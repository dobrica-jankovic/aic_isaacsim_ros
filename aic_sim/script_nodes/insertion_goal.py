"""ScriptNode body: ground-truth insertion goal poses and progress.

Mirrors the IsaacLab task's ``InsertionGoalCommand`` and the
``insertion_fraction`` observation, which together make up its ``cheatcode``
observation group. This is privileged information -- a real robot cannot
measure it. It is here for dataset recording, scripted demonstrations and
asymmetric critics, not as something a policy may consume.

The port entrance and seat are Xform prims *inside* the target asset, so their
transform relative to the target's rigid body never changes: it is read from USD
once, composed with the fixed EEF-in-port offset, and then carried by the
target's live pose every tick. Reading it from USD each tick would be wrong --
USD xforms go stale while PhysX owns the body.

CONFIG keys: ``target_prim``, ``entrance_prim``, ``seat_prim``, ``tip_prim``,
``eef_pos``, ``eef_quat`` (wxyz), ``lateral_threshold``.

Everything is nested inside ``compute`` on purpose -- see the package docstring.
"""


def compute(db):
    import math

    import numpy as np
    import omni.timeline

    CONFIG = {}

    def quat_mul(q1, q2):
        """Hamilton product of two wxyz quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    def quat_rotate(q, v):
        """Rotate vector ``v`` by wxyz quaternion ``q``."""
        w, x, y, z = q
        tx = 2.0 * (y * v[2] - z * v[1])
        ty = 2.0 * (z * v[0] - x * v[2])
        tz = 2.0 * (x * v[1] - y * v[0])
        return (
            v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx),
        )

    def compose(parent, child):
        """Compose a child pose onto a parent pose, both ``(pos, wxyz)``."""
        rotated = quat_rotate(parent[1], child[0])
        return (
            tuple(parent[0][i] + rotated[i] for i in range(3)),
            quat_mul(parent[1], child[1]),
        )

    def pose_in_target_frame(prim_path):
        """Static pose of a port frame relative to the target's rigid body."""
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.XformCache()
        target = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath(CONFIG["target_prim"])
        )
        frame = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(prim_path))
        relative = frame * target.GetInverse()

        translation = relative.ExtractTranslation()
        rotation = relative.ExtractRotationQuat().GetNormalized()
        imaginary = rotation.GetImaginary()
        return (
            (translation[0], translation[1], translation[2]),
            (rotation.GetReal(), imaginary[0], imaginary[1], imaginary[2]),
        )

    def init():
        from isaacsim.core.prims import RigidPrim

        state = db.per_instance_state
        state.target = RigidPrim(CONFIG["target_prim"])
        state.tip = RigidPrim(CONFIG["tip_prim"])

        offset = (tuple(CONFIG["eef_pos"]), tuple(CONFIG["eef_quat"]))
        state.entrance_local = compose(pose_in_target_frame(CONFIG["entrance_prim"]), offset)
        state.seat_local = compose(pose_in_target_frame(CONFIG["seat_prim"]), offset)

    def world_pose(rigid_prim):
        positions, orientations = rigid_prim.get_world_poses()
        position = np.asarray(positions)[0]
        orientation = np.asarray(orientations)[0]
        return (
            tuple(float(value) for value in position),
            tuple(float(value) for value in orientation),
        )

    def read():
        state = db.per_instance_state
        return world_pose(state.target), world_pose(state.tip)

    def insertion_fraction(entrance_pos, seat_pos, tip_pos):
        """Position-only insertion progress, gated on staying near the axis.

        Decomposes the seat-to-tip error along the entrance->seat axis: progress
        is how far along that axis the tip has travelled, clamped to [0, 1], and
        forced to zero once the tip strays further off the line than
        ``lateral_threshold``.
        """
        axis = [seat_pos[i] - entrance_pos[i] for i in range(3)]
        length = max(math.sqrt(sum(a * a for a in axis)), 1e-9)
        axis = [a / length for a in axis]

        error = [seat_pos[i] - tip_pos[i] for i in range(3)]
        axial = sum(error[i] * axis[i] for i in range(3))
        perpendicular = [error[i] - axial * axis[i] for i in range(3)]
        lateral = math.sqrt(sum(p * p for p in perpendicular))

        progress = 1.0 - min(max(axial / length, 0.0), 1.0)
        return progress if lateral <= CONFIG["lateral_threshold"] else 0.0

    try:
        target_pose, tip_pose = read()
    except Exception:
        # Physics view reset (see arm_state.py) -- rebuild and retry.
        init()
        target_pose, tip_pose = read()

    state = db.per_instance_state
    entrance = compose(target_pose, state.entrance_local)
    seat = compose(target_pose, state.seat_local)

    for name, (position, orientation) in (
        ("entrance", entrance),
        ("seat", seat),
        ("tip", tip_pose),
    ):
        setattr(db.outputs, name + "_px", position[0])
        setattr(db.outputs, name + "_py", position[1])
        setattr(db.outputs, name + "_pz", position[2])
        setattr(db.outputs, name + "_qw", orientation[0])
        setattr(db.outputs, name + "_qx", orientation[1])
        setattr(db.outputs, name + "_qy", orientation[2])
        setattr(db.outputs, name + "_qz", orientation[3])

    db.outputs.fraction = insertion_fraction(entrance[0], seat[0], tip_pose[0])

    now = omni.timeline.get_timeline_interface().get_current_time()
    db.outputs.sec = int(now)
    db.outputs.nanosec = int(round((now - int(now)) * 1e9))
    return True
