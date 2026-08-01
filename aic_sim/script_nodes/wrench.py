"""ScriptNode body: read the 6D joint-reaction wrench at the F/T sensor body.

OmniGraph has no native WrenchStamped publisher, so this feeds a generic
``ROS2Publisher`` with the message fields flattened (``wrench:force:x`` ...).
The wrench is expressed in the sensor body's own frame, which is what a
wrist-mounted F/T sensor reports in hardware.

CONFIG keys: ``robot_prim``, ``body``.

Everything is nested inside ``compute`` on purpose -- see the package docstring.
"""


def compute(db):
    import numpy as np
    import omni.timeline

    CONFIG = {}

    def init():
        from isaacsim.core.prims import Articulation

        articulation = Articulation(CONFIG["robot_prim"])
        articulation.initialize()
        db.per_instance_state.articulation = articulation
        db.per_instance_state.index = list(articulation.body_names).index(
            CONFIG["body"]
        )

    def read():
        state = db.per_instance_state
        return np.asarray(state.articulation.get_measured_joint_forces())[0, state.index]

    try:
        wrench = read()
    except Exception:
        # Physics view reset (see arm_state.py) -- rebuild and retry.
        init()
        wrench = read()

    db.outputs.fx = float(wrench[0])
    db.outputs.fy = float(wrench[1])
    db.outputs.fz = float(wrench[2])
    db.outputs.tx = float(wrench[3])
    db.outputs.ty = float(wrench[4])
    db.outputs.tz = float(wrench[5])

    now = omni.timeline.get_timeline_interface().get_current_time()
    db.outputs.sec = int(now)
    db.outputs.nanosec = int(round((now - int(now)) * 1e9))
    return True
