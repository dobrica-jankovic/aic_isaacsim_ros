"""ScriptNode body: read one joint group for a clean ``/joint_states``.

``ROS2PublishJointState`` is all-or-nothing -- pointed at this articulation it
emits all 46 DOF, including duplicated cable-joint names, which breaks MoveIt.
Running it in manual mode fed from here keeps the message to the arm joints.

CONFIG keys: ``robot_prim``, ``joints``.

Everything is nested inside ``compute`` on purpose -- see the package docstring.
"""


def compute(db):
    import numpy as np

    CONFIG = {}

    def init():
        from isaacsim.core.prims import Articulation

        articulation = Articulation(CONFIG["robot_prim"])
        articulation.initialize()
        dof_names = list(articulation.dof_names)
        db.per_instance_state.articulation = articulation
        db.per_instance_state.indices = [dof_names.index(n) for n in CONFIG["joints"]]

    def read():
        articulation = db.per_instance_state.articulation
        return (
            np.asarray(articulation.get_joint_positions())[0],
            np.asarray(articulation.get_joint_velocities())[0],
        )

    try:
        positions, velocities = read()
    except Exception:
        # Creating camera render products resets the physics simulation view,
        # which invalidates cached articulation handles. Rebuild and retry.
        init()
        positions, velocities = read()

    indices = db.per_instance_state.indices
    db.outputs.names = list(CONFIG["joints"])
    db.outputs.positions = [float(positions[i]) for i in indices]
    db.outputs.velocities = [float(velocities[i]) for i in indices]
    db.outputs.efforts = [0.0] * len(indices)
    db.outputs.dofTypes = [0] * len(indices)
    return True
