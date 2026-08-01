ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def _make(db):
    from isaacsim.core.prims import Articulation
    art = Articulation("/World/aic/Robot/aic_unified_robot")
    art.initialize()
    names = list(art.dof_names)
    db.per_instance_state.art = art
    db.per_instance_state.idx = [names.index(a) for a in ARM]


def compute(db):
    import numpy as np
    st = db.per_instance_state
    try:
        art = st.art
        q = np.asarray(art.get_joint_positions())[0]
        qd = np.asarray(art.get_joint_velocities())[0]
    except Exception:
        # physics view was reset (e.g. render products created) -> re-init
        _make(db)
        art = db.per_instance_state.art
        q = np.asarray(art.get_joint_positions())[0]
        qd = np.asarray(art.get_joint_velocities())[0]
    idx = db.per_instance_state.idx
    db.outputs.names = ARM
    db.outputs.positions = [float(q[i]) for i in idx]
    db.outputs.velocities = [float(qd[i]) for i in idx]
    db.outputs.efforts = [0.0] * len(idx)
    db.outputs.dofTypes = [0] * len(idx)
    return True
