def _make(db):
    from isaacsim.core.prims import Articulation
    art = Articulation("/World/aic/Robot/aic_unified_robot")
    art.initialize()
    db.per_instance_state.art = art
    db.per_instance_state.idx = list(art.body_names).index("ati_tool_link")


def compute(db):
    import numpy as np
    import omni.timeline
    st = db.per_instance_state
    try:
        art = st.art
        w = np.asarray(art.get_measured_joint_forces())[0, st.idx]
    except Exception:
        # physics view was reset (e.g. render products created) -> re-init
        _make(db)
        art = db.per_instance_state.art
        w = np.asarray(art.get_measured_joint_forces())[0, st.idx]
    db.outputs.fx = float(w[0])
    db.outputs.fy = float(w[1])
    db.outputs.fz = float(w[2])
    db.outputs.tx = float(w[3])
    db.outputs.ty = float(w[4])
    db.outputs.tz = float(w[5])
    t = omni.timeline.get_timeline_interface().get_current_time()
    db.outputs.sec = int(t)
    db.outputs.nanosec = int(round((t - int(t)) * 1e9))
    return True
