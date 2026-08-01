import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Gf, Sdf

ASSET_DIR = "/home/etfrobot/IsaacLab/etf_robotics_aic/source/aic_task/aic_task/assets"

# (scene prim path, usd file, pos, rot wxyz)  -- poses from asset_specs/scene.py
SLOTS = [
    ("/World/aic/Robot",     f"{ASSET_DIR}/robots/ur5e_cable/aic_unified_robot_cable_sdf.usd", (-0.18, -0.122, 0.0),      (0.0, 0.0, 0.0, 1.0)),
    ("/World/aic/workcell",  f"{ASSET_DIR}/workcells/aic/aic.usd",                              (0.0, 0.0, -1.15),         (1.0, 0.0, 0.0, 0.0)),
    ("/World/aic/board",     f"{ASSET_DIR}/workcells/task_board/task_board_rigid.usd",          (0.2837, 0.229, 0.0),      (1.0, 0.0, 0.0, 0.0)),
    ("/World/aic/sc_port_1", f"{ASSET_DIR}/targets/sc_port/sc_port.usd",                        (0.2904, 0.1928, 0.005),   (0.73136, 0.0, 0.0, -0.682)),
    ("/World/aic/sc_port_2", f"{ASSET_DIR}/targets/sc_port/sc_port.usd",                        (0.2913, 0.1507, 0.005),   (0.73136, 0.0, 0.0, -0.682)),
    ("/World/aic/target",    f"{ASSET_DIR}/targets/nic_card/nic_card.usd",                      (0.25135, 0.25229, 0.0743),(0.0, 0.0, -0.7068252, 0.7073883)),
]

ctx = omni.usd.get_context()
stage = ctx.get_stage()

# Ensure /World
if not stage.GetPrimAtPath("/World"):
    UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

# Physics scene
if not stage.GetPrimAtPath("/World/PhysicsScene"):
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

# Dome light
if not stage.GetPrimAtPath("/World/DomeLight"):
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000.0)

# Ground plane (Z-up collision plane)
if not stage.GetPrimAtPath("/World/GroundPlane"):
    gplane = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
    gplane.CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(gplane.GetPrim())

def set_pose(prim, pos, rot_wxyz):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    w, x, y, z = rot_wxyz
    xf.AddOrientOp().Set(Gf.Quatf(w, x, y, z))

report = []
for prim_path, usd_file, pos, rot in SLOTS:
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().ClearReferences()
    ok = prim.GetReferences().AddReference(usd_file)
    set_pose(prim, pos, rot)
    n_children = len(list(Usd.PrimRange(prim)))
    # articulation root?
    art = [p.GetPath().pathString for p in Usd.PrimRange(prim)
           if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    report.append((prim_path, n_children, art))

print("=== LOAD REPORT ===")
for prim_path, n, art in report:
    print(f"{prim_path:24s} prims={n:5d}  artRoot={art}")

total = len(list(Usd.PrimRange(stage.GetPrimAtPath('/World'))))
print(f"Total prims under /World: {total}")
