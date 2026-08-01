"""Build the AIC UR5e ROS 2 bridge action graph in a running Isaac Sim.

Send this to the running sim via the isaac-sim-remote skill:
    python3 scripts/isaacsim_send.py --file build_ros2_bridge.py

Prereq: the AIC scene must already be loaded (scripts/load_aic_scene.py) and
the timeline must be playing (the ROS 2 bridge only publishes while playing).

Topics created:
    /clock                         rosgraph_msgs/Clock
    /joint_states                  sensor_msgs/JointState      (6 UR5e arm joints)
    /joint_command   (subscribe)   sensor_msgs/JointState  ->  drives the arm
    /tf                            tf2_msgs/TFMessage          (all robot links)
    /aic/{center,left,right}_camera/rgb          sensor_msgs/Image
    /aic/{center,left,right}_camera/camera_info  sensor_msgs/CameraInfo
    /wrist_ft/wrench               geometry_msgs/WrenchStamped (ati_tool_link 6D)
"""
import omni.graph.core as og
import omni.usd, omni.kit.app
import carb.settings
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, Gf

REPO = "/home/etfrobot/Documents/dobrica/aic_isaacsim_ros"
ARMSTATE_BODY = REPO + "/scripts/armstate_body.py"
WRENCH_BODY = REPO + "/scripts/wrench_body.py"

GRAPH = "/World/aic/ROS2_Graph"
ROBOT = "/World/aic/Robot/aic_unified_robot"
ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
CAMS = ["center", "left", "right"]
# PinholeCamera intrinsics from the IsaacLab task (builders.py::_build_camera_cfg)
CAM = dict(focal=22.48, h_ap=20.955, v_ap=18.627, clip=(0.07, 20.0), res=(224, 224))
# UR5e actuator gains from the IsaacLab task (robots.py::UR5E_ARM_ACTUATOR)
ARM_STIFFNESS, ARM_DAMPING = 2000.0, 100.0

OUT = og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
stage = omni.usd.get_context().get_stage()
app = omni.kit.app.get_app()

# allow script-node execution headless
s = carb.settings.get_settings()
s.set("/app/omni.graph.scriptnode/enable_opt_in", False)

# articulation root prim (carries ArticulationRootAPI)
ART_ROOT = [p.GetPath().pathString for p in Usd.PrimRange(stage.GetPrimAtPath("/World/aic/Robot"))
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)][0]

# apply arm actuator gains so /joint_command tracks accurately
for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)):
    if p.GetName() in ARM and p.IsA(UsdPhysics.Joint):
        d = UsdPhysics.DriveAPI.Get(p, "angular")
        if d:
            d.GetStiffnessAttr().Set(ARM_STIFFNESS)
            d.GetDampingAttr().Set(ARM_DAMPING)

# fresh graph
if stage.GetPrimAtPath(GRAPH):
    stage.RemovePrim(GRAPH)

keys = og.Controller.Keys
graph = og.Controller.create_graph({"graph_path": GRAPH, "evaluator_name": "execution"})

def node(name, ntype):
    og.Controller.edit(graph, {keys.CREATE_NODES: [(name, ntype)]})
    return og.Controller.node(name, graph)

def A(name, attr):
    return og.Controller.attribute(attr, og.Controller.node(name, graph))

def conn(a_node, a_attr, b_node, b_attr):
    og.Controller.connect(A(a_node, a_attr), A(b_node, b_attr))

def set_rel(node_name, rel, targets):
    prim = stage.GetPrimAtPath(GRAPH + "/" + node_name)
    prim.CreateRelationship(rel).SetTargets([Sdf.Path(t) for t in targets])

def setv(node_name, attr, value):
    """Set an input value both at runtime and *authored into USD*.

    og.Controller.set alone only writes the runtime (Fabric) value — the stage
    keeps no opinion, so a saved-and-reopened graph silently falls back to OGN
    defaults (e.g. topicName "/rgb"). Authoring the USD attribute is what makes
    the stage reloadable.
    """
    og.Controller.set(A(node_name, attr), value)
    a = stage.GetPrimAtPath(GRAPH + "/" + node_name).GetAttribute(attr)
    if not a:
        raise RuntimeError("no USD attribute %s on %s" % (attr, node_name))
    a.Set(value)

# --- base nodes ---
node("OnTick", "omni.graph.action.OnPlaybackTick")
node("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime")
node("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock")
setv("PublishClock", "inputs:topicName", "/clock")
conn("OnTick", "outputs:tick", "PublishClock", "inputs:execIn")
conn("ReadSimTime", "outputs:simulationTime", "PublishClock", "inputs:timeStamp")

# --- /joint_states (arm-only, via script node feeding manual-mode publisher) ---
sn = node("ReadArmState", "omni.graph.scriptnode.ScriptNode")
setv("ReadArmState", "inputs:usePath", True)
setv("ReadArmState", "inputs:scriptPath", ARMSTATE_BODY)
for an, at in [("outputs:names", "token[]"), ("outputs:positions", "double[]"),
               ("outputs:velocities", "double[]"), ("outputs:efforts", "double[]"),
               ("outputs:dofTypes", "uchar[]")]:
    og.Controller.create_attribute(sn, an, at, OUT)
node("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState")
setv("PublishJointState", "inputs:topicName", "/joint_states")
setv("PublishJointState", "inputs:stageMetersPerUnit", 1.0)
conn("OnTick", "outputs:tick", "ReadArmState", "inputs:execIn")
conn("ReadArmState", "outputs:execOut", "PublishJointState", "inputs:execIn")
conn("ReadSimTime", "outputs:simulationTime", "PublishJointState", "inputs:timeStamp")
for a in ["names:jointNames", "positions:jointPositions", "velocities:jointVelocities",
          "efforts:jointEfforts", "dofTypes:jointDofTypes"]:
    src, dst = a.split(":")
    conn("ReadArmState", "outputs:" + src, "PublishJointState", "inputs:" + dst)

# --- /joint_command (subscribe) -> articulation controller ---
node("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState")
setv("SubscribeJointState", "inputs:topicName", "/joint_command")
node("ArtController", "isaacsim.core.nodes.IsaacArticulationController")
set_rel("ArtController", "inputs:targetPrim", [ART_ROOT])
conn("OnTick", "outputs:tick", "SubscribeJointState", "inputs:execIn")
conn("SubscribeJointState", "outputs:execOut", "ArtController", "inputs:execIn")
for f in ["jointNames", "positionCommand", "velocityCommand", "effortCommand"]:
    conn("SubscribeJointState", "outputs:" + f, "ArtController", "inputs:" + f)

# --- 3 cameras: prim + render product + rgb/info helpers ---
for name in CAMS:
    cam_path = f"{ROBOT}/{name}_camera_optical/{name}_camera"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    cam.CreateFocalLengthAttr(CAM["focal"])
    cam.CreateHorizontalApertureAttr(CAM["h_ap"])
    cam.CreateVerticalApertureAttr(CAM["v_ap"])
    cam.CreateClippingRangeAttr(Gf.Vec2f(*CAM["clip"]))
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddOrientOp().Set(Gf.Quatf(0.0, 1.0, 0.0, 0.0))  # 180 about X: USD cam -> ROS optical
    frame_id = f"{name}_camera_optical"

    # Render product is created *by the graph*, not by a Python-side Replicator
    # call: rep.create.render_product authors into the session layer, which is
    # dropped on save, leaving the helpers pointing at a dead path. This node
    # recreates it from USD data on every play, so the stage is saveable.
    crp = f"Cam_{name}_rp"
    node(crp, "isaacsim.core.nodes.IsaacCreateRenderProduct")
    set_rel(crp, "inputs:cameraPrim", [cam_path])
    setv(crp, "inputs:width", CAM["res"][0])
    setv(crp, "inputs:height", CAM["res"][1])
    conn("OnTick", "outputs:tick", crp, "inputs:execIn")

    rgb = f"Cam_{name}_rgb"
    node(rgb, "isaacsim.ros2.bridge.ROS2CameraHelper")
    conn(crp, "outputs:renderProductPath", rgb, "inputs:renderProductPath")
    setv(rgb, "inputs:type", "rgb")
    setv(rgb, "inputs:topicName", f"/aic/{name}_camera/rgb")
    setv(rgb, "inputs:frameId", frame_id)
    conn(crp, "outputs:execOut", rgb, "inputs:execIn")

    info = f"Cam_{name}_info"
    node(info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper")
    conn(crp, "outputs:renderProductPath", info, "inputs:renderProductPath")
    setv(info, "inputs:topicName", f"/aic/{name}_camera/camera_info")
    setv(info, "inputs:frameId", frame_id)
    conn(crp, "outputs:execOut", info, "inputs:execIn")

# --- /tf: every rigid-body link relative to World ---
node("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree")
setv("PublishTF", "inputs:topicName", "/tf")
links = [p.GetPath().pathString for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT))
         if p.HasAPI(UsdPhysics.RigidBodyAPI)]
set_rel("PublishTF", "inputs:parentPrim", ["/World"])
set_rel("PublishTF", "inputs:targetPrims", links)
conn("OnTick", "outputs:tick", "PublishTF", "inputs:execIn")
conn("ReadSimTime", "outputs:simulationTime", "PublishTF", "inputs:timeStamp")

# --- /wrist_ft/wrench: script node -> generic WrenchStamped publisher ---
wn = node("ReadWrench", "omni.graph.scriptnode.ScriptNode")
setv("ReadWrench", "inputs:usePath", True)
setv("ReadWrench", "inputs:scriptPath", WRENCH_BODY)
for an, at in [("outputs:fx", "double"), ("outputs:fy", "double"), ("outputs:fz", "double"),
               ("outputs:tx", "double"), ("outputs:ty", "double"), ("outputs:tz", "double"),
               ("outputs:sec", "int"), ("outputs:nanosec", "uint")]:
    og.Controller.create_attribute(wn, an, at, OUT)
node("WrenchPub", "isaacsim.ros2.bridge.ROS2Publisher")
setv("WrenchPub", "inputs:messagePackage", "geometry_msgs")
setv("WrenchPub", "inputs:messageSubfolder", "msg")
setv("WrenchPub", "inputs:messageName", "WrenchStamped")
setv("WrenchPub", "inputs:topicName", "/wrist_ft/wrench")

# tick so camera attrs settle and the WrenchStamped message fields get created
for _ in range(30):
    app.update()

setv("WrenchPub", "inputs:header:frame_id", "ati_tool_link")
conn("OnTick", "outputs:tick", "ReadWrench", "inputs:execIn")
conn("ReadWrench", "outputs:execOut", "WrenchPub", "inputs:execIn")
for src, dst in [("fx", "wrench:force:x"), ("fy", "wrench:force:y"), ("fz", "wrench:force:z"),
                 ("tx", "wrench:torque:x"), ("ty", "wrench:torque:y"), ("tz", "wrench:torque:z"),
                 ("sec", "header:stamp:sec"), ("nanosec", "header:stamp:nanosec")]:
    conn("ReadWrench", "outputs:" + src, "WrenchPub", "inputs:" + dst)

for _ in range(40):
    app.update()

print("=== BUILD REPORT ===")
for n in graph.get_nodes():
    nm = n.get_prim_path().split("/")[-1]
    errs = list(n.get_compute_messages(og.Severity.ERROR))
    if errs:
        print(f"  {nm}: {errs[0][:120]}")
print("nodes:", len(list(graph.get_nodes())))
print("ROS2 BRIDGE BUILT")
