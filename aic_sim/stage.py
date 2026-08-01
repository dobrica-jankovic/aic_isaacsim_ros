"""Build an AIC scene on a USD stage from a :class:`SceneLayoutSpec`.

Every pose, prim path and USD file comes from ``aic_sim.specs`` -- nothing here
is hardcoded per asset, so a new layout (or a new robot) is a spec change, not
a code change.

Needs ``pxr`` only, so it works both inside a running Isaac Sim (via the
isaac-sim-remote socket) and under a standalone ``SimulationApp``.
"""

from __future__ import annotations

import math
import os

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

from .specs import (
    AIC_DOME_LIGHT,
    DomeLightSpec,
    PoseSpec,
    Quaternion,
    RobotAssetSpec,
    SceneLayoutSpec,
    SceneSlotSpec,
    Vector3,
)


#: Scene root the layout's ``{ENV_REGEX_NS}`` placeholder expands to. IsaacLab
#: uses that placeholder to clone one env per worker; a plain Isaac Sim run has
#: exactly one, so it resolves to a fixed prim.
DEFAULT_ROOT = "/World/aic"

GROUND_PLANE_PATH = "/World/GroundPlane"
PHYSICS_SCENE_PATH = "/World/PhysicsScene"


def slot_prim_path(slot: SceneSlotSpec, root: str = DEFAULT_ROOT) -> str:
    """Resolve a slot's ``{ENV_REGEX_NS}``-templated prim path under ``root``."""

    return slot.prim_path.replace("{ENV_REGEX_NS}", root)


def set_pose(prim: Usd.Prim, pos: Vector3, rot: Quaternion) -> None:
    """Author a translate+orient xform op pair. ``rot`` is wxyz (IsaacLab)."""

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xform.AddOrientOp().Set(Gf.Quatf(rot[0], rot[1], rot[2], rot[3]))


def asset_reference(stage: Usd.Stage, usd_path: str) -> str:
    """Return ``usd_path`` relative to the stage's own layer when it has one.

    A stage that already lives on disk gets a repo-relative reference, so the
    saved ``.usd`` is portable to any clone. An anonymous (never-saved) stage
    has nothing to anchor against, so it keeps the absolute path -- save the
    stage *before* building if you want the portable form.
    """

    layer_path = stage.GetRootLayer().realPath
    if not layer_path:
        return usd_path
    return os.path.relpath(usd_path, os.path.dirname(layer_path))


def ensure_world(
    stage: Usd.Stage,
    *,
    light: DomeLightSpec | None = AIC_DOME_LIGHT,
    ground_plane: bool = True,
) -> None:
    """Create ``/World`` plus the physics scene, dome light and ground plane."""

    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH):
        scene = UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

    if light is not None:
        dome = UsdLux.DomeLight.Define(stage, light.prim_path)
        dome.CreateIntensityAttr(light.intensity)
        dome.CreateColorAttr(Gf.Vec3f(*light.color))

    if ground_plane and not stage.GetPrimAtPath(GROUND_PLANE_PATH):
        plane = UsdGeom.Plane.Define(stage, GROUND_PLANE_PATH)
        plane.CreateAxisAttr("Z")
        UsdPhysics.CollisionAPI.Apply(plane.GetPrim())


def spawn_slot(
    stage: Usd.Stage,
    slot: SceneSlotSpec,
    *,
    root: str = DEFAULT_ROOT,
    pose: PoseSpec | None = None,
) -> str:
    """Reference a slot's asset into the stage at its (or an override) pose."""

    prim_path = slot_prim_path(slot, root)
    prim = stage.DefinePrim(prim_path, "Xform")
    references = prim.GetReferences()
    references.ClearReferences()
    references.AddReference(asset_reference(stage, slot.asset.usd_path))
    set_pose(prim, *_pose_tuple(pose or slot.pose))
    if slot.kinematic:
        set_kinematic(stage, prim_path)
    return prim_path


def set_kinematic(stage: Usd.Stage, prim_path: str) -> None:
    """Make every rigid body under ``prim_path`` kinematic.

    The board, the ports and the target are fixtures: the task moves them
    between episodes but gravity must not. IsaacLab gets this from
    ``RigidObjectCfg.spawn.kinematic_enabled``; on a plain stage it has to be
    authored, or they sag out of the layout the specs describe.
    """

    for prim in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)


def build_scene(
    stage: Usd.Stage,
    layout: SceneLayoutSpec,
    *,
    root: str = DEFAULT_ROOT,
    include_robot: bool = True,
    light: DomeLightSpec | None = AIC_DOME_LIGHT,
    ground_plane: bool = True,
) -> dict[str, str]:
    """Build ``layout`` under ``root``. Returns ``{slot name: prim path}``.

    ``include_robot=False`` gives the passive workcell used for asset-pose and
    lighting randomization, where the arm is just in the way.
    """

    ensure_world(stage, light=light, ground_plane=ground_plane)
    UsdGeom.Xform.Define(stage, root)

    spawned: dict[str, str] = {}
    for slot in layout.all_slots():
        if slot.role == "robot" and not include_robot:
            continue
        spawned[slot.name] = spawn_slot(stage, slot, root=root)

    if include_robot:
        apply_default_joint_positions(
            stage, spawned[layout.robot_slot.name], layout.robot_slot.asset
        )
    return spawned


def describe_scene(stage: Usd.Stage, spawned: dict[str, str]) -> str:
    """One line per spawned slot: prim count and any articulation roots."""

    lines = []
    for name, prim_path in spawned.items():
        prim = stage.GetPrimAtPath(prim_path)
        prims = list(Usd.PrimRange(prim))
        articulations = [
            p.GetPath().pathString
            for p in prims
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        lines.append(
            f"{name:12s} {prim_path:24s} prims={len(prims):5d} artRoot={articulations}"
        )
    total = len(list(Usd.PrimRange(stage.GetPrimAtPath("/World"))))
    lines.append(f"total prims under /World: {total}")
    return "\n".join(lines)


def asset_root_prim_path(
    stage: Usd.Stage, slot_path: str, root_prim: str | None
) -> str:
    """Return the prim inside a spawned slot that its asset spec calls the root.

    Referencing a USD file pulls in its ``defaultPrim``. Most AIC assets nest
    their root under a wrapper (``sc_port.usd`` -> ``/World/sc_port_visual``),
    so the spec's ``root_prim`` becomes a child of the slot Xform. But
    ``nic_card.usd`` declares ``nic_card_link`` *as* its defaultPrim, so that
    level collapses and the slot Xform itself is the root. Try nested first,
    fall back to the slot.
    """

    if root_prim is None:
        return slot_path
    nested = f"{slot_path}/{root_prim}"
    if stage.GetPrimAtPath(nested).IsValid():
        return nested
    return slot_path


def articulation_root_path(stage: Usd.Stage, robot_prim_path: str) -> str:
    """Return the prim carrying ``ArticulationRootAPI`` under the robot slot.

    This is what ``IsaacArticulationController`` and the joint-state readers
    must target -- not the slot Xform above it.
    """

    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim.GetPath().pathString
    raise RuntimeError(f"No ArticulationRootAPI prim under '{robot_prim_path}'.")


def body_prim_path(stage: Usd.Stage, robot_prim_path: str, body_name: str) -> str:
    """Find the rigid-body prim for an articulation body name.

    Body names do *not* map onto ``<asset root>/<body name>``. One articulation
    can span sibling branches of the slot -- the UR5e's ``ati_tool_link`` sits
    under ``aic_unified_robot`` while its ``sfp_tip_link`` sits under ``cable``
    -- and a branch may hold a same-named prim that is not a body at all. So
    search the whole slot for a rigid body with exactly this name.
    """

    matches = [
        prim.GetPath().pathString
        for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path))
        if prim.GetName() == body_name and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one rigid body named '{body_name}' under "
            f"'{robot_prim_path}', found {matches}."
        )
    return matches[0]


def apply_default_joint_positions(
    stage: Usd.Stage, robot_prim_path: str, robot: RobotAssetSpec
) -> None:
    """Author the spec's default joint pose as the robot's spawn state.

    Without this the arm spawns in whatever pose the raw USD was authored in,
    not the task's default -- so the cameras look somewhere else and the first
    ``/joint_command`` yanks the arm across the workcell. IsaacLab gets this
    from ``ArticulationCfg.InitialStateCfg``; a plain stage has to author it.

    USD joint angles are in degrees, the specs are in radians.
    """

    defaults = {
        name: value
        for group in robot.joint_groups
        for name, value in group.default_positions.items()
    }
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
        radians = defaults.get(prim.GetName())
        if radians is None or not prim.IsA(UsdPhysics.Joint):
            continue
        degrees = math.degrees(radians)
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            drive.CreateTargetPositionAttr().Set(degrees)
        for attribute, value in (
            ("state:angular:physics:position", degrees),
            ("state:angular:physics:velocity", 0.0),
        ):
            joint_state = prim.GetAttribute(attribute)
            if joint_state:
                joint_state.Set(value)


def _pose_tuple(pose: PoseSpec) -> tuple[Vector3, Quaternion]:
    return pose.pos, pose.rot


__all__ = [
    "DEFAULT_ROOT",
    "apply_default_joint_positions",
    "articulation_root_path",
    "asset_reference",
    "asset_root_prim_path",
    "body_prim_path",
    "build_scene",
    "describe_scene",
    "ensure_world",
    "set_kinematic",
    "set_pose",
    "slot_prim_path",
    "spawn_slot",
]
