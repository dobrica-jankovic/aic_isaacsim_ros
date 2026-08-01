"""Shared primitives for reusable asset and scene-layout specs.

Vendored from the IsaacLab task at
``~/IsaacLab/etf_robotics_aic/source/aic_task/aic_task/asset_specs`` so this
repo runs without an IsaacLab checkout. Pure dataclasses -- no torch, no
isaaclab. Only ``ASSET_DIR`` differs from upstream: it points at this repo's
vendored ``assets/``. Mirror upstream changes by hand.

Asset specs describe facts that stay true wherever a USD asset is used.
Scene slot specs describe how one asset is instantiated in one layout.
Keeping those concepts separate lets the port-insertion task swap assets
without spreading USD paths, body names, or default poses through task code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal


REPO_DIR = Path(__file__).resolve().parents[2]
ASSET_DIR = REPO_DIR / "assets"


def asset_path(*parts: str) -> str:
    """Return an absolute path inside this repo's vendored ``assets/``."""

    return str(ASSET_DIR.joinpath(*parts))


AssetRole = Literal["robot", "target", "workcell", "fixture"]
SceneRole = Literal["robot", "target", "board", "workcell", "auxiliary"]
UsdAssetKind = Literal["articulation", "rigid_object", "static"]
Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseSpec:
    """Position and quaternion in IsaacLab's ``wxyz`` quaternion convention."""

    pos: Vector3 = (0.0, 0.0, 0.0)
    rot: Quaternion = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class AssetIdentity:
    """Stable identity for an asset family, independent of file location."""

    name: str
    role: AssetRole


@dataclass(frozen=True)
class UsdAssetInterface:
    """USD-level interface exposed by an asset file."""

    kind: UsdAssetKind
    root_prim: str | None = None


@dataclass(frozen=True)
class JointGroupSpec:
    """A named joint group belonging to a robot articulation."""

    name: str
    joint_names: tuple[str, ...]
    default_positions: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class BodyRoleSpec:
    """Semantic robot role resolved to a concrete articulation body."""

    role: str
    body_name: str
    purpose: str = ""


@dataclass(frozen=True)
class CameraFrameSpec:
    """Camera prim exposed by a robot asset."""

    name: str
    relative_prim_path: str
    height: int = 224
    width: int = 224
    data_types: tuple[str, ...] = ("rgb",)


@dataclass(frozen=True)
class ActuatorSpec:
    """Robot actuator defaults for one joint group."""

    name: str
    joint_group: str
    effort_limit_sim: float
    stiffness: float
    damping: float


@dataclass(frozen=True)
class RobotSpawnSpec:
    """Robot-specific physics defaults used when building an ArticulationCfg."""

    max_depenetration_velocity: float = 5.0
    enabled_self_collisions: bool = True
    solver_position_iteration_count: int = 32
    solver_velocity_iteration_count: int = 16
    activate_contact_sensors: bool = False


@dataclass(frozen=True)
class AssetSpec:
    """Reusable asset contract.

    This must stay true no matter where the asset is placed. Do not put task
    targets, randomization ranges, insertion offsets, or selected ports here.
    """

    identity: AssetIdentity
    usd: UsdAssetInterface
    usd_path: str

    @property
    def name(self) -> str:
        """Concrete asset family name, not the role it plays in a scene."""

        return self.identity.name


@dataclass(frozen=True)
class SceneSlotSpec:
    """One named asset instance in an environment layout.

    ``name`` is the stable key that scripts should use with ``env.scene[name]``.
    It should describe the role in this scene, for example ``target``, not the
    concrete model family, for example ``nic_card``.
    """

    name: str
    role: SceneRole
    asset: AssetSpec
    prim_path: str
    pose: PoseSpec = PoseSpec()
    kinematic: bool = False
    purpose: str = ""
