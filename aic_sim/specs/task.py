"""Task-level choices for the AIC port-insertion scene.

``base``/``robots``/``targets``/``workcells``/``scene`` in this package are a
straight mirror of the IsaacLab ``aic_task.asset_specs`` package -- keep them
that way so upstream changes can be applied by hand. This module is where
deliberate divergence lives.

It is *adapted*, not mirrored, from the IsaacLab task layer at
``~/IsaacLab/etf_robotics_aic/source/aic_task/aic_task/tasks/manager_based/
port_insertion``:

- ``specs.py::NIC_PORT_0_INSERTION_GOAL`` -> :data:`AIC_NIC_PORT_0_GOAL`
- ``specs.py::AIC_PORT_INSERTION_OBSERVATION`` -> the lateral threshold on it
- ``builders.py`` scene light + ``randomize_light`` event -> :data:`AIC_DOME_LIGHT`
  and :data:`AIC_DOME_LIGHT_RANDOMIZATION`
- ``builders.py::_build_camera_cfg``'s ``PinholeCameraCfg`` -> :data:`AIC_CAMERA_LENS`
  (resolution stays on ``CameraFrameSpec``, where upstream also keeps it)

Everything that only means something inside an IsaacLab manager (command names,
resampling windows, event-term modes, the diff-IK action) is dropped: a plain
Isaac Sim run has no command manager to feed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .base import PoseSpec, Vector3
from .robots import ROBOT_ROLE_EEF
from .scene import SCENE_SLOT_TARGET
from .targets import NIC_SFP_PORT_0


@dataclass(frozen=True)
class DomeLightSpec:
    """Scene dome light. Mirrors ``builders.py``'s ``DomeLightCfg``."""

    prim_path: str
    intensity: float
    color: Vector3


@dataclass(frozen=True)
class DomeLightRandomizationSpec:
    """Reset randomization ranges for the scene dome light."""

    intensity_range: tuple[float, float]
    color_range: tuple[Vector3, Vector3]


@dataclass(frozen=True)
class CameraLensSpec:
    """Pinhole lens shared by every camera on the robot, in USD units (mm)."""

    focal_length: float
    horizontal_aperture: float
    vertical_aperture: float
    clipping_range: tuple[float, float]


@dataclass(frozen=True)
class InsertionGoalSpec:
    """The single insertion goal the scene exposes as ground truth.

    ``entrance``/``seat`` are frames inside the *target* asset. The goal pose
    is that frame composed with :attr:`eef_pose_in_port_frame` -- i.e. where the
    robot's insertion tip should end up, not where the port frame itself is.
    """

    target_slot: str
    port_name: str
    tip_body_role: str
    eef_pose_in_port_frame: PoseSpec
    lateral_threshold_m: float


# Upstream ``builders.py``: DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0)
# at ``/World/light``. The prim path is ours -- nothing outside this repo names it.
AIC_DOME_LIGHT = DomeLightSpec(
    prim_path="/World/DomeLight",
    intensity=2500.0,
    color=(0.75, 0.75, 0.75),
)

# Upstream ``builders.py::randomize_light`` event-term params.
AIC_DOME_LIGHT_RANDOMIZATION = DomeLightRandomizationSpec(
    intensity_range=(1500.0, 3500.0),
    color_range=((0.5, 0.5, 0.5), (1.0, 1.0, 1.0)),
)

# Upstream ``builders.py::_build_camera_cfg``. Aperture and focal length give
# fx = fy = 224 * 22.48 / 20.955 = 240.3 px at the 224x224 resolution the
# camera frame specs carry; ``/camera_info`` reports exactly that.
AIC_CAMERA_LENS = CameraLensSpec(
    focal_length=22.48,
    horizontal_aperture=20.955,
    vertical_aperture=18.627,
    clipping_range=(0.07, 20.0),
)

AIC_NIC_PORT_0_GOAL = InsertionGoalSpec(
    target_slot=SCENE_SLOT_TARGET,
    port_name=NIC_SFP_PORT_0.name,
    tip_body_role=ROBOT_ROLE_EEF,
    eef_pose_in_port_frame=PoseSpec(
        pos=(0.0, 0.0, 0.0013),
        rot=(math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0),
    ),
    lateral_threshold_m=0.002,
)


__all__ = [
    "AIC_CAMERA_LENS",
    "AIC_DOME_LIGHT",
    "AIC_DOME_LIGHT_RANDOMIZATION",
    "AIC_NIC_PORT_0_GOAL",
    "CameraLensSpec",
    "DomeLightRandomizationSpec",
    "DomeLightSpec",
    "InsertionGoalSpec",
]
