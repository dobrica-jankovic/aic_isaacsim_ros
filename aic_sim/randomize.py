"""Reset randomization for the AIC layout, in plain USD.

Ported from the IsaacLab task's ``mdp/events.py`` (``randomize_board_and_parts``
and ``randomize_dome_light``) with torch and the env/manager plumbing removed.
The sampling maths is the same, so a seed here and a seed there produce the same
*distribution*, though not the same stream.

**Two deliberate differences from upstream:**

1. Upstream composes each part's new orientation onto the orientation cached
   from the *live* asset at the first reset. Here the base orientation comes
   straight from the slot spec, which is the pose that asset was spawned at --
   the same value, without needing a running physics view.
2. Upstream writes the pose to PhysX (``write_root_pose_to_sim``) *and* mirrors
   it onto the USD Xform. This module only authors USD. That is what goal 2
   needs -- randomize a passive scene with the timeline stopped -- but it means
   a pose applied mid-playback will not move an already-simulating rigid body.
   Sample with :func:`sample_layout`, then push through PhysX yourself if you
   ever need that.
3. Grid snapping rounds its range bounds with a tolerance, so a later edit to a
   range or step cannot quietly drop the top slot -- see :func:`_grid_index`.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

from .specs import (
    AIC_DOME_LIGHT,
    AIC_DOME_LIGHT_RANDOMIZATION,
    AxisRangeSpec,
    AxisSnapSpec,
    DomeLightRandomizationSpec,
    DomeLightSpec,
    PoseSpec,
    Quaternion,
    SceneLayoutSpec,
)
from .stage import DEFAULT_ROOT, slot_prim_path


#: Slack when rounding a range bound onto a snap grid, in grid steps.
_SNAP_TOL = 1e-9


@dataclass(frozen=True)
class LightSample:
    """A sampled dome light setting, returned so callers can log/replay it."""

    intensity: float
    color: tuple[float, float, float]


def sample_layout(
    layout: SceneLayoutSpec, rng: random.Random | None = None
) -> dict[str, PoseSpec]:
    """Sample new poses for the board and everything anchored to it.

    Returns ``{slot name: PoseSpec}`` in the scene-root frame, covering only the
    slots the randomization spec names. Sampling all-zero deltas reproduces the
    slots' spawn poses exactly -- the spec's board-local offsets and the slots'
    absolute poses agree by construction.
    """

    spec = layout.randomization
    if spec is None:
        return {}
    rng = rng or random.Random()

    board_slot = layout.slot(spec.board_slot_name)
    board_ranges = _ranges(spec.board_ranges)
    board_x, board_y, _ = board_slot.pose.pos
    board_pos = (
        board_x + _uniform(rng, board_ranges.get("x")),
        board_y + _uniform(rng, board_ranges.get("y")),
        board_slot.pose.pos[2],
    )
    board_yaw = _uniform(rng, board_ranges.get("yaw"))
    poses = {
        spec.board_slot_name: PoseSpec(
            pos=board_pos, rot=_quat_mul(_yaw_quat(board_yaw), board_slot.pose.rot)
        )
    }

    cos_yaw, sin_yaw = math.cos(board_yaw), math.sin(board_yaw)
    for part in spec.board_relative_parts:
        part_slot = layout.slot(part.slot_name)
        ranges = _ranges(part.pose_ranges)
        snaps = _snaps(part.snap_steps)
        offset_x, offset_y, offset_z = part.board_local_offset

        local_x = offset_x + _sample_axis(rng, ranges, snaps, "x")
        local_y = offset_y + _sample_axis(rng, ranges, snaps, "y")
        part_yaw = _sample_axis(rng, ranges, snaps, "yaw")

        poses[part.slot_name] = PoseSpec(
            pos=(
                board_pos[0] + cos_yaw * local_x - sin_yaw * local_y,
                board_pos[1] + sin_yaw * local_x + cos_yaw * local_y,
                board_pos[2] + offset_z,
            ),
            rot=_quat_mul(_yaw_quat(board_yaw + part_yaw), part_slot.pose.rot),
        )
    return poses


def apply_poses(
    stage: Usd.Stage,
    layout: SceneLayoutSpec,
    poses: dict[str, PoseSpec],
    *,
    root: str = DEFAULT_ROOT,
) -> None:
    """Write sampled poses onto the matching slot Xforms."""

    for slot_name, pose in poses.items():
        prim = stage.GetPrimAtPath(slot_prim_path(layout.slot(slot_name), root))
        if prim.IsValid():
            _write_xform_pose(prim, pose)


def randomize_dome_light(
    stage: Usd.Stage,
    *,
    light: DomeLightSpec = AIC_DOME_LIGHT,
    spec: DomeLightRandomizationSpec = AIC_DOME_LIGHT_RANDOMIZATION,
    rng: random.Random | None = None,
) -> LightSample | None:
    """Sample and apply a dome light intensity and color."""

    prim = stage.GetPrimAtPath(light.prim_path)
    if not prim.IsValid():
        return None
    rng = rng or random.Random()

    low, high = spec.color_range
    sample = LightSample(
        intensity=rng.uniform(*spec.intensity_range),
        color=tuple(rng.uniform(low[i], high[i]) for i in range(3)),
    )
    dome = UsdLux.DomeLight(prim)
    dome.GetIntensityAttr().Set(sample.intensity)
    dome.GetColorAttr().Set(Gf.Vec3f(*sample.color))
    return sample


def randomize(
    stage: Usd.Stage,
    layout: SceneLayoutSpec,
    *,
    root: str = DEFAULT_ROOT,
    rng: random.Random | None = None,
    light: DomeLightSpec | None = AIC_DOME_LIGHT,
    light_spec: DomeLightRandomizationSpec = AIC_DOME_LIGHT_RANDOMIZATION,
) -> tuple[dict[str, PoseSpec], LightSample | None]:
    """Sample and apply both layout poses and lighting in one call."""

    rng = rng or random.Random()
    poses = sample_layout(layout, rng)
    apply_poses(stage, layout, poses, root=root)
    sample = (
        randomize_dome_light(stage, light=light, spec=light_spec, rng=rng)
        if light is not None
        else None
    )
    return poses, sample


def _ranges(specs: tuple[AxisRangeSpec, ...]) -> dict[str, tuple[float, float]]:
    return {item.axis: item.bounds for item in specs}


def _snaps(specs: tuple[AxisSnapSpec, ...]) -> dict[str, float]:
    return {item.axis: item.step for item in specs}


def _uniform(rng: random.Random, bounds: tuple[float, float] | None) -> float:
    return rng.uniform(*bounds) if bounds else 0.0


def _sample_axis(
    rng: random.Random,
    ranges: dict[str, tuple[float, float]],
    snaps: dict[str, float],
    axis: str,
) -> float:
    """Sample one axis offset, snapping to a grid step when the spec asks."""

    low, high = ranges.get(axis, (0.0, 0.0))
    step = snaps.get(axis, 0.0)
    if step > 0.0 and high > low:
        return rng.randint(_grid_index(low, step, -1), _grid_index(high, step, 1)) * step
    return rng.uniform(low, high)


def _grid_index(bound: float, step: float, direction: int) -> int:
    """Grid index at ``bound``, rounded outward, tolerant of float error.

    The bare ``floor(high / step)`` upstream uses silently loses the top slot
    whenever the division lands just under an integer -- ``0.3 / 0.1`` is
    ``2.9999999999999996``, not ``3``. The current spec values dodge it
    (``0.12 / 0.04`` does round to exactly ``3.0``), so this is hardening, not a
    bug fix: it stops a later edit to a range or a step from quietly deleting
    one of the target's four positions.
    """

    scaled = bound / step
    return math.floor(scaled + _SNAP_TOL) if direction > 0 else math.ceil(scaled - _SNAP_TOL)


def _yaw_quat(angle: float) -> Quaternion:
    """Rotation about world Z as a wxyz quaternion."""

    return (math.cos(angle * 0.5), 0.0, 0.0, math.sin(angle * 0.5))


def _quat_mul(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """Hamilton product of two wxyz quaternions."""

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _write_xform_pose(prim: Usd.Prim, pose: PoseSpec) -> None:
    """Update the prim's existing translate/orient ops, preserving their types.

    Rewriting the op order instead would drop any scale or extra ops the asset
    author put there, so edit in place and only add ops that are missing.
    """

    xform = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    translate = next((op for name, op in ops.items() if "translate" in name), None)
    orient = next((op for name, op in ops.items() if "orient" in name), None)

    if translate is None:
        translate = xform.AddTranslateOp()
    if orient is None:
        orient = xform.AddOrientOp()

    x, y, z = pose.pos
    translate.Set(
        Gf.Vec3f(x, y, z)
        if translate.GetTypeName() == Sdf.ValueTypeNames.Float3
        else Gf.Vec3d(x, y, z)
    )
    w, qx, qy, qz = pose.rot
    orient.Set(
        Gf.Quatf(w, qx, qy, qz)
        if orient.GetTypeName() == Sdf.ValueTypeNames.Quatf
        else Gf.Quatd(w, qx, qy, qz)
    )


__all__ = [
    "LightSample",
    "apply_poses",
    "randomize",
    "randomize_dome_light",
    "sample_layout",
]
