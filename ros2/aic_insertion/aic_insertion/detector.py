"""Classical port-opening detector: dark quadrilaterals with metric gates.

One instance per camera stream. The detector is deliberately dumb about
appearance — it thresholds for "darker than the local surroundings" inside a
prior-predicted region — and strict about geometry: every candidate is
back-projected onto the known entrance plane and must measure like an SFP
opening there. Swapping this class for a learned or foundation-model detector
only has to reproduce :meth:`detect`'s return value.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .fitting import (
    backproject_to_plane,
    order_cyclic_ccw,
    project_plane_points,
    rect_gate,
    rect_metrics,
)
from .specs import DetectorSpec, PORT_RECT_HEIGHT, PORT_RECT_WIDTH


@dataclass
class RectObservation:
    """One candidate opening seen by one camera."""

    camera: str
    stamp: float
    image_pts: np.ndarray  # (4, 2) pixel corners, CCW in plane order
    plane_pts: np.ndarray  # (4, 2) world-xy on the entrance plane
    metrics: dict


class ClassicalPortDetector:
    def __init__(self, spec: DetectorSpec, plane_z: float):
        self.spec = spec
        self.plane_z = plane_z

    def detect(
        self,
        gray: np.ndarray,
        K: np.ndarray,
        cam_pose: tuple,
        roi_world_xy: np.ndarray,
        camera: str,
        stamp: float,
    ) -> list[RectObservation]:
        """Find opening candidates inside the world-space ROI polygon."""

        roi = self._roi_rect(gray.shape, K, cam_pose, roi_world_xy)
        if roi is None:
            return []
        x0, y0, x1, y1 = roi
        patch = gray[y0:y1, x0:x1]

        block = self.spec.adaptive_block | 1
        binary = cv2.adaptiveThreshold(
            patch, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV, block, self.spec.adaptive_c,
        )
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        out: list[RectObservation] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.spec.min_area_px or area > 0.5 * patch.size:
                continue
            quad = self._as_quad(contour)
            if quad is None:
                continue
            corners = self._refine_subpix(gray, quad + np.array([x0, y0], dtype=np.float32))
            plane_pts, valid = backproject_to_plane(corners, K, cam_pose, self.plane_z)
            if not np.all(valid):
                continue
            order = _order_indices(plane_pts)
            plane_pts = plane_pts[order]
            corners = corners[order]
            metrics = rect_metrics(plane_pts)
            if not rect_gate(
                metrics, PORT_RECT_WIDTH, PORT_RECT_HEIGHT,
                self.spec.side_tolerance, self.spec.angle_tolerance_deg,
            ):
                continue
            out.append(
                RectObservation(
                    camera=camera, stamp=stamp,
                    image_pts=corners, plane_pts=plane_pts, metrics=metrics,
                )
            )
        return out

    def _roi_rect(self, shape, K, cam_pose, roi_world_xy) -> tuple | None:
        """Project the world ROI polygon into the image, return a clipped box."""

        px = project_plane_points(roi_world_xy, self.plane_z, K, cam_pose)
        if not np.all(np.isfinite(px)):
            return None
        x0 = int(np.floor(px[:, 0].min())) - 2
        y0 = int(np.floor(px[:, 1].min())) - 2
        x1 = int(np.ceil(px[:, 0].max())) + 2
        y1 = int(np.ceil(px[:, 1].max())) + 2
        h, w = shape[:2]
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(x1, w), min(y1, h)
        if x1 - x0 < 8 or y1 - y0 < 8:
            return None
        return x0, y0, x1, y1

    def _as_quad(self, contour) -> np.ndarray | None:
        """Reduce a contour to a convex quadrilateral, or reject it."""

        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.06 * peri, True)
        if len(approx) != 4:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            if cv2.contourArea(contour) < 0.75 * max(rect[1][0] * rect[1][1], 1e-6):
                return None
            approx = box.reshape(-1, 1, 2)
        if not cv2.isContourConvex(approx.astype(np.int32)):
            return None
        return approx.reshape(4, 2).astype(np.float32)

    def _refine_subpix(self, gray: np.ndarray, corners: np.ndarray) -> np.ndarray:
        win = max(self.spec.subpix_window, 2)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.01)
        pts = corners.reshape(-1, 1, 2).astype(np.float32)
        try:
            cv2.cornerSubPix(gray, pts, (win, win), (-1, -1), criteria)
        except cv2.error:
            pass
        return pts.reshape(4, 2).astype(float)


def _order_indices(plane_pts: np.ndarray) -> np.ndarray:
    """Indices that order the 4 plane points CCW around their centroid."""

    centroid = plane_pts.mean(axis=0)
    angles = np.arctan2(plane_pts[:, 1] - centroid[1], plane_pts[:, 0] - centroid[0])
    return np.argsort(angles)


def draw_overlay(
    bgr: np.ndarray,
    rects: list[RectObservation],
    predicted_px: np.ndarray | None,
) -> np.ndarray:
    """Debug image: accepted rectangles (green) and the track prediction (cyan)."""

    out = bgr.copy()
    for rect in rects:
        pts = rect.image_pts.astype(np.int32)
        cv2.polylines(out, [pts.reshape(-1, 1, 2)], True, (0, 255, 0), 1)
    if predicted_px is not None and np.all(np.isfinite(predicted_px)):
        pts = predicted_px.astype(np.int32)
        for i in range(0, len(pts), 4):
            cv2.polylines(out, [pts[i:i + 4].reshape(-1, 1, 2)], True, (255, 255, 0), 1)
    return out
