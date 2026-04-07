import cv2
import numpy as np

from config import (
    LANE_BOTTOM_MARGIN_RATIO,
    LANE_CANNY_HIGH,
    LANE_CANNY_HIGH_MAX,
    LANE_CANNY_LOW,
    LANE_CANNY_LOW_MIN,
    LANE_HOUGH_THRESHOLD,
    LANE_MIN_ABS_SLOPE,
    LANE_MAX_LINE_GAP,
    LANE_MAX_FILTERED_LINES,
    LANE_MAX_FILTERED_LINES_LOW_VIS,
    LANE_MIN_LINE_LENGTH,
    LANE_SIDE_MARGIN_RATIO,
    LANE_TOP_CUT_RATIO,
)
from utils.visibility import build_lane_guidance


def _adaptive_canny_thresholds(guidance_gray, visibility_context=None):
    visibility_context = visibility_context or {}
    nonzero_pixels = guidance_gray[guidance_gray > 0]
    if nonzero_pixels.size == 0:
        return LANE_CANNY_LOW, LANE_CANNY_HIGH

    median_intensity = float(np.median(nonzero_pixels))
    visibility_score = float(visibility_context.get("visibility_score", 0.0) or 0.0)
    condition = visibility_context.get("condition", "day")

    low = int(np.clip(0.66 * median_intensity, LANE_CANNY_LOW_MIN, LANE_CANNY_HIGH_MAX - 30))
    high = int(np.clip(1.32 * median_intensity, low + 30, LANE_CANNY_HIGH_MAX))

    if condition in {"dim", "night", "fog", "low_visibility"} or visibility_score >= 0.45:
        low = max(LANE_CANNY_LOW_MIN, low - 10)
        high = min(LANE_CANNY_HIGH_MAX, high + 12)

    return low, high


def _adaptive_hough_params(visibility_context=None):
    visibility_context = visibility_context or {}
    condition = visibility_context.get("condition", "day")
    visibility_score = float(visibility_context.get("visibility_score", 0.0) or 0.0)

    if condition in {"day"} and visibility_score < 0.35:
        return LANE_HOUGH_THRESHOLD, LANE_MIN_LINE_LENGTH, LANE_MAX_LINE_GAP

    return (
        max(28, int(LANE_HOUGH_THRESHOLD * 0.78)),
        max(55, int(LANE_MIN_LINE_LENGTH * 0.70)),
        max(LANE_MAX_LINE_GAP, 62),
    )


def detect_lanes(frame, visibility_context=None):
    height, width = frame.shape[:2]

    top_cut = int(height * LANE_TOP_CUT_RATIO)
    margin_x = int(width * LANE_SIDE_MARGIN_RATIO)
    margin_bottom = int(height * LANE_BOTTOM_MARGIN_RATIO)

    mask = np.zeros_like(frame)

    polygon = np.array([[
        (margin_x, height - margin_bottom),
        (margin_x, top_cut),
        (width - margin_x, top_cut),
        (width - margin_x, height - margin_bottom)
    ]], dtype=np.int32)

    cv2.fillPoly(mask, polygon, (255, 255, 255))
    roi = cv2.bitwise_and(frame, mask)

    guidance_gray, lane_mask = build_lane_guidance(roi, visibility_context)
    canny_low, canny_high = _adaptive_canny_thresholds(guidance_gray, visibility_context)
    hough_threshold, min_line_length, max_line_gap = _adaptive_hough_params(visibility_context)
    edges = cv2.Canny(guidance_gray, canny_low, canny_high)
    edges = cv2.bitwise_and(edges, lane_mask)

    if visibility_context and visibility_context.get("low_visibility", False):
        edges = cv2.dilate(edges, np.ones((3, 3), dtype=np.uint8), iterations=1)

    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        hough_threshold,
        minLineLength=min_line_length,
        maxLineGap=max_line_gap
    )

    filtered_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)

            if abs(slope) > LANE_MIN_ABS_SLOPE:
                filtered_lines.append(line)

    filtered_lines.sort(
        key=lambda line: np.hypot(line[0][2] - line[0][0], line[0][3] - line[0][1]),
        reverse=True,
    )
    max_lines = (
        LANE_MAX_FILTERED_LINES_LOW_VIS
        if visibility_context and visibility_context.get("low_visibility", False)
        else LANE_MAX_FILTERED_LINES
    )
    filtered_lines = filtered_lines[:max_lines]

    return filtered_lines, polygon
