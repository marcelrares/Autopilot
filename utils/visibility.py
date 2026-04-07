import cv2
import numpy as np

from config import (
    DETECTION_CONFIDENCE,
    DETECTION_CONFIDENCE_VISIBILITY_FLOOR,
    LANE_MARKING_WHITE_THRESHOLD,
    LANE_MARKING_WHITE_THRESHOLD_LOW_VIS,
    LANE_MARKING_YELLOW_S_MIN,
    LANE_MARKING_YELLOW_V_MIN,
    VISIBILITY_ANALYSIS_WIDTH,
    VISIBILITY_CLAHE_CLIP_LIMIT,
    VISIBILITY_CLAHE_TILE_SIZE,
    VISIBILITY_DIM_BRIGHTNESS_THRESHOLD,
    VISIBILITY_DIM_GAMMA,
    VISIBILITY_FOG_CONTRAST_THRESHOLD,
    VISIBILITY_FOG_EDGE_DENSITY_THRESHOLD,
    VISIBILITY_FOG_SATURATION_THRESHOLD,
    VISIBILITY_FOG_UNSHARP_AMOUNT,
    VISIBILITY_LOW_VISIBILITY_SCORE_THRESHOLD,
    VISIBILITY_NIGHT_BRIGHTNESS_THRESHOLD,
    VISIBILITY_NIGHT_GAMMA,
    VISIBILITY_SMOOTHING_ALPHA,
)


def _clip01(value):
    return float(np.clip(value, 0.0, 1.0))


def _smooth(previous_value, current_value):
    if previous_value is None:
        return float(current_value)
    return float(
        ((1.0 - VISIBILITY_SMOOTHING_ALPHA) * float(previous_value))
        + (VISIBILITY_SMOOTHING_ALPHA * float(current_value))
    )


def _apply_gamma(frame, gamma):
    gamma = max(0.1, float(gamma))
    lut = np.array(
        [((idx / 255.0) ** gamma) * 255.0 for idx in range(256)],
        dtype=np.uint8,
    )
    return cv2.LUT(frame, lut)


def _clahe_luminance(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)
    clahe = cv2.createCLAHE(
        clipLimit=max(1.0, float(VISIBILITY_CLAHE_CLIP_LIMIT)),
        tileGridSize=(VISIBILITY_CLAHE_TILE_SIZE, VISIBILITY_CLAHE_TILE_SIZE),
    )
    l_channel = clahe.apply(l_channel)
    enhanced_lab = cv2.merge((l_channel, a_channel, b_channel))
    return cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)


def _unsharp(frame, amount):
    amount = max(0.0, float(amount))
    if amount <= 1e-3:
        return frame
    blurred = cv2.GaussianBlur(frame, (0, 0), 1.15)
    return cv2.addWeighted(frame, 1.0 + amount, blurred, -amount, 0)


class VisibilityConditionEstimator:
    def __init__(self):
        self.previous_metrics = {}

    def analyze(self, frame):
        if frame is None or frame.size == 0:
            return self._default_context()

        frame_h, frame_w = frame.shape[:2]
        analysis_scale = min(1.0, VISIBILITY_ANALYSIS_WIDTH / max(1, frame_w))
        sample_w = max(48, int(frame_w * analysis_scale))
        sample_h = max(32, int(frame_h * analysis_scale))
        sample = cv2.resize(frame, (sample_w, sample_h), interpolation=cv2.INTER_AREA)

        hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(sample, cv2.COLOR_BGR2LAB)
        l_channel = lab[:, :, 0].astype(np.float32) / 255.0
        saturation = hsv[:, :, 1].astype(np.float32) / 255.0
        value = hsv[:, :, 2].astype(np.float32) / 255.0

        brightness = float(np.mean(value))
        contrast = _clip01(float(np.std(l_channel) / 0.22))
        saturation_mean = float(np.mean(saturation))
        edges = cv2.Canny((l_channel * 255.0).astype(np.uint8), 60, 150)
        edge_density = float(np.mean(edges > 0))
        highlight_ratio = float(np.mean(value >= 0.82))

        brightness = _smooth(self.previous_metrics.get("brightness"), brightness)
        contrast = _smooth(self.previous_metrics.get("contrast"), contrast)
        saturation_mean = _smooth(self.previous_metrics.get("saturation"), saturation_mean)
        edge_density = _smooth(self.previous_metrics.get("edge_density"), edge_density)
        highlight_ratio = _smooth(self.previous_metrics.get("highlight_ratio"), highlight_ratio)
        self.previous_metrics = {
            "brightness": brightness,
            "contrast": contrast,
            "saturation": saturation_mean,
            "edge_density": edge_density,
            "highlight_ratio": highlight_ratio,
        }

        darkness_score = _clip01(
            (VISIBILITY_DIM_BRIGHTNESS_THRESHOLD - brightness)
            / max(1e-3, VISIBILITY_DIM_BRIGHTNESS_THRESHOLD)
        )
        fog_contrast_penalty = _clip01(
            (VISIBILITY_FOG_CONTRAST_THRESHOLD - contrast)
            / max(1e-3, VISIBILITY_FOG_CONTRAST_THRESHOLD)
        )
        fog_saturation_penalty = _clip01(
            (VISIBILITY_FOG_SATURATION_THRESHOLD - saturation_mean)
            / max(1e-3, VISIBILITY_FOG_SATURATION_THRESHOLD)
        )
        fog_edge_penalty = _clip01(
            (VISIBILITY_FOG_EDGE_DENSITY_THRESHOLD - edge_density)
            / max(1e-3, VISIBILITY_FOG_EDGE_DENSITY_THRESHOLD)
        )
        fog_brightness_bonus = _clip01((brightness - 0.58) / 0.22)
        low_light_score = _clip01(
            (0.68 * darkness_score)
            + (0.20 * (1.0 - contrast))
            + (0.12 * np.clip(highlight_ratio * 1.7, 0.0, 1.0))
        )
        fog_score = _clip01(
            (0.18 * fog_contrast_penalty)
            + (0.46 * fog_saturation_penalty)
            + (0.22 * fog_edge_penalty)
            + (0.14 * fog_brightness_bonus)
        )
        visibility_score = _clip01(max(low_light_score, fog_score, (0.58 * low_light_score) + (0.42 * fog_score)))

        condition = "day"
        if brightness <= VISIBILITY_NIGHT_BRIGHTNESS_THRESHOLD * 0.78 and contrast <= VISIBILITY_FOG_CONTRAST_THRESHOLD:
            condition = "low_visibility"
        elif (
            fog_score >= 0.52
            and brightness > VISIBILITY_NIGHT_BRIGHTNESS_THRESHOLD * 1.10
            and saturation_mean <= VISIBILITY_FOG_SATURATION_THRESHOLD * 0.55
        ):
            condition = "fog"
        elif brightness <= VISIBILITY_NIGHT_BRIGHTNESS_THRESHOLD:
            condition = "night"
        elif brightness <= VISIBILITY_DIM_BRIGHTNESS_THRESHOLD:
            condition = "dim"
        elif visibility_score >= VISIBILITY_LOW_VISIBILITY_SCORE_THRESHOLD:
            condition = "low_visibility"

        low_visibility = condition in {"night", "fog", "low_visibility"}
        if condition == "day" and visibility_score >= 0.48:
            condition = "dim"

        detection_confidence = max(
            DETECTION_CONFIDENCE_VISIBILITY_FLOOR,
            DETECTION_CONFIDENCE - (visibility_score * 0.12),
        )

        return {
            "condition": condition,
            "low_visibility": low_visibility,
            "visibility_score": round(visibility_score, 3),
            "brightness": round(brightness, 3),
            "contrast": round(contrast, 3),
            "saturation": round(saturation_mean, 3),
            "edge_density": round(edge_density, 4),
            "low_light_score": round(low_light_score, 3),
            "fog_score": round(fog_score, 3),
            "detection_confidence": round(detection_confidence, 3),
        }

    @staticmethod
    def _default_context():
        return {
            "condition": "day",
            "low_visibility": False,
            "visibility_score": 0.0,
            "brightness": 0.5,
            "contrast": 0.5,
            "saturation": 0.5,
            "edge_density": 0.05,
            "low_light_score": 0.0,
            "fog_score": 0.0,
            "detection_confidence": DETECTION_CONFIDENCE,
        }


def preprocess_frame_for_detection(frame, visibility_context=None):
    visibility_context = visibility_context or {}
    condition = visibility_context.get("condition", "day")
    visibility_score = float(visibility_context.get("visibility_score", 0.0) or 0.0)

    enhanced = frame.copy()
    if condition != "day" or visibility_score >= 0.28:
        enhanced = _clahe_luminance(enhanced)

    if condition == "dim":
        enhanced = _apply_gamma(enhanced, VISIBILITY_DIM_GAMMA)
    elif condition in {"night", "low_visibility"}:
        enhanced = _apply_gamma(enhanced, VISIBILITY_NIGHT_GAMMA)

    if condition in {"fog", "low_visibility"} or visibility_score >= 0.52:
        enhanced = _unsharp(enhanced, VISIBILITY_FOG_UNSHARP_AMOUNT)

    return enhanced


def preprocess_traffic_light_roi(roi, visibility_context=None):
    visibility_context = visibility_context or {}
    if roi is None or roi.size == 0:
        return roi

    enhanced = roi.copy()
    if visibility_context.get("condition") in {"dim", "night", "fog", "low_visibility"}:
        enhanced = _clahe_luminance(enhanced)
    if visibility_context.get("condition") in {"night", "low_visibility"}:
        enhanced = _apply_gamma(enhanced, 0.78)
    return enhanced


def build_lane_guidance(frame, visibility_context=None):
    visibility_context = visibility_context or {}
    condition = visibility_context.get("condition", "day")
    visibility_score = float(visibility_context.get("visibility_score", 0.0) or 0.0)

    enhanced = preprocess_frame_for_detection(frame, visibility_context)
    hls = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HLS)
    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(enhanced, cv2.COLOR_BGR2LAB)
    gray = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)

    lightness = hls[:, :, 1]
    yellow_lower = np.array([12, LANE_MARKING_YELLOW_S_MIN, LANE_MARKING_YELLOW_V_MIN], dtype=np.uint8)
    yellow_upper = np.array([42, 255, 255], dtype=np.uint8)
    white_threshold = (
        LANE_MARKING_WHITE_THRESHOLD_LOW_VIS
        if condition in {"dim", "night", "fog", "low_visibility"} or visibility_score >= 0.40
        else LANE_MARKING_WHITE_THRESHOLD
    )
    white_mask = cv2.inRange(lightness, white_threshold, 255)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
    lane_mask = cv2.morphologyEx(
        lane_mask,
        cv2.MORPH_CLOSE,
        np.ones((5, 5), dtype=np.uint8),
        iterations=1,
    )

    combined = cv2.addWeighted(gray, 0.55, lightness, 0.30, 0.0)
    combined = cv2.addWeighted(combined, 0.85, lab[:, :, 0], 0.15, 0.0)
    if condition in {"fog", "low_visibility"}:
        combined = cv2.addWeighted(combined, 0.80, lane_mask, 0.20, 0.0)

    combined = cv2.GaussianBlur(combined, (5, 5), 0)
    return combined, lane_mask
