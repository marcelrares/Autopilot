import cv2
import numpy as np

from config import (
    CAMERA_HEIGHT_M,
    FOCAL_LENGTH,
    LANE_BOTTOM_MARGIN_RATIO,
    LANE_TOP_CUT_RATIO,
    PERSON_CLASSES,
    ROAD_CONTEXT_ADJACENT_LANE_LIMIT,
    ROAD_CONTEXT_CURVE_MAX_LATERAL_M,
    ROAD_CONTEXT_CURVE_SAMPLE_COUNT,
    ROAD_CONTEXT_CURVE_SMOOTHING_ALPHA,
    ROAD_CONTEXT_DEFAULT_HORIZON_RATIO,
    ROAD_CONTEXT_DISTANCE_SMOOTHING_ALPHA,
    ROAD_CONTEXT_EGO_OVERLAP_THRESHOLD,
    ROAD_CONTEXT_FALLBACK_LANE_WIDTH_RATIO,
    ROAD_CONTEXT_HEADING_CLAMP,
    ROAD_CONTEXT_HORIZON_SMOOTHING_ALPHA,
    ROAD_CONTEXT_MAX_DISTANCE_DELTA_LOW_VIS_M,
    ROAD_CONTEXT_MAX_DISTANCE_DELTA_PER_FRAME_M,
    ROAD_CONTEXT_MIN_CLOSING_SPEED_MPS,
    ROAD_CONTEXT_REAL_LANE_WIDTH_M,
    ROAD_CONTEXT_TTC_MAX_S,
    VEHICLE_CLASSES,
)


def _fit_lane_models(lanes):
    left_points = []
    right_points = []

    for line in lanes or []:
        x1, y1, x2, y2 = line[0]
        if x2 == x1:
            continue

        slope = (y2 - y1) / (x2 - x1)
        target = left_points if slope < 0 else right_points
        target.extend([(y1, x1), (y2, x2)])

    models = {}
    for side, points in (("left", left_points), ("right", right_points)):
        if len(points) < 2:
            continue

        ys = np.array([p[0] for p in points], dtype=np.float32)
        xs = np.array([p[1] for p in points], dtype=np.float32)
        unique_y = np.unique(np.round(ys, 2))
        degree = 2 if unique_y.size >= 3 and len(points) >= 4 else 1
        models[side] = np.polyfit(ys, xs, degree)

    return models


def _lane_x(model, y):
    return float(np.polyval(model, float(y)))


def _lane_slope(model, y):
    derivative = np.polyder(model)
    return float(np.polyval(derivative, float(y)))


def _lane_progress(y, top_y, bottom_y):
    if bottom_y <= top_y:
        return 1.0
    return float(np.clip((y - top_y) / (bottom_y - top_y), 0.0, 1.0))


def _lane_geometry_at_y(
    models,
    y,
    top_y,
    bottom_y,
    lane_center_top,
    lane_center_bottom,
    lane_width_top,
    lane_width_bottom,
    frame_w,
):
    progress = _lane_progress(y, top_y, bottom_y)
    fallback_center = ((1.0 - progress) * lane_center_top) + (progress * lane_center_bottom)
    fallback_width = ((1.0 - progress) * lane_width_top) + (progress * lane_width_bottom)
    fallback_width = max(60.0, fallback_width)

    has_left_lane = "left" in models
    has_right_lane = "right" in models
    left_x = _lane_x(models["left"], y) if has_left_lane else None
    right_x = _lane_x(models["right"], y) if has_right_lane else None

    valid_dual_lane = (
        left_x is not None
        and right_x is not None
        and (right_x - left_x) >= max(50.0, fallback_width * 0.45)
    )

    if valid_dual_lane:
        lane_width_px = float(np.clip(right_x - left_x, fallback_width * 0.70, frame_w * 0.95))
        lane_center_x = (left_x + right_x) / 2.0
    elif left_x is not None:
        lane_width_px = float(fallback_width)
        lane_center_x = left_x + (lane_width_px / 2.0)
    elif right_x is not None:
        lane_width_px = float(fallback_width)
        lane_center_x = right_x - (lane_width_px / 2.0)
    else:
        lane_width_px = float(fallback_width)
        lane_center_x = float(fallback_center)

    lane_center_x = float(np.clip(lane_center_x, frame_w * 0.05, frame_w * 0.95))
    lane_width_px = float(np.clip(lane_width_px, 60.0, frame_w * 0.95))
    left_bound_x = lane_center_x - (lane_width_px / 2.0)
    right_bound_x = lane_center_x + (lane_width_px / 2.0)

    return {
        "left_x": float(left_bound_x),
        "right_x": float(right_bound_x),
        "center_x": float(lane_center_x),
        "lane_width_px": float(lane_width_px),
        "progress": progress,
    }


def _compute_horizon_y(models, frame_h, top_y, previous_horizon_y=None):
    default_horizon_y = frame_h * ROAD_CONTEXT_DEFAULT_HORIZON_RATIO
    fallback_horizon_y = default_horizon_y if previous_horizon_y is None else previous_horizon_y

    if "left" in models and "right" in models:
        left_slope = _lane_slope(models["left"], top_y)
        right_slope = _lane_slope(models["right"], top_y)
        left_intercept = _lane_x(models["left"], top_y) - (left_slope * top_y)
        right_intercept = _lane_x(models["right"], top_y) - (right_slope * top_y)
        slope_delta = left_slope - right_slope

        if abs(slope_delta) > 1e-4:
            raw_horizon_y = (right_intercept - left_intercept) / slope_delta
        else:
            raw_horizon_y = fallback_horizon_y
    else:
        raw_horizon_y = fallback_horizon_y

    min_horizon_y = frame_h * 0.10
    max_horizon_y = max(min_horizon_y + 8.0, top_y - 8.0)
    horizon_y = float(np.clip(raw_horizon_y, min_horizon_y, max_horizon_y))

    if previous_horizon_y is not None:
        horizon_y = float(
            ((1.0 - ROAD_CONTEXT_HORIZON_SMOOTHING_ALPHA) * previous_horizon_y)
            + (ROAD_CONTEXT_HORIZON_SMOOTHING_ALPHA * horizon_y)
        )

    return horizon_y


def _safe_forward_distance_from_horizon(y, horizon_y):
    pixel_delta = max(1.0, y - horizon_y)
    distance_m = (CAMERA_HEIGHT_M * FOCAL_LENGTH) / pixel_delta
    return float(np.clip(distance_m, 0.5, 120.0))


def _project_point_with_homography(homography, point_xy):
    point = np.array([[[float(point_xy[0]), float(point_xy[1])]]], dtype=np.float32)
    projected = cv2.perspectiveTransform(point, homography)[0][0]
    lateral_m = float(projected[0])
    distance_m = float(projected[1])

    if not np.isfinite(lateral_m) or not np.isfinite(distance_m):
        return None
    if distance_m <= 0.0:
        return None

    return {
        "lateral_m": lateral_m,
        "distance_m": distance_m,
    }


def _build_centerline_profile(
    models,
    homography,
    top_y,
    bottom_y,
    lane_center_top,
    lane_center_bottom,
    lane_width_top,
    lane_width_bottom,
    frame_w,
):
    ys = np.linspace(bottom_y, top_y, ROAD_CONTEXT_CURVE_SAMPLE_COUNT)
    samples = []

    for y in ys:
        geometry = _lane_geometry_at_y(
            models,
            float(y),
            top_y,
            bottom_y,
            lane_center_top,
            lane_center_bottom,
            lane_width_top,
            lane_width_bottom,
            frame_w,
        )
        projected = _project_point_with_homography(homography, (geometry["center_x"], y))
        if projected is None:
            continue

        samples.append((float(projected["distance_m"]), float(projected["lateral_m"])))

    if not samples:
        return []

    samples.sort(key=lambda item: item[0])
    base_lateral_m = samples[0][1]
    profile = [{"distance_m": 0.0, "lateral_shift_m": 0.0}]
    last_distance_m = 0.0

    for distance_m, lateral_m in samples:
        distance_m = float(np.clip(distance_m, 0.0, 120.0))
        lateral_shift_m = float(
            np.clip(
                lateral_m - base_lateral_m,
                -ROAD_CONTEXT_CURVE_MAX_LATERAL_M,
                ROAD_CONTEXT_CURVE_MAX_LATERAL_M,
            )
        )

        if distance_m <= last_distance_m + 0.05:
            continue

        profile.append({
            "distance_m": round(distance_m, 3),
            "lateral_shift_m": round(lateral_shift_m, 3),
        })
        last_distance_m = distance_m

    if len(profile) == 1:
        profile.append({"distance_m": 30.0, "lateral_shift_m": 0.0})

    return profile


def _interpolate_centerline_shift(profile, distance_m):
    if not profile:
        return 0.0

    target_distance = max(0.0, float(distance_m))
    first_point = profile[0]
    if target_distance <= first_point["distance_m"]:
        return float(first_point["lateral_shift_m"])

    for previous_point, current_point in zip(profile, profile[1:]):
        previous_distance = float(previous_point["distance_m"])
        current_distance = float(current_point["distance_m"])
        if target_distance > current_distance:
            continue

        span = max(1e-6, current_distance - previous_distance)
        alpha = (target_distance - previous_distance) / span
        return float(
            ((1.0 - alpha) * float(previous_point["lateral_shift_m"]))
            + (alpha * float(current_point["lateral_shift_m"]))
        )

    return float(profile[-1]["lateral_shift_m"])


def build_road_context(lanes, frame_shape, previous_horizon_y=None):
    frame_h, frame_w = frame_shape[:2]
    top_y = int(frame_h * LANE_TOP_CUT_RATIO)
    bottom_y = int(frame_h * (1.0 - LANE_BOTTOM_MARGIN_RATIO))
    frame_center_x = frame_w / 2.0
    fallback_width_bottom = max(220.0, frame_w * ROAD_CONTEXT_FALLBACK_LANE_WIDTH_RATIO)
    fallback_width_top = fallback_width_bottom * 0.52
    models = _fit_lane_models(lanes)
    horizon_y = _compute_horizon_y(models, frame_h, top_y, previous_horizon_y)

    top_geometry = _lane_geometry_at_y(
        models,
        top_y,
        top_y,
        bottom_y,
        frame_center_x,
        frame_center_x,
        fallback_width_top,
        fallback_width_bottom,
        frame_w,
    )
    bottom_geometry = _lane_geometry_at_y(
        models,
        bottom_y,
        top_y,
        bottom_y,
        frame_center_x,
        frame_center_x,
        fallback_width_top,
        fallback_width_bottom,
        frame_w,
    )

    lane_center_top = top_geometry["center_x"]
    lane_center_bottom = bottom_geometry["center_x"]
    lane_width_top = top_geometry["lane_width_px"]
    lane_width_bottom = bottom_geometry["lane_width_px"]

    road_heading_norm = float(
        np.clip(
            (lane_center_top - lane_center_bottom) / max(1.0, frame_center_x),
            -ROAD_CONTEXT_HEADING_CLAMP,
            ROAD_CONTEXT_HEADING_CLAMP,
        )
    )

    top_distance_m = _safe_forward_distance_from_horizon(top_y, horizon_y)
    bottom_distance_m = _safe_forward_distance_from_horizon(bottom_y, horizon_y)

    src_points = np.array([
        [lane_center_bottom - (lane_width_bottom / 2.0), bottom_y],
        [lane_center_bottom + (lane_width_bottom / 2.0), bottom_y],
        [lane_center_top - (lane_width_top / 2.0), top_y],
        [lane_center_top + (lane_width_top / 2.0), top_y],
    ], dtype=np.float32)
    dst_points = np.array([
        [-ROAD_CONTEXT_REAL_LANE_WIDTH_M / 2.0, bottom_distance_m],
        [ROAD_CONTEXT_REAL_LANE_WIDTH_M / 2.0, bottom_distance_m],
        [-ROAD_CONTEXT_REAL_LANE_WIDTH_M / 2.0, top_distance_m],
        [ROAD_CONTEXT_REAL_LANE_WIDTH_M / 2.0, top_distance_m],
    ], dtype=np.float32)
    homography_image_to_ground = cv2.getPerspectiveTransform(src_points, dst_points)
    road_centerline_profile = _build_centerline_profile(
        models,
        homography_image_to_ground,
        top_y,
        bottom_y,
        lane_center_top,
        lane_center_bottom,
        lane_width_top,
        lane_width_bottom,
        frame_w,
    )

    return {
        "lane_visibility_low": not ("left" in models and "right" in models),
        "has_left_lane": "left" in models,
        "has_right_lane": "right" in models,
        "lane_line_count": len(lanes or []),
        "top_y": top_y,
        "bottom_y": bottom_y,
        "frame_width": frame_w,
        "frame_height": frame_h,
        "horizon_y": horizon_y,
        "lane_center_top_x": lane_center_top,
        "lane_center_bottom_x": lane_center_bottom,
        "lane_width_top_px": lane_width_top,
        "lane_width_bottom_px": lane_width_bottom,
        "lane_models": models,
        "road_heading_norm": road_heading_norm,
        "ego_lane_center_norm": float((lane_center_bottom - frame_center_x) / max(1.0, frame_center_x)),
        "ego_lane_width_norm": float(lane_width_bottom / max(1.0, frame_center_x)),
        "lane_width_m": ROAD_CONTEXT_REAL_LANE_WIDTH_M,
        "top_distance_m": top_distance_m,
        "bottom_distance_m": bottom_distance_m,
        "road_centerline_profile": road_centerline_profile,
        "homography_image_to_ground": homography_image_to_ground,
    }


def project_image_point_to_ground(road_context, point_xy):
    homography = road_context.get("homography_image_to_ground")
    if homography is None:
        return None
    return _project_point_with_homography(homography, point_xy)


def centerline_lateral_shift_at_distance(road_context, distance_m):
    return _interpolate_centerline_shift(road_context.get("road_centerline_profile", []), distance_m)


def lane_bounds_at_y(road_context, y):
    return _lane_geometry_at_y(
        road_context.get("lane_models", {}),
        float(y),
        road_context["top_y"],
        road_context["bottom_y"],
        road_context["lane_center_top_x"],
        road_context["lane_center_bottom_x"],
        road_context["lane_width_top_px"],
        road_context["lane_width_bottom_px"],
        road_context["frame_width"],
    )


class RoadContextEstimator:
    def __init__(self, frame_dt=1.0 / 30.0):
        self.frame_dt = max(1e-3, float(frame_dt))
        self.prev_horizon_y = None
        self.prev_centerline_profile = None
        self.prev_metric_distances = {}
        self.prev_relative_speeds = {}

    def set_frame_dt(self, frame_dt):
        self.frame_dt = max(1e-3, float(frame_dt))

    @staticmethod
    def _smooth(previous_value, current_value, alpha):
        return ((1.0 - alpha) * previous_value) + (alpha * current_value)

    def _smooth_centerline_profile(self, profile):
        if not profile:
            self.prev_centerline_profile = None
            return []

        if self.prev_centerline_profile is None or len(self.prev_centerline_profile) != len(profile):
            self.prev_centerline_profile = [dict(point) for point in profile]
            return profile

        smoothed_profile = []
        for previous_point, current_point in zip(self.prev_centerline_profile, profile):
            smoothed_profile.append({
                "distance_m": float(current_point["distance_m"]),
                "lateral_shift_m": round(
                    self._smooth(
                        float(previous_point["lateral_shift_m"]),
                        float(current_point["lateral_shift_m"]),
                        ROAD_CONTEXT_CURVE_SMOOTHING_ALPHA,
                    ),
                    3,
                ),
            })

        self.prev_centerline_profile = [dict(point) for point in smoothed_profile]
        return smoothed_profile

    def _estimate_metric_components(self, obj, road_context, lane_bounds):
        x1, _, x2, y2 = obj["bbox"]
        foot_x = (x1 + x2) / 2.0
        foot_y = float(np.clip(y2, road_context["top_y"], road_context["bottom_y"]))
        size_distance_m = float(obj.get("distance", 9999.0))

        horizon_distance_m = None
        if foot_y > road_context["horizon_y"] + 1.0:
            horizon_distance_m = _safe_forward_distance_from_horizon(foot_y, road_context["horizon_y"])

        lane_scale_distance_m = None
        lane_width_px = lane_bounds["lane_width_px"]
        if lane_width_px > 1.0:
            lane_scale_distance_m = float((ROAD_CONTEXT_REAL_LANE_WIDTH_M * FOCAL_LENGTH) / lane_width_px)

        projected_ground = project_image_point_to_ground(road_context, (foot_x, foot_y))
        projected_distance_m = projected_ground["distance_m"] if projected_ground is not None else None
        raw_lateral_distance_m = projected_ground["lateral_m"] if projected_ground is not None else None
        road_center_shift_m = (
            centerline_lateral_shift_at_distance(road_context, projected_distance_m)
            if projected_distance_m is not None else 0.0
        )
        lateral_distance_m = (
            raw_lateral_distance_m - road_center_shift_m
            if raw_lateral_distance_m is not None else None
        )

        obj_class = obj.get("class", "object")
        if obj_class in VEHICLE_CLASSES or obj_class in PERSON_CLASSES:
            estimates = [
                (projected_distance_m, 0.48),
                (horizon_distance_m, 0.22),
                (lane_scale_distance_m, 0.15),
                (size_distance_m, 0.15),
            ]
        else:
            estimates = [
                (size_distance_m, 0.70),
                (projected_distance_m, 0.15),
                (lane_scale_distance_m, 0.10),
                (horizon_distance_m, 0.05),
            ]

        weighted_sum = 0.0
        total_weight = 0.0
        for estimate_value, weight in estimates:
            if estimate_value is None:
                continue
            if not np.isfinite(estimate_value):
                continue
            if estimate_value <= 0.0:
                continue
            weighted_sum += float(estimate_value) * weight
            total_weight += weight

        fused_distance_m = size_distance_m if total_weight <= 0.0 else weighted_sum / total_weight
        fused_distance_m = float(np.clip(fused_distance_m, 0.5, 120.0))

        if lateral_distance_m is None:
            lane_half_width_m = ROAD_CONTEXT_REAL_LANE_WIDTH_M / 2.0
            bbox_center_x = (x1 + x2) / 2.0
            lane_offset_norm = (bbox_center_x - lane_bounds["center_x"]) / max(1.0, lane_bounds["lane_width_px"] / 2.0)
            lateral_distance_m = float(lane_offset_norm * lane_half_width_m)
            raw_lateral_distance_m = float(lateral_distance_m + road_center_shift_m)

        return {
            "distance_metric_m": projected_distance_m,
            "distance_horizon_m": horizon_distance_m,
            "distance_lane_scale_m": lane_scale_distance_m,
            "distance_size_m": size_distance_m,
            "lateral_distance_m": lateral_distance_m,
            "lateral_distance_raw_m": raw_lateral_distance_m,
            "road_center_shift_m": road_center_shift_m,
            "fused_distance_m": fused_distance_m,
        }

    @staticmethod
    def _classify_lane_relation(lateral_distance_m, lane_overlap, lane_offset):
        lane_half_width_m = ROAD_CONTEXT_REAL_LANE_WIDTH_M / 2.0

        if lateral_distance_m is not None:
            if abs(lateral_distance_m) <= (lane_half_width_m * 0.98) or lane_overlap >= ROAD_CONTEXT_EGO_OVERLAP_THRESHOLD:
                return "ego_lane", 0
            if lateral_distance_m < -(ROAD_CONTEXT_REAL_LANE_WIDTH_M * 1.6):
                return "far_left", -2
            if lateral_distance_m < -lane_half_width_m:
                return "left_adjacent", -1
            if lateral_distance_m > (ROAD_CONTEXT_REAL_LANE_WIDTH_M * 1.6):
                return "far_right", 2
            return "right_adjacent", 1

        if lane_overlap >= ROAD_CONTEXT_EGO_OVERLAP_THRESHOLD or abs(lane_offset) <= 0.95:
            return "ego_lane", 0
        if lane_offset < -ROAD_CONTEXT_ADJACENT_LANE_LIMIT:
            return "far_left", -2
        if lane_offset < -0.95:
            return "left_adjacent", -1
        if lane_offset > ROAD_CONTEXT_ADJACENT_LANE_LIMIT:
            return "far_right", 2
        return "right_adjacent", 1

    def _update_longitudinal_kinematics(self, track_id, fused_distance_m, visibility_low=False):
        previous_distance = self.prev_metric_distances.get(track_id)
        previous_relative_speed = self.prev_relative_speeds.get(track_id)

        if previous_distance is None:
            self.prev_metric_distances[track_id] = fused_distance_m
            return None, None

        raw_distance_delta = previous_distance - fused_distance_m
        max_distance_delta = (
            ROAD_CONTEXT_MAX_DISTANCE_DELTA_LOW_VIS_M
            if visibility_low else ROAD_CONTEXT_MAX_DISTANCE_DELTA_PER_FRAME_M
        )
        distance_delta = float(np.clip(raw_distance_delta, -max_distance_delta, max_distance_delta))
        closing_speed_mps = distance_delta / self.frame_dt
        if previous_relative_speed is not None:
            closing_speed_mps = self._smooth(previous_relative_speed, closing_speed_mps, 0.38)

        self.prev_metric_distances[track_id] = fused_distance_m
        self.prev_relative_speeds[track_id] = closing_speed_mps

        if closing_speed_mps <= ROAD_CONTEXT_MIN_CLOSING_SPEED_MPS:
            return float(closing_speed_mps), None

        ttc_s = fused_distance_m / closing_speed_mps
        if ttc_s > ROAD_CONTEXT_TTC_MAX_S:
            return float(closing_speed_mps), None

        return float(closing_speed_mps), float(ttc_s)

    def annotate(self, objects, lanes, frame_shape, visibility_context=None):
        road_context = build_road_context(lanes, frame_shape, previous_horizon_y=self.prev_horizon_y)
        road_context["road_centerline_profile"] = self._smooth_centerline_profile(
            road_context.get("road_centerline_profile", [])
        )
        visibility_context = visibility_context or {}
        visibility_condition = visibility_context.get("condition", "day")
        road_context["visibility_condition"] = visibility_condition
        road_context["visibility_score"] = float(visibility_context.get("visibility_score", 0.0) or 0.0)
        road_context["visibility_low"] = bool(visibility_context.get("low_visibility", False))
        road_context["scene_brightness"] = float(visibility_context.get("brightness", 0.5) or 0.5)
        road_context["scene_contrast"] = float(visibility_context.get("contrast", 0.5) or 0.5)
        road_context["scene_fog_score"] = float(visibility_context.get("fog_score", 0.0) or 0.0)
        road_context["scene_low_light_score"] = float(visibility_context.get("low_light_score", 0.0) or 0.0)
        if road_context["visibility_low"] and road_context.get("lane_line_count", 0) < 5:
            road_context["lane_visibility_low"] = True
        distance_smoothing_alpha = (
            ROAD_CONTEXT_DISTANCE_SMOOTHING_ALPHA * 0.65
            if road_context["visibility_low"] else ROAD_CONTEXT_DISTANCE_SMOOTHING_ALPHA
        )
        self.prev_horizon_y = road_context["horizon_y"]
        annotated_objects = []
        active_track_ids = set()

        for obj in objects:
            annotated = dict(obj)
            track_id = annotated["id"]
            active_track_ids.add(track_id)

            x1, _, x2, y2 = annotated["bbox"]
            y_ref = int(np.clip(y2, road_context["top_y"], road_context["bottom_y"]))
            lane_bounds = lane_bounds_at_y(road_context, y_ref)

            bbox_center_x = (x1 + x2) / 2.0
            bbox_width = max(1.0, x2 - x1)
            lane_half_width_px = max(1.0, lane_bounds["lane_width_px"] / 2.0)
            lane_offset = (bbox_center_x - lane_bounds["center_x"]) / lane_half_width_px
            lane_overlap = max(0.0, min(x2, lane_bounds["right_x"]) - max(x1, lane_bounds["left_x"])) / bbox_width

            metric = self._estimate_metric_components(annotated, road_context, lane_bounds)
            fused_distance_m = metric["fused_distance_m"]
            previous_distance = self.prev_metric_distances.get(track_id)
            if previous_distance is not None:
                fused_distance_m = self._smooth(
                    previous_distance,
                    fused_distance_m,
                    distance_smoothing_alpha,
                )

            lane_relation, lane_index = self._classify_lane_relation(
                metric["lateral_distance_m"],
                lane_overlap,
                lane_offset,
            )
            relative_speed_mps, ttc_s = self._update_longitudinal_kinematics(
                track_id,
                fused_distance_m,
                visibility_low=road_context["visibility_low"],
            )

            annotated["lane_relation"] = lane_relation
            annotated["lane_index"] = lane_index
            annotated["lane_offset"] = float(lane_offset)
            annotated["lane_overlap"] = float(lane_overlap)
            annotated["path_conflict"] = bool(
                lane_overlap >= 0.15
                or lane_index == 0
                or (metric["lateral_distance_m"] is not None and abs(metric["lateral_distance_m"]) <= ROAD_CONTEXT_REAL_LANE_WIDTH_M * 0.70)
            )
            annotated["distance_size_m"] = round(metric["distance_size_m"], 3)
            annotated["distance_metric_m"] = None if metric["distance_metric_m"] is None else round(metric["distance_metric_m"], 3)
            annotated["distance_horizon_m"] = None if metric["distance_horizon_m"] is None else round(metric["distance_horizon_m"], 3)
            annotated["distance_lane_scale_m"] = None if metric["distance_lane_scale_m"] is None else round(metric["distance_lane_scale_m"], 3)
            annotated["lateral_distance_m"] = None if metric["lateral_distance_m"] is None else round(metric["lateral_distance_m"], 3)
            annotated["lateral_distance_raw_m"] = None if metric["lateral_distance_raw_m"] is None else round(metric["lateral_distance_raw_m"], 3)
            annotated["road_center_shift_m"] = round(metric["road_center_shift_m"], 3)
            annotated["relative_speed_mps"] = None if relative_speed_mps is None else round(relative_speed_mps, 3)
            annotated["ttc_s"] = None if ttc_s is None else round(ttc_s, 3)
            annotated["distance"] = float(round(fused_distance_m, 3))
            annotated_objects.append(annotated)

        stale_track_ids = set(self.prev_metric_distances) - active_track_ids
        for stale_track_id in stale_track_ids:
            self.prev_metric_distances.pop(stale_track_id, None)
            self.prev_relative_speeds.pop(stale_track_id, None)

        return annotated_objects, road_context


def annotate_objects_with_road_context(objects, lanes, frame_shape, estimator=None, visibility_context=None):
    if estimator is None:
        estimator = RoadContextEstimator()
    return estimator.annotate(objects, lanes, frame_shape, visibility_context=visibility_context)
