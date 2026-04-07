import cv2
import numpy as np

from config import (
    BIRDVIEW_MAX_DISTANCE,
    BIRDVIEW_OBJECT_SMOOTHING_ALPHA,
    BIRDVIEW_PANEL_WIDTH,
    BIRDVIEW_ROAD_CURVE_SMOOTHING_ALPHA,
    BIRDVIEW_ROAD_HEADING_SMOOTHING_ALPHA,
    BIRDVIEW_WINDOW_HEIGHT,
    BIRDVIEW_WINDOW_WIDTH,
    DECISION_SPEEDOMETER_MAX_KMH,
    ROAD_CONTEXT_REAL_LANE_WIDTH_M,
    VEHICLE_CLASSES,
)


def _pretty_label(value):
    return str(value).replace("_", " ").title()


def _object_color(distance, lane_index):
    if lane_index == 0:
        if distance < 10:
            return (60, 96, 255)
        if distance < 16:
            return (0, 193, 255)
        return (58, 211, 125)

    if distance < 10:
        return (120, 170, 255)
    return (130, 200, 220)


def _signal_color(state):
    return {
        "red": (60, 96, 255),
        "yellow": (0, 220, 255),
        "green": (40, 210, 120),
    }.get(state, (165, 175, 185))


def _status_color(kind, value):
    if kind == "risk":
        return {
            "low": (58, 211, 125),
            "medium": (0, 193, 255),
            "high": (60, 96, 255),
        }.get(value, (200, 200, 200))

    if kind == "brake":
        return {
            "no_brake": (58, 211, 125),
            "engine_brake": (100, 205, 180),
            "light_brake": (0, 193, 255),
            "progressive_brake": (0, 160, 255),
            "controlled_stop": (50, 120, 255),
            "maximum_brake": (60, 96, 255),
            "emergency_brake": (60, 70, 235),
        }.get(value, (200, 200, 200))

    if kind == "lane":
        return {
            "keep_lane": (58, 211, 125),
            "keep_lane_with_caution": (0, 193, 255),
        }.get(value, (200, 200, 200))

    if kind == "throttle":
        if value >= 55:
            return (58, 211, 125)
        if value >= 25:
            return (0, 193, 255)
        return (120, 170, 255)

    return (200, 200, 200)


def _visibility_color(condition):
    return {
        "day": (58, 211, 125),
        "dim": (0, 193, 255),
        "night": (120, 170, 255),
        "fog": (0, 160, 255),
        "low_visibility": (60, 96, 255),
    }.get(condition, (170, 190, 205))


def _ttc_color(ttc_s):
    if ttc_s is None:
        return (120, 170, 255)
    if ttc_s <= 1.8:
        return (60, 96, 255)
    if ttc_s <= 3.4:
        return (0, 193, 255)
    return (58, 211, 125)


class BirdViewRenderer:
    def __init__(self):
        self.object_states = {}
        self.road_heading_norm = 0.0
        self.road_profile = []

    @staticmethod
    def _blend(previous, current, alpha):
        return ((1.0 - alpha) * previous) + (alpha * current)

    def _update_road_heading(self, road_context):
        target_heading = float(road_context.get("road_heading_norm", 0.0))
        self.road_heading_norm = self._blend(
            self.road_heading_norm,
            target_heading,
            BIRDVIEW_ROAD_HEADING_SMOOTHING_ALPHA,
        )
        return self.road_heading_norm

    def _update_road_profile(self, road_context):
        target_profile = road_context.get("road_centerline_profile", []) or []
        if not target_profile:
            self.road_profile = []
            return self.road_profile

        if not self.road_profile or len(self.road_profile) != len(target_profile):
            self.road_profile = [dict(point) for point in target_profile]
            return self.road_profile

        smoothed_profile = []
        for previous_point, current_point in zip(self.road_profile, target_profile):
            smoothed_profile.append({
                "distance_m": float(current_point["distance_m"]),
                "lateral_shift_m": self._blend(
                    float(previous_point["lateral_shift_m"]),
                    float(current_point["lateral_shift_m"]),
                    BIRDVIEW_ROAD_CURVE_SMOOTHING_ALPHA,
                ),
            })

        self.road_profile = smoothed_profile
        return self.road_profile

    def _layout(self):
        panel_left = BIRDVIEW_WINDOW_WIDTH - BIRDVIEW_PANEL_WIDTH
        road_left = 38
        road_right = panel_left - 18
        road_top = 58
        road_bottom = BIRDVIEW_WINDOW_HEIGHT - 82
        lane_width = (road_right - road_left) / 3.0

        return {
            "panel_left": panel_left,
            "road_left": road_left,
            "road_right": road_right,
            "road_top": road_top,
            "road_bottom": road_bottom,
            "lane_width": lane_width,
        }

    def _road_shift_from_profile_m(self, distance_m):
        if not self.road_profile:
            return None

        target_distance = max(0.0, float(distance_m))
        if target_distance <= float(self.road_profile[0]["distance_m"]):
            return float(self.road_profile[0]["lateral_shift_m"])

        for previous_point, current_point in zip(self.road_profile, self.road_profile[1:]):
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

        return float(self.road_profile[-1]["lateral_shift_m"])

    def _road_shift(self, progress, layout):
        progress = float(np.clip(progress, 0.0, 1.0))
        distance_m = progress * BIRDVIEW_MAX_DISTANCE
        profile_shift_m = self._road_shift_from_profile_m(distance_m)
        if profile_shift_m is not None:
            px_per_meter = layout["lane_width"] / max(1e-6, ROAD_CONTEXT_REAL_LANE_WIDTH_M)
            return profile_shift_m * px_per_meter

        eased = progress ** 1.35
        return self.road_heading_norm * layout["lane_width"] * 0.95 * eased

    def _road_edge_x(self, boundary_index, progress, layout):
        base_x = layout["road_left"] + (boundary_index * layout["lane_width"])
        return int(base_x + self._road_shift(progress, layout))

    def _lane_center_x(self, lane_slot, layout, progress=0.0):
        base_x = layout["road_left"] + ((lane_slot + 0.5) * layout["lane_width"])
        return int(base_x + self._road_shift(progress, layout))

    def _roadside_x(self, side, progress, layout):
        edge_x = self._road_edge_x(0 if side < 0 else 3, progress, layout)
        if side < 0:
            return edge_x - 28
        return edge_x + 28

    def _lane_boundary_polyline(self, boundary_index, layout, sample_count=26):
        points = []
        for idx in range(sample_count):
            progress = idx / max(1, sample_count - 1)
            y = int(layout["road_bottom"] - progress * (layout["road_bottom"] - layout["road_top"]))
            x = self._road_edge_x(boundary_index, progress, layout)
            points.append([x, y])
        return np.array(points, dtype=np.int32)

    def _draw_background(self, canvas, layout):
        height, width = canvas.shape[:2]
        gradient = np.linspace(0, 1, height, dtype=np.float32)[:, None]
        top_color = np.array([18, 20, 26], dtype=np.float32)
        bottom_color = np.array([7, 36, 52], dtype=np.float32)
        row_colors = ((1.0 - gradient) * top_color) + (gradient * bottom_color)
        canvas[:] = np.repeat(row_colors[:, None, :], width, axis=1).astype(np.uint8)

        cv2.rectangle(
            canvas,
            (layout["panel_left"], 0),
            (width, height),
            (14, 18, 26),
            -1,
        )
        cv2.line(canvas, (layout["panel_left"], 0), (layout["panel_left"], height), (46, 78, 95), 2)

    def _draw_road(self, canvas, road_context, layout):
        road_left = layout["road_left"]
        road_right = layout["road_right"]
        road_top = layout["road_top"]
        road_bottom = layout["road_bottom"]
        lane_width = layout["lane_width"]
        left_outer = self._lane_boundary_polyline(0, layout)
        ego_left = self._lane_boundary_polyline(1, layout)
        ego_right = self._lane_boundary_polyline(2, layout)
        right_outer = self._lane_boundary_polyline(3, layout)
        road_poly = np.vstack([left_outer, right_outer[::-1]])
        ego_poly = np.vstack([ego_left, ego_right[::-1]])

        cv2.fillPoly(canvas, [road_poly], (38, 44, 50))
        cv2.fillPoly(canvas, [ego_poly], (22, 64, 82))
        cv2.polylines(canvas, [road_poly], True, (70, 86, 96), 2)

        for boundary in (ego_left, ego_right):
            for idx in range(len(boundary) - 1):
                if idx % 2 == 0:
                    cv2.line(canvas, tuple(boundary[idx]), tuple(boundary[idx + 1]), (200, 220, 228), 2)

        for meters in range(10, BIRDVIEW_MAX_DISTANCE + 10, 10):
            progress = min(1.0, meters / BIRDVIEW_MAX_DISTANCE)
            y = int(road_bottom - progress * (road_bottom - road_top))
            left_x = self._road_edge_x(0, progress, layout)
            right_x = self._road_edge_x(3, progress, layout)
            cv2.line(canvas, (left_x, y), (right_x, y), (54, 66, 78), 1)
            cv2.putText(
                canvas,
                f"{meters} m",
                (10, y + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (150, 180, 195),
                1,
            )

        for lane_slot, label in ((0, "ONCOMING"), (1, "EGO"), (2, "RIGHT")):
            label_x = self._lane_center_x(lane_slot, layout, progress=1.0) - 34
            cv2.putText(
                canvas,
                label,
                (label_x, road_top - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (170, 205, 220),
                1,
            )

        lane_status = "Lane model: full" if not road_context.get("lane_visibility_low", True) else "Lane model: fallback"
        cv2.putText(
            canvas,
            lane_status,
            (road_left, road_bottom + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (170, 205, 220),
            1,
        )

    def _object_target_state(self, obj, layout):
        distance = float(obj.get("distance", BIRDVIEW_MAX_DISTANCE))
        progress = np.clip(distance / BIRDVIEW_MAX_DISTANCE, 0.0, 1.0)
        obj_class = obj["class"]
        lane_offset = float(obj.get("lane_offset", 0.0))
        lane_relation = obj.get("lane_relation", "ego_lane")
        lateral_distance_m = obj.get("lateral_distance_m")
        ego_lane_center_x = self._lane_center_x(1, layout, progress)

        if obj_class in {"traffic light", "stop sign"}:
            roadside_side = -1 if lane_relation in {"left_adjacent", "far_left"} or lane_offset < -0.1 else 1
            if lateral_distance_m is not None:
                roadside_side = -1 if lateral_distance_m < 0.0 else 1
            x = self._roadside_x(roadside_side, progress, layout)
            y = int(layout["road_bottom"] - progress * (layout["road_bottom"] - layout["road_top"]))

            if obj_class == "traffic light":
                width = 24
                height = 58
            else:
                width = 28
                height = 36

            return {
                "x": float(x),
                "y": float(y),
                "w": float(width),
                "h": float(height),
                "distance": distance,
                "lane_index": 0,
                "class": obj_class,
                "render_shape": obj_class.replace(" ", "_"),
                "traffic_light_color": obj.get("traffic_light_color", "unknown"),
            }

        lane_index = int(np.clip(obj.get("lane_index", 0), -1, 1))
        if lateral_distance_m is not None:
            lateral_distance_m = float(np.clip(lateral_distance_m, -ROAD_CONTEXT_REAL_LANE_WIDTH_M * 1.45, ROAD_CONTEXT_REAL_LANE_WIDTH_M * 1.45))
            x = ego_lane_center_x + int((lateral_distance_m / ROAD_CONTEXT_REAL_LANE_WIDTH_M) * layout["lane_width"])
        else:
            lane_slot = lane_index + 1
            lane_center_x = self._lane_center_x(lane_slot, layout, progress)
            lane_offset = float(np.clip(lane_offset - lane_index, -0.7, 0.7))
            x = lane_center_x + int(lane_offset * layout["lane_width"] * 0.24)
        y = int(layout["road_bottom"] - progress * (layout["road_bottom"] - layout["road_top"]))

        scale = 1.0 - (0.42 * progress)
        if obj["class"] in VEHICLE_CLASSES:
            width = max(18, int(32 * scale))
            height = max(30, int(56 * scale))
        else:
            width = max(12, int(18 * scale))
            height = max(22, int(30 * scale))

        return {
            "x": float(x),
            "y": float(y),
            "w": float(width),
            "h": float(height),
            "distance": distance,
            "lane_index": lane_index,
            "class": obj_class,
            "render_shape": "box",
            "traffic_light_color": obj.get("traffic_light_color"),
        }

    def _update_object_states(self, objects, layout):
        active_ids = set()
        drawables = []

        for obj in sorted(objects, key=lambda item: item.get("distance", 9999), reverse=True):
            distance = float(obj.get("distance", BIRDVIEW_MAX_DISTANCE))
            if distance > BIRDVIEW_MAX_DISTANCE:
                continue

            object_id = str(obj.get("id", len(active_ids)))
            active_ids.add(object_id)
            target_state = self._object_target_state(obj, layout)
            previous_state = self.object_states.get(object_id)

            if previous_state is not None and previous_state.get("class") == target_state["class"]:
                alpha = BIRDVIEW_OBJECT_SMOOTHING_ALPHA
                for key in ("x", "y", "w", "h", "distance"):
                    target_state[key] = self._blend(previous_state[key], target_state[key], alpha)

            target_state["class"] = obj["class"]
            target_state["missed"] = 0
            self.object_states[object_id] = target_state

            drawables.append({
                "id": object_id,
                "class": target_state["class"],
                "distance": target_state["distance"],
                "lane_index": target_state["lane_index"],
                "x": int(target_state["x"]),
                "y": int(target_state["y"]),
                "w": int(target_state["w"]),
                "h": int(target_state["h"]),
                "color": _object_color(target_state["distance"], target_state["lane_index"]),
                "render_shape": target_state["render_shape"],
                "traffic_light_color": target_state.get("traffic_light_color"),
            })

        stale_ids = []
        for object_id, state in self.object_states.items():
            if object_id in active_ids:
                continue
            state["missed"] = state.get("missed", 0) + 1
            if state["missed"] > 8:
                stale_ids.append(object_id)

        for object_id in stale_ids:
            self.object_states.pop(object_id, None)

        drawables.sort(key=lambda item: item["y"])
        return drawables

    @staticmethod
    def _draw_traffic_light_icon(canvas, x, y, width, height, signal_state):
        housing_w = max(18, width)
        housing_h = max(42, height)
        x1 = x - housing_w // 2
        y1 = y - housing_h // 2
        x2 = x1 + housing_w
        y2 = y1 + housing_h

        cv2.rectangle(canvas, (x1, y1), (x2, y2), (30, 34, 38), -1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (240, 244, 246), 1)
        cv2.line(canvas, (x, y2), (x, y2 + 16), (140, 148, 155), 2)

        radius = max(4, housing_w // 5)
        lights = [
            ("red", (x, y1 + housing_h // 4)),
            ("yellow", (x, y1 + housing_h // 2)),
            ("green", (x, y1 + (3 * housing_h) // 4)),
        ]
        for light_name, center in lights:
            color = (55, 55, 55)
            if light_name == signal_state:
                color = _signal_color(signal_state)
            cv2.circle(canvas, center, radius, color, -1)
            cv2.circle(canvas, center, radius, (220, 225, 228), 1)

    @staticmethod
    def _draw_stop_sign_icon(canvas, x, y, size):
        radius = max(10, size // 2)
        pts = np.array([
            [
                int(x + radius * np.cos(angle)),
                int(y + radius * np.sin(angle)),
            ]
            for angle in np.deg2rad(np.arange(22.5, 360.0, 45.0))
        ], dtype=np.int32)
        cv2.fillPoly(canvas, [pts], (0, 0, 220))
        cv2.polylines(canvas, [pts], True, (255, 255, 255), 2)
        cv2.line(canvas, (x, y + radius), (x, y + radius + 15), (160, 160, 160), 2)

    def _draw_objects(self, canvas, drawables):
        for obj in drawables:
            x = obj["x"]
            y = obj["y"]
            box_w = obj["w"]
            box_h = obj["h"]
            color = obj["color"]
            render_shape = obj.get("render_shape", "box")

            if render_shape == "traffic_light":
                self._draw_traffic_light_icon(canvas, x, y, box_w, box_h, obj.get("traffic_light_color", "unknown"))
                label_color = _signal_color(obj.get("traffic_light_color", "unknown"))
                cv2.putText(
                    canvas,
                    f"traffic light {obj.get('traffic_light_color', 'unknown')} {obj['distance']:.1f}m",
                    (x - 54, y - box_h // 2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    label_color,
                    1,
                )
                continue

            if render_shape == "stop_sign":
                self._draw_stop_sign_icon(canvas, x, y, max(box_w, box_h))
                cv2.putText(
                    canvas,
                    f"stop sign {obj['distance']:.1f}m",
                    (x - 40, y - box_h // 2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (255, 255, 255),
                    1,
                )
                continue

            top_left = (x - box_w // 2, y - box_h // 2)
            bottom_right = (x + box_w // 2, y + box_h // 2)

            overlay = canvas.copy()
            cv2.rectangle(overlay, top_left, bottom_right, color, -1)
            cv2.addWeighted(overlay, 0.24, canvas, 0.76, 0, canvas)
            cv2.rectangle(canvas, top_left, bottom_right, color, 2)
            cv2.rectangle(canvas, top_left, bottom_right, (245, 248, 250), 1)

            cv2.putText(
                canvas,
                f"{obj['class']} {obj['distance']:.1f}m",
                (top_left[0] - 4, top_left[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
            )

    def _draw_ego(self, canvas, layout):
        ego_center_x = self._lane_center_x(1, layout)
        ego_top_y = layout["road_bottom"] + 10
        ego_w = 42
        ego_h = 62

        cv2.rectangle(
            canvas,
            (ego_center_x - ego_w // 2, ego_top_y),
            (ego_center_x + ego_w // 2, ego_top_y + ego_h),
            (222, 230, 235),
            -1,
        )
        cv2.rectangle(
            canvas,
            (ego_center_x - ego_w // 2, ego_top_y),
            (ego_center_x + ego_w // 2, ego_top_y + ego_h),
            (90, 102, 110),
            2,
        )
        cv2.putText(
            canvas,
            "EGO",
            (ego_center_x - 16, ego_top_y + ego_h + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (220, 228, 235),
            1,
        )

    def _draw_bar(self, canvas, x, y, width, height, label, value, color):
        cv2.putText(
            canvas,
            label,
            (x, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (170, 190, 205),
            1,
        )
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (34, 42, 50), -1)
        fill_width = int(width * np.clip(value / 100.0, 0.0, 1.0))
        if fill_width > 0:
            cv2.rectangle(canvas, (x, y), (x + fill_width, y + height), color, -1)
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (70, 92, 108), 1)
        cv2.putText(
            canvas,
            f"{int(value)}%",
            (x + width - 42, y + height - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (240, 245, 248),
            1,
        )

    def _draw_status_card(self, canvas, x, y, width, height, title, value, accent_color):
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (20, 28, 36), -1)
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (55, 78, 90), 1)
        cv2.rectangle(canvas, (x, y), (x + 8, y + height), accent_color, -1)
        cv2.putText(
            canvas,
            title,
            (x + 18, y + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (152, 176, 190),
            1,
        )
        cv2.putText(
            canvas,
            value,
            (x + 18, y + 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (240, 245, 248),
            2,
        )

    def _draw_speedometer(self, canvas, x, y, radius, speed_kmh, max_speed_kmh):
        speed_kmh = float(np.clip(speed_kmh, 0.0, max_speed_kmh))
        center = (x, y)
        start_angle = 210
        end_angle = -30

        cv2.circle(canvas, center, radius + 8, (20, 28, 36), -1)
        cv2.circle(canvas, center, radius + 8, (55, 78, 90), 1)
        cv2.ellipse(canvas, center, (radius, radius), 0, start_angle, end_angle, (55, 68, 78), 10)

        speed_ratio = 0.0 if max_speed_kmh <= 0.0 else speed_kmh / max_speed_kmh
        sweep_angle = start_angle + ((end_angle - start_angle) * speed_ratio)
        accent_color = _status_color("throttle", min(100.0, (speed_ratio * 100.0)))
        cv2.ellipse(canvas, center, (radius, radius), 0, start_angle, sweep_angle, accent_color, 10)

        for tick_speed in range(0, int(max_speed_kmh) + 1, 20):
            tick_ratio = tick_speed / max_speed_kmh
            tick_angle_deg = start_angle + ((end_angle - start_angle) * tick_ratio)
            tick_angle = np.deg2rad(tick_angle_deg)
            outer_point = (
                int(center[0] + np.cos(tick_angle) * (radius + 4)),
                int(center[1] + np.sin(tick_angle) * (radius + 4)),
            )
            inner_point = (
                int(center[0] + np.cos(tick_angle) * (radius - 12)),
                int(center[1] + np.sin(tick_angle) * (radius - 12)),
            )
            cv2.line(canvas, outer_point, inner_point, (190, 205, 216), 2)
            text_point = (
                int(center[0] + np.cos(tick_angle) * (radius - 28)) - 10,
                int(center[1] + np.sin(tick_angle) * (radius - 28)) + 5,
            )
            cv2.putText(
                canvas,
                str(tick_speed),
                text_point,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (180, 198, 210),
                1,
            )

        needle_angle = np.deg2rad(sweep_angle)
        needle_tip = (
            int(center[0] + np.cos(needle_angle) * (radius - 18)),
            int(center[1] + np.sin(needle_angle) * (radius - 18)),
        )
        cv2.line(canvas, center, needle_tip, accent_color, 3)
        cv2.circle(canvas, center, 6, (240, 245, 248), -1)
        cv2.circle(canvas, center, 8, (60, 78, 92), 2)

        cv2.putText(
            canvas,
            "EST. SPEED",
            (x - 42, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (160, 182, 196),
            1,
        )
        cv2.putText(
            canvas,
            f"{speed_kmh:.0f}",
            (x - 26, y + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.00,
            (245, 248, 250),
            2,
        )
        cv2.putText(
            canvas,
            "km/h",
            (x - 18, y + 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (170, 190, 205),
            1,
        )

    def _draw_ttc_card(self, canvas, x, y, width, height, ttc_s, closing_speed_mps, focus_distance):
        accent_color = _ttc_color(ttc_s)
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (20, 28, 36), -1)
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (55, 78, 90), 1)
        cv2.rectangle(canvas, (x, y), (x + 8, y + height), accent_color, -1)

        cv2.putText(
            canvas,
            "Time To Collision",
            (x + 18, y + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (152, 176, 190),
            1,
        )

        if ttc_s is None:
            value_text = "--.-"
            subtitle = "No imminent collision"
        else:
            value_text = f"{ttc_s:.1f}s"
            subtitle = "Potential collision window"

        cv2.putText(
            canvas,
            value_text,
            (x + 18, y + 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.90,
            (245, 248, 250),
            2,
        )
        cv2.putText(
            canvas,
            subtitle,
            (x + 18, y + 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            accent_color,
            1,
        )

        if focus_distance is not None:
            cv2.putText(
                canvas,
                f"Target {focus_distance:.1f}m",
                (x + 18, y + height - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (190, 208, 220),
                1,
            )
        if closing_speed_mps is not None:
            cv2.putText(
                canvas,
                f"Closing {closing_speed_mps:.1f} m/s",
                (x + 18, y + height - 38),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (190, 208, 220),
                1,
            )

    def _draw_dashboard_panel(self, canvas, decision, objects, road_context, layout):
        panel_left = layout["panel_left"] + 18
        panel_width = BIRDVIEW_PANEL_WIDTH - 36
        nearest_distance = min((obj.get("distance", BIRDVIEW_MAX_DISTANCE) for obj in objects), default=BIRDVIEW_MAX_DISTANCE)
        focus_id = decision.get("focus_object_id")
        focus_lane = decision.get("focus_lane_relation")
        focus_distance = decision.get("focus_distance_m")
        focus_ttc = decision.get("focus_ttc_s")
        focus_relative_speed = decision.get("focus_relative_speed_mps")
        traffic_light_state = decision.get("traffic_light_state")
        estimated_speed_kmh = float(decision.get("estimated_speed_kmh", 0.0) or 0.0)
        speed_target_kmh = float(decision.get("speed_target_kmh", estimated_speed_kmh) or estimated_speed_kmh)
        visibility_condition = decision.get("visibility_condition", road_context.get("visibility_condition", "day"))
        visibility_score = float(decision.get("visibility_score", road_context.get("visibility_score", 0.0)) or 0.0)

        cv2.putText(
            canvas,
            "AUTOPILOT BOARD",
            (panel_left, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (245, 248, 250),
            2,
        )
        cv2.putText(
            canvas,
            "Lane-aware longitudinal control",
            (panel_left, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (150, 176, 190),
            1,
        )
        cv2.putText(
            canvas,
            f"Visibility: {_pretty_label(visibility_condition)}  {visibility_score:.2f}",
            (panel_left, 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            _visibility_color(visibility_condition),
            1,
        )

        self._draw_speedometer(canvas, panel_left + 72, 148, 54, estimated_speed_kmh, DECISION_SPEEDOMETER_MAX_KMH)
        self._draw_ttc_card(
            canvas,
            panel_left + 130,
            96,
            panel_width - 130,
            112,
            focus_ttc,
            focus_relative_speed,
            focus_distance,
        )
        cv2.putText(
            canvas,
            f"Target speed {speed_target_kmh:.0f} km/h",
            (panel_left + 16, 224),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (170, 190, 205),
            1,
        )

        card_h = 60
        self._draw_status_card(
            canvas,
            panel_left,
            238,
            panel_width,
            card_h,
            "Brake",
            _pretty_label(decision.get("brake", "no_brake")),
            _status_color("brake", decision.get("brake")),
        )
        self._draw_status_card(
            canvas,
            panel_left,
            308,
            panel_width,
            card_h,
            "Throttle",
            _pretty_label(decision.get("throttle", "maintain_throttle")),
            _status_color("throttle", decision.get("throttle_pct", 0)),
        )
        self._draw_status_card(
            canvas,
            panel_left,
            378,
            panel_width,
            card_h,
            "Risk",
            _pretty_label(decision.get("risk", "low")),
            _status_color("risk", decision.get("risk")),
        )

        self._draw_bar(
            canvas,
            panel_left,
            450,
            panel_width,
            18,
            "Brake Cmd",
            decision.get("brake_pct", 0),
            _status_color("brake", decision.get("brake")),
        )
        self._draw_bar(
            canvas,
            panel_left,
            498,
            panel_width,
            18,
            "Throttle Cmd",
            decision.get("throttle_pct", 0),
            _status_color("throttle", decision.get("throttle_pct", 0)),
        )

        focus_top = 536
        focus_bottom = 606
        cv2.rectangle(canvas, (panel_left, focus_top), (panel_left + panel_width, focus_bottom), (20, 28, 36), -1)
        cv2.rectangle(canvas, (panel_left, focus_top), (panel_left + panel_width, focus_bottom), (55, 78, 90), 1)
        cv2.putText(
            canvas,
            "Focus",
            (panel_left + 14, focus_top + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (152, 176, 190),
            1,
        )
        cv2.putText(
            canvas,
            f"Objects: {len(objects)}",
            (panel_left + 14, focus_top + 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (240, 245, 248),
            1,
        )
        cv2.putText(
            canvas,
            f"Nearest: {nearest_distance:.1f}m",
            (panel_left + 124, focus_top + 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (240, 245, 248),
            1,
        )

        if focus_id is not None and focus_distance is not None:
            cv2.putText(
                canvas,
                f"Target ID: {focus_id}",
                (panel_left + 14, focus_top + 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (240, 245, 248),
                1,
            )
            cv2.putText(
                canvas,
                f"Lane: {str(focus_lane).replace('_', ' ')}",
                (panel_left + 104, focus_top + 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (190, 208, 220),
                1,
            )
            cv2.putText(
                canvas,
                f"Distance {focus_distance:.1f}m",
                (panel_left + 14, focus_bottom - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (190, 208, 220),
                1,
            )
            if focus_ttc is not None:
                cv2.putText(
                    canvas,
                    f"TTC {focus_ttc:.1f}s",
                    (panel_left + 126, focus_bottom - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    _ttc_color(focus_ttc),
                    1,
                )
        else:
            model_text = "fallback lanes" if road_context.get("lane_visibility_low", True) else "full lane lock"
            cv2.putText(
                canvas,
                f"Lane model: {model_text}",
                (panel_left + 14, focus_top + 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (190, 208, 220),
                1,
            )

        if traffic_light_state is not None:
            cv2.putText(
                canvas,
                f"Signal: {traffic_light_state}",
                (panel_left + 136, focus_top + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                _signal_color(traffic_light_state),
                1,
            )

    def render(self, objects, road_context, decision):
        canvas = np.zeros((BIRDVIEW_WINDOW_HEIGHT, BIRDVIEW_WINDOW_WIDTH, 3), dtype=np.uint8)
        layout = self._layout()

        self._draw_background(canvas, layout)
        self._update_road_heading(road_context)
        self._update_road_profile(road_context)
        self._draw_road(canvas, road_context, layout)
        drawables = self._update_object_states(objects, layout)
        self._draw_objects(canvas, drawables)
        self._draw_ego(canvas, layout)
        self._draw_dashboard_panel(canvas, decision, objects, road_context, layout)

        cv2.putText(
            canvas,
            "Autopilot Surround View",
            (24, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (245, 248, 250),
            2,
        )

        return canvas
