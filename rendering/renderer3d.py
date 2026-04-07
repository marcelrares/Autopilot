import time

import numpy as np
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

from config import (
    PERSON_CLASSES,
    RENDER3D_CAMERA_BACK_Z,
    RENDER3D_CAMERA_HEIGHT,
    RENDER3D_CAMERA_LOOKAHEAD,
    RENDER3D_EGO_Z,
    RENDER3D_FAR_PLANE,
    RENDER3D_LANE_WIDTH,
    RENDER3D_MAX_DISTANCE,
    RENDER3D_OBJECT_SMOOTHING_ALPHA,
    RENDER3D_ROAD_CURVE_SMOOTHING_ALPHA,
    RENDER3D_ROAD_HEADING_SMOOTHING_ALPHA,
    RENDER3D_WINDOW_HEIGHT,
    RENDER3D_WINDOW_WIDTH,
    VEHICLE_CLASSES,
)


def _rgb(color):
    return tuple(channel / 255.0 for channel in color)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _object_color(distance, lane_index, focused=False):
    if focused:
        return (255, 210, 90)
    if lane_index == 0:
        if distance < 10:
            return (255, 110, 90)
        if distance < 16:
            return (255, 195, 80)
        return (70, 220, 140)
    if lane_index < 0:
        return (135, 200, 255)
    return (120, 220, 230)


def _signal_color(state):
    return {
        "red": (255, 92, 72),
        "yellow": (255, 214, 90),
        "green": (78, 230, 140),
    }.get(state, (155, 165, 178))


def _vehicle_dimensions(class_name):
    if class_name == "truck":
        return 2.4, 2.5, 5.8
    if class_name == "bus":
        return 2.55, 3.0, 7.5
    if class_name == "motorcycle":
        return 0.90, 1.25, 2.10
    return 1.9, 1.6, 4.3


class Scene3DRenderer:
    def __init__(self):
        self.object_states = {}
        self.last_caption_update = 0.0
        self.road_heading_norm = 0.0
        self.road_profile = []
        if not pygame.font.get_init():
            pygame.font.init()
        self.label_font = pygame.font.SysFont("Segoe UI", 18, bold=True)
        self.label_small_font = pygame.font.SysFont("Segoe UI", 14)

    @staticmethod
    def _blend(previous, current, alpha):
        return ((1.0 - alpha) * previous) + (alpha * current)

    @staticmethod
    def _road_bounds():
        half_width = RENDER3D_LANE_WIDTH * 2.75
        return -half_width, half_width

    @staticmethod
    def _lane_center_x(lane_index):
        lane_index = int(np.clip(lane_index, -2, 2))
        return lane_index * RENDER3D_LANE_WIDTH

    def update_road_heading(self, road_context):
        target_heading = float(road_context.get("road_heading_norm", 0.0))
        self.road_heading_norm = self._blend(
            self.road_heading_norm,
            target_heading,
            RENDER3D_ROAD_HEADING_SMOOTHING_ALPHA,
        )
        return self.road_heading_norm

    def update_road_profile(self, road_context):
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
                    RENDER3D_ROAD_CURVE_SMOOTHING_ALPHA,
                ),
            })

        self.road_profile = smoothed_profile
        return self.road_profile

    def _road_shift_from_profile(self, distance):
        if not self.road_profile:
            return None
        target_distance = max(0.0, float(distance))
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

    def road_shift_for_distance(self, distance):
        profile_shift = self._road_shift_from_profile(distance)
        if profile_shift is not None:
            return profile_shift
        progress = float(np.clip(distance / max(1.0, RENDER3D_MAX_DISTANCE), 0.0, 1.0))
        eased = progress ** 1.25
        return self.road_heading_norm * RENDER3D_LANE_WIDTH * 2.6 * eased

    @staticmethod
    def _label_lines_for(target_state):
        if not target_state.get("focused"):
            return None, None
        obj_class = str(target_state["class"]).replace("_", " ")
        primary_text = f"{obj_class.title()} {target_state['distance']:.1f}m"
        ttc_s = target_state.get("ttc_s")
        relative_speed_mps = target_state.get("relative_speed_mps")
        signal_state = target_state.get("traffic_light_color", "unknown")
        if target_state["class"] == "traffic light":
            secondary_text = f"Signal {signal_state}"
        elif ttc_s is not None:
            secondary_text = f"TTC {ttc_s:.1f}s"
        elif relative_speed_mps is not None and relative_speed_mps > 0.25:
            secondary_text = f"Closing {relative_speed_mps:.1f} m/s"
        else:
            secondary_text = None
        return primary_text, secondary_text

    def _target_state(self, obj):
        lane_index = int(np.clip(obj.get("lane_index", 0), -2, 2))
        distance = float(np.clip(obj.get("distance", RENDER3D_MAX_DISTANCE), 0.0, RENDER3D_MAX_DISTANCE))
        lane_offset = float(np.clip(obj.get("lane_offset", 0.0) - lane_index, -0.80, 0.80))
        lateral_distance_m = obj.get("lateral_distance_m")
        obj_class = obj["class"]

        curve_shift = self.road_shift_for_distance(distance)
        if lateral_distance_m is not None:
            road_left, road_right = self._road_bounds()
            x = float(np.clip(lateral_distance_m, road_left - 1.8, road_right + 1.8)) + curve_shift
        else:
            x = self._lane_center_x(lane_index) + (lane_offset * RENDER3D_LANE_WIDTH * 0.52) + curve_shift
        z = RENDER3D_EGO_Z - distance
        render_shape = "generic"
        traffic_light_color = obj.get("traffic_light_color", "unknown")

        if obj_class == "traffic light":
            road_left, road_right = self._road_bounds()
            side = -1 if obj.get("lane_relation") in {"left_adjacent", "far_left"} or obj.get("lane_offset", 0.0) < -0.15 else 1
            x = ((road_left - 1.35) if side < 0 else (road_right + 1.35)) + curve_shift
            width, height, length = 0.70, 4.90, 0.58
            render_shape = "traffic_light"
            label_anchor_y = 5.3
        elif obj_class == "stop sign":
            road_left, road_right = self._road_bounds()
            side = -1 if obj.get("lane_relation") in {"left_adjacent", "far_left"} or obj.get("lane_offset", 0.0) < -0.15 else 1
            x = ((road_left - 1.0) if side < 0 else (road_right + 1.0)) + curve_shift
            width, height, length = 0.52, 3.35, 0.34
            render_shape = "stop_sign"
            label_anchor_y = 3.7
        elif obj_class == "truck":
            width, height, length = _vehicle_dimensions(obj_class)
            render_shape = "truck"
            label_anchor_y = height + 0.9
        elif obj_class == "bus":
            width, height, length = _vehicle_dimensions(obj_class)
            render_shape = "bus"
            label_anchor_y = height + 0.85
        elif obj_class == "car":
            width, height, length = _vehicle_dimensions(obj_class)
            render_shape = "car"
            label_anchor_y = height + 0.8
        elif obj_class == "motorcycle":
            width, height, length = _vehicle_dimensions(obj_class)
            render_shape = "motorcycle"
            label_anchor_y = height + 0.7
        elif obj_class == "person":
            width, height, length = 0.72, 1.78, 0.72
            render_shape = "person"
            label_anchor_y = height + 0.7
        elif obj_class == "bicycle":
            width, height, length = 0.86, 1.35, 1.75
            render_shape = "bicycle"
            label_anchor_y = height + 0.7
        else:
            width, height, length = 1.0, 1.3, 1.0
            render_shape = "generic"
            label_anchor_y = height + 0.8

        return {
            "x": float(x),
            "z": float(z),
            "distance": distance,
            "lane_index": lane_index,
            "lateral_distance_m": None if lateral_distance_m is None else float(lateral_distance_m),
            "width": float(width),
            "height": float(height),
            "length": float(length),
            "label_anchor_y": float(label_anchor_y),
            "class": obj_class,
            "lane_relation": obj.get("lane_relation"),
            "id": str(obj.get("id")),
            "render_shape": render_shape,
            "traffic_light_color": traffic_light_color,
            "ttc_s": obj.get("ttc_s"),
            "relative_speed_mps": obj.get("relative_speed_mps"),
        }

    def build_scene_objects(self, objects, road_context, decision):
        self.update_road_heading(road_context)
        self.update_road_profile(road_context)
        active_ids = set()
        scene_objects = []
        focus_id = None if decision is None else str(decision.get("focus_object_id"))
        brake_pct = 0 if decision is None else int(decision.get("brake_pct", 0))

        for obj in sorted(objects, key=lambda item: item.get("distance", 9999), reverse=True):
            target_state = self._target_state(obj)
            object_id = target_state["id"]
            active_ids.add(object_id)

            previous_state = self.object_states.get(object_id)
            if previous_state is not None and previous_state.get("class") == target_state["class"]:
                alpha = RENDER3D_OBJECT_SMOOTHING_ALPHA
                for key in ("x", "z", "distance", "width", "height", "length", "label_anchor_y"):
                    target_state[key] = self._blend(previous_state[key], target_state[key], alpha)

            target_state["missed"] = 0
            self.object_states[object_id] = target_state
            target_state["focused"] = object_id == focus_id
            target_state["color"] = _object_color(
                target_state["distance"],
                target_state["lane_index"],
                target_state["focused"],
            )

            relative_speed_mps = target_state.get("relative_speed_mps")
            ttc_s = target_state.get("ttc_s")
            brake_intensity = 0.0
            if target_state["class"] in VEHICLE_CLASSES and target_state["lane_index"] == 0:
                if ttc_s is not None:
                    brake_intensity = max(brake_intensity, _clamp((4.2 - float(ttc_s)) / 2.6, 0.0, 1.0))
                if relative_speed_mps is not None and float(relative_speed_mps) > 0.8:
                    brake_intensity = max(brake_intensity, _clamp(float(relative_speed_mps) / 5.0, 0.0, 1.0))
                if target_state["focused"] and brake_pct > 0:
                    brake_intensity = max(brake_intensity, _clamp(0.35 + (brake_pct / 100.0), 0.0, 1.0))

            target_state["brake_light_intensity"] = float(_clamp(brake_intensity, 0.0, 1.0))
            primary_label, secondary_label = self._label_lines_for(target_state)
            target_state["label_primary"] = primary_label
            target_state["label_secondary"] = secondary_label
            scene_objects.append(target_state)

        stale_ids = []
        for object_id, state in self.object_states.items():
            if object_id in active_ids:
                continue
            state["missed"] = state.get("missed", 0) + 1
            if state["missed"] > 10:
                stale_ids.append(object_id)

        for object_id in stale_ids:
            self.object_states.pop(object_id, None)

        return scene_objects

    def update_caption(self, scene_snapshot):
        now = time.time()
        if now - self.last_caption_update < 0.2:
            return
        decision = scene_snapshot.get("decision", {})
        focus_distance = decision.get("focus_distance_m")
        focus_lane = decision.get("focus_lane_relation")
        focus_ttc = decision.get("focus_ttc_s")
        traffic_light_state = decision.get("traffic_light_state")
        focus_text = "none"
        if focus_distance is not None:
            focus_text = f"{str(focus_lane).replace('_', ' ')} {focus_distance:.1f}m"
            if focus_ttc is not None:
                focus_text += f" TTC {focus_ttc:.1f}s"

        pygame.display.set_caption(
            "3D Autopilot | "
            f"Brake {int(decision.get('brake_pct', 0))}% | "
            f"Throttle {int(decision.get('throttle_pct', 0))}% | "
            f"Target {focus_text} | "
            f"Signal {traffic_light_state or 'none'}"
        )
        self.last_caption_update = now


def _setup_gl(display_width, display_height):
    glViewport(0, 0, display_width, display_height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(58.0, display_width / max(1.0, display_height), 0.1, RENDER3D_FAR_PLANE)
    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LINE_SMOOTH)
    glClearColor(0.05, 0.08, 0.12, 1.0)


def _set_camera():
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(
        0.0,
        RENDER3D_CAMERA_HEIGHT,
        RENDER3D_CAMERA_BACK_Z,
        0.0,
        1.0,
        RENDER3D_EGO_Z - RENDER3D_CAMERA_LOOKAHEAD,
        0.0,
        1.0,
        0.0,
    )


def _draw_quad_3d(vertices, color, alpha=1.0):
    glColor4f(color[0], color[1], color[2], alpha)
    glBegin(GL_QUADS)
    for vertex in vertices:
        glVertex3f(*vertex)
    glEnd()


def _draw_ground():
    ground_color = _rgb((10, 36, 50))
    _draw_quad_3d(
        [
            (-36.0, -0.02, 20.0),
            (36.0, -0.02, 20.0),
            (36.0, -0.02, -120.0),
            (-36.0, -0.02, -120.0),
        ],
        ground_color,
        1.0,
    )


def _road_center_shift(renderer, z):
    ahead_distance = max(0.0, RENDER3D_EGO_Z - z)
    return renderer.road_shift_for_distance(ahead_distance)


def _draw_road(renderer):
    road_left, road_right = Scene3DRenderer._road_bounds()
    road_near = 16.0
    road_far = -110.0
    road_color = _rgb((42, 46, 52))
    ego_lane_color = _rgb((25, 68, 88))
    shoulder_color = _rgb((28, 34, 40))
    edge_color = _rgb((80, 220, 255))

    z_samples = np.arange(road_near, road_far - 2.0, -3.0)
    for z0, z1 in zip(z_samples[:-1], z_samples[1:]):
        shift0 = _road_center_shift(renderer, z0)
        shift1 = _road_center_shift(renderer, z1)

        _draw_quad_3d(
            [
                (road_left - 2.5 + shift0, 0.0, z0),
                (road_right + 2.5 + shift0, 0.0, z0),
                (road_right + 2.5 + shift1, 0.0, z1),
                (road_left - 2.5 + shift1, 0.0, z1),
            ],
            shoulder_color,
            1.0,
        )
        _draw_quad_3d(
            [
                (road_left + shift0, 0.01, z0),
                (road_right + shift0, 0.01, z0),
                (road_right + shift1, 0.01, z1),
                (road_left + shift1, 0.01, z1),
            ],
            road_color,
            1.0,
        )
        _draw_quad_3d(
            [
                (-RENDER3D_LANE_WIDTH / 2.0 + shift0, 0.012, z0),
                (RENDER3D_LANE_WIDTH / 2.0 + shift0, 0.012, z0),
                (RENDER3D_LANE_WIDTH / 2.0 + shift1, 0.012, z1),
                (-RENDER3D_LANE_WIDTH / 2.0 + shift1, 0.012, z1),
            ],
            ego_lane_color,
            0.95,
        )

    glLineWidth(3.0)
    glColor4f(edge_color[0], edge_color[1], edge_color[2], 0.95)
    glBegin(GL_LINES)
    for z0, z1 in zip(z_samples[:-1], z_samples[1:]):
        shift0 = _road_center_shift(renderer, z0)
        shift1 = _road_center_shift(renderer, z1)
        for edge_x in (road_left, road_right):
            glVertex3f(edge_x + shift0, 0.03, z0)
            glVertex3f(edge_x + shift1, 0.03, z1)
    glEnd()

    glColor4f(0.90, 0.93, 0.96, 1.0)
    glLineWidth(2.0)
    for boundary_x in (-1.5 * RENDER3D_LANE_WIDTH, -0.5 * RENDER3D_LANE_WIDTH, 0.5 * RENDER3D_LANE_WIDTH, 1.5 * RENDER3D_LANE_WIDTH):
        glBegin(GL_LINES)
        for segment_z in np.arange(road_far, road_near, 7.0):
            segment_end_z = min(road_near, segment_z + 3.5)
            shift0 = _road_center_shift(renderer, segment_z)
            shift1 = _road_center_shift(renderer, segment_end_z)
            glVertex3f(boundary_x + shift0, 0.03, segment_z)
            glVertex3f(boundary_x + shift1, 0.03, segment_end_z)
        glEnd()

    glColor4f(0.35, 0.45, 0.55, 1.0)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    for meters in range(5, RENDER3D_MAX_DISTANCE + 5, 5):
        z = RENDER3D_EGO_Z - meters
        shift = renderer.road_shift_for_distance(meters)
        glVertex3f(road_left + shift, 0.02, z)
        glVertex3f(road_right + shift, 0.02, z)
    glEnd()


def _draw_box(width, height, length, color, roof_tint=1.12):
    half_w = width / 2.0
    half_l = length / 2.0
    top = tuple(min(1.0, channel * roof_tint) for channel in color)
    side = tuple(channel * 0.80 for channel in color)
    rear = tuple(channel * 0.68 for channel in color)

    glBegin(GL_QUADS)
    glColor3f(*top)
    glVertex3f(-half_w, height, -half_l)
    glVertex3f(half_w, height, -half_l)
    glVertex3f(half_w, height, half_l)
    glVertex3f(-half_w, height, half_l)

    glColor3f(*side)
    glVertex3f(-half_w, 0.0, -half_l)
    glVertex3f(half_w, 0.0, -half_l)
    glVertex3f(half_w, height, -half_l)
    glVertex3f(-half_w, height, -half_l)

    glColor3f(*rear)
    glVertex3f(-half_w, 0.0, half_l)
    glVertex3f(half_w, 0.0, half_l)
    glVertex3f(half_w, height, half_l)
    glVertex3f(-half_w, height, half_l)

    glColor3f(*(channel * 0.74 for channel in color))
    glVertex3f(-half_w, 0.0, -half_l)
    glVertex3f(-half_w, 0.0, half_l)
    glVertex3f(-half_w, height, half_l)
    glVertex3f(-half_w, height, -half_l)

    glColor3f(*(channel * 0.88 for channel in color))
    glVertex3f(half_w, 0.0, -half_l)
    glVertex3f(half_w, 0.0, half_l)
    glVertex3f(half_w, height, half_l)
    glVertex3f(half_w, height, -half_l)

    glColor3f(*(channel * 0.55 for channel in color))
    glVertex3f(-half_w, 0.0, -half_l)
    glVertex3f(half_w, 0.0, -half_l)
    glVertex3f(half_w, 0.0, half_l)
    glVertex3f(-half_w, 0.0, half_l)
    glEnd()


def _draw_glass_band(width, lower_y, upper_y, length, alpha=0.95):
    glass = (0.12, 0.16, 0.20)
    half_w = width / 2.0
    half_l = length / 2.0
    inset = min(width, length) * 0.04
    glColor4f(glass[0], glass[1], glass[2], alpha)
    glBegin(GL_QUADS)
    glVertex3f(-half_w - 0.001, lower_y, -half_l + inset)
    glVertex3f(-half_w - 0.001, lower_y, half_l - inset)
    glVertex3f(-half_w - 0.001, upper_y, half_l - inset)
    glVertex3f(-half_w - 0.001, upper_y, -half_l + inset)

    glVertex3f(half_w + 0.001, lower_y, -half_l + inset)
    glVertex3f(half_w + 0.001, lower_y, half_l - inset)
    glVertex3f(half_w + 0.001, upper_y, half_l - inset)
    glVertex3f(half_w + 0.001, upper_y, -half_l + inset)

    glVertex3f(-half_w + inset, lower_y, -half_l - 0.001)
    glVertex3f(half_w - inset, lower_y, -half_l - 0.001)
    glVertex3f(half_w - inset, upper_y, -half_l - 0.001)
    glVertex3f(-half_w + inset, upper_y, -half_l - 0.001)

    glVertex3f(-half_w + inset, lower_y, half_l + 0.001)
    glVertex3f(half_w - inset, lower_y, half_l + 0.001)
    glVertex3f(half_w - inset, upper_y, half_l + 0.001)
    glVertex3f(-half_w + inset, upper_y, half_l + 0.001)
    glEnd()


def _draw_window_strip(width, height, length, lower_ratio=0.52, upper_ratio=0.80):
    _draw_glass_band(width, height * lower_ratio, height * upper_ratio, length)


def _draw_vehicle_lights(width, height, length, facing_ego, brake_intensity=0.0, z_offset=0.0, include_high_mount=True):
    half_w = width / 2.0
    half_l = length / 2.0
    front_z = z_offset + (half_l if facing_ego else -half_l)
    rear_z = z_offset + (-half_l if facing_ego else half_l)
    brake_intensity = _clamp(float(brake_intensity), 0.0, 1.0)

    glBegin(GL_QUADS)
    glColor3f(0.98, 0.96, 0.90)
    glVertex3f(-half_w * 0.65, height * 0.34, front_z)
    glVertex3f(-half_w * 0.15, height * 0.34, front_z)
    glVertex3f(-half_w * 0.15, height * 0.56, front_z)
    glVertex3f(-half_w * 0.65, height * 0.56, front_z)

    glVertex3f(half_w * 0.15, height * 0.34, front_z)
    glVertex3f(half_w * 0.65, height * 0.34, front_z)
    glVertex3f(half_w * 0.65, height * 0.56, front_z)
    glVertex3f(half_w * 0.15, height * 0.56, front_z)

    rear_color = (
        0.55 + (0.45 * brake_intensity),
        0.06 + (0.16 * brake_intensity),
        0.05 + (0.10 * brake_intensity),
    )
    glColor3f(*rear_color)
    glVertex3f(-half_w * 0.68, height * 0.24, rear_z)
    glVertex3f(-half_w * 0.24, height * 0.24, rear_z)
    glVertex3f(-half_w * 0.24, height * 0.44, rear_z)
    glVertex3f(-half_w * 0.68, height * 0.44, rear_z)

    glVertex3f(half_w * 0.24, height * 0.24, rear_z)
    glVertex3f(half_w * 0.68, height * 0.24, rear_z)
    glVertex3f(half_w * 0.68, height * 0.44, rear_z)
    glVertex3f(half_w * 0.24, height * 0.44, rear_z)
    glEnd()

    if include_high_mount and brake_intensity > 0.18:
        glow_alpha = 0.16 + (0.24 * brake_intensity)
        glColor4f(1.0, 0.18, 0.12, glow_alpha)
        glBegin(GL_QUADS)
        glVertex3f(-half_w * 0.18, height * 0.70, rear_z + 0.001)
        glVertex3f(half_w * 0.18, height * 0.70, rear_z + 0.001)
        glVertex3f(half_w * 0.18, height * 0.78, rear_z + 0.001)
        glVertex3f(-half_w * 0.18, height * 0.78, rear_z + 0.001)
        glEnd()

    if brake_intensity > 0.22:
        glow_alpha = 0.12 + (0.28 * brake_intensity)
        glColor4f(1.0, 0.22, 0.12, glow_alpha)
        glBegin(GL_QUADS)
        glVertex3f(-half_w * 0.76, height * 0.18, rear_z + 0.002)
        glVertex3f(-half_w * 0.16, height * 0.18, rear_z + 0.002)
        glVertex3f(-half_w * 0.16, height * 0.50, rear_z + 0.002)
        glVertex3f(-half_w * 0.76, height * 0.50, rear_z + 0.002)

        glVertex3f(half_w * 0.16, height * 0.18, rear_z + 0.002)
        glVertex3f(half_w * 0.76, height * 0.18, rear_z + 0.002)
        glVertex3f(half_w * 0.76, height * 0.50, rear_z + 0.002)
        glVertex3f(half_w * 0.16, height * 0.50, rear_z + 0.002)
        glEnd()


def _draw_disc(center_x, center_y, center_z, radius, color, alpha=1.0, segments=18):
    glColor4f(color[0], color[1], color[2], alpha)
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(center_x, center_y, center_z)
    for idx in range(segments + 1):
        angle = (2.0 * np.pi * idx) / segments
        glVertex3f(
            center_x + (radius * np.cos(angle)),
            center_y + (radius * np.sin(angle)),
            center_z,
        )
    glEnd()


def _draw_octagon(center_x, center_y, center_z, radius, fill_color, outline_color):
    vertices = [
        (
            center_x + (radius * np.cos(angle)),
            center_y + (radius * np.sin(angle)),
            center_z,
        )
        for angle in np.deg2rad(np.arange(22.5, 360.0, 45.0))
    ]

    glColor3f(*fill_color)
    glBegin(GL_POLYGON)
    for vertex in vertices:
        glVertex3f(*vertex)
    glEnd()

    glColor3f(*outline_color)
    glLineWidth(2.0)
    glBegin(GL_LINE_LOOP)
    for vertex in vertices:
        glVertex3f(*vertex)
    glEnd()


def _draw_extruded_octagon(radius, depth, fill_color, outline_color):
    angles = np.deg2rad(np.arange(22.5, 360.0, 45.0))
    front_vertices = [(radius * np.cos(angle), radius * np.sin(angle), depth / 2.0) for angle in angles]
    back_vertices = [(radius * np.cos(angle), radius * np.sin(angle), -depth / 2.0) for angle in angles]

    glColor3f(*fill_color)
    glBegin(GL_POLYGON)
    for vertex in front_vertices:
        glVertex3f(*vertex)
    glEnd()

    glBegin(GL_POLYGON)
    for vertex in back_vertices:
        glVertex3f(*vertex)
    glEnd()

    glBegin(GL_QUADS)
    for index in range(len(front_vertices)):
        next_index = (index + 1) % len(front_vertices)
        front_a = front_vertices[index]
        front_b = front_vertices[next_index]
        back_b = back_vertices[next_index]
        back_a = back_vertices[index]
        glVertex3f(*front_a)
        glVertex3f(*front_b)
        glVertex3f(*back_b)
        glVertex3f(*back_a)
    glEnd()

    glColor3f(*outline_color)
    glLineWidth(2.0)
    glBegin(GL_LINE_LOOP)
    for vertex in front_vertices:
        glVertex3f(*vertex)
    glEnd()
    glBegin(GL_LINE_LOOP)
    for vertex in back_vertices:
        glVertex3f(*vertex)
    glEnd()


def _draw_focus_marker(width, height, length):
    half_w = width / 2.0
    half_l = length / 2.0
    glColor4f(1.0, 0.82, 0.35, 0.95)
    glLineWidth(2.0)
    glBegin(GL_LINE_LOOP)
    glVertex3f(-half_w - 0.18, height + 0.18, -half_l - 0.22)
    glVertex3f(half_w + 0.18, height + 0.18, -half_l - 0.22)
    glVertex3f(half_w + 0.18, height + 0.18, half_l + 0.22)
    glVertex3f(-half_w - 0.18, height + 0.18, half_l + 0.22)
    glEnd()


def _draw_car(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    width = scene_object["width"]
    height = scene_object["height"]
    length = scene_object["length"]
    color = _rgb(scene_object["color"])
    facing_ego = scene_object["lane_index"] < 0
    brake_intensity = scene_object.get("brake_light_intensity", 0.0)
    body_height = height * 0.58
    cabin_height = height * 0.42

    glPushMatrix()
    glTranslatef(x, 0.0, z)
    if facing_ego:
        glRotatef(180.0, 0.0, 1.0, 0.0)
    _draw_box(width, body_height, length, color)

    glPushMatrix()
    glTranslatef(0.0, body_height * 0.62, -length * 0.06)
    _draw_box(width * 0.78, cabin_height, length * 0.56, color, roof_tint=1.04)
    _draw_window_strip(width * 0.78, cabin_height, length * 0.56, 0.36, 0.84)
    glPopMatrix()

    glPushMatrix()
    glTranslatef(0.0, body_height * 0.28, -length * 0.20)
    _draw_box(width * 0.82, body_height * 0.16, length * 0.22, color, roof_tint=1.03)
    glPopMatrix()

    _draw_vehicle_lights(width, body_height + (cabin_height * 0.45), length, facing_ego, brake_intensity)
    if scene_object.get("focused"):
        _draw_focus_marker(width, height, length)
    glPopMatrix()


def _draw_truck(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    width = scene_object["width"]
    height = scene_object["height"]
    length = scene_object["length"]
    color = _rgb(scene_object["color"])
    facing_ego = scene_object["lane_index"] < 0
    brake_intensity = scene_object.get("brake_light_intensity", 0.0)
    trailer_length = length * 0.64
    trailer_height = height * 0.78
    trailer_z = length * 0.12
    cab_length = length * 0.26
    cab_z = -(length * 0.29)

    glPushMatrix()
    glTranslatef(x, 0.0, z)
    if facing_ego:
        glRotatef(180.0, 0.0, 1.0, 0.0)

    glPushMatrix()
    glTranslatef(0.0, 0.0, trailer_z)
    _draw_box(width * 0.96, trailer_height, trailer_length, color, roof_tint=1.02)
    glPopMatrix()

    glPushMatrix()
    glTranslatef(0.0, 0.0, cab_z)
    _draw_box(width * 0.82, height, cab_length, color, roof_tint=1.06)
    glTranslatef(0.0, height * 0.52, -cab_length * 0.06)
    _draw_box(width * 0.66, height * 0.28, cab_length * 0.62, color, roof_tint=1.02)
    _draw_window_strip(width * 0.66, height * 0.28, cab_length * 0.62, 0.30, 0.86)
    glPopMatrix()

    glPushMatrix()
    glTranslatef(0.0, trailer_height * 0.22, -length * 0.06)
    _draw_box(width * 0.38, trailer_height * 0.14, length * 0.18, color, roof_tint=1.01)
    glPopMatrix()

    _draw_vehicle_lights(width * 0.96, trailer_height, trailer_length, facing_ego, brake_intensity, z_offset=trailer_z)
    if scene_object.get("focused"):
        _draw_focus_marker(width, height, length)
    glPopMatrix()


def _draw_bus(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    width = scene_object["width"]
    height = scene_object["height"]
    length = scene_object["length"]
    color = _rgb(scene_object["color"])
    facing_ego = scene_object["lane_index"] < 0
    brake_intensity = scene_object.get("brake_light_intensity", 0.0)
    body_height = height * 0.86

    glPushMatrix()
    glTranslatef(x, 0.0, z)
    if facing_ego:
        glRotatef(180.0, 0.0, 1.0, 0.0)

    _draw_box(width * 0.98, body_height, length, color, roof_tint=1.03)
    glPushMatrix()
    glTranslatef(0.0, body_height * 0.86, 0.0)
    _draw_box(width * 0.88, height * 0.14, length * 0.74, color, roof_tint=1.01)
    glPopMatrix()
    _draw_window_strip(width * 0.98, body_height, length * 0.94, 0.44, 0.82)

    glColor4f(0.10, 0.14, 0.18, 0.96)
    glBegin(GL_QUADS)
    glVertex3f(-width * 0.32, body_height * 0.54, -(length / 2.0) - 0.001)
    glVertex3f(width * 0.32, body_height * 0.54, -(length / 2.0) - 0.001)
    glVertex3f(width * 0.32, body_height * 0.80, -(length / 2.0) - 0.001)
    glVertex3f(-width * 0.32, body_height * 0.80, -(length / 2.0) - 0.001)
    glEnd()

    _draw_vehicle_lights(width * 0.98, body_height, length, facing_ego, brake_intensity)
    if scene_object.get("focused"):
        _draw_focus_marker(width, height, length)
    glPopMatrix()


def _draw_motorcycle(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    width = scene_object["width"]
    height = scene_object["height"]
    length = scene_object["length"]
    color = _rgb(scene_object["color"])
    facing_ego = scene_object["lane_index"] < 0
    brake_intensity = scene_object.get("brake_light_intensity", 0.0)

    glPushMatrix()
    glTranslatef(x, 0.0, z)
    if facing_ego:
        glRotatef(180.0, 0.0, 1.0, 0.0)

    glPushMatrix()
    glTranslatef(0.0, height * 0.18, 0.0)
    _draw_box(width * 0.30, height * 0.18, length * 0.54, color, roof_tint=1.02)
    glPopMatrix()

    glPushMatrix()
    glTranslatef(0.0, height * 0.44, -length * 0.08)
    _draw_box(width * 0.26, height * 0.36, length * 0.22, color, roof_tint=1.04)
    glPopMatrix()

    wheel_color = _rgb((38, 44, 50))
    for wheel_z in (-length * 0.30, length * 0.30):
        glPushMatrix()
        glTranslatef(0.0, height * 0.11, wheel_z)
        _draw_box(width * 0.76, height * 0.12, length * 0.08, wheel_color, roof_tint=1.0)
        glPopMatrix()

    _draw_vehicle_lights(width * 0.54, height * 0.72, length * 0.44, facing_ego, brake_intensity)
    if scene_object.get("focused"):
        _draw_focus_marker(width, height, length)
    glPopMatrix()


def _draw_person(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    width = scene_object["width"]
    height = scene_object["height"]
    color = _rgb(scene_object["color"])
    head_color = _rgb((240, 214, 190))

    glPushMatrix()
    glTranslatef(x, 0.0, z)

    leg_width = width * 0.16
    leg_length = width * 0.18
    leg_height = height * 0.42
    torso_height = height * 0.38
    arm_height = height * 0.28

    for leg_x in (-width * 0.12, width * 0.12):
        glPushMatrix()
        glTranslatef(leg_x, 0.0, 0.0)
        _draw_box(leg_width, leg_height, leg_length, color, roof_tint=1.0)
        glPopMatrix()

    glPushMatrix()
    glTranslatef(0.0, leg_height, 0.0)
    _draw_box(width * 0.36, torso_height, width * 0.22, color, roof_tint=1.04)
    glPopMatrix()

    for arm_x in (-width * 0.26, width * 0.26):
        glPushMatrix()
        glTranslatef(arm_x, leg_height + (torso_height * 0.18), 0.0)
        _draw_box(width * 0.10, arm_height, width * 0.12, color, roof_tint=1.0)
        glPopMatrix()

    glPushMatrix()
    glTranslatef(0.0, leg_height + torso_height + (height * 0.06), 0.0)
    _draw_box(width * 0.22, height * 0.18, width * 0.22, head_color, roof_tint=1.0)
    glPopMatrix()

    if scene_object.get("focused"):
        _draw_focus_marker(width * 0.6, height, width * 0.5)
    glPopMatrix()


def _draw_bicycle(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    width = scene_object["width"]
    height = scene_object["height"]
    length = scene_object["length"]
    frame_color = _rgb(scene_object["color"])
    wheel_color = _rgb((36, 42, 46))

    glPushMatrix()
    glTranslatef(x, 0.0, z)

    glPushMatrix()
    glTranslatef(0.0, height * 0.26, 0.0)
    _draw_box(width * 0.20, height * 0.10, length * 0.56, frame_color, roof_tint=1.02)
    glPopMatrix()

    glPushMatrix()
    glTranslatef(0.0, height * 0.52, -length * 0.04)
    _draw_box(width * 0.18, height * 0.32, length * 0.18, frame_color, roof_tint=1.04)
    glPopMatrix()

    for wheel_z in (-length * 0.30, length * 0.30):
        glPushMatrix()
        glTranslatef(0.0, height * 0.10, wheel_z)
        _draw_box(width * 0.72, height * 0.10, length * 0.08, wheel_color, roof_tint=1.0)
        glPopMatrix()

    if scene_object.get("focused"):
        _draw_focus_marker(width, height, length)
    glPopMatrix()


def _draw_generic(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    width = scene_object["width"]
    height = scene_object["height"]
    length = scene_object["length"]
    color = _rgb(scene_object["color"])

    glPushMatrix()
    glTranslatef(x, 0.0, z)
    _draw_box(width, height, length, color, roof_tint=1.04)
    if scene_object.get("focused"):
        _draw_focus_marker(width, height, length)
    glPopMatrix()


def _draw_traffic_light(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    signal_state = scene_object.get("traffic_light_color", "unknown")
    pole_color = _rgb((145, 150, 156))
    housing_color = _rgb((34, 38, 42))
    side = -1.0 if x < 0.0 else 1.0

    glPushMatrix()
    glTranslatef(x, 0.0, z)
    _draw_box(0.14, 4.30, 0.14, pole_color, roof_tint=1.0)

    glPushMatrix()
    glTranslatef(-side * 0.92, 4.02, 0.0)
    _draw_box(1.92, 0.08, 0.08, pole_color, roof_tint=1.0)
    glPopMatrix()

    glPushMatrix()
    glTranslatef(-side * 1.56, 3.28, 0.0)
    _draw_box(0.86, 1.72, 0.56, housing_color, roof_tint=1.02)
    _draw_box(0.96, 0.10, 0.62, pole_color, roof_tint=1.0)
    lamps = [("red", 0.48), ("yellow", 0.0), ("green", -0.48)]
    for lamp_name, lamp_y in lamps:
        glPushMatrix()
        glTranslatef(0.0, lamp_y, 0.29)
        _draw_box(0.34, 0.12, 0.18, housing_color, roof_tint=1.0)
        glTranslatef(0.0, 0.0, 0.04)
        is_active = lamp_name == signal_state
        lamp_color = _rgb(_signal_color(lamp_name if is_active else "unknown"))
        if not is_active:
            lamp_color = _rgb((72, 74, 76))
        _draw_disc(0.0, 0.0, 0.0, 0.13, lamp_color, 1.0)
        if is_active:
            _draw_disc(0.0, 0.0, 0.01, 0.21, lamp_color, 0.24)
        glPopMatrix()
    if scene_object.get("focused"):
        _draw_focus_marker(0.86, 1.72, 0.56)
    glPopMatrix()
    glPopMatrix()


def _draw_stop_sign(scene_object):
    x = scene_object["x"]
    z = scene_object["z"]
    pole_color = _rgb((160, 164, 170))
    fill_color = _rgb((220, 38, 38))
    outline_color = _rgb((245, 248, 250))

    glPushMatrix()
    glTranslatef(x, 0.0, z)
    _draw_box(0.10, 2.25, 0.10, pole_color, roof_tint=1.0)

    glPushMatrix()
    glTranslatef(0.0, 2.82, 0.08)
    _draw_extruded_octagon(0.46, 0.10, fill_color, outline_color)
    glPushMatrix()
    glScalef(0.82, 0.82, 1.0)
    glColor3f(*outline_color)
    glLineWidth(2.0)
    glBegin(GL_LINE_LOOP)
    for angle in np.deg2rad(np.arange(22.5, 360.0, 45.0)):
        glVertex3f(np.cos(angle), np.sin(angle), 0.055)
    glEnd()
    glPopMatrix()
    glPopMatrix()

    if scene_object.get("focused"):
        _draw_focus_marker(0.98, 0.94, 0.24)
    glPopMatrix()


def _draw_focus_path(decision, scene_objects):
    focus_id = None if decision is None else str(decision.get("focus_object_id"))
    if focus_id is None:
        return
    focus_objects = [obj for obj in scene_objects if obj["id"] == focus_id]
    if not focus_objects:
        return

    focus_object = focus_objects[0]
    glColor4f(1.0, 0.78, 0.35, 0.70)
    glLineWidth(2.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.05, RENDER3D_EGO_Z - 1.0)
    glVertex3f(focus_object["x"], 0.05, focus_object["z"])
    glEnd()


def _project_world_to_screen(point_xyz):
    modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
    projection = glGetDoublev(GL_PROJECTION_MATRIX)
    viewport = glGetIntegerv(GL_VIEWPORT)
    screen_point = gluProject(
        float(point_xyz[0]),
        float(point_xyz[1]),
        float(point_xyz[2]),
        modelview,
        projection,
        viewport,
    )
    if screen_point is None:
        return None
    screen_x, screen_y, screen_z = screen_point
    if screen_z < 0.0 or screen_z > 1.0:
        return None
    return float(screen_x), float(screen_y)


def _build_label_surface(renderer, primary_text, secondary_text, accent_color):
    padding_x = 10
    padding_y = 6
    gap = 2
    primary_surface = renderer.label_font.render(primary_text, True, (244, 248, 250))
    secondary_surface = None
    if secondary_text:
        secondary_surface = renderer.label_small_font.render(secondary_text, True, (188, 205, 216))

    width = primary_surface.get_width() + (padding_x * 2)
    height = primary_surface.get_height() + (padding_y * 2)
    if secondary_surface is not None:
        width = max(width, secondary_surface.get_width() + (padding_x * 2))
        height += secondary_surface.get_height() + gap

    surface = pygame.Surface((width, height), pygame.SRCALPHA)
    pygame.draw.rect(surface, (14, 20, 28, 222), (0, 0, width, height), border_radius=8)
    pygame.draw.rect(surface, accent_color + (255,), (0, 0, width, 4), border_radius=8)
    pygame.draw.rect(surface, (64, 88, 102, 220), (0, 0, width, height), width=1, border_radius=8)
    surface.blit(primary_surface, (padding_x, padding_y))
    if secondary_surface is not None:
        surface.blit(secondary_surface, (padding_x, padding_y + primary_surface.get_height() + gap))
    return surface


def _draw_surface_pixels(surface, screen_x, screen_y):
    pixel_data = pygame.image.tostring(surface, "RGBA", True)
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
    glWindowPos2f(float(screen_x), float(screen_y))
    glDrawPixels(surface.get_width(), surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, pixel_data)


def _draw_scene_labels(scene_objects, renderer):
    for scene_object in scene_objects:
        primary_text = scene_object.get("label_primary")
        if not primary_text:
            continue
        screen_point = _project_world_to_screen((
            scene_object["x"],
            scene_object["label_anchor_y"],
            scene_object["z"],
        ))
        if screen_point is None:
            continue

        label_surface = _build_label_surface(
            renderer,
            primary_text,
            scene_object.get("label_secondary"),
            tuple(int(channel) for channel in scene_object["color"]),
        )
        label_x = screen_point[0] - (label_surface.get_width() / 2.0)
        label_y = screen_point[1] + 10.0
        _draw_surface_pixels(label_surface, label_x, label_y)


def _draw_scene(scene_objects, scene_snapshot):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    _set_camera()
    _draw_ground()
    renderer = scene_snapshot["renderer"]
    _draw_road(renderer)
    _draw_focus_path(scene_snapshot.get("decision", {}), scene_objects)

    for scene_object in scene_objects:
        render_shape = scene_object.get("render_shape")
        if render_shape == "traffic_light":
            _draw_traffic_light(scene_object)
        elif render_shape == "stop_sign":
            _draw_stop_sign(scene_object)
        elif render_shape == "truck":
            _draw_truck(scene_object)
        elif render_shape == "bus":
            _draw_bus(scene_object)
        elif render_shape == "car":
            _draw_car(scene_object)
        elif render_shape == "motorcycle":
            _draw_motorcycle(scene_object)
        elif render_shape == "person":
            _draw_person(scene_object)
        elif render_shape == "bicycle":
            _draw_bicycle(scene_object)
        else:
            _draw_generic(scene_object)

    ego_color = _rgb((222, 230, 236))
    glPushMatrix()
    glTranslatef(0.0, 0.0, RENDER3D_EGO_Z)
    _draw_box(2.0, 1.5, 4.6, ego_color, roof_tint=1.05)
    _draw_vehicle_lights(2.0, 1.5, 4.6, facing_ego=False, brake_intensity=0.0)
    glPopMatrix()
    _draw_scene_labels(scene_objects, renderer)


def run_3d(get_scene):
    pygame.init()
    display = (RENDER3D_WINDOW_WIDTH, RENDER3D_WINDOW_HEIGHT)
    pygame.display.set_mode(display, pygame.DOUBLEBUF | pygame.OPENGL)

    _setup_gl(*display)
    renderer = Scene3DRenderer()
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        scene_snapshot = get_scene() or {}
        renderer.update_caption(scene_snapshot)
        scene_objects = renderer.build_scene_objects(
            scene_snapshot.get("objects", []),
            scene_snapshot.get("road_context", {}),
            scene_snapshot.get("decision", {}),
        )
        draw_snapshot = dict(scene_snapshot)
        draw_snapshot["renderer"] = renderer
        _draw_scene(scene_objects, draw_snapshot)

        pygame.display.flip()
        clock.tick(60)
