import cv2
import numpy as np

from config import VEHICLE_CLASSES


def _traffic_light_display_color(state):
    return {
        "red": (0, 0, 255),
        "yellow": (0, 220, 255),
        "green": (0, 220, 0),
    }.get(state, (170, 170, 170))


def _draw_traffic_light_symbol(frame, x1, y1, x2, y2, state):
    box_w = max(12, x2 - x1)
    box_h = max(24, y2 - y1)
    center_x = (x1 + x2) // 2
    housing_w = max(14, min(26, int(box_w * 0.55)))
    housing_h = max(36, min(70, int(box_h * 0.90)))
    housing_x1 = center_x - housing_w // 2
    housing_y1 = max(5, y1)
    housing_x2 = housing_x1 + housing_w
    housing_y2 = housing_y1 + housing_h
    radius = max(4, housing_w // 5)

    cv2.rectangle(frame, (housing_x1, housing_y1), (housing_x2, housing_y2), (35, 35, 35), -1)
    cv2.rectangle(frame, (housing_x1, housing_y1), (housing_x2, housing_y2), (225, 225, 225), 1)
    cv2.line(frame, (center_x, housing_y2), (center_x, min(frame.shape[0] - 1, housing_y2 + 14)), (110, 110, 110), 2)

    light_centers = [
        (center_x, housing_y1 + housing_h // 4),
        (center_x, housing_y1 + housing_h // 2),
        (center_x, housing_y1 + (3 * housing_h) // 4),
    ]
    light_names = ["red", "yellow", "green"]

    for light_name, light_center in zip(light_names, light_centers):
        color = (55, 55, 55)
        if light_name == state:
            color = _traffic_light_display_color(state)
        cv2.circle(frame, light_center, radius, color, -1)
        cv2.circle(frame, light_center, radius, (220, 220, 220), 1)


def _draw_stop_sign_symbol(frame, x1, y1, x2, y2):
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    radius = max(10, min((x2 - x1) // 2, (y2 - y1) // 2))
    angles = np.deg2rad(np.arange(22.5, 360.0, 45.0))
    pts = np.array([
        [
            int(center_x + radius * np.cos(angle)),
            int(center_y + radius * np.sin(angle)),
        ]
        for angle in angles
    ], dtype=np.int32)

    cv2.fillPoly(frame, [pts], (0, 0, 220))
    cv2.polylines(frame, [pts], True, (255, 255, 255), 2)
    if radius >= 14:
        cv2.putText(
            frame,
            "STOP",
            (center_x - radius + 2, center_y + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (255, 255, 255),
            1,
        )


def draw_objects(frame, objects):
    for obj in objects:
        x1, y1, x2, y2 = obj["bbox"]
        obj_class = obj["class"]
        traffic_light_color = obj.get("traffic_light_color", "unknown")

        color = (0, 255, 0)
        if obj["distance"] < 15:
            color = (0, 165, 255)
        if obj["distance"] < 10:
            color = (0, 0, 255)

        label = f"{obj_class} ID:{obj['id']} {int(obj['distance'])}m"
        if obj_class == "traffic light":
            label = f"traffic light {traffic_light_color} {int(obj['distance'])}m"
            color = _traffic_light_display_color(traffic_light_color)
        elif obj_class == "stop sign":
            label = f"stop sign {int(obj['distance'])}m"
            color = (0, 0, 255)
        polygon = obj.get("polygon")

        if obj_class in VEHICLE_CLASSES and polygon is not None and len(polygon) >= 3:
            epsilon = 0.002 * cv2.arcLength(polygon, True)
            smooth_polygon = cv2.approxPolyDP(polygon, epsilon, True)

            overlay = frame.copy()
            cv2.fillPoly(overlay, [smooth_polygon], color)
            frame = cv2.addWeighted(overlay, 0.18, frame, 0.82, 0)
            cv2.polylines(frame, [smooth_polygon], True, color, 3)
        elif obj_class == "traffic light":
            _draw_traffic_light_symbol(frame, x1, y1, x2, y2, traffic_light_color)
        elif obj_class == "stop sign":
            _draw_stop_sign_symbol(frame, x1, y1, x2, y2)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        cv2.putText(
            frame,
            label,
            (x1, max(25, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2
        )

    return frame


def draw_lanes(frame, lanes, roi_polygon):
    if lanes is not None:
        for line in lanes:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    cv2.polylines(frame, roi_polygon, True, (0, 255, 255), 2)
    return frame


def draw_decision_ui(frame, decision):
    cv2.putText(frame, f"Brake: {decision['brake']}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(frame, f"Lane: {decision['lane']}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    cv2.putText(frame, f"Speed: {decision['speed']}", (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Risk: {decision['risk']}", (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    y = 160
    for reason in decision.get("reason", []):
        cv2.putText(frame, f"- {reason}", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y += 25

    return frame
