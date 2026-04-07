import cv2
import time
import threading

from config import (
    BIRDVIEW_WINDOW_HEIGHT,
    BIRDVIEW_WINDOW_WIDTH,
    DISPLAY_WINDOW_HEIGHT,
    DISPLAY_WINDOW_WIDTH,
    JSON_INTERVAL,
    VIDEO_PATH,
)
from perception.detector import ObjectDetector
from perception.lane_detection import detect_lanes
from perception.tracker import ObjectTracker
from decision.decision_engine import DecisionEngine
from output.birdview import BirdViewRenderer
from output.json_logger import write_json
from output.visualizer import draw_objects, draw_lanes
from rendering.renderer3d import run_3d
from utils.road_context import RoadContextEstimator
from utils.visibility import VisibilityConditionEstimator


current_decision = {
    "brake": "no_brake",
    "brake_pct": 0,
    "throttle": "maintain_throttle",
    "throttle_pct": 62,
    "lane": "keep_lane",
    "speed": "maintain_speed",
    "risk": "low",
    "reason": ["System initialized"],
    "focus_object_id": None,
    "focus_lane_relation": None,
    "focus_distance_m": None,
    "focus_ttc_s": None,
    "focus_relative_speed_mps": None,
    "estimated_speed_kmh": 52.0,
    "speed_target_kmh": 52.0,
    "traffic_light_state": None,
    "visibility_condition": "day",
    "visibility_score": 0.0,
}


def get_decision():
    return current_decision


current_scene = {
    "decision": dict(current_decision),
    "objects": [],
    "road_context": {
        "lane_visibility_low": True,
        "has_left_lane": False,
        "has_right_lane": False,
        "visibility_condition": "day",
        "visibility_score": 0.0,
    },
}


def get_scene():
    return current_scene


def main():
    global current_decision, current_scene

    cap = cv2.VideoCapture(VIDEO_PATH)
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_dt = 1.0 / fps if fps and fps > 1 else 1.0 / 30.0
    detector = ObjectDetector()
    tracker = ObjectTracker()
    decision_engine = DecisionEngine(frame_dt=frame_dt)
    birdview_renderer = BirdViewRenderer()
    road_estimator = RoadContextEstimator(frame_dt=frame_dt)
    visibility_estimator = VisibilityConditionEstimator()
    last_json_time = time.time()
    window_name = "2D Drive Assist"
    birdview_window_name = "Autopilot Dashboard"

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, DISPLAY_WINDOW_WIDTH, DISPLAY_WINDOW_HEIGHT)
    cv2.namedWindow(birdview_window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(birdview_window_name, BIRDVIEW_WINDOW_WIDTH, BIRDVIEW_WINDOW_HEIGHT)

    threading.Thread(target=run_3d, args=(get_scene,), daemon=True).start()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        visibility_context = visibility_estimator.analyze(frame)
        detections, segmented_objects = detector.detect(frame, visibility_context=visibility_context)
        objects = tracker.update(detections, frame, segmented_objects, dt=frame_dt)
        lanes, roi_polygon = detect_lanes(frame, visibility_context=visibility_context)
        objects, road_context = road_estimator.annotate(
            objects,
            lanes,
            frame.shape,
            visibility_context=visibility_context,
        )

        current_decision = decision_engine.make_decision(objects, road_context)
        current_scene = {
            "decision": dict(current_decision),
            "objects": [dict(obj) for obj in objects],
            "road_context": dict(road_context),
        }

        if time.time() - last_json_time >= JSON_INTERVAL:
            write_json(current_decision, objects)
            last_json_time = time.time()

        frame = draw_objects(frame, objects)
        frame = draw_lanes(frame, lanes, roi_polygon)
        birdview_frame = birdview_renderer.render(objects, road_context, current_decision)

        cv2.imshow(window_name, frame)
        cv2.imshow(birdview_window_name, birdview_frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
