import cv2
import os
import time
import threading

from config import (
    BIRDVIEW_WINDOW_HEIGHT,
    BIRDVIEW_WINDOW_WIDTH,
    DISPLAY_WINDOW_HEIGHT,
    DISPLAY_WINDOW_WIDTH,
    JSON_INTERVAL,
    PERF_LANE_INTERVAL,
    PERF_VISIBILITY_INTERVAL,
    VIDEO_PATH,
)
from models import Decision, RoadContext
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


current_decision = Decision(reason=["System initialized"])


def get_decision():
    return current_decision


current_scene = {
    "decision": current_decision.copy(),
    "objects": [],
    "road_context": RoadContext(),
    "version": 0,
}


def get_scene():
    return current_scene


def main():
    global current_decision, current_scene

    cv2.setUseOptimized(True)
    cv2.setNumThreads(max(1, int(os.cpu_count() or 1)))

    cap = cv2.VideoCapture(VIDEO_PATH)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_dt = 1.0 / fps if fps and fps > 1 else 1.0 / 30.0
    detector = ObjectDetector()
    tracker = ObjectTracker()
    decision_engine = DecisionEngine(frame_dt=frame_dt)
    birdview_renderer = BirdViewRenderer()
    road_estimator = RoadContextEstimator(frame_dt=frame_dt)
    visibility_estimator = VisibilityConditionEstimator()
    last_json_time = time.time()
    cached_visibility_context = None
    cached_lanes = []
    cached_roi_polygon = None
    frame_index = 0
    scene_version = 0
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

        if cached_visibility_context is None or (frame_index % PERF_VISIBILITY_INTERVAL) == 0:
            cached_visibility_context = visibility_estimator.analyze(frame)
        visibility_context = cached_visibility_context

        detections = detector.detect(frame, visibility_context=visibility_context)
        objects = tracker.update(detections, frame, dt=frame_dt)

        if cached_roi_polygon is None or (frame_index % PERF_LANE_INTERVAL) == 0:
            cached_lanes, cached_roi_polygon = detect_lanes(frame, visibility_context=visibility_context)

        lanes = cached_lanes
        roi_polygon = cached_roi_polygon

        objects, road_context = road_estimator.annotate(
            objects,
            lanes,
            frame.shape,
            visibility_context=visibility_context,
        )

        current_decision = decision_engine.make_decision(objects, road_context)
        current_scene = {
            "decision": current_decision.copy(),
            "objects": [obj.copy() for obj in objects],
            "road_context": road_context.copy(),
            "version": scene_version,
        }
        scene_version += 1

        if time.time() - last_json_time >= JSON_INTERVAL:
            write_json(current_decision, objects)
            last_json_time = time.time()

        display_frame = draw_objects(frame, objects)
        display_frame = draw_lanes(display_frame, lanes, roi_polygon)
        birdview_frame = birdview_renderer.render(objects, road_context, current_decision)

        cv2.imshow(window_name, display_frame)
        cv2.imshow(birdview_window_name, birdview_frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

        frame_index += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
