import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort

from config import (
    DISTANCE_SMOOTHING_ALPHA,
    FOCAL_LENGTH,
    REAL_HEIGHTS,
    REAL_WIDTHS,
    TRACK_SEGMENT_IOU_THRESHOLD,
    VEHICLE_CLASSES,
)


class ObjectTracker:
    def __init__(self):
        self.tracker = DeepSort(max_age=30)
        self.prev_positions = {}
        self.prev_distances = {}
        self.prev_signal_states = {}

    @staticmethod
    def clamp_bbox(box, frame_w, frame_h):
        l, t, r, b = box

        l = int(np.clip(l, 0, max(0, frame_w - 1)))
        t = int(np.clip(t, 0, max(0, frame_h - 1)))
        r = int(np.clip(r, l + 1, frame_w))
        b = int(np.clip(b, t + 1, frame_h))

        return [l, t, r, b]

    @staticmethod
    def bbox_center(box):
        l, t, r, b = box
        return ((l + r) / 2.0, (t + b) / 2.0)

    @staticmethod
    def estimate_distance(bbox, class_name, focal_length=FOCAL_LENGTH):
        l, t, r, b = bbox
        bbox_h = max(1, b - t)
        bbox_w = max(1, r - l)

        height_estimate = None
        width_estimate = None

        real_height = REAL_HEIGHTS.get(class_name)
        if real_height is not None:
            height_estimate = (real_height * focal_length) / bbox_h

        real_width = REAL_WIDTHS.get(class_name)
        if real_width is not None:
            width_estimate = (real_width * focal_length) / bbox_w

        if height_estimate is None and width_estimate is None:
            fallback_height = REAL_HEIGHTS.get("car", 1.50)
            return float((fallback_height * focal_length) / bbox_h)

        if height_estimate is not None and width_estimate is not None:
            if class_name in VEHICLE_CLASSES:
                return float((0.65 * height_estimate) + (0.35 * width_estimate))
            return float((0.80 * height_estimate) + (0.20 * width_estimate))

        return float(height_estimate if height_estimate is not None else width_estimate)

    @staticmethod
    def estimate_speed(prev_pos, curr_pos, dt=1.0 / 30.0):
        if prev_pos is None:
            return 0.0
        return float(np.linalg.norm(np.array(curr_pos) - np.array(prev_pos)) / dt)

    @staticmethod
    def compute_iou(box_a, box_b):
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
        area_b = max(1, (bx2 - bx1) * (by2 - by1))
        union_area = area_a + area_b - inter_area

        return inter_area / union_area

    def match_segmented_object(self, track_bbox, segmented_objects, preferred_class=None):
        candidates = segmented_objects

        if preferred_class is not None:
            same_class_candidates = [
                seg_obj for seg_obj in segmented_objects
                if seg_obj["class"] == preferred_class
            ]
            if same_class_candidates:
                candidates = same_class_candidates

        best_match = None
        best_iou = 0.0
        best_center_distance = float("inf")
        track_cx, track_cy = self.bbox_center(track_bbox)

        for seg_obj in candidates:
            iou = self.compute_iou(track_bbox, seg_obj["bbox"])
            seg_cx, seg_cy = self.bbox_center(seg_obj["bbox"])
            center_distance = float(np.hypot(track_cx - seg_cx, track_cy - seg_cy))

            if iou > best_iou or (np.isclose(iou, best_iou) and center_distance < best_center_distance):
                best_match = seg_obj
                best_iou = iou
                best_center_distance = center_distance

        if best_match is None:
            return None

        bbox_w = max(1, track_bbox[2] - track_bbox[0])
        bbox_h = max(1, track_bbox[3] - track_bbox[1])
        max_center_distance = 0.5 * np.hypot(bbox_w, bbox_h)

        if best_iou >= TRACK_SEGMENT_IOU_THRESHOLD or best_center_distance <= max_center_distance:
            return best_match

        return None

    def smooth_distance(self, track_id, distance):
        previous_distance = self.prev_distances.get(track_id)
        if previous_distance is None:
            self.prev_distances[track_id] = distance
            return distance

        smoothed_distance = (
            ((1.0 - DISTANCE_SMOOTHING_ALPHA) * previous_distance)
            + (DISTANCE_SMOOTHING_ALPHA * distance)
        )
        self.prev_distances[track_id] = smoothed_distance
        return smoothed_distance

    def update(self, detections, frame, segmented_objects, dt=1.0 / 30.0):
        tracks = self.tracker.update_tracks(detections, frame=frame)
        objects = []
        frame_h, frame_w = frame.shape[:2]
        active_track_ids = set()

        for track in tracks:
            if not track.is_confirmed() or track.time_since_update > 0:
                continue

            track_id = track.track_id
            active_track_ids.add(track_id)
            track_bbox = self.clamp_bbox(list(map(int, track.to_ltrb())), frame_w, frame_h)
            track_class = track.det_class if track.det_class is not None else "object"

            matched_obj = self.match_segmented_object(track_bbox, segmented_objects, track_class)

            if matched_obj is not None:
                output_bbox = self.clamp_bbox(matched_obj["bbox"], frame_w, frame_h)
                matched_class = matched_obj["class"]
                matched_polygon = matched_obj["polygon"]
                traffic_light_color = matched_obj.get("traffic_light_color")
            else:
                output_bbox = track_bbox
                matched_class = track_class
                matched_polygon = None
                traffic_light_color = None

            cx, cy = map(int, self.bbox_center(output_bbox))

            prev = self.prev_positions.get(track_id)
            speed = self.estimate_speed(prev, (cx, cy), dt=dt)
            self.prev_positions[track_id] = (cx, cy)

            distance = self.estimate_distance(output_bbox, matched_class)
            distance = self.smooth_distance(track_id, distance)

            if matched_class == "traffic light":
                if traffic_light_color is None or traffic_light_color == "unknown":
                    traffic_light_color = self.prev_signal_states.get(track_id, "unknown")
                else:
                    self.prev_signal_states[track_id] = traffic_light_color
            else:
                traffic_light_color = None

            objects.append({
                "id": track_id,
                "bbox": output_bbox,
                "class": matched_class,
                "speed": speed,
                "distance": distance,
                "polygon": matched_polygon,
                "traffic_light_color": traffic_light_color,
            })

        stale_track_ids = set(self.prev_positions) - active_track_ids
        for stale_track_id in stale_track_ids:
            self.prev_positions.pop(stale_track_id, None)
            self.prev_distances.pop(stale_track_id, None)
            self.prev_signal_states.pop(stale_track_id, None)

        return objects
