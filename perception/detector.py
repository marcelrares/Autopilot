import cv2
import numpy as np
import torch
from ultralytics import YOLO

from config import (
    DETECTION_CONFIDENCE,
    MODEL_PATH,
    PERF_DETECTOR_CLASSES,
    PERF_DETECTOR_IMGSZ,
    TRAFFIC_LIGHT_MIN_ACTIVE_PIXELS,
    TRAFFIC_LIGHT_MIN_BOX_SIZE,
    TRAFFIC_LIGHT_MIN_SATURATION,
    TRAFFIC_LIGHT_MIN_VALUE,
    TRAFFIC_LIGHT_SCORE_MARGIN,
    VEHICLE_CLASSES,
)
from models import Detection
from utils.visibility import preprocess_frame_for_detection, preprocess_traffic_light_roi


class ObjectDetector:
    def __init__(self):
        self.model = YOLO(MODEL_PATH)
        self.device_name = "0" if torch.cuda.is_available() else "cpu"
        self.use_half = bool(torch.cuda.is_available())
        self.inference_imgsz = int(PERF_DETECTOR_IMGSZ)
        self.classes = list(PERF_DETECTOR_CLASSES)

        if torch.cuda.is_available():
            torch.backends.cudnn.benchmark = True
            try:
                torch.set_float32_matmul_precision("high")
            except AttributeError:
                pass
            self._warmup()

    def _warmup(self):
        dummy_frame = np.zeros((self.inference_imgsz, self.inference_imgsz, 3), dtype=np.uint8)
        _ = self.model.predict(
            source=dummy_frame,
            verbose=False,
            device=self.device_name,
            half=self.use_half,
            imgsz=self.inference_imgsz,
            classes=self.classes,
            conf=DETECTION_CONFIDENCE,
        )
        torch.cuda.synchronize()

    @staticmethod
    def classify_traffic_light_color(frame, bbox, visibility_context=None):
        x1, y1, x2, y2 = bbox
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return "unknown"

        roi_h, roi_w = roi.shape[:2]
        if roi_h < TRAFFIC_LIGHT_MIN_BOX_SIZE or roi_w < max(4, TRAFFIC_LIGHT_MIN_BOX_SIZE // 2):
            return "unknown"

        roi = preprocess_traffic_light_roi(roi, visibility_context)
        blurred = cv2.GaussianBlur(roi, (3, 3), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        valid_mask = (sat >= TRAFFIC_LIGHT_MIN_SATURATION) & (val >= TRAFFIC_LIGHT_MIN_VALUE)

        red_mask = valid_mask & ((hue <= 10) | (hue >= 170))
        yellow_mask = valid_mask & (hue >= 15) & (hue <= 40)
        green_mask = valid_mask & (hue >= 42) & (hue <= 95)

        top_end = max(1, int(roi_h * 0.42))
        yellow_start = int(roi_h * 0.22)
        yellow_end = max(yellow_start + 1, int(roi_h * 0.78))
        green_start = min(roi_h - 1, int(roi_h * 0.58))

        band_slices = {
            "red": np.s_[:top_end, :],
            "yellow": np.s_[yellow_start:yellow_end, :],
            "green": np.s_[green_start:, :],
        }
        masks = {
            "red": red_mask,
            "yellow": yellow_mask,
            "green": green_mask,
        }

        scores = {}
        dynamic_min_pixels = max(TRAFFIC_LIGHT_MIN_ACTIVE_PIXELS, int(roi_h * roi_w * 0.02))
        for color_name, mask in masks.items():
            total_pixels = int(mask.sum())
            if total_pixels <= 0:
                scores[color_name] = {
                    "total": 0,
                    "expected": 0,
                    "score": 0.0,
                }
                continue

            expected_pixels = int(mask[band_slices[color_name]].sum())
            outside_pixels = total_pixels - expected_pixels
            score = (expected_pixels * 2.4) - (outside_pixels * 0.55)

            scores[color_name] = {
                "total": total_pixels,
                "expected": expected_pixels,
                "score": score,
            }

        ranked = sorted(scores.items(), key=lambda item: item[1]["score"], reverse=True)
        best_name, best_metrics = ranked[0]
        second_score = ranked[1][1]["score"] if len(ranked) > 1 else 0.0

        if best_metrics["total"] < dynamic_min_pixels:
            return "unknown"
        if best_metrics["expected"] < max(3, int(best_metrics["total"] * 0.35)):
            return "unknown"
        if best_metrics["score"] <= 0:
            return "unknown"
        if second_score > 0 and best_metrics["score"] < (second_score * TRAFFIC_LIGHT_SCORE_MARGIN):
            return "unknown"

        return best_name

    def detect(self, frame, visibility_context=None):
        visibility_context = visibility_context or {}
        inference_frame = preprocess_frame_for_detection(frame, visibility_context)
        confidence_threshold = float(visibility_context.get("detection_confidence", DETECTION_CONFIDENCE))
        results = self.model.predict(
            source=inference_frame,
            verbose=False,
            device=self.device_name,
            half=self.use_half,
            imgsz=self.inference_imgsz,
            classes=self.classes,
            conf=confidence_threshold,
        )

        detections = []

        for result in results:
            boxes = result.boxes
            masks = result.masks

            if boxes is None:
                continue

            for i, box in enumerate(boxes):
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls]

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                traffic_light_color = None
                if class_name == "traffic light":
                    traffic_light_color = self.classify_traffic_light_color(
                        frame,
                        (x1, y1, x2, y2),
                        visibility_context=visibility_context,
                    )

                polygon = None
                if masks is not None and class_name in VEHICLE_CLASSES:
                    pts = masks.xy[i]
                    if pts is not None and len(pts) >= 3:
                        polygon = np.array(pts, dtype=np.int32)

                detections.append(
                    Detection(
                        bbox=(x1, y1, x2, y2),
                        confidence=conf,
                        label=class_name,
                        polygon=polygon,
                        traffic_light_color=traffic_light_color,
                    )
                )

        return detections
