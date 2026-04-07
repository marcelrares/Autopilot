import json
import time

from config import JSON_OUTPUT_PATH


def write_json(decision, objects):
    payload = {
        "timestamp": time.time(),
        "decision": decision,
        "objects": [
            {
                "id": obj["id"],
                "class": obj["class"],
                "distance": round(obj["distance"], 2),
                "speed": round(obj["speed"], 2),
                "bbox": obj["bbox"],
                "lane_relation": obj.get("lane_relation"),
                "lane_index": obj.get("lane_index"),
                "traffic_light_color": obj.get("traffic_light_color"),
                "lateral_distance_m": obj.get("lateral_distance_m"),
                "distance_metric_m": obj.get("distance_metric_m"),
                "distance_horizon_m": obj.get("distance_horizon_m"),
                "distance_lane_scale_m": obj.get("distance_lane_scale_m"),
                "distance_size_m": obj.get("distance_size_m"),
                "relative_speed_mps": obj.get("relative_speed_mps"),
                "ttc_s": obj.get("ttc_s"),
            }
            for obj in objects
        ]
    }

    with open(JSON_OUTPUT_PATH, "a", encoding="utf-8") as f:
        json.dump(payload, f)
        f.write("\n")
