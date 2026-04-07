import json
import time

from config import JSON_OUTPUT_PATH


def write_json(decision, objects):
    payload = {
        "timestamp": time.time(),
        "decision": {
            "brake": decision.brake,
            "brake_pct": decision.brake_pct,
            "throttle": decision.throttle,
            "throttle_pct": decision.throttle_pct,
            "lane": decision.lane,
            "speed": decision.speed,
            "risk": decision.risk,
            "reason": list(decision.reason),
            "action_priority": list(decision.action_priority),
            "focus_object_id": decision.focus_object_id,
            "focus_lane_relation": decision.focus_lane_relation,
            "focus_distance_m": decision.focus_distance_m,
            "focus_ttc_s": decision.focus_ttc_s,
            "focus_relative_speed_mps": decision.focus_relative_speed_mps,
            "estimated_speed_kmh": decision.estimated_speed_kmh,
            "speed_target_kmh": decision.speed_target_kmh,
            "traffic_light_state": decision.traffic_light_state,
            "visibility_condition": decision.visibility_condition,
            "visibility_score": decision.visibility_score,
        },
        "objects": [
            {
                "id": obj.id,
                "label": obj.label,
                "distance": round(obj.distance, 2),
                "speed": round(obj.speed, 2),
                "bbox": list(obj.bbox),
                "lane_relation": obj.lane_relation,
                "lane_index": obj.lane_index,
                "traffic_light_color": obj.traffic_light_color,
                "lateral_distance_m": obj.lateral_distance_m,
                "distance_metric_m": obj.distance_metric_m,
                "distance_horizon_m": obj.distance_horizon_m,
                "distance_lane_scale_m": obj.distance_lane_scale_m,
                "distance_size_m": obj.distance_size_m,
                "relative_speed_mps": obj.relative_speed_mps,
                "ttc_s": obj.ttc_s,
            }
            for obj in objects
        ]
    }

    with open(JSON_OUTPUT_PATH, "a", encoding="utf-8") as f:
        json.dump(payload, f)
        f.write("\n")
