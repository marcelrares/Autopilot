from config import (
    DECISION_FOLLOW_DISTANCE_COAST_ENTER,
    DECISION_FOLLOW_DISTANCE_COAST_EXIT,
    DECISION_FOLLOW_DISTANCE_EMERGENCY_BRAKE_ENTER,
    DECISION_FOLLOW_DISTANCE_EMERGENCY_BRAKE_EXIT,
    DECISION_FOLLOW_DISTANCE_LIGHT_BRAKE_ENTER,
    DECISION_FOLLOW_DISTANCE_LIGHT_BRAKE_EXIT,
    DECISION_FOLLOW_DISTANCE_MAXIMUM_BRAKE_ENTER,
    DECISION_FOLLOW_DISTANCE_MAXIMUM_BRAKE_EXIT,
    DECISION_FOLLOW_DISTANCE_PROGRESSIVE_BRAKE_ENTER,
    DECISION_FOLLOW_DISTANCE_PROGRESSIVE_BRAKE_EXIT,
    DECISION_FOLLOW_DISTANCE_REDUCED_ENTER,
    DECISION_FOLLOW_DISTANCE_REDUCED_EXIT,
    DECISION_FOLLOW_REL_SPEED_COAST_ENTER,
    DECISION_FOLLOW_REL_SPEED_COAST_EXIT,
    DECISION_FOLLOW_REL_SPEED_EMERGENCY_BRAKE_ENTER,
    DECISION_FOLLOW_REL_SPEED_EMERGENCY_BRAKE_EXIT,
    DECISION_FOLLOW_REL_SPEED_LIGHT_BRAKE_ENTER,
    DECISION_FOLLOW_REL_SPEED_LIGHT_BRAKE_EXIT,
    DECISION_FOLLOW_REL_SPEED_MAXIMUM_BRAKE_ENTER,
    DECISION_FOLLOW_REL_SPEED_MAXIMUM_BRAKE_EXIT,
    DECISION_FOLLOW_REL_SPEED_PROGRESSIVE_BRAKE_ENTER,
    DECISION_FOLLOW_REL_SPEED_PROGRESSIVE_BRAKE_EXIT,
    DECISION_FOLLOW_REL_SPEED_REDUCED_ENTER,
    DECISION_FOLLOW_REL_SPEED_REDUCED_EXIT,
    DECISION_FOLLOW_TTC_COAST_ENTER,
    DECISION_FOLLOW_TTC_COAST_EXIT,
    DECISION_FOLLOW_TTC_EMERGENCY_BRAKE_ENTER,
    DECISION_FOLLOW_TTC_EMERGENCY_BRAKE_EXIT,
    DECISION_FOLLOW_TTC_LIGHT_BRAKE_ENTER,
    DECISION_FOLLOW_TTC_LIGHT_BRAKE_EXIT,
    DECISION_FOLLOW_TTC_MAXIMUM_BRAKE_ENTER,
    DECISION_FOLLOW_TTC_MAXIMUM_BRAKE_EXIT,
    DECISION_FOLLOW_TTC_PROGRESSIVE_BRAKE_ENTER,
    DECISION_FOLLOW_TTC_PROGRESSIVE_BRAKE_EXIT,
    DECISION_FOLLOW_TTC_REDUCED_ENTER,
    DECISION_FOLLOW_TTC_REDUCED_EXIT,
    DECISION_VISIBILITY_DIM_SPEED_CAP_KMH,
    DECISION_VISIBILITY_FOG_SPEED_CAP_KMH,
    DECISION_VISIBILITY_LOW_SPEED_CAP_KMH,
    DECISION_VISIBILITY_NIGHT_SPEED_CAP_KMH,
    DECISION_SPEEDOMETER_MAX_KMH,
    DECISION_SPEED_INITIAL_KMH,
    DECISION_SPEED_RESPONSE_DOWN_KMHPS,
    DECISION_SPEED_RESPONSE_UP_KMHPS,
    PERSON_CLASSES,
    VEHICLE_CLASSES,
)


LONGITUDINAL_PRESETS = {
    "cruise": {
        "brake": "no_brake",
        "brake_pct": 0,
        "throttle": "maintain_throttle",
        "throttle_pct": 62,
        "speed": "maintain_speed",
    },
    "reduced_throttle": {
        "brake": "no_brake",
        "brake_pct": 0,
        "throttle": "reduced_throttle",
        "throttle_pct": 38,
        "speed": "slight_deceleration",
    },
    "coast": {
        "brake": "no_brake",
        "brake_pct": 0,
        "throttle": "closed_throttle",
        "throttle_pct": 0,
        "speed": "slight_deceleration",
    },
    "engine_brake": {
        "brake": "engine_brake",
        "brake_pct": 12,
        "throttle": "closed_throttle",
        "throttle_pct": 0,
        "speed": "reduce_speed_for_visibility",
    },
    "light_brake": {
        "brake": "light_brake",
        "brake_pct": 28,
        "throttle": "closed_throttle",
        "throttle_pct": 0,
        "speed": "moderate_deceleration",
    },
    "progressive_brake": {
        "brake": "progressive_brake",
        "brake_pct": 52,
        "throttle": "closed_throttle",
        "throttle_pct": 0,
        "speed": "strong_deceleration",
    },
    "maximum_brake": {
        "brake": "maximum_brake",
        "brake_pct": 82,
        "throttle": "closed_throttle",
        "throttle_pct": 0,
        "speed": "rapid_deceleration",
    },
    "controlled_stop": {
        "brake": "controlled_stop",
        "brake_pct": 68,
        "throttle": "closed_throttle",
        "throttle_pct": 0,
        "speed": "decelerate_to_stop",
    },
    "emergency_brake": {
        "brake": "emergency_brake",
        "brake_pct": 100,
        "throttle": "closed_throttle",
        "throttle_pct": 0,
        "speed": "rapid_deceleration",
    },
}


LONGITUDINAL_ORDER = {
    "cruise": 0,
    "reduced_throttle": 1,
    "coast": 2,
    "engine_brake": 3,
    "light_brake": 4,
    "progressive_brake": 5,
    "controlled_stop": 6,
    "maximum_brake": 7,
    "emergency_brake": 8,
}


LONGITUDINAL_TARGET_SPEED_KMH = {
    "cruise": 62.0,
    "reduced_throttle": 48.0,
    "coast": 38.0,
    "engine_brake": 28.0,
    "light_brake": 18.0,
    "progressive_brake": 10.0,
    "maximum_brake": 3.0,
    "controlled_stop": 0.0,
    "emergency_brake": 0.0,
}


VISIBILITY_SPEED_CAPS = {
    "dim": DECISION_VISIBILITY_DIM_SPEED_CAP_KMH,
    "night": DECISION_VISIBILITY_NIGHT_SPEED_CAP_KMH,
    "fog": DECISION_VISIBILITY_FOG_SPEED_CAP_KMH,
    "low_visibility": DECISION_VISIBILITY_LOW_SPEED_CAP_KMH,
}


FOLLOWING_MODE_ORDER = {
    "cruise": 0,
    "reduced_throttle": 1,
    "coast": 2,
    "light_brake": 3,
    "progressive_brake": 4,
    "maximum_brake": 5,
    "emergency_brake": 6,
}


FOLLOWING_STAGE_CONFIGS = [
    {
        "mode": "reduced_throttle",
        "ttc_enter": DECISION_FOLLOW_TTC_REDUCED_ENTER,
        "ttc_exit": DECISION_FOLLOW_TTC_REDUCED_EXIT,
        "distance_enter": DECISION_FOLLOW_DISTANCE_REDUCED_ENTER,
        "distance_exit": DECISION_FOLLOW_DISTANCE_REDUCED_EXIT,
        "relative_speed_enter": DECISION_FOLLOW_REL_SPEED_REDUCED_ENTER,
        "relative_speed_exit": DECISION_FOLLOW_REL_SPEED_REDUCED_EXIT,
        "risk": "low",
    },
    {
        "mode": "coast",
        "ttc_enter": DECISION_FOLLOW_TTC_COAST_ENTER,
        "ttc_exit": DECISION_FOLLOW_TTC_COAST_EXIT,
        "distance_enter": DECISION_FOLLOW_DISTANCE_COAST_ENTER,
        "distance_exit": DECISION_FOLLOW_DISTANCE_COAST_EXIT,
        "relative_speed_enter": DECISION_FOLLOW_REL_SPEED_COAST_ENTER,
        "relative_speed_exit": DECISION_FOLLOW_REL_SPEED_COAST_EXIT,
        "risk": "medium",
    },
    {
        "mode": "light_brake",
        "ttc_enter": DECISION_FOLLOW_TTC_LIGHT_BRAKE_ENTER,
        "ttc_exit": DECISION_FOLLOW_TTC_LIGHT_BRAKE_EXIT,
        "distance_enter": DECISION_FOLLOW_DISTANCE_LIGHT_BRAKE_ENTER,
        "distance_exit": DECISION_FOLLOW_DISTANCE_LIGHT_BRAKE_EXIT,
        "relative_speed_enter": DECISION_FOLLOW_REL_SPEED_LIGHT_BRAKE_ENTER,
        "relative_speed_exit": DECISION_FOLLOW_REL_SPEED_LIGHT_BRAKE_EXIT,
        "risk": "medium",
    },
    {
        "mode": "progressive_brake",
        "ttc_enter": DECISION_FOLLOW_TTC_PROGRESSIVE_BRAKE_ENTER,
        "ttc_exit": DECISION_FOLLOW_TTC_PROGRESSIVE_BRAKE_EXIT,
        "distance_enter": DECISION_FOLLOW_DISTANCE_PROGRESSIVE_BRAKE_ENTER,
        "distance_exit": DECISION_FOLLOW_DISTANCE_PROGRESSIVE_BRAKE_EXIT,
        "relative_speed_enter": DECISION_FOLLOW_REL_SPEED_PROGRESSIVE_BRAKE_ENTER,
        "relative_speed_exit": DECISION_FOLLOW_REL_SPEED_PROGRESSIVE_BRAKE_EXIT,
        "risk": "high",
    },
    {
        "mode": "maximum_brake",
        "ttc_enter": DECISION_FOLLOW_TTC_MAXIMUM_BRAKE_ENTER,
        "ttc_exit": DECISION_FOLLOW_TTC_MAXIMUM_BRAKE_EXIT,
        "distance_enter": DECISION_FOLLOW_DISTANCE_MAXIMUM_BRAKE_ENTER,
        "distance_exit": DECISION_FOLLOW_DISTANCE_MAXIMUM_BRAKE_EXIT,
        "relative_speed_enter": DECISION_FOLLOW_REL_SPEED_MAXIMUM_BRAKE_ENTER,
        "relative_speed_exit": DECISION_FOLLOW_REL_SPEED_MAXIMUM_BRAKE_EXIT,
        "risk": "high",
    },
    {
        "mode": "emergency_brake",
        "ttc_enter": DECISION_FOLLOW_TTC_EMERGENCY_BRAKE_ENTER,
        "ttc_exit": DECISION_FOLLOW_TTC_EMERGENCY_BRAKE_EXIT,
        "distance_enter": DECISION_FOLLOW_DISTANCE_EMERGENCY_BRAKE_ENTER,
        "distance_exit": DECISION_FOLLOW_DISTANCE_EMERGENCY_BRAKE_EXIT,
        "relative_speed_enter": DECISION_FOLLOW_REL_SPEED_EMERGENCY_BRAKE_ENTER,
        "relative_speed_exit": DECISION_FOLLOW_REL_SPEED_EMERGENCY_BRAKE_EXIT,
        "risk": "high",
    },
]


class DecisionEngine:
    def __init__(self, frame_dt=1.0 / 30.0):
        self.frame_dt = max(1e-3, float(frame_dt))
        self.previous_follow_mode = "cruise"
        self.previous_follow_object_id = None
        self.estimated_speed_kmh = float(DECISION_SPEED_INITIAL_KMH)

    def set_frame_dt(self, frame_dt):
        self.frame_dt = max(1e-3, float(frame_dt))

    def _empty_decision(self):
        return {
            "longitudinal_mode": "cruise",
            "brake": "no_brake",
            "brake_pct": 0,
            "throttle": "maintain_throttle",
            "throttle_pct": 62,
            "lane": "keep_lane",
            "speed": "maintain_speed",
            "risk": "low",
            "reason": [],
            "action_priority": [],
            "focus_object_id": None,
            "focus_lane_relation": None,
            "focus_distance_m": None,
            "focus_ttc_s": None,
            "focus_relative_speed_mps": None,
            "estimated_speed_kmh": round(self.estimated_speed_kmh, 1),
            "speed_target_kmh": round(self.estimated_speed_kmh, 1),
            "traffic_light_state": None,
            "visibility_condition": "day",
            "visibility_score": 0.0,
            "speed_cap_kmh": None,
        }

    @staticmethod
    def _closing_speed_value(relative_speed_mps):
        if relative_speed_mps is None:
            return None
        return float(relative_speed_mps)

    def _select_following_mode(self, lead_vehicle):
        if lead_vehicle is None:
            self.previous_follow_mode = "cruise"
            self.previous_follow_object_id = None
            return None

        lead_id = str(lead_vehicle.get("id"))
        distance = float(lead_vehicle.get("distance", 9999.0))
        ttc_s = lead_vehicle.get("ttc_s")
        relative_speed_mps = self._closing_speed_value(lead_vehicle.get("relative_speed_mps"))
        same_lead_object = lead_id == self.previous_follow_object_id
        previous_mode = self.previous_follow_mode if same_lead_object else "cruise"
        previous_mode_order = FOLLOWING_MODE_ORDER.get(previous_mode, 0)

        selected_mode = "cruise"
        selected_risk = "low"
        selected_source = None

        for idx, config in enumerate(FOLLOWING_STAGE_CONFIGS, start=1):
            use_exit_thresholds = previous_mode_order >= idx
            ttc_threshold = config["ttc_exit"] if use_exit_thresholds else config["ttc_enter"]
            distance_threshold = config["distance_exit"] if use_exit_thresholds else config["distance_enter"]
            relative_speed_threshold = (
                config["relative_speed_exit"] if use_exit_thresholds else config["relative_speed_enter"]
            )

            ttc_match = ttc_s is not None and ttc_s <= ttc_threshold
            distance_match = (
                distance <= distance_threshold
                and (
                    relative_speed_mps is None
                    or relative_speed_mps >= relative_speed_threshold
                )
            )

            if ttc_match or distance_match:
                selected_mode = config["mode"]
                selected_risk = config["risk"]
                selected_source = "ttc" if ttc_match else "distance"

        self.previous_follow_mode = selected_mode
        self.previous_follow_object_id = lead_id

        if selected_mode == "cruise":
            return None

        return {
            "mode": selected_mode,
            "risk": selected_risk,
            "priority": "brake" if LONGITUDINAL_ORDER[selected_mode] >= LONGITUDINAL_ORDER["light_brake"] else "speed",
            "reason": self._following_reason(selected_mode, distance, relative_speed_mps, ttc_s, selected_source),
            "focus_obj": lead_vehicle,
        }

    @staticmethod
    def _following_reason(mode, distance, relative_speed_mps, ttc_s, source):
        mode_text = mode.replace("_", " ")
        if source == "ttc" and ttc_s is not None:
            if relative_speed_mps is not None and relative_speed_mps > 0.0:
                return f"Lead vehicle {mode_text}: TTC {ttc_s:.1f}s, closing {relative_speed_mps:.1f}m/s"
            return f"Lead vehicle {mode_text}: TTC {ttc_s:.1f}s"

        if relative_speed_mps is None:
            return f"Lead vehicle {mode_text}: gap {distance:.1f}m"

        if relative_speed_mps > 0.05:
            return f"Lead vehicle {mode_text}: gap {distance:.1f}m, closing {relative_speed_mps:.1f}m/s"

        if relative_speed_mps < -0.25:
            return f"Lead vehicle {mode_text}: gap {distance:.1f}m, opening"

        return f"Lead vehicle {mode_text}: gap {distance:.1f}m, stable closing"

    def _update_estimated_speed(self, decision):
        mode = decision["longitudinal_mode"]
        target_speed_kmh = float(LONGITUDINAL_TARGET_SPEED_KMH.get(mode, DECISION_SPEED_INITIAL_KMH))
        speed_cap_kmh = decision.get("speed_cap_kmh")
        if speed_cap_kmh is not None:
            target_speed_kmh = min(target_speed_kmh, float(speed_cap_kmh))
        current_speed_kmh = float(self.estimated_speed_kmh)
        delta_kmh = target_speed_kmh - current_speed_kmh
        response_limit = (
            DECISION_SPEED_RESPONSE_UP_KMHPS
            if delta_kmh >= 0.0 else DECISION_SPEED_RESPONSE_DOWN_KMHPS
        ) * self.frame_dt
        if abs(delta_kmh) <= response_limit:
            updated_speed_kmh = target_speed_kmh
        else:
            updated_speed_kmh = current_speed_kmh + (response_limit if delta_kmh > 0.0 else -response_limit)

        updated_speed_kmh = float(max(0.0, min(DECISION_SPEEDOMETER_MAX_KMH, updated_speed_kmh)))
        self.estimated_speed_kmh = updated_speed_kmh
        decision["estimated_speed_kmh"] = round(updated_speed_kmh, 1)
        decision["speed_target_kmh"] = round(target_speed_kmh, 1)

    def _apply_visibility_policy(self, decision, road_context):
        visibility_condition = road_context.get("visibility_condition", "day")
        visibility_score = float(road_context.get("visibility_score", 0.0) or 0.0)
        lane_visibility_low = road_context.get("lane_visibility_low", True)
        speed_cap_kmh = VISIBILITY_SPEED_CAPS.get(visibility_condition)

        decision["visibility_condition"] = visibility_condition
        decision["visibility_score"] = round(visibility_score, 2)
        decision["speed_cap_kmh"] = speed_cap_kmh

        if visibility_condition == "day":
            return

        if visibility_condition == "dim":
            consider_mode(
                decision,
                "reduced_throttle",
                risk="low",
                reason="Dim ambient light, moderating cruise speed",
                priority="speed",
            )
            return

        if visibility_condition == "night":
            consider_mode(
                decision,
                "reduced_throttle",
                risk="medium" if lane_visibility_low else "low",
                reason="Night driving: increasing safety margin",
                priority="speed",
            )
            if lane_visibility_low:
                consider_mode(
                    decision,
                    "coast",
                    risk="medium",
                    reason="Night lane model uncertain",
                    priority="lane",
                )
            return

        if visibility_condition == "fog":
            consider_mode(
                decision,
                "reduced_throttle",
                risk="medium",
                reason="Fog detected: reducing speed for visibility",
                priority="speed",
            )
            if visibility_score >= 0.68 or lane_visibility_low:
                consider_mode(
                    decision,
                    "coast",
                    risk="medium",
                    reason="Fog density increasing, preparing for limited sight distance",
                    priority="speed",
                )
            return

        consider_mode(
            decision,
            "engine_brake",
            risk="high" if visibility_score >= 0.72 else "medium",
            reason="Severely reduced visibility ahead",
            priority="speed",
        )

    def make_decision(self, objects, road_context):
        decision = self._empty_decision()

        lane_visibility_low = road_context.get("lane_visibility_low", True)
        self._apply_visibility_policy(decision, road_context)
        same_lane_vehicles = sorted(
            [
                obj for obj in objects
                if obj.get("class") in VEHICLE_CLASSES and obj.get("lane_index") == 0
            ],
            key=lambda obj: obj.get("distance", 9999),
        )
        same_path_vulnerable = sorted(
            [
                obj for obj in objects
                if obj.get("class") in PERSON_CLASSES and obj.get("path_conflict", False)
            ],
            key=lambda obj: obj.get("distance", 9999),
        )
        adjacent_vehicles = sorted(
            [
                obj for obj in objects
                if obj.get("class") in VEHICLE_CLASSES and obj.get("lane_index") != 0
            ],
            key=lambda obj: obj.get("distance", 9999),
        )
        close_objects_count = sum(1 for obj in objects if obj.get("distance", 9999) <= 25)
        stop_sign_detected = any(obj.get("class") == "stop sign" for obj in objects)
        traffic_lights = sorted(
            [obj for obj in objects if obj.get("class") == "traffic light"],
            key=lambda obj: obj.get("distance", 9999),
        )

        lead_vulnerable = same_path_vulnerable[0] if same_path_vulnerable else None
        lead_vehicle = same_lane_vehicles[0] if same_lane_vehicles else None
        nearest_adjacent_vehicle = adjacent_vehicles[0] if adjacent_vehicles else None
        primary_traffic_light = traffic_lights[0] if traffic_lights else None

        if lead_vulnerable is not None:
            distance = lead_vulnerable.get("distance", 9999)
            if distance <= 7.0:
                consider_mode(
                    decision,
                    "emergency_brake",
                    risk="high",
                    reason=f"Vulnerable road user in path at {distance:.1f}m",
                    priority="brake",
                    focus_obj=lead_vulnerable,
                )
            elif distance <= 10.0:
                consider_mode(
                    decision,
                    "maximum_brake",
                    risk="high",
                    reason=f"Pedestrian or cyclist ahead at {distance:.1f}m",
                    priority="brake",
                    focus_obj=lead_vulnerable,
                )
            elif distance <= 15.0:
                consider_mode(
                    decision,
                    "progressive_brake",
                    risk="high",
                    reason=f"Pedestrian or cyclist entering ego lane at {distance:.1f}m",
                    priority="brake",
                    focus_obj=lead_vulnerable,
                )
            elif distance <= 22.0:
                consider_mode(
                    decision,
                    "engine_brake",
                    risk="medium",
                    reason=f"Pedestrian or cyclist near ego lane at {distance:.1f}m",
                    priority="speed",
                    focus_obj=lead_vulnerable,
                )

        follow_plan = self._select_following_mode(lead_vehicle)
        if follow_plan is not None:
            consider_mode(
                decision,
                follow_plan["mode"],
                risk=follow_plan["risk"],
                reason=follow_plan["reason"],
                priority=follow_plan["priority"],
                focus_obj=follow_plan["focus_obj"],
            )

        if stop_sign_detected:
            consider_mode(
                decision,
                "controlled_stop",
                risk="medium",
                reason="Stop sign detected ahead",
                priority="brake",
            )

        if primary_traffic_light is not None:
            signal_state = primary_traffic_light.get("traffic_light_color", "unknown")
            signal_distance = primary_traffic_light.get("distance", 9999)
            decision["traffic_light_state"] = signal_state

            if signal_state == "red" and signal_distance <= 42.0:
                consider_mode(
                    decision,
                    "controlled_stop",
                    risk="medium",
                    reason=f"Red traffic light at {signal_distance:.1f}m",
                    priority="brake",
                    focus_obj=primary_traffic_light,
                )
            elif signal_state == "yellow":
                if signal_distance <= 20.0:
                    consider_mode(
                        decision,
                        "controlled_stop",
                        risk="medium",
                        reason=f"Yellow traffic light close ahead at {signal_distance:.1f}m",
                        priority="brake",
                        focus_obj=primary_traffic_light,
                    )
                elif signal_distance <= 34.0:
                    consider_mode(
                        decision,
                        "light_brake",
                        risk="medium",
                        reason=f"Yellow traffic light detected at {signal_distance:.1f}m",
                        priority="brake",
                        focus_obj=primary_traffic_light,
                    )
            elif signal_state == "unknown" and signal_distance <= 20.0:
                consider_mode(
                    decision,
                    "reduced_throttle",
                    risk="low",
                    reason=f"Traffic light visible but color uncertain at {signal_distance:.1f}m",
                    priority="speed",
                    focus_obj=primary_traffic_light,
                )

        if nearest_adjacent_vehicle is not None:
            distance = nearest_adjacent_vehicle.get("distance", 9999)
            lane_relation = nearest_adjacent_vehicle.get("lane_relation", "adjacent")

            if distance <= 4.0:
                consider_mode(
                    decision,
                    "coast",
                    risk="medium",
                    reason=f"Vehicle very close in {lane_relation.replace('_', ' ')} at {distance:.1f}m",
                    priority="lane",
                    focus_obj=nearest_adjacent_vehicle,
                )
            elif distance <= 8.0:
                consider_mode(
                    decision,
                    "reduced_throttle",
                    risk="medium",
                    reason=f"Vehicle nearby in {lane_relation.replace('_', ' ')} at {distance:.1f}m",
                    priority="lane",
                    focus_obj=nearest_adjacent_vehicle,
                )
            elif distance <= 15.0:
                consider_mode(
                    decision,
                    "reduced_throttle",
                    risk="low",
                    reason=f"Traffic close on adjacent lane at {distance:.1f}m",
                    priority="lane",
                    focus_obj=nearest_adjacent_vehicle,
                )

        if lane_visibility_low:
            decision["lane"] = "keep_lane_with_caution"
            consider_mode(
                decision,
                "reduced_throttle",
                risk="medium",
                reason="Lane boundaries uncertain",
                priority="lane",
            )

        if close_objects_count >= 4:
            consider_mode(
                decision,
                "reduced_throttle",
                risk="medium",
                reason="Dense nearby traffic detected",
                priority="speed",
            )

        if not decision["reason"]:
            decision["reason"].append("Road situation stable")
            decision["action_priority"].append("maintain")

        self._update_estimated_speed(decision)
        decision.pop("longitudinal_mode", None)
        decision.pop("speed_cap_kmh", None)
        return decision


def consider_mode(decision, mode, risk=None, reason=None, priority=None, focus_obj=None):
    current_order = LONGITUDINAL_ORDER[decision["longitudinal_mode"]]
    new_order = LONGITUDINAL_ORDER[mode]

    if new_order > current_order:
        decision["longitudinal_mode"] = mode
        decision.update(LONGITUDINAL_PRESETS[mode])

    if risk is not None:
        decision["risk"] = max_risk(decision["risk"], risk)

    if reason and reason not in decision["reason"]:
        decision["reason"].append(reason)

    if priority and priority not in decision["action_priority"]:
        decision["action_priority"].append(priority)

    if focus_obj is not None and (new_order > current_order or decision["focus_object_id"] is None):
        decision["focus_object_id"] = focus_obj.get("id")
        decision["focus_lane_relation"] = focus_obj.get("lane_relation")
        decision["focus_distance_m"] = round(focus_obj.get("distance", 0.0), 2)
        decision["focus_ttc_s"] = focus_obj.get("ttc_s")
        decision["focus_relative_speed_mps"] = focus_obj.get("relative_speed_mps")


def max_risk(current, new):
    order = {"low": 0, "medium": 1, "high": 2}
    return new if order[new] > order[current] else current


_DEFAULT_ENGINE = DecisionEngine()


def make_decision(objects, road_context, engine=None):
    active_engine = _DEFAULT_ENGINE if engine is None else engine
    return active_engine.make_decision(objects, road_context)
