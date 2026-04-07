from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, fields, is_dataclass
from typing import Any

import numpy as np


BBox = tuple[int, int, int, int]


class ModelMixin:
    def _key_aliases(self) -> dict[str, str]:
        return {}

    def _resolve_key(self, key: str) -> str:
        return self._key_aliases().get(key, key)

    def __getitem__(self, key: str) -> Any:
        attr_name = self._resolve_key(key)
        if not hasattr(self, attr_name):
            raise KeyError(key)
        return getattr(self, attr_name)

    def __setitem__(self, key: str, value: Any) -> None:
        setattr(self, self._resolve_key(key), value)

    def get(self, key: str, default: Any = None) -> Any:
        value = getattr(self, self._resolve_key(key), default)
        return default if value is None else value

    def copy(self):
        return deepcopy(self)

    def to_dict(self) -> dict[str, Any]:
        return {
            field_info.name: _serialize_value(getattr(self, field_info.name))
            for field_info in fields(self)
        }


def _serialize_value(value: Any) -> Any:
    if is_dataclass(value):
        return {
            field_info.name: _serialize_value(getattr(value, field_info.name))
            for field_info in fields(value)
        }
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, tuple):
        return [_serialize_value(item) for item in value]
    if isinstance(value, list):
        return [_serialize_value(item) for item in value]
    if isinstance(value, dict):
        return {key: _serialize_value(item) for key, item in value.items()}
    return value


@dataclass
class Detection(ModelMixin):
    bbox: BBox
    confidence: float
    label: str
    polygon: np.ndarray | None = None
    traffic_light_color: str | None = None

    def to_deepsort_input(self) -> tuple[list[int], float, str]:
        x1, y1, x2, y2 = self.bbox
        return [x1, y1, x2 - x1, y2 - y1], self.confidence, self.label


@dataclass
class TrackedObject(ModelMixin):
    id: int | str
    bbox: BBox
    label: str
    speed: float
    distance: float
    polygon: np.ndarray | None = None
    traffic_light_color: str | None = None
    lane_relation: str | None = None
    lane_index: int | None = None
    lane_offset: float | None = None
    lane_overlap: float | None = None
    path_conflict: bool = False
    distance_size_m: float | None = None
    distance_metric_m: float | None = None
    distance_horizon_m: float | None = None
    distance_lane_scale_m: float | None = None
    lateral_distance_m: float | None = None
    lateral_distance_raw_m: float | None = None
    road_center_shift_m: float | None = None
    relative_speed_mps: float | None = None
    ttc_s: float | None = None


@dataclass
class RoadContext(ModelMixin):
    lane_visibility_low: bool = True
    has_left_lane: bool = False
    has_right_lane: bool = False
    lane_line_count: int = 0
    top_y: int = 0
    bottom_y: int = 0
    frame_width: int = 0
    frame_height: int = 0
    horizon_y: float = 0.0
    lane_center_top_x: float = 0.0
    lane_center_bottom_x: float = 0.0
    lane_width_top_px: float = 0.0
    lane_width_bottom_px: float = 0.0
    lane_models: dict[str, np.ndarray] = field(default_factory=dict)
    road_heading_norm: float = 0.0
    ego_lane_center_norm: float = 0.0
    ego_lane_width_norm: float = 0.0
    lane_width_m: float = 0.0
    top_distance_m: float = 0.0
    bottom_distance_m: float = 0.0
    road_centerline_profile: list[dict[str, float]] = field(default_factory=list)
    homography_image_to_ground: np.ndarray | None = None
    visibility_condition: str = "day"
    visibility_score: float = 0.0
    visibility_low: bool = False
    scene_brightness: float = 0.5
    scene_contrast: float = 0.5
    scene_fog_score: float = 0.0
    scene_low_light_score: float = 0.0


@dataclass
class Decision(ModelMixin):
    longitudinal_mode: str = "cruise"
    brake: str = "no_brake"
    brake_pct: int = 0
    throttle: str = "maintain_throttle"
    throttle_pct: int = 62
    lane: str = "keep_lane"
    speed: str = "maintain_speed"
    risk: str = "low"
    reason: list[str] = field(default_factory=list)
    action_priority: list[str] = field(default_factory=list)
    focus_object_id: int | str | None = None
    focus_lane_relation: str | None = None
    focus_distance_m: float | None = None
    focus_ttc_s: float | None = None
    focus_relative_speed_mps: float | None = None
    estimated_speed_kmh: float = 52.0
    speed_target_kmh: float = 52.0
    traffic_light_state: str | None = None
    visibility_condition: str = "day"
    visibility_score: float = 0.0
    speed_cap_kmh: float | None = None
