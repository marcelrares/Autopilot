import numpy as np

def estimate_distance(bbox_height, real_height=1.5, focal_length=700):
    if bbox_height <= 0:
        return 999.0
    return (real_height * focal_length) / bbox_height

def estimate_speed(prev, curr, dt=0.5):
    if prev is None:
        return 0.0
    return float(np.linalg.norm(np.array(curr) - np.array(prev)) / dt)

def bbox_iou(box_a, box_b):
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