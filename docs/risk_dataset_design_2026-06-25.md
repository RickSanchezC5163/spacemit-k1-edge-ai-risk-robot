# Risk Dataset Design - 2026-06-25

Goal: define the first dataset structure for constrained-space inspection risk
events.

## Classes

- `soft_obstacle`
- `hard_obstacle`
- `blocked_path`
- `low_light`
- `cable_or_wire`
- `reflective_noise`

## Directory Structure

```text
datasets/risk_events_v0/
  images/
    train/
    val/
    test/
  labels_yolo/
    train/
    val/
    test/
  events/
    train/
    val/
    test/
  execution_results/
  README.md
```

## YOLO Labels

Use normalized YOLO boxes:

```text
class_id x_center y_center width height
```

Class mapping should match `configs/risk_classes.yaml`.

## Risk Event JSON

```json
{
  "event_id": "risk_000001",
  "timestamp": "2026-06-25T15:00:00+08:00",
  "event_type": "soft_obstacle",
  "distance_m": 0.8,
  "confidence": 0.9,
  "source": "camera_lidar_fusion",
  "image_path": "images/train/risk_000001.jpg",
  "label_path": "labels_yolo/train/risk_000001.txt"
}
```

## Execution Result JSON

```json
{
  "event_id": "risk_000001",
  "risk_level": "medium",
  "recommended_action": "stop_and_recheck",
  "actual_result": "manual_confirmed",
  "notes": "No arm action executed in this stage."
}
```

## Split Plan

- train: 70 percent
- val: 20 percent
- test: 10 percent

## First Collection Target

Collect 50-100 examples per class in the first phase:

- different lighting
- different distances
- partial occlusion
- sensor noise
- safe lab or dorm setup only

## Safety Rules

- Do not use real hazards.
- Do not use real water near power.
- Use film, tape, boxes, cloth, and safe reflective materials for simulation.
