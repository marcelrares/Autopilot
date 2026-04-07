# Autopilot

Computer-vision driving-assist prototype built around object detection, tracking, lane understanding, scene context estimation, and rule-based longitudinal decisions.

## Repository layout

- `main.py`: application entry point
- `config.py`: runtime configuration and thresholds
- `perception/`: detection, lane extraction, and tracking
- `utils/`: visibility estimation and road-context math
- `decision/`: rule-based driving decision engine
- `output/`: 2D overlays, bird's-eye dashboard, and JSON logging
- `rendering/`: 3D scene renderer


## Local-only files

The current code expects these local assets by default:

- `VIDEO_PATH = "1.mp4"`
- `MODEL_PATH = "yolov8n-seg.pt"`

## Setup

Windows PowerShell:

```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
python main.py
```