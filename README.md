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

## What belongs in Git

Commit these:

- Python source files
- package metadata such as `requirements.txt`
- documentation such as this `README.md`
- small config files and scripts needed to run the project

Do not commit these:

- virtual environments such as `venv/`
- generated caches such as `__pycache__/` and `*.pyc`
- runtime logs and generated outputs such as `output.json`
- large local assets such as model weights and input videos

## Local-only files

The current code expects these local assets by default:

- `VIDEO_PATH = "1.mp4"`
- `MODEL_PATH = "yolov8n-seg.pt"`

They are intentionally ignored by Git. Keep them on your machine, or change the paths in `config.py` to point to your local files.

## Setup

Windows PowerShell:

```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
python main.py
```

## Notes for first-time Git use

- Git should track code and documentation.
- Git should usually not track generated files, downloaded models, datasets, videos, or local environments.
- If a file is big and you can recreate or download it again, it usually should not live in normal Git history.
- If you ever need to version a large binary on purpose, use Git LFS instead of standard Git.

