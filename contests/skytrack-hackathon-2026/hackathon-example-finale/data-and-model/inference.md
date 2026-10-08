# ONNX inference - FP32 Only

These graphs were exported from our best training model with highest validation mAP50-95 via Ultralytics.
```python
.(format='onnx', nms=False, haft=False, opset=17, simplify=True)
```

Supported files:
- `w4e20_640_fp32.onnx` — input **640×640**, anchors **N = 8400**
- `w4e20_320_fp32.onnx` — input **320×320**, anchors **N = 2100**

## Input and Output

| Tensor | Name | Shape | Dtype |
|--------|------|-------|-------|
| Input | `images` | `[1, 3, H, H]` | float32 |
| Output | `output0` | `[1, 5, N]` | float32 |

- **H** is 640 or 320 depending on the file.
- **N** is the number of detection anchors (8400 at 640, 2100 at 320).
- **Class:** single head channel; index **0 = `human`** (person-like: real humans and mannequins share this label in training).

Output layout matches Ultralytics v8 detect export: for each anchor, the five channels are **center-x, center-y, width, height, objectness/class score** in **letterboxed input pixel space**.

## Preprocessing

1. Read image as RGB (OpenCV loads BGR — convert before normalization).
2. **Letterbox** to H×H: preserve aspect ratio, scale to fit, pad with gray **(114, 114, 114)** (RGB).
3. Scale pixel values to **`[0, 1]`** (`float32`).
4. Layout **NCHW**: `[1, 3, H, H]`.

Training used `imgsz=640`; the 320 ONNX is for lighter deployment — use the matching H in preprocessing.

## Post-processing

The ONNX file does **not** apply:

- confidence threshold,
- IoU-based **NMS**,
- coordinate transform back to original image size.

Typical player pipeline:

1. Filter anchors by score (e.g. `conf > 0.25`).
2. Convert xywh from letterboxed space to original image (undo pad and divide by letterbox ratio).
3. Run NMS (OpenCV, torchvision, or custom).
4. Draw boxes; class id is always **0** (`human`).

See `run_onnx_example.py` for a minimal ORT run and commented decode hints.

## Quick run

```bash
pip install onnxruntime opencv-python numpy
python run_onnx_example.py --onnx w4e20_640_fp32.onnx --image /path/to/image.jpg
```

Use `--imgsz 320` with the 320 ONNX if the graph input size is not inferred from the session.
