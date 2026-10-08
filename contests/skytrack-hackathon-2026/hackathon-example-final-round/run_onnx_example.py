#!/usr/bin/env python3
"""Minimal ONNX Runtime example for w4e20 (FP32)."""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


def letterbox(im: np.ndarray, new_shape: int = 640, color: tuple[int, int, int] = (114, 114, 114)):
    h, w = im.shape[:2]
    r = min(new_shape / h, new_shape / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    im_resized = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = new_shape - nw, new_shape - nh
    top, bottom = pad_h // 2, pad_h - pad_h // 2
    left, right = pad_w // 2, pad_w - pad_w // 2
    out = cv2.copyMakeBorder(im_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    ratio = r
    pad = (left, top)
    return out, ratio, pad


def preprocess(path: Path, imgsz: int) -> tuple[np.ndarray, np.ndarray, float, tuple[int, int]]:
    bgr = cv2.imread(str(path))
    if bgr is None:
        raise SystemExit(f"Could not read image: {path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    lb, ratio, pad = letterbox(rgb, imgsz)
    x = lb.astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))[None, ...]  # NCHW
    return x, bgr, ratio, pad


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--onnx",
        type=Path,
        default=Path(__file__).with_name("w4e20_640_fp32.onnx"),
        help="Path to w4e20_*_fp32.onnx",
    )
    parser.add_argument("--image", type=Path, required=True, help="Input image path")
    parser.add_argument("--imgsz", type=int, default=None, help="Override square input size (640 or 320)")
    args = parser.parse_args()

    sess = ort.InferenceSession(str(args.onnx), providers=["CPUExecutionProvider"])
    inp_name = sess.get_inputs()[0].name
    out_name = sess.get_outputs()[0].name
    shape = sess.get_inputs()[0].shape
    imgsz = args.imgsz or int(shape[2])
    print(f"session input={inp_name} shape={shape} output={out_name}")

    x, _bgr, ratio, pad = preprocess(args.image, imgsz)
    out = sess.run([out_name], {inp_name: x})[0]
    print(f"raw output shape: {out.shape}  (layout [1, 5, N] = cx,cy,w,h,class0 score per anchor)")

    # Optional post-processing (not run by default):
    # preds = np.transpose(out[0], (1, 0))  # [N, 5]
    # conf = preds[:, 4]
    # keep = conf > 0.25
    # boxes = preds[keep, :4]  # xywh in letterboxed space — scale back with ratio/pad, then NMS

    print(f"letterbox ratio={ratio:.4f} pad={pad}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
