# Hackathon Autumn 2026 - Dataset Information

## Publish Datasets

We **used** the following Roboflow Universe exports (CC BY 4.0) as sources to build and train, a merged version derivative with a single class. These are one two-source merge inputs; and one remaps their labels to one person-like class.

| Source | Roboflow Universe |
|--------|-------------------|
| [Drone Challenge](https://universe.roboflow.com/mlv-bzm7v/drone-challenge-shhay) | MLV, 2022 — CC BY 4.0 |
| [Mannequins detection](https://universe.roboflow.com/nolly-ai-university-of-salford/mannequins-detection) | Nolly AI / University of Salford, 2023 — CC BY 4.0 |

Merged dataset is our training set (merged derivative, not a third Roboflow project). Cite these Roboflow sources under CC BY 4.0 when redistributing or publishing results based on these dataset.

- First stage, the two Roboflow exports above into one YOLO detection set with two classes (live **human** vs **mannequin**).
- Second stage, collapses all boxes to a **single class** `human`. The model learns **person-like** appearance (humans and mannequins), not fine-grained mannequin vs person discrimination.

## Schema

- **Task:** object detection (axis-aligned boxes)
- **Classes:** `["human"]`.
- **Label format:** YOLO normalized `class cx cy w h` per line

## Scale and splits

- **Train:** 402 images / 1403 boxes
- **Val:** 50 images / 189 boxes
- **Test:** 51 images / 196 boxes
- **Total:** 503 images / 1788 boxes

## How did we build dataset

1. Start from the cleaned v3 two-source merge (deduplicated images, consistent paths in `data.yaml`).
2. Remap every label file having all class ids **0**.
3. Splits **402 / 50 / 51**; no extra synthetic images in the stored dataset.

## Augmentation (training-time, not in files)

We do not conduct augmentation in dataset, the augmentation should happends in Ultralytics dataloader during training steps.

For more information: The shipped model (known as w4e20), training used the standard YOLOv8 detection stack, including from run config contain mosaic, random flip, HSV jitter, RandAugment-style with auto augment, random erasing, translate/scale, and close mosaic in the last epochs.


## Acknowledgements

Training data derives from the Roboflow-hosted projects linked in **Publish datasets** (CC BY 4.0). Cite the dataset name **hackathon-marneqquin-v4** and the Wave-4 paper in `papers/` when publishing results.
