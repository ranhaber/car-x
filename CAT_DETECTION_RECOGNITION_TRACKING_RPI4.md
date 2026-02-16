# Cat Detection, Recognition, and Tracking on Raspberry Pi 4 (4 GB)

This document explains your options for **cat detection**, **cat recognition**, and **tracking a running cat** when using a **Raspberry Pi 4 Model B with 4 GB RAM** (e.g. on PiCar-X with the OV5647 camera and vilib/picar-x stack).

---

## Definitions

- **Detection** — “Where is a cat in this image?” → one or more **bounding boxes** (and usually a class label “cat”). No identity across time.
- **Recognition** — “Is this a cat?” (classification) or “Which cat is this?” (identity). Often a single label per image or per crop.
- **Tracking** — “Where is *the same* cat in the next frame?” → maintain **identity and position** across frames (e.g. one bounding box or centroid that follows the same cat over time).

---

# 1. Cat detection options (find where the cat is)

**Goal:** For each frame (or image), get bounding box(es) and a “cat” label.

## 1.1 Use existing vilib object detection (COCO, class “cat”)

**What you have:** In your car-x repo, **vilib** already has TFLite object detection with **COCO labels**. In `vilib/workspace/coco_labels.txt`, **class id 16 is “cat”**.

- **API:** `Vilib.object_detect_switch(True)`, optionally `Vilib.object_detect_set_model(path)` and `Vilib.object_detect_set_labels(path)`. Default model is `/opt/vilib/detect.tflite`, default labels are COCO.
- **Output:** List of detections; each has `class_id`, `bounding_box`, `score`, and `class_name`. Filter where `class_name == "cat"` (or `class_id == 16`).
- **Pros:** No new model or code; works with your existing pipeline; COCO includes “cat”.
- **Cons:** Model is generic (80 COCO classes); input size and model type depend on what’s in `/opt/vilib/` (often 300×300 or similar). On Pi 4, TFLite object detection is typically **~3–10 FPS** depending on resolution and model size. Not tuned specifically for cats.

**Best for:** Quick start, “any cat” in the scene.

---

## 1.2 MediaPipe object detection (EfficientDet-Lite) in vilib

**What you have:** `vilib/vilib/mediapipe_object_detection.py` uses a TFLite object detector (e.g. EfficientDet-Lite). You’d need to use a **model/labels that include “cat”** (e.g. COCO-based).

- **Pros:** Modern pipeline; can use EfficientDet-Lite0 (reasonably fast on Pi 4).
- **Cons:** Same as above: COCO-based, so “cat” is there if the model is trained on COCO; FPS still limited by inference (~5–15 FPS range on Pi 4 depending on input size and model).

**Best for:** If you prefer the MediaPipe API or a different TFLite detector; still “cat” as one of many classes.

---

## 1.3 Custom or optimized TFLite model (cat-focused)

**Idea:** Use a **small TFLite object detection model** that either (a) is trained only on “cat” (and maybe “dog”/background) or (b) is a lighter COCO model (e.g. SSD MobileNet, EfficientDet-Lite) to improve FPS.

- **Options:**
  - **TensorFlow Model Maker / custom dataset:** Train a detector on cat images, export TFLite. Then use the same `detect_objects()`-style pipeline (resize → interpreter.invoke() → parse boxes). Single-class “cat” can be smaller and faster.
  - **EfficientDet-Lite 0/1 (COCO):** Official TFLite models; run on Pi 4 at reduced resolution (e.g. 320×320) for higher FPS; COCO class 16 = cat.
  - **YOLO (e.g. YOLOv8/v11 nano) → TFLite or ONNX:** Convert to TFLite or use a lightweight runtime; run at small input (e.g. 320×320 or 256×256) for real-time. Cat is in COCO. Pi 4 4 GB can run small YOLO at ~5–15 FPS depending on size and backend.

- **Pros:** Better FPS and/or accuracy for “cat” if you train or pick a smaller model.
- **Cons:** Requires model conversion and/or training; integration into vilib (or a separate script) is on you.

**Best for:** When you need higher FPS or better cat-specific accuracy.

---

## 1.4 Coral USB Accelerator (Edge TPU) + TFLite

**Idea:** Use a **Coral USB Edge TPU** with a TFLite model compiled for Edge TPU. Inference runs on the accelerator, not the Pi’s CPU, so FPS can be **much higher** (often 20+ FPS for small detectors).

- **Pros:** Big FPS gain; still “detection” (boxes + classes); COCO models (with “cat”) are available for Edge TPU.
- **Cons:** Extra hardware cost; you must use TPU-compatible TFLite models and the Edge TPU runtime.

**Best for:** When you need high-FPS detection and are willing to add the Coral dongle.

---

## 1.5 Summary: detection on Pi 4 (4 GB)

| Option | FPS (approx.) | Effort | Notes |
|--------|----------------|--------|--------|
| vilib COCO TFLite (default) | ~3–10 | None | Filter class 16 = cat |
| MediaPipe EfficientDet (COCO) | ~5–15 | Low | Use COCO labels for cat |
| Custom/small TFLite detector | ~8–20 | Medium–High | Train or pick smaller model |
| YOLO nano → TFLite/small input | ~5–15 | Medium | Convert + integrate |
| Coral Edge TPU + TFLite | 20+ | Medium | Need Coral hardware |

**Recommendation:** Start with **vilib object detection + COCO labels**, filter for `class_id == 16` (“cat”). If FPS is too low, reduce camera resolution (e.g. 320×320 or 480×360 for the model input) or switch to a smaller TFLite/Edge TPU model.

---

# 2. Cat recognition options (is it a cat? which cat?)

**Goal:** Classify an image (or crop) as “cat” vs not, or identify “which cat” (e.g. my cat vs others).

## 2.1 Use detection output as “recognition”

**Idea:** Run object detection; if any detection has `class_name == "cat"` and score above a threshold, you “recognize” that there is a cat (and where). No separate recognition model.

- **Pros:** Same pipeline as §1; no extra model.
- **Cons:** “Recognition” is just “detection with class cat”; no notion of “which cat” (re-identification).

**Best for:** “Is there a cat in the frame?” and “where is it?”.

---

## 2.2 Image classification (whole frame or crop): “cat” vs not

**What you have:** vilib **image classification** with TFLite (e.g. MobileNet) and labels. Your repo has `labels_mobilenet_quant_v1_224.txt` with entries like “tiger cat”, “Persian cat”, “Siamese cat”, “Egyptian cat”.

- **Use case:** Crop the image to the cat region (e.g. from a detector or a fixed ROI), then run the classifier. If the top class is one of the cat breeds (or you add a single “cat” class), you have “this crop is a cat”.
- **Pros:** Already in vilib; lightweight (224×224 input); good for “is this a cat?” on a crop.
- **Cons:** No bounding box by itself; you need a crop (e.g. from detection or tracking). “Which cat?” is not supported unless you add your own classes.

**Best for:** “Is this region a cat?” (e.g. after a first-stage detector or tracker gives a region).

---

## 2.3 “Which cat?” (re-identification / identity)

**Idea:** Recognize **individual** cats (e.g. “my cat” vs “other cat”). This is **re-ID** or **fine-grained classification**, not standard object detection.

- **Options:**
  - **Custom classifier:** Collect many images of “my cat” and “other cats / no cat”; train a small classifier (e.g. MobileNet fine-tuned, or small CNN) and export to TFLite. Run on the tracked/detected cat crop each time. On Pi 4, one 224×224 classification is cheap (~10–50 ms), so you can run it every few frames.
  - **Embedding + similarity:** Use a small network that outputs an embedding for the cat crop; compare with stored embeddings of “my cat”. If similarity is above a threshold, label as “my cat”. Requires a bit more code and data.

- **Pros:** Real “recognition” of identity.
- **Cons:** Need your own dataset and training or embedding pipeline.

**Best for:** “Only react to my cat” or “ignore the neighbor’s cat”.

---

## 2.4 Summary: recognition on Pi 4 (4 GB)

| Goal | Option | Notes |
|------|--------|--------|
| “Is there a cat?” | Detection (COCO class 16) | Same as §1; no extra step |
| “Is this crop a cat?” | vilib image classification (MobileNet + cat labels) | Use detector/tracker to get crop, then classify |
| “Which cat?” (identity) | Custom classifier or embedding model | Your data + training; run on crop |

**Recommendation:** For “is it a cat?”, use **detection** (and optionally confirm with **image classification** on the cat crop). For “which cat?”, add a **custom small classifier or embedding model** on top of detection/tracking.

---

# 3. Tracking a running cat options (follow the same cat over time)

**Goal:** From frame to frame, keep updating the position (and optionally size) of **the same** cat, so you can e.g. drive the pan-tilt or the car to follow it. A running cat moves fast, so you want **enough FPS** and a **stable** tracker.

## 3.1 Detect every frame (no dedicated tracker)

**Idea:** Run cat detection on every frame; take the “best” cat box (e.g. highest score, or largest, or nearest to previous center). Use that box as the current “target” position. No OpenCV tracker.

- **Pros:** Simple; always uses “cat” from the detector; no drift.
- **Cons:** Limited by detection FPS (often 3–10 FPS on Pi 4 with TFLite). Between two detection frames the cat can move a lot; you may “jump” between cats if there are several, or lose the target when the detector misses a frame.

**Best for:** Low FPS is acceptable; single cat; or as a fallback when tracker fails.

---

## 3.2 OpenCV single-object tracker (KCF, CSRT, MOSSE)

**Idea:**  
1. **Initialize:** Run cat detection once (or on key press); get one bounding box for “cat”.  
2. **Track:** Use an OpenCV tracker (e.g. `cv2.legacy.TrackerKCF_create()` or `cv2.legacy.TrackerCSRT_create()`) and call `tracker.update(frame)` every frame. The tracker returns the new bbox.  
3. **Re-initialize:** Optionally run detection again every N seconds or when confidence drops, then re-init the tracker with the new “cat” bbox.

- **Pros:** Tracking step is **very fast** (KCF/MOSSE can run at 20–30+ FPS on Pi 4); you get smooth, high-FPS position updates. Good for pan-tilt following.
- **Cons:** Trackers can drift (e.g. onto background or another cat); they don’t know “cat”, so periodic re-detection is important. CSRT is more accurate but slower than KCF; KCF is a good speed/accuracy tradeoff on Pi 4.

**Best for:** **Tracking a running cat** with smooth motion: detect at 5–10 FPS, track at 20–30 FPS, re-init from detection when needed.

---

## 3.3 Hybrid: detect periodically + track in between

**Idea:**  
- Run **cat detection** every K frames (e.g. every 5–10) or every 0.2–0.5 s.  
- When you get a “cat” box, **initialize or re-initialize** an OpenCV tracker.  
- On all other frames, run only **tracker.update()** and use the tracker’s bbox.  
- If the tracker fails (e.g. low confidence or bbox out of frame), run detection again immediately.

- **Pros:** Good balance: high effective FPS (tracker) + correct “cat” identity (detector). Fits Pi 4 4 GB well.
- **Cons:** More logic (state machine: detect vs track; re-init rules).

**Best for:** **Recommended** for a running cat on Pi 4: smooth tracking and cat-specific re-lock.

---

## 3.4 Tracking with vilib and picar-x

**Integration:**  
- Use **vilib** for camera (e.g. `Vilib.camera_start(…)`) and read `Vilib.img` (or use Picamera2 directly for lower latency if you prefer).  
- Use **vilib object detection** (or your chosen TFLite detector) to get “cat” boxes; filter `class_id == 16` (or `class_name == "cat"`).  
- Implement a small loop: **if no tracker or re-init needed → run detector, pick best cat box, init tracker; else → tracker.update(), get bbox.**  
- Map bbox center (or centroid) to **pan-tilt** (e.g. P0, P1 in picar-x) so the camera (or car) follows the cat. You can reuse the same “stare at you” / “bull fight” style logic but with “cat” and a tracker.

**FPS:**  
- Camera: 640×480 @ 60 fps or 720p @ 30 fps is fine; you can process every 2nd or 3rd frame if needed.  
- Detection: aim for at least **5–10 FPS** (reduce model input size if necessary).  
- Tracker: **20–30 FPS** is realistic on Pi 4 for KCF/MOSSE.

---

## 3.5 Summary: tracking a running cat on Pi 4 (4 GB)

| Approach | Effective FPS | Robustness | Complexity |
|----------|----------------|------------|------------|
| Detect every frame only | ~3–10 | High (always “cat”) | Low |
| OpenCV tracker only (manual first box) | 20–30+ | Medium (drift) | Low |
| **Hybrid: detect every K frames + tracker** | **~20–30** | **High** | **Medium** |

**Recommendation:** Use **hybrid tracking**:  
- **Cat detection** (vilib COCO or a small TFLite model) every 5–10 frames or every ~0.2–0.3 s.  
- **OpenCV KCF** (or CSRT if you accept lower FPS) between detections, re-initialized from the last “cat” box.  
- Pan-tilt (and optionally car motion) driven by the current bbox center.  
- Run camera at 30–60 FPS; process at 10–30 FPS (tracker + occasional detection) so the running cat is followed smoothly.

---

# 4. End-to-end suggestion for Pi 4 (4 GB)

1. **Detection:** Use **vilib object detection** with COCO labels; filter for **class 16 (“cat”)**. Optionally reduce model input size (e.g. 300×300 or 320×320) to gain FPS.
2. **Recognition:** For “is it a cat?”, the **detector’s “cat” class is enough**. For “is this crop a cat?”, add **vilib image classification** on the cat crop. For “which cat?” (identity), add a **custom small classifier or embedding** later.
3. **Tracking:** Implement **hybrid tracking**: **detect cat** every 5–10 frames (or every ~0.2 s), **init/update OpenCV KCF** (or CSRT) on every frame, drive **pan-tilt** from the bbox center. Re-init tracker when detection runs or when tracking fails.

This keeps everything feasible on a Pi 4 with 4 GB RAM and uses your existing vilib/picar-x stack where possible.
