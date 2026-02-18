# Design: Bbox–Distance Calibration Tool

**Goal:** Let the user collect **(area_px, distance_cm)** pairs and write them into `calibration/bbox_distance.json` so `get_distance_cm_from_bbox_area()` is accurate for their camera and cat at **640×480**.

---

## 1. How calibration works (recap)

- **Input:** Bbox area in pixels² = `width × height` (from detector/tracker at 640×480).
- **Output:** Estimated distance in cm (interpolated from the table).
- **Table format:** `area_to_cm`: list of `[area_px, distance_cm]`. Larger area → closer (smaller distance_cm).
- **User’s job:** At several **known distances** (e.g. 15, 20, 30, 50 cm), get a bbox around the cat and record the area. Each pair becomes one row in the table.

---

## 2. How the user can use it (user flow)

### Option A: Manual measurement, tool only logs and saves

1. User places cat (or cat-sized object) at a **known distance** (e.g. 30 cm, measured with ruler/tape).
2. User gets a bbox somehow:
   - **Live:** Run main_loop or a “calibration mode” that shows the stream + current bbox; user presses “Record” to log (area, distance).
   - **Offline:** User has a screenshot or saved image; they run a small script that either (a) runs the detector on that image and shows bbox, or (b) lets them draw a rectangle and the script computes area.
3. User enters **distance_cm** (e.g. 30).
4. Tool records **(area_px, distance_cm)** and adds it to a list.
5. Repeat for 3–5+ distances (spread between closest and farthest).
6. User clicks **“Save to calibration”** (or script writes) → tool updates `bbox_distance.json` with the new `area_to_cm` (and keeps `target_distance_cm` / notes as configured).

**Pros:** Simple, works without car moving. **Cons:** User must measure distance and have a way to get bbox (live or from image).

### Option B: Semi‑assisted (recommended)

1. **Calibration mode in Web UI** (or a separate small app):
   - Stream at 640×480 with current bbox overlay (from detector or manual draw).
   - Fields: **Distance (cm)** [input], **“Add sample”** [button].
   - On “Add sample”: take current bbox area from shared state (or from last detector output), pair with user-entered distance, append to a temporary list.
   - Show **current samples** as a table (area, distance) and optionally a small plot (distance vs area).
   - **“Save to calibration”** writes `area_to_cm` (sorted by area) to `bbox_distance.json`; optionally user can set **target_distance_cm** in the same screen (already “configurable later”).
2. User measures distance with ruler, places cat, enters distance, clicks “Add sample”, repeats, then saves.

**Pros:** One place to do everything; no editing JSON by hand. **Cons:** Requires a calibration page or mode in the Web UI.

### Option C: CLI script only

- Script that:
  - Reads an image (path) or connects to camera, runs detector (or user passes bbox), gets area.
  - Prompts for distance_cm, appends to a list, then can write `bbox_distance.json`.
- Optional: interactive loop (image path → detect → show bbox → enter distance → add; repeat; save).

**Pros:** No UI; good for headless or scripted use. **Cons:** Less visual; user must provide images or camera access.

---

## 3. Design choices to decide

| Topic | Options | Recommendation |
|-------|--------|-----------------|
| **Where it lives** | (a) New tab in existing Web UI, (b) Standalone script, (c) Both | Start with (a) so one place for all calibration; add (b) later if needed for automation. |
| **Source of bbox** | (a) Live detector/tracker from main pipeline, (b) User draws rectangle on a still image, (c) Run detector on a single image | (a) if main_loop is running and detector available; (b) or (c) for offline/single-image. Supporting (a) + (b) covers most cases. |
| **Who measures distance** | User with ruler/tape at known distances. | No change; tool only records (area, distance) and saves. |
| **Editing existing table** | Allow delete row, edit distance, or “replace entire table” from current samples. | At least “replace entire table from current samples”; optional row delete/edit. |
| **target_distance_cm** | Editable in same screen or leave in JSON only. | Make it editable in the same calibration screen (configurable later as you wanted). |

---

## 4. Proposed design (concrete)

### 4.1 Web UI: “Bbox distance” calibration tab

- **URL:** e.g. `/calibration` or `/calibration/bbox-distance` (reuse or extend existing calibration stub).
- **Content:**
  - Short blurb: “Locked to 640×480. Place cat at known distance (cm), then Add sample.”
  - **Live stream** at 640×480 with bbox overlay (from shared state: detector or tracker bbox). If no bbox, show “No bbox — run detector or draw on image.”
  - **Distance (cm):** number input.
  - **“Add sample”** button: reads current bbox from shared state (if valid), computes area = w×h, appends `[area, distance_cm]` to a **samples list** (in memory or in a small backend store for the session).
  - **Optional:** “Load image” + simple canvas to draw a rectangle → compute area and use that instead of live bbox (for offline calibration).
  - **Current samples** table: columns Area (px²), Distance (cm); sort by area; allow delete row.
  - **target_distance_cm:** number input (default from current JSON); applied on Save.
  - **“Save to calibration”** button: sort samples by area, set `area_to_cm = samples`, set `target_distance_cm`, write `calibration/bbox_distance.json` (keep existing `notes` or a short fixed one).
- **Backend:**  
  - `GET /api/calibration/bbox-distance` → return current `area_to_cm`, `target_distance_cm`.  
  - `POST /api/calibration/bbox-distance` → body: `{ "area_to_cm": [[a,d], ...], "target_distance_cm": 15 }` → write JSON file.  
  - For “Add sample” from live bbox: either (1) frontend sends `{ area, distance_cm }` and backend appends to a session list and returns updated list, or (2) frontend keeps list in Alpine.js and only sends full list on Save. (2) is simpler and stateless.

### 4.2 Implementation steps

1. **Backend**
   - Add `GET/POST /api/calibration/bbox-distance` in `web_ui/app.py`:
     - GET: read `bbox_distance.json`, return `area_to_cm`, `target_distance_cm` (and maybe `notes`).
     - POST: validate body, write `area_to_cm` (sorted by area) and `target_distance_cm` to `bbox_distance.json` (path from calibration loader or env).
   - Ensure calibration dir path is resolvable from the app (e.g. same as `Calibration` uses).

2. **Frontend**
   - Calibration tab (new or existing): Alpine.js state: `samples = []`, `targetDistanceCm = 15`, `distanceInput = ""`.
   - On load: fetch GET, init `samples` and `targetDistanceCm`.
   - “Add sample”: if using live bbox, get bbox from same source as main stream (e.g. from status or a dedicated `/api/bbox` that returns last bbox from shared state). Compute `area = w*h`, push `[area, parseFloat(distanceInput)]`, sort by area, clear input.
   - “Delete row”: remove from `samples`.
   - “Save to calibration”: POST `{ area_to_cm: samples, target_distance_cm: targetDistanceCm }`. Show success/error.

3. **Getting bbox in the calibration tab**
   - **Option 1:** Add `GET /api/status` or existing status to include `bbox_tracker` or `bbox_detector` (if already there, reuse). Frontend uses that bbox for “Add sample” when stream is live.
   - **Option 2:** “Load image” + draw rectangle: canvas with mouse drag → get (x,y,w,h) → area = w*h; no backend needed for the rectangle, only for Save.

4. **Safety**
   - POST should only allow updating `area_to_cm` and `target_distance_cm`; keep `notes` or overwrite with a short fixed string so we don’t lose the “640×480 / examples” note unless we explicitly change the design.
   - Optional: backup `bbox_distance.json` before write (e.g. `bbox_distance.json.bak`).

---

## 5. Summary

- **User:** Measures distance with ruler, places cat, enters distance in UI, clicks “Add sample” (and optionally deletes/edits rows), sets target distance, clicks “Save to calibration.”
- **Tool:** Provides a calibration tab with (optionally) live stream + bbox, sample list, target_distance_cm, and GET/POST API to read/write `bbox_distance.json`.
- **Implementation:** Backend GET/POST for bbox-distance, calibration tab in Web UI, frontend state for samples and save; optionally “draw bbox on image” for offline use. Start with Web UI; add CLI later if needed.

If you want to adjust (e.g. CLI-first, or no live stream and only “load image + draw”), we can trim or extend this design accordingly.
