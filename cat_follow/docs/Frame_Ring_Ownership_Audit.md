# Frame Ring Ownership Audit

**Status:** NumPy ring path implemented; native DMA-BUF path and H.264 ownership repairs deployed and validated on ROCK 4D (2026-07-26)
**Date:** 2026-07-26
**Scope:** Production perception frame sharing in `cat_follow` vs `picarx_cat tracker` (Cat Dome)

This document consolidates three architecture audits:

- [Audit car-x frame sharing](e0561200-1f12-4d4a-a035-6a06e9a615ab) — current car-x ring, copy costs, race analysis
- [Compare zero-copy designs](5693520a-e3f1-46f0-8c5f-1989d651ff72) — design options for multi-reader car-x
- [Audit Cat Dome zero-copy](25fbca8b-fd35-4cd5-b8f3-a961e8d53927) — picarx reference implementation

Detection and tracking **must remain independent of the web UI** (see workspace architecture rules). Streaming copies are optional; the camera → detector path is not.

---

## Executive summary

| Aspect | car-x today | picarx_cat tracker (Cat Dome) |
|--------|-------------|-------------------------------|
| Ring size | 4 slots, 640×480 NV12 (~450 KiB/slot) | 3 slots, 4K NV12 (~12 MB/slot) |
| Handoff | **Refcounted leases** + per-slot generation | **Zero-copy views** + generation guards |
| Readers | Detector, H264 (multi-thread) | Single Proc consumer on ring |
| Tear safety | Refcount pins + per-slot odd/even generation | Per-slot `_ring_gen` odd/even + `_ring_slot_changed` |
| Detector path | Lease → RGA/CPU 320×320 crop → RGB RKNN | Proc holds ring view; AI uses separate crop pool |
| Recommended target | **Refcounted ring leases + per-slot generation** | Adapt picarx gen/tear + explicit multi-reader safety |

The NumPy path pins ring slots across detector and stream reads. The camera
never reuses the latest or a pinned slot and drops capture frames when all
slots are busy. The detector's former ~921 KB `snapshot_detector_frame` copy
and legacy `frame_for_detector` allocation are removed.

The OpenCV V4L2 backend now requests unconverted bytes and packs the validated
`/dev/video11` 640×480 NV12 source directly into the ring. Motion uses the
zero-copy Y-plane view; RKNN consumes a paired 320×320 crop. The selectable
native DMA-BUF path performs the center-bottom NV12→RGB crop in RGA and imports
the crop fd into RKNN. H.264 imports a leased camera DMA-BUF directly into MPP;
the web UI draws bounding boxes client-side over WebCodecs-decoded video. An
`mppjpegdec` stage is not applicable to this raw RKISP source.

---

## 0. Vision zero-copy options (A / B / C)

The last deployed/validated production baseline is **Option A**. **Option B**
is now selectable with `CAT_FOLLOW_PERCEPTION_ZEROCOPY=dmabuf` and has been
deployed/revalidated on the ROCK 4D. The older Python probe maps the camera
buffer and therefore does not by itself prove fd-to-fd operation.

| | **C — CPU crop + RGB RKNN** | **A — RGA crop + RGB RKNN (production)** | **B — dmabuf Cam→RGA→NPU** |
|--|--|--|--|
| Capture | CPU-mapped NV12 `Cam[0..3]` ring | Same ring | V4L2/GStreamer **dmabuf fds** (no CPU map of full frames) |
| Crop | CPU `extract_nv12_crop()` | Hardware RGA into paired `Crop[0..3]` (CPU fallback if RGA absent) | Hardware RGA fd→fd |
| NPU input | RGB NumPy tensor | Same | RGB RKNN from crop **dmabuf fd** |
| Color convert | CPU NV12→RGB | CPU NV12→RGB after RGA crop | RGA NV12→RGB fd→fd |
| Complexity | Lowest | Medium | Highest; `io-mode=4` rejected for CPU ring packing today |
| Risk | Low | Medium | High (fd lifetime, RKNN fd import) |
| When | RGA unavailable interim | **Ship now** | Only after standalone B test is fully green |

**Promotion gate for B:** compare at least one frame (normally 60 same captured
frames) with `scripts/compare_zerocopy_vs_numpy.py`; aggregate detection-count
delta must be **0** and median top-box IoU must be **≥0.90** when comparable
boxes exist. Also require ≥30 consecutive DMA-BUF frames, zero steady-state and
lifecycle fd growth, and concurrent detector/H.264 soak without QBUF failure,
starvation, or torn ownership. Until those board gates pass, production keeps
the last validated NumPy NV12 path.

The RGB-model fd proof passed 30 consecutive frames with no fd growth on
2026-07-25. Detection parity and concurrent H.264 stress remain open before
Option B can replace the production ring. That result predates the current
native ownership/crop/ABI repairs and is historical evidence, not validation of
the changed library.

### 0.1 Implemented repair contracts (host-verified, board-pending)

- Camera readiness in DMA-BUF mode is reported only after one startup
  `dequeue -> RGA/RKNN infer -> QBUF` self-test succeeds.
- Native camera buffers, mappings, exported fds, RGA imports, DMA-heap crop,
  RKNN memory, and context use RAII cleanup. Per-buffer queued/dequeued state
  and ownership locks serialize infer/copy/requeue; Python session close is
  camera-owner-thread-only and waits for borrowed calls.
- The C ABI and `ctypes` declarations are synchronized for model
  load/unload/status, detection copy-out, crop offsets, and frame metadata.
  Crop geometry is validated as positive, in-bounds, even-sized/even-origin,
  and equal to RKNN input dimensions.
- Idle unload releases **RKNN/model resources only**. V4L2 streaming, camera
  DMA-BUFs, RGA imports, and the crop DMA-BUF stay alive; reload failure is
  fatal rather than a silent empty-detection mode.
- With DMA-BUF capture and motion gating enabled, `/dev/video12` (or another
  real lores/luma source) is a deployment requirement. A DMA-BUF fd is not
  motion evidence. If lores is unavailable, disable motion gating explicitly;
  do not treat every fd-only frame as motion.
- Monitoring admits exactly one WebSocket viewer/encoder owner. A second
  viewer is rejected. At most one camera lease may wait on MPP, and polling
  drains delayed access units even when no newer camera frame arrives.
- Web UI, TLS, H.264 route/encoder, and authentication configuration failures
  degrade monitoring/control-plane availability without stopping the headless
  camera/detector/tracker/control core. Mutating routes fail closed when the
  production tokens are incomplete; stop and emergency-stop remain open.

---

## 1. car-x current architecture

### 1.1 Memory pool

[`cat_follow/memory/pool.py`](../memory/pool.py):

- `FRAME_RING_N = 4`, shape `(4, 720, 640)` uint8 packed NV12
- `CROP_RING_N = 4`, shape `(4, 480, 320)` uint8 packed NV12 (320×320 crops)
- `frame_for_detector` has been removed
- Static frame RAM: ~1.8 MiB capture + ~0.6 MiB crop pool

### 1.2 Synchronization

[`cat_follow/memory/shared_state.py`](../memory/shared_state.py):

| API | Behavior |
|-----|----------|
| `try_get_write_buffer()` | Under lock, reserves a non-latest slot with `refcount == 0`; returns `None` if full |
| `publish_latest_from_write()` | Marks generation stable, advances capture sequence and latest index |
| `acquire_latest_frame()` | Increments slot refcount and returns `FrameLease(view, frame_seq, generation)` |
| `get_frame_latest(dst)` | Lock → `np.copyto(dst, ring[latest_idx])` |

### 1.3 Data flow

```mermaid
flowchart LR
  subgraph camera [CameraThread]
    CAP[cap.read NV12 staging]
    WB[reserve write slot and pack NV12]
    INJ[optional NV12 to BGR inject to NV12]
    PUB[publish_latest_from_write]
    CAP --> WB --> INJ --> PUB
  end
  subgraph ring [frame_ring N=4 packed NV12]
    S0[slot0]
    S1[slot1]
    S2[slot2]
  end
  subgraph detector [DetectorThread]
    SNAP[acquire_latest_frame lease]
    MOT[Y-plane motion gating]
    CROP[RGA or CPU crop to Crop_i]
    RKNN[RGB RKNN tensor]
    SNAP --> MOT --> CROP --> RKNN
  end
  subgraph stream [H264 only]
    ENC[lease NV12 to mpph264enc]
  end
  WB --> ring
  PUB --> ring
  ring -->|pinned view| SNAP
  ring -->|pinned view| ENC
```

**Tracker** ([`cat_follow/threads/tracker.py`](../threads/tracker.py)) does not read frames — only detector outputs matched by `_detector_frame_gen`.

**Live injection** ([`cat_follow/perception/live_cat_injector.py`](../perception/live_cat_injector.py)) converts and mutates the writer-owned reserved slot before publish; leased slots are never changed.

### 1.4 Copy-cost inventory (640×480 NV12 = 450 KiB)

| Stage | When | Full-frame copies | Notes |
|-------|------|-------------------|-------|
| Camera → ring | ~30 FPS | 1× 450 KiB `copyto` | `cap.read()` owns its buffer; no full-frame color conversion at 640×480 |
| Detector | `DETECT_FPS` (default 5) | 0× full-frame | Copies packed 320×320 Y+UV crop; no intermediate BGR image |
| RKNN preprocess | On detect ticks | 0× full 640×480 | Packed 320×320 NV12 into `_input_buf`; no CPU RGB convert |
| H264 client | ~15 FPS | 1× packed NV12 `tobytes` | Direct `format=NV12` appsrc; overlays drawn in browser |

**Headless production (no stream):** one 450 KiB OpenCV staging → ring copy per
camera frame; no full-frame BGR conversion and no detector full-frame copy.

### 1.5 Why direct ring views are unsafe today

If the detector did:

```python
with lock:
    idx = self._latest_idx
view = pool.frame_ring[idx]  # release lock
backend.infer_all(view, ...)  # slow, lock-free
```

After **3** camera publishes, `write_idx` wraps to `idx` and the camera `copyto`s into that slot while NPU still reads it → **torn/corrupt frame**. At 30 FPS the window is ~100 ms; RKNN + postprocess can exceed that.

Copy-out readers avoid this because each consumer owns its `dst` after the locked copy.

### 1.6 Latent issues (lower severity)

| Risk | Location | Notes |
|------|----------|-------|
| Unlocked `get_write_buffer()` | `memory/shared_state.py` | OK with single camera writer; breaks if second writer added |
| OpenCV `cap.read()` + resize alloc | `threads/camera.py` | Per-frame alloc outside pool discipline |
| OpenCV capture staging | `threads/camera.py` | `cap.read()` still owns one raw NV12 buffer before the ring pack |

---

## 2. picarx_cat tracker reference (Cat Dome)

Primary source: `picarx_cat tracker/web/app.py` (`VideoProcessor`). Audited in [Audit Cat Dome zero-copy](25fbca8b-fd35-4cd5-b8f3-a961e8d53927).

### 2.1 Thread model

| Thread | Role |
|--------|------|
| `CatDome-Cap` | VPU decode, RGA lores, ring pack, H264 submit |
| `CatDome-Proc` | Motion, phases, AI crop submit, tracker — **sole ring consumer** |
| `CatDome-AI` / `CatDome-Post` | RKNN invoke + CPU postprocess on **owned crop buffers**, not ring |
| `CatDome-H264` | Encode from **owned-buffer pool** (copy out of ring) |

### 2.2 Ring slot lifecycle

1. Cap picks write slot `wi`; **skips** if `wi == _ring_last_read` (Proc still holding that slot)
2. `_ring_gen[wi] += 1` → **odd** (write in progress)
3. Decode/RGA/pack into slot
4. On success: `_ring_gen[wi] += 1` → **even** (stable); `_ring_last_written = wi`
5. `_frame_seq += 1` + `Condition.notify()` — Proc never misses a publish

Proc reads `_ring_last_written`, snapshots `_gen_before` (must be even), holds **view** into `_ring_main[ri]`. Before pixel publish or AI submit: `_ring_slot_changed(ri, _gen_before)` → skip if Cap overwrote slot.

**Inject:** copies ring view before paste (`frame.copy()`) — never mutates shared ring slot while readers exist.

### 2.3 Zero-copy terminology (picarx)

| Category | Meaning | Example |
|----------|---------|---------|
| True HW zero-copy | CPU never maps 4K for motion; RGA reads decoder dmabuf fd | fd RGA lores path |
| Copy avoidance | Preallocated `dst=`; one pack into ring; no per-frame alloc | `read_main(dst=_ring_main[wi])` |
| Unavoidable copy | Isolation, format change, async consumer | AI crop pool, H264 `np.copyto`, RKNN output pool |

### 2.4 Async consumer pattern

H264, recording, and AI **do not** hold ring pointers across slow work:

- **AI crop pool:** `AI_QUEUE_DEPTH + 2` owned buffers; Proc extracts crop + `cvtColor` into pool
- **H264 pool:** `H264_BUFFER_POOL_SIZE = 2`; `np.copyto` into free buf → worker encodes → return buf; queue full → latest-wins drop
- **RKNN output pool:** copies NPU tensors before Post reads them

### 2.5 picarx patterns worth porting to car-x (priority)

1. **3-slot ring + per-slot generation + reader-skip + torn-read gates** — car-x has ring indices but lacks gen counters and reader-skip
2. **Condition + monotonic `_frame_seq`** — avoids binary-Event coalesce races (less critical for car-x's polling detector, still useful)
3. **Owned-buffer pools for async encoders** — any H264/recording worker must copy out of ring (picarx `MppH264Service` pattern)
4. **Separate validity flags** — e.g. `_ring_full_frame_valid` for IDLE 4K-skip (adapt as motion-only lores gating on 1 GB boards)
5. **NV12 + fd RGA capture** — future MPP path on RK3576; not required for current OpenCV BGR pipeline

**Do not port blindly:** picarx assumes **one ring consumer** (Proc). car-x has **multiple independent reader threads** (detector + H264). The legacy MJPEG reader has been removed; there is no MJPEG/software fallback.

---

## 3. Design comparison for car-x multi-reader layout

Evaluated in [Compare zero-copy designs](5693520a-e3f1-46f0-8c5f-1989d651ff72).

### 3.1 Option A — Refcounted ring leases (recommended)

Each slot: `{generation, refcount}`. Camera publishes into a write slot; readers `acquire` → `FrameLease(view, gen)` → `release`. Camera reuses slot only when `refcount == 0`.

| Path | Behavior |
|------|----------|
| camera → detector | Hold lease through RKNN infer; **zero full-frame copy** |
| camera → H264 | Non-blocking `SharedState.acquire_latest_frame()`; skip tick if busy (latest-wins) |
| inject | Mutate camera staging only; never inject into slot with `refcount > 0` |
| lag | Camera drops frames when all slots leased; readers never see torn data |

**Pros:** Safest multi-reader model; combines picarx tear semantics with explicit pinning.
**Cons:** RAII discipline in Python; likely need `FRAME_RING_N = 4–5` for detector + dual streams.

### 3.2 Option B — Reader index acknowledgements

Per-consumer read cursor; writer skips slot if any consumer has not acked.

**Pros:** Simple for 2-thread systems; picarx proves single `_ring_last_read`.
**Cons:** Awkward with 3+ readers at different rates; still needs gen counter for zero-copy views.

### 3.3 Option C — Latest-slot copy-out (historical car-x baseline)

Readers always `copyto` under lock.

**Pros:** Simplest correct design; no lease bugs.
**Cons:** Highest bandwidth; detector copies every tick regardless of motion gate.

### 3.4 Verdict

**Refcounted leases + per-slot generation** is implemented for detector and
H.264. `get_frame_latest(dst)` remains as a compatibility copy-out escape hatch.

---

## 4. Copies: what stays vs what goes under the target design

### 4.1 Target design decisions (2026-07-23)

These constraints revise the earlier “unavoidable” list:

| Decision | Implication |
|----------|-------------|
| Capture = OpenCV V4L2 raw NV12 into ring slot | Keeps one `cap.read()` staging buffer and one packed ring write; optional GStreamer mmap CPU pack remains fallback |
| Camera locked at **640×480** | **No capture resize** — perception frame size equals camera size |
| Native **NV12** ring (picarx-style) | Motion stays on Y; AI feeds NV12 crop to RKNN; BGR only at inject boundaries |
| Detector input = **center-bottom 320×320 crop** from 640×480 | **No letterbox/resize** — same rule as picarx `AI_CROP_SIZE == RKNN_INPUT_SIZE` |
| Overlays = client Canvas + JSON (picarx-style) | No server-side pixel burn-in on ring for live UI |
| Inject = copy-before-paste / writer-owned only | Never mutate a leased ring slot |

### 4.2 Still required (even in picarx)

| Copy / convert | How picarx handles it | Notes for car-x |
|----------------|----------------------|-----------------|
| OpenCV capture → ring NV12 pack | `cap.read()` + `np.copyto(..., ring_slot)` | Implemented baseline; optional GStreamer mmap CPU pack (`io-mode=2`) is fallback only. |
| NV12 crop → RKNN (AI path) | RGA or `extract_nv12_crop` into `Crop[i]` | Packed NV12 into preallocated RKNN `_input_buf`; no RGB conversion |
| `rknn.inference(inputs=[buf])` | Prealloc `_input_buf`; **output** pool only | Input still CPU NumPy → NPU map; not solved as dmabuf zero-copy |
| H.264 boundary | Direct NV12 `Gst.Buffer.new_wrapped(tobytes())` | Implemented; synchronous bytes ownership permits lease release after submit |
| Inject | `frame.copy()` / NV12→BGR then paste; lores ROI paste into scratch | Writer-owned / Proc-owned copies only |
| Snapshot JPEG | `.copy()` + NV12→BGR + encode | Off hot path |

### 4.3 Eliminated under car-x target (vs today’s OpenCV BGR path)

| Former “unavoidable” | Why it goes away |
|----------------------|------------------|
| Capture resize 640×480 | Camera is already 640×480 |
| Full-frame letterbox to 320×320 | Center-bottom crop, scale=1.0 |
| Full-frame NV12→BGR every frame | Keep NV12 on ring; feed NV12 crop to RKNN; BGR only for inject/snapshot |
| Server-side stream overlays | H.264 + client overlay JSON (WebCodecs canvas) |
| Detector `snapshot_detector_frame` full copy | Ring lease / crop extract instead |
| Software full-frame lores when RGA fd available | Hardware lores Y-plane view (picarx) |

### 4.4 Lores / inject minimize options

1. **Preferred:** RGA dmabuf-fd downscale into lores NV12 slot; motion uses Y-plane **view** (no extra gray alloc).
2. **Inject:** do not re-downscale full frame — `copyto` lores scratch + **ROI-only** sprite paste (picarx `paste_on_lores`).
3. **While inject active:** force pack of main only if AI/inject needs pixels; otherwise keep lores path for motion.
4. **Optional:** run capture already at lores size for motion-only modes (picarx `CAT_DOME_LORES_DECODE`).
5. **Avoid:** software `cv2.resize` full BGR→gray every frame when hardware lores is healthy.

Zero-copy means **eliminating redundant full-frame copies and allocs**, not zero CPU touches everywhere.

---

## 5. Deferred migration sequence

Incremental phases and current status:

| Phase | Scope | Key files | Outcome |
|-------|-------|-----------|---------|
| **0 — Instrument** | Counters only | `memory/shared_state.py`, metrics | Pending board metrics |
| **1 — Gen counter** | Per-slot odd/even generation | `memory/shared_state.py` | **Implemented** |
| **2 — Refcount wired** | Camera skips pinned/latest slots | `memory/pool.py`, `memory/shared_state.py` | **Implemented** |
| **3 — Detector lease** | Lease through RKNN; center-bottom 320×320 crop | `threads/detector.py` | **Implemented** |
| **4 — Stream leases** | H264 leases | `routes_h264.py`, `h264_encoder.py` | **Implemented**; one viewer, one pending camera lease, delayed-AU polling |
| **8 — Crop pool + RGA** | `Cam[i]→Crop[i]` paired buffers | `memory/pool.py`, `vision/rga_crop.py`, `threads/detector.py` | **Implemented** |
| **9 — RKNN input** | RGB model + RGA/CPU NV12→RGB input | `vision/rga_crop.py`, `vision/rknn_backend.py` | **Implemented**; packed-NV12 model claim rejected by Toolkit calibration |
| **10 — H264-only UI** | WebCodecs decode + canvas overlays | `web_ui/templates/main.html`, `web_ui/app.py` | **Implemented** |
| **11 — dmabuf B test** | Native session + parity gate | `native/zerocopy`, `compare_zerocopy_vs_numpy.py` | **Implemented; changed native code is not yet deployed/validated on board** |
| **5 — Native RKISP NV12** | Raw `/dev/video11` NV12 → DMA-BUF leases | camera/shared state/detector/stream | **Implemented on host; current native revision needs ROCK 4D validation** |
| **6 — Lores gen coupling** | Pair lores gen with main gen; ROI inject patch | shared state/inject | Pending |
| **7 — Retire legacy** | Remove `frame_for_detector`, stale APIs | pool/tests | **Implemented** |

### Thread contracts (implemented)

- **Camera:** `try_get_write_buffer()` → staging/inject → `publish_latest_from_write()`; drop when full
- **Detector:** one `acquire_latest_frame()` per tick when needed; hold through motion fallback + RKNN + publish; `release()` in `finally`
- **H264:** sole admitted viewer acquires one latest lease per encode tick;
  DMA-BUF ownership transfers to MPP until matching output PTS, additional
  pending frames drop, and `poll()` drains delayed AUs without a new frame
- **Tracker:** unchanged — reacts to `_detector_detections_gen` changes only

### 5.1 Native NV12 capture roadmap ([Assess MPP NV12 port](688ed57d-eb40-4d5f-b804-65b14d8c2aa3))

**Do not use picarx `mppjpegdec` for car-x.** RKISP `/dev/video11` already delivers raw 640×480 NV12; picarx MPP decode targets USB MJPEG 4K.

| Patch | Scope | Notes |
|-------|-------|-------|
| **3 — NV12 ring + crop-at-detect** | NV12 pool `(4,720,640)`; stop full-frame BGR convert; 320×320 crop → RGB tensor → RKNN | **Implemented and validated** with production venv OpenCV |
| **4 — GStreamer capture fallback** | Optional `v4l2src io-mode=2` (mmap CPU pack); feature flag `CAT_FOLLOW_CAMERA_CAPTURE_BACKEND=gst_nv12` | Implemented; canonical 460,800-byte sample validated via `extract_dup`. `io-mode=4` (fd-only dmabuf export) is rejected for CPU ring packing because Python map/VideoInfo crashed the allocator on this board. |
| **5 — HW lores / NV12 H.264** | Direct NV12 MPP input implemented; RKISP lores self-path (`/dev/video12`) remains | `mpph264enc` is absent on the board. Current optional streaming therefore remains unavailable on that path. Under the canonical target, this also means no monitoring stream and degraded no-recording operation until a hardware H.264 encoder is provisioned; there is no software/MJPEG target fallback. Prefer the lores device over RGA at 640×480. |

**Model input correction (2026-07-25):** the YOLO ONNX tensor is RGB
`[1,3,H,W]`. RKNN Toolkit2 2.3.2 may export an FP artifact when
`input_size_list=[1,H*3/2,W,1]` is supplied, but INT8 calibration rejects the
packed NV12 arrays and still requires the original RGB tensor. Therefore this
is not a valid native-NV12 model conversion. The supported zero-copy path uses
RGA to convert NV12→RGB directly between DMA-BUFs before RKNN fd import.

### 5.3 Standalone dmabuf validation (Option B)

Run on the ROCK 4D only (not linked to `main_loop` / Flask):

#### Capture-only Python probe

```bash
python3 scripts/validate_dmabuf_rga_rknn.py \
  --device /dev/video11 --frames 30 \
  --model models/yolov8n_coco_320_rk3576_int8.rknn
```

Emits JSON: `status` (`pass` | `partial` | `fail`), `rga_ms`, `npu_ms`, `detections`.
`partial` means dmabuf capture + RGA succeeded but RKNN fd import is unavailable.
This probe maps the GStreamer buffer to NumPy and is not the Option B promotion
gate, even if it reports `status=pass`.

#### Native fd-to-fd proof

Build on the board after installing `librga` development headers and the RKNN
2.3.2 `rknn_api.h`:

```bash
g++ -std=c++17 -O2 -Wall -Wextra \
  scripts/validate_dmabuf_rga_rknn_fd.cpp \
  -o scripts/validate_dmabuf_rga_rknn_fd -lrga -lrknnrt

sudo systemctl stop cat-follow.service
scripts/validate_dmabuf_rga_rknn_fd \
  /dev/video11 models/yolov8n_coco_320_rk3576_int8.rknn 30
sudo systemctl start cat-follow.service
```

This path exports V4L2 capture buffers with `VIDIOC_EXPBUF`, imports the camera
fd into RGA, writes the center-bottom crop into a DMA-heap fd, and imports that
same crop fd with `rknn_create_mem_from_fd`. The crop fd has RKNN-required
virtual-address metadata but no CPU pixel access.

Historical ROCK 4D result on 2026-07-25 (before the current native
RAII/crop/ABI/lifecycle repairs):

```json
{"status":"pass","path":"v4l2-expbuf->rga-fd->rknn-fd","camera_cpu_mapped":false,"crop_va_mapped":true,"crop_cpu_access":false,"frames_ok":30,"rga_ms_p50":0.363705,"npu_ms_p50":6.52327,"fd_delta":0,"input_size_with_stride":307200,"input_format":"rgb"}
```

This proved fd-to-fd transport with RGA NV12→RGB conversion for that revision
and provisioned RGB model. It does **not** validate the currently changed
native library. Do **not** replace the last validated NumPy capture path until
the current code is deployed and the parity, lifecycle/fd, QBUF, and concurrent
H.264 stress gates are green on the ROCK 4D.

### 5.2 ROCK 4D validation (2026-07-23)

- Production venv OpenCV captured `/dev/video11` as contiguous raw NV12:
  shape `(1, 460800)`, reshaped ring slot `(720,640)`, stride 640.
- Full-frame conversion and center-bottom crop conversion matched exactly
  (`max_bgr_delta=0`) for region `(160,160,320,320)`.
- Headless camera + real RKNN detector ran without Flask/streams/motors:
  camera generation 28, detector generation 27, NPU invoke about 21–22 ms.
- Direct NV12→RGB RKNN input was validated at 30 FPS over 354 inferences:
  color conversion p50 2.08 ms (from 3.63 ms for NV12→BGR→RGB),
  detector processing p50 16.75 ms (from 18.34 ms), p95 19.57 ms, and
  zero 33.3 ms detector-processing deadline misses.
- GStreamer `v4l2src io-mode=2` (mmap CPU pack) is an optional fallback for
  system OpenCV builds that cannot open this multiplanar node. Reserve
  `io-mode=4` for a future fd-only hardware path (RGA/MPP), not ring packing.
  Production remains on verified `CAT_FOLLOW_CAMERA_CAPTURE_BACKEND=opencv`.

### Relationship to 30 FPS soak

The direct color path removes the intermediate BGR allocation and saves about
1.6 ms at p50, but does **not** remove the NPU invoke (~13–16 ms on A53). The
M4f **DETECT_FPS=30 + ROS2 soak** remains a separate validation item
([Software_Integration](Software_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md) M4f).

---

## 6. Tests

Host regression status: `python -m pytest tests -q` completed with **511
passing** on 2026-07-26. These tests cover contracts and mocks; they do not
exercise ROCK 4D V4L2, RGA, RKNN fd import, MPP, or device permissions.

| Test | File | Status |
|------|------|--------|
| Slot pinned until release | `tests/test_shared_state.py::test_frame_lease_pins_slot_until_release` | Implemented |
| Drop write when all slots leased | `tests/test_shared_state.py::test_frame_ring_drops_write_when_all_slots_are_pinned` | Implemented |
| Slow reader no tear | `tests/test_shared_state.py::test_slow_frame_reader_never_observes_torn_pixels` | Implemented |
| Center-bottom Y/UV crop mapping | `tests/test_detector_thread.py::test_center_bottom_model_crop_copies_matching_y_and_uv_planes` | Implemented |
| Detector bbox remap + crop shape | `tests/test_detector_thread.py::test_detector_preserves_primary_confidence` | Implemented |
| Ring publish + lease | `tests/test_camera_ring.py` | Implemented |
| NV12 geometry, planes, crop, conversion | `tests/test_nv12_utils.py` | Implemented |
| Center-bottom crop BGR golden equivalence | `tests/test_nv12_utils.py::test_center_bottom_crop_bgr_matches_full_frame_slice` | Implemented |
| Direct NV12→RGB equivalence/reuse | `tests/test_nv12_utils.py::test_nv12_to_rgb_matches_bgr_channel_swap_and_reuses_destination` | Implemented |
| Chroma alignment regression on odd crops | `tests/test_nv12_utils.py::test_align_nv12_crop_required_before_odd_region_extract` | Implemented |
| Native ABI declarations and model lifecycle | `tests/test_zerocopy_model_lifecycle.py`, native-source contract tests | Host pass |
| H.264 one-viewer, lease cap, delayed AU polling | H.264 route/encoder tests | Host pass |
| Option A/B parity thresholds | `tests/test_compare_zerocopy_parity.py` | Host pass (threshold logic only) |

**ROCK 4D temporary-build evidence (2026-07-26):** native compilation and ABI
symbol load passed; 10 RGA/RKNN frames completed with steady fd delta 0; native
model unload/reload passed; 10 same-input parity frames had count delta 0;
repeat-session lifecycle fd delta was 0 (the first RKNN/RGA initialization
retained two process-global descriptors); and DMA-BUF MPP produced 30/30 H.264
access units. The service was restored active after validation.

**Board gates still Pending:** deploy the repaired revision, exercise scenes
with comparable detections for the IoU gate, run long fd/QBUF and concurrent
detector/stream soak, validate `/dev/video12` motion gating, inject camera
faults, and prove secure-context browser playback.

---

## 7. Code review checklist (C2)

When reviewing frame-ring changes, apply [`Code_Review_Plan.md`](Code_Review_Plan.md) C2 against this document:

- Verify single-writer on ring pixels; readers use lease or copy-out
- Do not hold `_lock_frame` across RKNN, encode, or network I/O
- Bump generation on publish; check `stale` before downstream pixel use
- Async encoders must use owned pools (picarx H264 pattern), not ring pointers
- Detection must work headless without web streams

---

## 8. References

| Resource | Path |
|----------|------|
| car-x pool | [`cat_follow/memory/pool.py`](../memory/pool.py) |
| car-x shared state | [`cat_follow/memory/shared_state.py`](../memory/shared_state.py) |
| car-x camera | [`cat_follow/threads/camera.py`](../threads/camera.py) |
| car-x detector | [`cat_follow/threads/detector.py`](../threads/detector.py) |
| picarx orchestration | `picarx_cat tracker/web/app.py` |
| picarx MPP capture | `picarx_cat tracker/camera/gst_mpp_capture.py` |
| picarx H264 pool | `picarx_cat tracker/camera/h264_mpp.py` |
