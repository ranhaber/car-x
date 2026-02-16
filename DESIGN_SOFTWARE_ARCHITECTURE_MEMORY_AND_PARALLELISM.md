# Software Architecture: Memory Allocation, Shared Memory, and Parallel Processes

This document defines a **software architecture** for the cat-follow system with:

1. **Pre-allocated memory** at application start (no per-frame alloc in hot paths).
2. **Shared memory** between threads/processes where possible (minimize copies and allocator pressure).
3. **Explicit parallel process/thread design** with clear ownership and synchronization.

Target platform: **Raspberry Pi 4, 4 GB RAM**. Language: **Python 3** (picar-x, vilib, OpenCV, TFLite).

---

## 1. High-Level Parallel Design

### 1.1 Process vs Thread Choice

- **Threads:** Share the same heap; good for I/O-bound work and for passing references to pre-allocated buffers. Python GIL is released during many NumPy/OpenCV/C extension operations, so camera capture and OpenCV tracker can run in threads without blocking each other too much. TFLite inference may hold the GIL.
- **Processes:** True parallelism for CPU-heavy work (e.g. TFLite detector). Use **shared memory** (e.g. `multiprocessing.shared_memory.SharedMemory` or `multiprocessing.Array`) for frames and results to avoid serialization and copies.

**Recommended:** Start with **threads + pre-allocated buffers** for simplicity; move **detector to a separate process** only if the main thread is starved (FPS drops). All pre-allocated and shared structures are defined below so a later move to multi-process only changes who writes/reads the shared buffers.

### 1.2 Logical Units and Concurrency

| Unit | Role | Suggested concurrency | Shares |
|------|------|------------------------|--------|
| **Camera** | Capture frame at 30 FPS into a pre-allocated buffer | 1 thread | Writes `SHARED.frame_*` |
| **Tracker** | Update OpenCV tracker every frame; output bbox | 1 thread (30 FPS) | Reads latest frame; writes `SHARED.bbox_*`, `SHARED.bbox_valid` |
| **Detector** | Run TFLite every K frames; output cat bbox for re-init | 1 thread or 1 process | Reads frame copy or shared frame; writes `SHARED.detect_bbox_*`, `SHARED.detect_valid` |
| **State machine + Motion** | Poll commands, dispatch, run motion (picar-x) | Main thread | Reads `SHARED.bbox_*`, `SHARED.detect_*`, odometry; writes motion |
| **Commands** | Incoming cat_location, stop | Main thread (poll) or 1 thread | Lock-protected queue or shared flags |

---

## 2. Pre-Allocated Memory (Upfront at Startup)

All large buffers are allocated **once at startup** and reused. No per-frame `malloc`/alloc in the hot path.

### 2.1 Frame Buffers (Largest Consumers)

- **Format:** RGB or BGR, 640×480×3, uint8.
- **Size per frame:** 640 × 480 × 3 = **921,600 bytes** (~0.9 MB).
- **Strategy:** **Ring of N frames** (e.g. N=3). Camera writes into slot `write_index`; tracker/detector read from slot `read_index` or “latest” slot. Slots are pre-allocated arrays (NumPy or `multiprocessing.Array` if cross-process).

**Pre-allocated:**

- `frame_pool[0..N-1]`: N arrays of shape `(480, 640, 3)`, dtype `uint8`.
- Total: N × 921600 bytes (e.g. 3 × 0.9 MB ≈ 2.7 MB).

**Ownership:**

- **Writer (camera):** writes into `frame_pool[write_index % N]`; then advances `write_index`.
- **Readers (tracker, detector):** read from `frame_pool[read_index % N]` or from a dedicated “latest” slot that the camera copies into (single copy). Prefer **single “latest” frame** + one extra for detector so detector can read a snapshot while tracker uses “latest” (see below).

**Simpler variant (recommended for threads):**

- **Two buffers:** `frame_latest`, `frame_for_detector`. Camera always writes into `frame_latest` (in-place or copy from driver). When detector is due, copy `frame_latest` → `frame_for_detector` (one copy every K frames). Tracker reads `frame_latest` only. All pre-allocated at startup.

### 2.2 Detection / Tracker Output (Shared State)

- **Bbox:** 4 floats (x, y, w, h) or (xmin, ymin, xmax, ymax).
- **Valid flag:** 1 bool or int.
- **Detector re-init bbox:** same 4 floats + 1 bool.

Pre-allocate:

- `bbox_tracker`: 4 floats + 1 int (valid).
- `bbox_detector`: 4 floats + 1 int (valid).

Use a small **struct** or **NumPy array** (e.g. shape `(2, 5)` for two bboxes + valid) in shared memory or in a shared object protected by a lock.

### 2.3 Odometry and State Machine State

- **Odometry:** (x, y, heading) = 3 floats. Pre-allocate one struct; odometry thread (or main) updates in place.
- **State machine:** current state (enum/int), target_xy (2 floats), last_bbox (4 floats). Pre-allocate; main thread updates.

### 2.4 Queues and Small Buffers

- **Command queue:** Bounded queue (e.g. capacity 8) for “cat_location (x,y)” and “stop”. Pre-allocate the queue and its message slots (e.g. fixed array of structs).
- **TFLite:** Interpreter input tensor buffer is often tied to the model; allocate interpreter once at startup. Input buffer can be a view over a pre-allocated frame (resize into fixed buffer) so no per-run alloc.

### 2.5 Summary Table (Pre-Allocated at Startup)

| Buffer | Type | Size (approx) | Owner (writer) | Readers |
|--------|------|----------------|----------------|---------|
| `frame_latest` | ndarray (H,W,3) uint8 | 0.9 MB | Camera thread | Tracker, (copy to detector) |
| `frame_for_detector` | ndarray (H,W,3) uint8 | 0.9 MB | Main/Camera | Detector thread/process |
| `bbox_tracker` | 4 float + 1 int | 20 B | Tracker thread | Main |
| `bbox_detector` | 4 float + 1 int | 20 B | Detector | Main, Tracker (re-init) |
| `odometry_xyh` | 3 float | 12 B | Main or odom thread | Main |
| `state_machine_state` | enum + 6 float | ~56 B | Main | Main |
| `command_queue` | Fixed array of 8 msgs | ~200 B | Command source | Main |
| **Total (threads)** | | **~1.8 MB** + small | | |

If detector runs in another **process**, use `multiprocessing.shared_memory.SharedMemory` for `frame_for_detector` and for `bbox_detector` (or a small shared array). Sizes unchanged; only the allocation API and sync change.

---

## 3. Shared Memory Layout and Access

### 3.1 Single-Process (All Threads)

- **Shared object** (e.g. `SharedState` class or a module-level dict) holding:
  - `frame_latest`: NumPy array (pre-allocated). Camera writes; tracker reads. Protect with a **lock** (e.g. `threading.Lock`) for “swap latest” or use a **double buffer** and swap indices so reader always sees a consistent frame.
  - `frame_for_detector`: NumPy array (pre-allocated). Main or camera thread copies from `frame_latest` every K frames; detector reads. One lock or atomic “dirty” flag.
  - `bbox_tracker`, `bbox_detector`: 4 float + 1 int each. Writers (tracker, detector) write; main reads. Use one lock for all bbox/valid fields or one lock per bbox.
  - `odometry_xyh`: 3 float. Main or dedicated thread updates; main reads.
  - `state`, `target_xy`, `last_bbox`: main only.
  - `stop_requested`, `cat_location`: set by command poll; main reads and clears.

**Minimize locking:** Use a single **pipeline lock** per “slot” (e.g. lock for “current frame”, lock for “bboxes”) and keep critical sections short (copy pointer or swap index, not heavy work inside lock).

### 3.2 Double-Buffer for Latest Frame (Lock-Free Read)

- **Two slots:** `frame_buf[0]`, `frame_buf[1]`, each pre-allocated.
- **Camera:** Writes into `frame_buf[write_idx]`. After write, atomically set `latest_index = write_idx` and then `write_idx = 1 - write_idx`. (On Python, use a shared int and ensure single writer.)
- **Tracker:** Reads `frame_buf[latest_index]` (no lock if only camera updates `latest_index`). Tracker never writes the frame.

This avoids the reader waiting on the writer.

### 3.3 Cross-Process (Detector in Separate Process)

- **Shared memory** for:
  - One frame buffer (e.g. 640×480×3 bytes). **Producer:** main process (camera). **Consumer:** detector process. Use a **semaphore** or “frame_id” so detector knows when a new frame is ready; avoid reading while producer is writing (double buffer: producer writes A, then flips “active” to A; detector reads “active”).
  - Detection result: 4 float + 1 int in a small `multiprocessing.Array`.
- **Allocation:** At startup, create `SharedMemory(size=frame_size)` and `SharedMemory(size=bbox_size)`, or `multiprocessing.Array('d', 5)`. Pass names to the detector process. NumPy can wrap the shared memory: `np.ndarray(shape, dtype, buffer=shm.buf)`.

---

## 4. Parallel Process/Thread Plan

### 4.1 Thread Layout (Recommended First Version)

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Main process                                                            │
│                                                                          │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐               │
│  │ Camera thread│    │Tracker thread│    │Detector thread│               │
│  │  30 FPS      │    │  30 FPS      │    │  every K     │               │
│  │              │    │              │    │  frames      │               │
│  │ write        │    │ read latest  │    │ read         │               │
│  │ frame_latest │    │ update KCF   │    │ frame_for_   │               │
│  │              │    │ write        │    │ detector     │               │
│  │              │    │ bbox_tracker │    │ write        │               │
│  │              │    │              │    │ bbox_detector│               │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘               │
│         │                    │                   │                        │
│         └────────────────────┴───────────────────┘                      │
│                              │                                            │
│                    ┌─────────▼─────────┐                                  │
│                    │  SharedState     │                                  │
│                    │  frame_latest    │                                  │
│                    │  frame_for_det   │                                  │
│                    │  bbox_tracker    │                                  │
│                    │  bbox_detector   │                                  │
│                    └─────────┬─────────┘                                  │
│                              │                                            │
│  ┌───────────────────────────▼───────────────────────────┐             │
│  │  Main thread (loop at 30 FPS or motion rate)             │             │
│  │  - poll commands → state_machine.dispatch()              │             │
│  │  - read bbox_tracker / bbox_detector, odometry            │             │
│  │  - if APPROACH/TRACK: motion.center_cat_control(bbox)     │             │
│  │  - copy frame_latest → frame_for_detector every K frames │             │
│  │  - update odometry                                       │             │
│  └──────────────────────────────────────────────────────────┘            │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4.2 Pipeline Stages and Data Flow

1. **Camera thread:**  
   - Capture into pre-allocated `frame_latest` (or double-buffer slot).  
   - Optionally signal “new frame” (event or increment frame_id).

2. **Main thread (each tick):**  
   - Copy `frame_latest` → `frame_for_detector` every K-th frame (so detector always has a fresh snapshot without blocking camera).  
   - Poll commands; dispatch state machine.  
   - Read `bbox_tracker` (and if detector just wrote, `bbox_detector` for re-init).  
   - In APPROACH/TRACK: call `center_cat_control(bbox_tracker)`; update odometry.  
   - Trigger detector (e.g. set “detector_go” flag or put frame_id in a queue) if it’s the K-th frame.

3. **Tracker thread:**  
   - Wait for “new frame” (or run at fixed 30 FPS and read whatever is in `frame_latest`).  
   - Run `tracker.update(frame_latest)`; write result to `bbox_tracker` and valid flag.  
   - If main thread set “re_init_bbox” from detector, re-init OpenCV tracker with that bbox.

4. **Detector thread:**  
   - When “detector_go” and `frame_for_detector` is ready, run TFLite on `frame_for_detector`.  
   - Write best “cat” bbox to `bbox_detector` and valid flag.  
   - Main or tracker uses this to re-init tracker or to confirm “cat_found”.

### 4.3 Optional: Detector in Separate Process

If the GIL or CPU contention makes 30 FPS hard:

- **Process 1 (main):** Camera thread + Tracker thread + Main thread (as above).  
- **Process 2 (detector):** Only TFLite. Receives frame via **shared memory**; sends back bbox via **shared memory** or `multiprocessing.Queue` (small message).  
- **Startup:** Main creates `SharedMemory` for one frame and for bbox; starts detector process; passes shared memory names. Main writes frame into shared buffer every K frames; detector reads, runs TFLite, writes bbox into shared result.  
- **Synchronization:** “Frame ready” can be a shared semaphore or a frame_id (detector only runs when frame_id changes).

---

## 5. Startup Sequence (Memory and Threads)

**Order of operations at application start:**

1. **Load configuration and calibration** (no large alloc).
2. **Pre-allocate all buffers:**
   - `frame_latest = np.zeros((480, 640, 3), dtype=np.uint8)`
   - `frame_for_detector = np.zeros((480, 640, 3), dtype=np.uint8)`
   - `bbox_tracker = multiprocessing.Array('d', 5)` or shared struct (4 float + 1 int)
   - `bbox_detector = multiprocessing.Array('d', 5)`
   - Odometry and state struct (small).
   - Command queue (fixed capacity).
3. **Create shared state object** (hold refs to the above; expose to threads/processes).
4. **Initialize TFLite interpreter once** (and its input buffer if separate).
5. **Start threads (or processes):**
   - Camera thread (receives shared state).
   - Tracker thread (receives shared state + OpenCV tracker).
   - Detector thread (receives shared state; or start detector process with shared memory).
6. **Main loop** runs in main thread; reads shared state and drives state machine + motion.

**No allocation in hot path:** Camera writes into existing array; detector/tracker write into existing bbox arrays; main only reads and calls motion (picar-x calls are external).

---

## 6. Synchronization Summary

| Resource | Writer | Reader | Sync |
|----------|--------|--------|------|
| `frame_latest` | Camera | Tracker, Main (for copy) | Double buffer or lock; keep critical section to “swap index” or “copy into slot”. |
| `frame_for_detector` | Main (copy from latest) | Detector | “Ready” flag or frame_id; detector runs when ready. |
| `bbox_tracker` | Tracker | Main | Lock or atomic write (4 float + 1 int). |
| `bbox_detector` | Detector | Main, Tracker | Lock or atomic write. |
| `odometry_xyh` | Main (or odom thread) | Main | Single writer; no lock if only main reads. |
| Commands | Command poll | Main | Lock-protected queue or atomic flags. |

Use **one lock per logical resource** (e.g. one for “frame”, one for “bboxes”) and keep critical sections minimal (assign values, swap indices, not heavy computation).

---

## 7. Highlights for Further Code Writing

### 7.1 Module: `memory_pool` or `shared_state`

- **Allocate at import or in `main()` before any thread starts:**
  - `frame_latest`, `frame_for_detector` (NumPy).
  - `bbox_tracker`, `bbox_detector` (array of 5 doubles or ctypes struct in `multiprocessing.Array` if cross-process).
  - Locks: `lock_frame`, `lock_bbox_tracker`, `lock_bbox_detector`.
- **API:** `get_frame_latest()`, `set_frame_latest(buf)` (or swap), `get_bbox_tracker()`, `set_bbox_tracker(x,y,w,h,valid)`, same for detector. All use pre-allocated memory; no alloc in these functions.

### 7.2 Camera Thread

- **Loop:** Capture into a **pre-allocated** buffer (picamera2 or vilib gives a frame; copy into `frame_latest` or write into current slot of double buffer). Then release lock or flip index. Do not allocate new arrays per frame.

### 7.3 Tracker Thread

- **Input:** Read `frame_latest` (under lock or from current index).  
- **Output:** Write to `bbox_tracker` (4 floats + valid).  
- **Re-init:** When main sets “re_init_bbox” from detector, call `tracker.init(frame, bbox)` and clear flag. Use one pre-allocated tracker object.

### 7.4 Detector Thread or Process

- **Input:** Read `frame_for_detector` (or shared memory). Resize into TFLite input tensor (use a **fixed pre-allocated** input buffer if possible).  
- **Output:** Write to `bbox_detector` (4 floats + valid).  
- **Run:** Only when main signals “new frame for detector” (every K frames). No per-run allocation for the result.

### 7.5 Main Loop

- **No alloc in loop:** Read from shared state (bbox, odometry, commands); call state machine; call motion. Optionally copy `frame_latest` → `frame_for_detector` in place (reuse buffers).  
- **Odometry:** Update a pre-allocated (x, y, heading) struct from time, speed, steer.

### 7.6 State Machine and Motion

- **State machine:** Pure logic; no large state (enum + a few floats). Can live in main thread.  
- **Motion:** Calls picar-x; no internal buffers beyond what driver needs. Calibration is read-only after load.

### 7.7 Optional: Shared Memory for Detector Process

- **Create at startup:**  
  `shm_frame = shared_memory.SharedMemory(create=True, size=640*480*3)`  
  `shm_bbox = shared_memory.SharedMemory(create=True, size=5*8)`  # 5 doubles  
- **NumPy view:**  
  `frame_buf = np.ndarray((480, 640, 3), dtype=np.uint8, buffer=shm_frame.buf)`  
- **Pass `shm_frame.name` and `shm_bbox.name` to detector process.** Detector attaches with `SharedMemory(name=...)` and reads/writes the same memory.  
- **Sync:** Main writes frame; increments “frame_id” in a small shared int; detector waits for frame_id change, runs, writes bbox, then waits again.

---

## 8. File Layout Suggestion (Reflecting This Architecture)

```
cat_follow/
  memory/
    __init__.py
    pool.py          # Pre-allocate all buffers; expose get/set API with locks
    shared_state.py  # SharedState class holding refs + locks (used by threads)
  threads/
    __init__.py
    camera.py        # Camera thread: capture -> frame_latest
    tracker.py       # Tracker thread: frame_latest -> bbox_tracker
    detector.py      # Detector thread: frame_for_detector -> bbox_detector
  state_machine.py   # unchanged
  motion/            # unchanged
  calibration/       # unchanged
  main_loop.py       # Startup: alloc pool, start threads; loop: read shared state, dispatch, motion
  ...
```

**Startup in `main_loop.py` (pseudocode):**

```python
from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.threads.camera import start_camera_thread
from cat_follow.threads.tracker import start_tracker_thread
from cat_follow.threads.detector import start_detector_thread

def main():
    pool = allocate_pool()       # Pre-allocate frames, bbox arrays, etc.
    shared = SharedState(pool)   # Wrap with locks / accessors
    start_camera_thread(shared)
    start_tracker_thread(shared)
    start_detector_thread(shared)
    # Main loop: shared.get_bbox_tracker(), dispatch, motion
```

---

## 9. Summary

| Goal | Approach |
|------|----------|
| **Pre-alloc at start** | All frame buffers, bbox arrays, odometry/state structs, command queue allocated once in `memory/pool` (or `SharedState` init) before any thread starts. |
| **Shared memory** | Single process: shared NumPy arrays and small arrays (locks per resource). Cross-process: `SharedMemory` for frame + bbox; same sizes and layout. |
| **Parallel design** | Camera thread (writer of frame); Tracker thread (reader of frame, writer of bbox); Detector thread or process (reader of frame copy, writer of bbox); Main thread (reader of bboxes/odometry, state machine, motion). Optional detector process with shared memory for true CPU parallelism. |
| **Hot path** | No allocation in camera, tracker, or main loop; only copy into pre-allocated slots and read/write shared state. |

This gives a clear blueprint for implementing the cat-follow pipeline with predictable memory use and explicit concurrency.
