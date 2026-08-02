## Board Checklist: Chase Recording (ROCK 4D)

**Scope:** verify the hardware chase-recording path that cannot be exercised on
a development host. Everything below runs on the ROCK 4D over
`ssh picarx@192.168.7.67`.

Host CI already covers the ownership and segmentation contract with a mocked
GStreamer (`tests/test_recording_encoder.py`). This checklist covers what only
real hardware can prove: that MPP accepts a second encoder instance, that the
files are valid Matroska, and that recording does not disturb detection.

### 1. Preconditions

- [ ] `gst-inspect-1.0 mpph264enc` and `gst-inspect-1.0 matroskamux` both
  resolve. If either is missing, recording is expected to report
  `encoder_unavailable` rather than write files.
- [ ] `/dev/mpp_service` exists and is readable by the service user.
- [ ] `CAT_FOLLOW_RECORDING_ALLOW_STUB` is **not** set in
  `cat_follow/scripts/car-x.env`. A stub on the board would produce unplayable
  files that still report healthy.
- [ ] `CAT_FOLLOW_RECORDING_QUOTA_BYTES` and
  `CAT_FOLLOW_RECORDING_MIN_FREE_BYTES` are set (defaults: 8 GiB and 1 GiB).
- [ ] `df -h` shows free space above the configured reserve before starting.

### 2. Encoder availability and fail-closed behaviour

- [ ] Start the service and trigger a chase. Confirm the log line
  `MPP H.264 encoder started (NV12 ...)` appears a second time for recording,
  distinct from the monitoring stream instance.
- [ ] Temporarily rename `mpph264enc` out of the GStreamer registry (or set an
  invalid `GST_PLUGIN_PATH`), restart, and confirm the recording status reports
  `degraded_reason=encoder_unavailable`, that `capture_active` drops when
  recording is the only camera consumer, and that **no** `.mkv` files appear.
- [ ] Restore the plugin path and confirm recording recovers on the next chase
  without a service restart.

### 3. File validity

- [ ] After a chase, run `ffprobe <segment>.mkv` on a finalized file. Confirm
  the codec is `h264`, resolution matches the camera, and the duration is
  non-zero.
- [ ] Play a finalized segment end to end and confirm it is real camera video,
  not synthetic frames or a frozen image.
- [ ] Force a rotation (set `CAT_FOLLOW_RECORDING_SEGMENT_SEC=10`) and confirm
  **each** rotated segment probes and plays independently. A segment that
  reuses the previous muxer's headers will fail `ffprobe`.
- [ ] Kill the service mid-segment (`systemctl kill -s SIGKILL`), restart, and
  confirm the `.mkv.part` is recovered into a finalized segment at startup and
  that `ffprobe` reports the recovered portion.

### 4. Frame-ring and detection interference

This is the main risk of adding a third frame consumer to a four-slot ring.

- [ ] With recording active, confirm detection FPS in the perception
  diagnostics stays within 10% of its no-recording baseline.
- [ ] Confirm the camera log shows no sustained increase in dropped frames
  ("no free frame-ring write slot" path) while recording.
- [ ] Confirm `pending_dmabuf_requeues` stays empty: any entry means a QBUF
  failed and a V4L2 buffer is still held by the ring.
- [ ] Run a chase with the web UI stream **also** open, so detector, stream,
  and recording all hold leases. Confirm detection continues and no
  `DmabufRequeueError` appears.

### 5. Control-loop isolation

- [ ] Confirm the recording I/O thread `CatFollow-Recording` exists
  (`ps -T -p $(pgrep -f cat_follow)`).
- [ ] Confirm control-loop tick timing while recording stays within its budget;
  a regression here means disk I/O leaked back onto the control thread.
- [ ] Simulate a slow card (`echo 3 > /proc/sys/vm/drop_caches` then record to a
  busy filesystem) and confirm motor command latency is unaffected while
  `bytes_written` continues to advance.

### 6. Retention and disk pressure

- [ ] Set a small quota (e.g. `CAT_FOLLOW_RECORDING_QUOTA_BYTES=52428800`) and
  confirm the oldest finalized segments are deleted while recording continues.
- [ ] Fill the card until free space drops below the reserve and confirm
  recording reports `low_space`, stops writing, and resumes automatically once
  space is freed.

### 7. Thermal and duration

- [ ] Record continuously for 30 minutes. Confirm no encoder restarts, no
  growth in RSS, and SoC temperature stays within the thermal policy.
