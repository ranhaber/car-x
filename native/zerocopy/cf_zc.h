// cat_follow zerocopy pipeline — stable C ABI for Python ctypes.
//
// V4L2 EXPBUF -> RGA NV12->RGB crop -> RKNN create_mem_from_fd inference.
// Build on ROCK 4D only; x86 CI skips runtime probes.

#ifndef CF_ZC_H_
#define CF_ZC_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CfZcSession CfZcSession;

typedef struct {
  int cam_fd;
  int crop_rgb_fd;
  uint32_t frame_seq;
  uint32_t buffer_index;
  uint32_t image_size;
  uint32_t src_width;
  uint32_t src_height;
  uint32_t stride;
} CfZcFrame;

typedef struct {
  float x1;
  float y1;
  float x2;
  float y2;
  float score;
  int32_t class_id;
} CfZcDetection;

typedef struct {
  int ok;
  double rga_ms;
  double npu_ms;
  double post_ms;
  uint32_t frame_seq;
  int num_detections;
} CfZcProcessResult;

// Returns 1 when /dev/rga exists (quick probe, no session alloc).
int cf_zc_runtime_available(void);

// Open camera + DMA-heap RGB crop + RKNN fd input. crop_x/y may be -1 to
// select center-bottom placement. Source/crop dimensions and the resolved NV12
// crop origin must be even, the crop must be in bounds, and crop dimensions
// must match the model input dimensions.
CfZcSession* cf_zc_open(const char* device, const char* model_path,
                        int src_w, int src_h, int crop_w, int crop_h,
                        int crop_x, int crop_y);

void cf_zc_close(CfZcSession* session);

// RKNN/model lifecycle. These calls preserve the V4L2 stream, exported camera
// DMA-BUFs, RGB crop DMA-BUF, and RGA imports. All lifecycle operations are
// serialized with frame ownership and inference.
// cf_zc_model_loaded returns 1 when loaded, 0 when unloaded, and -1 on error.
int cf_zc_model_load(CfZcSession* session);
int cf_zc_model_unload(CfZcSession* session);
int cf_zc_model_loaded(CfZcSession* session);

// Dequeue one camera buffer (poll outside camera lock; DQBUF inside).
// Returns 0 on success; cam_fd remains valid until cf_zc_requeue().
int cf_zc_dequeue(CfZcSession* session, CfZcFrame* out, int timeout_ms);

// Requeue a previously dequeued buffer index. The implementation serializes
// requeue with inference/copy ownership, so a concurrent requeue waits until
// the active operation finishes. Callers must not use a buffer after requeue.
int cf_zc_requeue(CfZcSession* session, uint32_t buffer_index);

// Copy NV12 bytes from a dequeued buffer into dst (CPU pack for inject only).
int cf_zc_copy_camera_nv12(CfZcSession* session, uint32_t buffer_index,
                           void* dst, size_t dst_size);

// RGA crop + RKNN run + native INT8 YOLO decode/NMS + copy detections.
// out may be NULL when max_out <= 0 (timing-only / validator path).
// A dequeued buffer remains caller-owned through this call; requeue is
// serialized and may occur only after inference has relinquished ownership.
int cf_zc_infer_detections(CfZcSession* session, uint32_t buffer_index,
                           float score_threshold, int animal_mode,
                           CfZcDetection* out, int max_out,
                           CfZcProcessResult* result);

// Legacy timing-only wrapper (no detection copy-out).
int cf_zc_infer(CfZcSession* session, uint32_t buffer_index,
                float score_threshold, int animal_mode,
                CfZcProcessResult* result);

int cf_zc_detection_count(CfZcSession* session);
int cf_zc_copy_detections(CfZcSession* session, CfZcDetection* out,
                          int max_out);

// Crop offset in source pixels (for mapping detections back to full frame).
int cf_zc_crop_offset(CfZcSession* session, int* out_x, int* out_y);

// Last error string (static buffer, do not free).
const char* cf_zc_last_error(void);

// Standalone validator: dequeue/infer/requeue for N frames, print JSON stats.
int cf_zc_validate_run(const char* device, const char* model_path, int frames);

#ifdef __cplusplus
}
#endif

#endif  // CF_ZC_H_
