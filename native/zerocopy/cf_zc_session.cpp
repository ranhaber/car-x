#include <unistd.h>

#include "cf_zc.h"
#include "cf_zc_internal.hpp"
#include "cf_zc_yolo_post.hpp"

#include <fcntl.h>
#include <dirent.h>
#include <linux/dma-heap.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <rga/im2d.hpp>
#include <rknn_api.h>

#include <atomic>
#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

namespace cf_zc {

namespace {

constexpr uint32_t kDefaultBufferCount = 6;

struct CameraBuffer {
  int fd = -1;
  void* mapped = MAP_FAILED;
  rga_buffer_handle_t rga_handle = 0;
  rga_buffer_t rga_buffer{};
};

enum class BufferState { kQueued, kDequeued };

class V4l2DmabufCamera {
 public:
  V4l2DmabufCamera(const std::string& device, uint32_t width, uint32_t height,
                   uint32_t buffer_count = kDefaultBufferCount)
      : width_(width), height_(height) {
    fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("open camera: " +
                               std::string(std::strerror(errno)));
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    format.fmt.pix_mp.width = width_;
    format.fmt.pix_mp.height = height_;
    format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    format.fmt.pix_mp.field = V4L2_FIELD_NONE;
    format.fmt.pix_mp.num_planes = 1;
    checked_ioctl(fd_, VIDIOC_S_FMT, &format, "VIDIOC_S_FMT");
    if (format.fmt.pix_mp.width != width_ ||
        format.fmt.pix_mp.height != height_ ||
        format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12 ||
        format.fmt.pix_mp.num_planes != 1) {
      throw std::runtime_error("camera did not accept requested NV12 format");
    }
    image_size_ = format.fmt.pix_mp.plane_fmt[0].sizeimage;
    stride_ = format.fmt.pix_mp.plane_fmt[0].bytesperline;

    v4l2_requestbuffers request{};
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    request.memory = V4L2_MEMORY_MMAP;
    request.count = buffer_count;
    checked_ioctl(fd_, VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS");
    if (request.count < buffer_count) {
      throw std::runtime_error(
          "camera allocated fewer than requested V4L2 buffers: got " +
          std::to_string(request.count) + ", need " +
          std::to_string(buffer_count));
    }
    if (request.count < 2) {
      throw std::runtime_error("camera allocated fewer than two buffers");
    }

    buffers_.resize(request.count);
    buffer_states_.assign(request.count, BufferState::kQueued);
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_plane plane{};
      v4l2_buffer buffer{};
      buffer.type = request.type;
      buffer.memory = request.memory;
      buffer.index = index;
      buffer.length = 1;
      buffer.m.planes = &plane;
      checked_ioctl(fd_, VIDIOC_QUERYBUF, &buffer, "VIDIOC_QUERYBUF");

      auto& item = buffers_[index];
      item.mapped = mmap(nullptr, image_size_, PROT_READ, MAP_SHARED, fd_,
                         plane.m.mem_offset);
      if (item.mapped == MAP_FAILED) {
        throw std::runtime_error("mmap camera buffer: " +
                                 std::string(std::strerror(errno)));
      }

      v4l2_exportbuffer export_buffer{};
      export_buffer.type = request.type;
      export_buffer.index = index;
      export_buffer.plane = 0;
      export_buffer.flags = O_RDWR | O_CLOEXEC;
      checked_ioctl(fd_, VIDIOC_EXPBUF, &export_buffer, "VIDIOC_EXPBUF");

      item.fd = export_buffer.fd;
      item.rga_handle = importbuffer_fd(item.fd, static_cast<int>(image_size_));
      if (!item.rga_handle) {
        throw std::runtime_error("RGA failed to import camera DMA-BUF");
      }
      item.rga_buffer = wrapbuffer_handle(
          item.rga_handle, width_, height_, RK_FORMAT_YCbCr_420_SP);

      checked_ioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF");
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    checked_ioctl(fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
    streaming_ = true;
  }

  ~V4l2DmabufCamera() {
    if (streaming_) {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      ioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (auto& buffer : buffers_) {
      if (buffer.rga_handle) releasebuffer_handle(buffer.rga_handle);
      if (buffer.mapped != MAP_FAILED) munmap(buffer.mapped, image_size_);
      if (buffer.fd >= 0) close(buffer.fd);
    }
    if (fd_ >= 0) close(fd_);
  }

  uint32_t dequeue(int timeout_ms) {
    pollfd poll_fd{fd_, POLLIN, 0};
    const int poll_result = poll(&poll_fd, 1, timeout_ms);
    if (poll_result <= 0) {
      throw std::runtime_error(poll_result == 0 ? "camera timeout"
                                                : std::strerror(errno));
    }

    std::lock_guard<std::mutex> lock(camera_mu_);
    v4l2_plane plane{};
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.length = 1;
    buffer.m.planes = &plane;
    checked_ioctl(fd_, VIDIOC_DQBUF, &buffer, "VIDIOC_DQBUF");
    const uint32_t index = buffer.index;
    if (index >= buffer_states_.size()) {
      throw std::runtime_error("DQBUF returned invalid buffer index");
    }
    if (buffer_states_[index] != BufferState::kQueued) {
      throw std::runtime_error("DQBUF returned buffer that was not queued");
    }
    buffer_states_[index] = BufferState::kDequeued;
    return index;
  }

  void requeue(uint32_t index) {
    std::lock_guard<std::mutex> lock(camera_mu_);
    if (index >= buffer_states_.size()) {
      throw std::runtime_error("requeue: invalid buffer index");
    }
    if (buffer_states_[index] != BufferState::kDequeued) {
      throw std::runtime_error("requeue: buffer is not dequeued");
    }
    v4l2_plane plane{};
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    buffer.length = 1;
    buffer.m.planes = &plane;
    checked_ioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF");
    buffer_states_[index] = BufferState::kQueued;
  }

  void require_dequeued(uint32_t index) const {
    std::lock_guard<std::mutex> lock(camera_mu_);
    if (index >= buffer_states_.size()) {
      throw std::runtime_error("invalid buffer index");
    }
    if (buffer_states_[index] != BufferState::kDequeued) {
      throw std::runtime_error("buffer is not dequeued");
    }
  }

  const rga_buffer_t& rga_buffer(uint32_t index) const {
    return buffers_.at(index).rga_buffer;
  }

  int buffer_fd(uint32_t index) const { return buffers_.at(index).fd; }

  void copy_nv12(uint32_t index, void* dst, size_t dst_size) const {
    require_dequeued(index);
    if (dst_size < image_size_) {
      throw std::runtime_error("NV12 destination too small");
    }
    std::memcpy(dst, buffers_.at(index).mapped, image_size_);
  }

  uint32_t image_size() const { return image_size_; }
  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  uint32_t stride() const { return stride_; }

 private:
  int fd_ = -1;
  bool streaming_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t image_size_ = 0;
  uint32_t stride_ = 0;
  mutable std::mutex camera_mu_;
  std::vector<CameraBuffer> buffers_;
  std::vector<BufferState> buffer_states_;
};

class DmaHeapBuffer {
 public:
  explicit DmaHeapBuffer(size_t size) : size_(size) {
    int heap_fd = open("/dev/dma_heap/system-uncached", O_RDWR | O_CLOEXEC);
    if (heap_fd < 0) {
      throw std::runtime_error("open system-uncached dma_heap: " +
                               std::string(std::strerror(errno)));
    }
    dma_heap_allocation_data allocation{};
    allocation.len = size_;
    allocation.fd_flags = O_RDWR | O_CLOEXEC;
    if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &allocation) < 0) {
      const std::string error = std::strerror(errno);
      close(heap_fd);
      throw std::runtime_error("DMA_HEAP_IOCTL_ALLOC: " + error);
    }
    close(heap_fd);
    fd_ = static_cast<int>(allocation.fd);
    address_ =
        mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (address_ == MAP_FAILED) {
      address_ = nullptr;
      const std::string error = std::strerror(errno);
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("mmap crop DMA-BUF: " + error);
    }
  }

  ~DmaHeapBuffer() {
    if (address_) munmap(address_, size_);
    if (fd_ >= 0) close(fd_);
  }

  int fd() const { return fd_; }
  size_t size() const { return size_; }
  void* address() const { return address_; }

 private:
  int fd_ = -1;
  size_t size_ = 0;
  void* address_ = nullptr;
};

inline void tensor_hw(const rknn_tensor_attr& attr, int* height, int* width) {
  if (attr.n_dims < 4) {
    throw std::runtime_error("YOLO output tensor must be 4-D");
  }
  if (attr.fmt == RKNN_TENSOR_NHWC) {
    *height = static_cast<int>(attr.dims[1]);
    *width = static_cast<int>(attr.dims[2]);
  } else {
    // NCHW (and unknown) — model-zoo head is NCHW.
    *height = static_cast<int>(attr.dims[2]);
    *width = static_cast<int>(attr.dims[3]);
  }
}

class RknnFdSession {
 public:
  RknnFdSession(const std::string& model, int input_fd, void* input_address,
                size_t input_size, int expected_input_w,
                int expected_input_h) {
    try {
      int result = rknn_init(&context_, const_cast<char*>(model.c_str()), 0, 0,
                             nullptr);
      if (result < 0) {
        throw std::runtime_error("rknn_init failed: " + std::to_string(result));
      }

      rknn_input_output_num io_count{};
      result = rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_count,
                          sizeof(io_count));
      if (result < 0 || io_count.n_input != 1) {
        throw std::runtime_error("unexpected RKNN input count");
      }
      if (io_count.n_output != 9) {
        throw std::runtime_error("expected 9 YOLO output tensors, got " +
                                 std::to_string(io_count.n_output));
      }
      output_count_ = io_count.n_output;
      output_attrs_.resize(output_count_);
      for (uint32_t index = 0; index < output_count_; ++index) {
        output_attrs_[index].index = index;
        result = rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR,
                            &output_attrs_[index], sizeof(rknn_tensor_attr));
        if (result < 0) {
          throw std::runtime_error("RKNN_QUERY_OUTPUT_ATTR failed");
        }
        if (output_attrs_[index].type != RKNN_TENSOR_INT8) {
          throw std::runtime_error("YOLO outputs must be INT8 for native post");
        }
      }

      input_attr_.index = 0;
      result = rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input_attr_,
                          sizeof(input_attr_));
      if (result < 0) {
        throw std::runtime_error("RKNN_QUERY_INPUT_ATTR failed");
      }

      if (input_attr_.n_dims >= 4) {
        if (input_attr_.fmt == RKNN_TENSOR_NHWC) {
          input_h_ = static_cast<int>(input_attr_.dims[1]);
          input_w_ = static_cast<int>(input_attr_.dims[2]);
        } else {
          input_h_ = static_cast<int>(input_attr_.dims[2]);
          input_w_ = static_cast<int>(input_attr_.dims[3]);
        }
      }
      if (input_w_ <= 0 || input_h_ <= 0) {
        throw std::runtime_error("invalid RKNN input spatial dims");
      }
      if (input_w_ != expected_input_w || input_h_ != expected_input_h) {
        throw std::runtime_error(
            "crop dimensions do not match RKNN input dimensions");
      }

      input_attr_.type = RKNN_TENSOR_UINT8;
      input_attr_.fmt = RKNN_TENSOR_NHWC;
      input_attr_.pass_through = 0;
      if (input_size < input_attr_.size_with_stride) {
        throw std::runtime_error("DMA-BUF is smaller than RKNN input stride");
      }

      input_mem_ =
          rknn_create_mem_from_fd(context_, input_fd, input_address,
                                  static_cast<uint32_t>(input_size), 0);
      if (!input_mem_) {
        throw std::runtime_error("rknn_create_mem_from_fd failed");
      }
      result = rknn_set_io_mem(context_, input_mem_, &input_attr_);
      if (result < 0) {
        throw std::runtime_error("rknn_set_io_mem failed: " +
                                 std::to_string(result));
      }
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~RknnFdSession() { cleanup(); }

  void cleanup() {
    if (input_mem_) rknn_destroy_mem(context_, input_mem_);
    if (context_) rknn_destroy(context_);
    input_mem_ = nullptr;
    context_ = 0;
  }

  // Runs NPU + INT8 postprocess. Returns postprocess wall time in ms.
  double run(float score_threshold, bool animal_mode) {
    int result = rknn_run(context_, nullptr);
    if (result < 0) {
      throw std::runtime_error("rknn_run failed: " + std::to_string(result));
    }

    std::vector<rknn_output> outputs(output_count_);
    for (uint32_t index = 0; index < output_count_; ++index) {
      outputs[index].index = index;
      outputs[index].want_float = 0;
      outputs[index].is_prealloc = 0;
    }
    result = rknn_outputs_get(context_, output_count_, outputs.data(), nullptr);
    if (result < 0) {
      throw std::runtime_error("rknn_outputs_get failed: " +
                               std::to_string(result));
    }

    const int8_t* tensors[9];
    float scales[9];
    int32_t zero_points[9];
    int heights[9];
    int widths[9];
    for (uint32_t index = 0; index < output_count_; ++index) {
      if (outputs[index].buf == nullptr) {
        rknn_outputs_release(context_, output_count_, outputs.data());
        throw std::runtime_error("RKNN output buffer is null");
      }
      tensors[index] = static_cast<const int8_t*>(outputs[index].buf);
      scales[index] = output_attrs_[index].scale;
      zero_points[index] = output_attrs_[index].zp;
      tensor_hw(output_attrs_[index], &heights[index], &widths[index]);
    }

    const auto post_start = std::chrono::steady_clock::now();
    detections_ = decode_yolov8_int8(tensors, scales, zero_points, heights,
                                     widths, input_w_, input_h_, score_threshold,
                                     animal_mode);
    const double post_ms = elapsed_ms(post_start);
    rknn_outputs_release(context_, output_count_, outputs.data());
    return post_ms;
  }

  const std::vector<Detection>& detections() const { return detections_; }

 private:
  rknn_context context_ = 0;
  rknn_tensor_attr input_attr_{};
  rknn_tensor_mem* input_mem_ = nullptr;
  uint32_t output_count_ = 0;
  std::vector<rknn_tensor_attr> output_attrs_;
  std::vector<Detection> detections_;
  int input_w_ = 0;
  int input_h_ = 0;
};

struct SessionImpl {
  ~SessionImpl() {
    if (crop_handle) releasebuffer_handle(crop_handle);
  }

  std::unique_ptr<V4l2DmabufCamera> camera;
  std::unique_ptr<DmaHeapBuffer> crop_buffer;
  rga_buffer_handle_t crop_handle = 0;
  rga_buffer_t crop_rga{};
  im_rect crop_rect{};
  std::unique_ptr<RknnFdSession> npu;
  std::string model_path;
  int model_input_w = 0;
  int model_input_h = 0;
  std::atomic<uint32_t> frame_seq{0};
  // Serializes access to dequeued camera-buffer ownership. In particular,
  // requeue must never race RGA/RKNN inference (or a diagnostic CPU copy).
  std::mutex buffer_ownership_mu;
  std::mutex infer_mu;
  int crop_x = 0;
  int crop_y = 0;

  void load_model() {
    if (npu) return;
    try {
      npu = std::make_unique<RknnFdSession>(
          model_path, crop_buffer->fd(), crop_buffer->address(),
          crop_buffer->size(), model_input_w, model_input_h);
    } catch (const std::exception& error) {
      throw std::runtime_error("RKNN model reload failed for '" + model_path +
                               "': " + error.what());
    }
  }
};

}  // namespace

thread_local std::string g_last_error;

static void set_error(const std::string& message) { g_last_error = message; }

static im_rect make_crop_rect(int src_w, int src_h, int crop_w, int crop_h,
                              int crop_x, int crop_y) {
  im_rect rect{};
  if (crop_x < 0) {
    rect.x = (src_w - crop_w) / 2;
  } else {
    rect.x = crop_x;
  }
  if (crop_y < 0) {
    rect.y = src_h - crop_h;
  } else {
    rect.y = crop_y;
  }
  rect.width = crop_w;
  rect.height = crop_h;
  return rect;
}

static im_rect validate_geometry(int src_w, int src_h, int crop_w, int crop_h,
                                 int crop_x, int crop_y) {
  if (src_w <= 0 || src_h <= 0 || crop_w <= 0 || crop_h <= 0) {
    throw std::invalid_argument("source and crop dimensions must be positive");
  }
  if ((src_w & 1) || (src_h & 1) || (crop_w & 1) || (crop_h & 1)) {
    throw std::invalid_argument(
        "NV12 source and crop dimensions must be even");
  }
  if (crop_w > src_w || crop_h > src_h) {
    throw std::invalid_argument("crop dimensions exceed source bounds");
  }
  const im_rect rect =
      make_crop_rect(src_w, src_h, crop_w, crop_h, crop_x, crop_y);
  if (rect.x < 0 || rect.y < 0 || rect.x > src_w - crop_w ||
      rect.y > src_h - crop_h) {
    throw std::invalid_argument("crop rectangle exceeds source bounds");
  }
  if ((rect.x & 1) || (rect.y & 1)) {
    throw std::invalid_argument("NV12 crop origin must be even-aligned");
  }
  const size_t pixels =
      static_cast<size_t>(crop_w) * static_cast<size_t>(crop_h);
  if (pixels > std::numeric_limits<size_t>::max() / 3 ||
      pixels * 3 > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("crop buffer size is unsupported");
  }
  return rect;
}

}  // namespace cf_zc

struct CfZcSession {
  cf_zc::SessionImpl impl;
};

extern "C" {

int cf_zc_runtime_available(void) {
  return access("/dev/rga", R_OK | W_OK) == 0 ? 1 : 0;
}

const char* cf_zc_last_error(void) { return cf_zc::g_last_error.c_str(); }

CfZcSession* cf_zc_open(const char* device, const char* model_path, int src_w,
                        int src_h, int crop_w, int crop_h, int crop_x,
                        int crop_y) {
  try {
    if (device == nullptr || model_path == nullptr || device[0] == '\0' ||
        model_path[0] == '\0') {
      throw std::invalid_argument("device and model_path must be non-empty");
    }
    const im_rect crop_rect = cf_zc::validate_geometry(
        src_w, src_h, crop_w, crop_h, crop_x, crop_y);
    auto session = std::make_unique<CfZcSession>();
    session->impl.camera = std::make_unique<cf_zc::V4l2DmabufCamera>(
        device, static_cast<uint32_t>(src_w), static_cast<uint32_t>(src_h));
    const size_t rgb_bytes =
        static_cast<size_t>(crop_w) * static_cast<size_t>(crop_h) * 3;
    session->impl.crop_buffer =
        std::make_unique<cf_zc::DmaHeapBuffer>(rgb_bytes);
    session->impl.crop_handle = importbuffer_fd(
        session->impl.crop_buffer->fd(),
        static_cast<int>(session->impl.crop_buffer->size()));
    if (!session->impl.crop_handle) {
      throw std::runtime_error("RGA crop fd import failed");
    }
    session->impl.crop_rga = wrapbuffer_handle(
        session->impl.crop_handle, crop_w, crop_h, RK_FORMAT_RGB_888);
    session->impl.crop_rect = crop_rect;
    session->impl.crop_x = session->impl.crop_rect.x;
    session->impl.crop_y = session->impl.crop_rect.y;
    session->impl.model_path = model_path;
    session->impl.model_input_w = crop_w;
    session->impl.model_input_h = crop_h;
    session->impl.load_model();
    return session.release();
  } catch (const std::exception& error) {
    cf_zc::set_error(error.what());
    return nullptr;
  }
}

void cf_zc_close(CfZcSession* session) {
  if (session == nullptr) return;
  delete session;
}

int cf_zc_model_load(CfZcSession* session) {
  if (session == nullptr) {
    cf_zc::set_error("cf_zc_model_load: null session");
    return -1;
  }
  try {
    std::lock_guard<std::mutex> ownership_lock(
        session->impl.buffer_ownership_mu);
    std::lock_guard<std::mutex> infer_lock(session->impl.infer_mu);
    session->impl.load_model();
    return 0;
  } catch (const std::exception& error) {
    cf_zc::set_error(std::string("cf_zc_model_load: ") + error.what());
    return -1;
  }
}

int cf_zc_model_unload(CfZcSession* session) {
  if (session == nullptr) {
    cf_zc::set_error("cf_zc_model_unload: null session");
    return -1;
  }
  try {
    std::lock_guard<std::mutex> ownership_lock(
        session->impl.buffer_ownership_mu);
    std::lock_guard<std::mutex> infer_lock(session->impl.infer_mu);
    session->impl.npu.reset();
    return 0;
  } catch (const std::exception& error) {
    cf_zc::set_error(std::string("cf_zc_model_unload: ") + error.what());
    return -1;
  }
}

int cf_zc_model_loaded(CfZcSession* session) {
  if (session == nullptr) {
    cf_zc::set_error("cf_zc_model_loaded: null session");
    return -1;
  }
  try {
    std::lock_guard<std::mutex> ownership_lock(
        session->impl.buffer_ownership_mu);
    std::lock_guard<std::mutex> infer_lock(session->impl.infer_mu);
    return session->impl.npu ? 1 : 0;
  } catch (const std::exception& error) {
    cf_zc::set_error(std::string("cf_zc_model_loaded: ") + error.what());
    return -1;
  }
}

int cf_zc_dequeue(CfZcSession* session, CfZcFrame* out, int timeout_ms) {
  if (session == nullptr || out == nullptr) {
    cf_zc::set_error("cf_zc_dequeue: null argument");
    return -1;
  }
  try {
    const uint32_t index =
        session->impl.camera->dequeue(timeout_ms > 0 ? timeout_ms : 3000);
    const uint32_t seq = session->impl.frame_seq.fetch_add(1) + 1;
    out->cam_fd = session->impl.camera->buffer_fd(index);
    out->crop_rgb_fd = session->impl.crop_buffer->fd();
    out->frame_seq = seq;
    out->buffer_index = index;
    out->image_size = session->impl.camera->image_size();
    out->src_width = session->impl.camera->width();
    out->src_height = session->impl.camera->height();
    out->stride = session->impl.camera->stride();
    return 0;
  } catch (const std::exception& error) {
    cf_zc::set_error(error.what());
    return -1;
  }
}

int cf_zc_requeue(CfZcSession* session, uint32_t buffer_index) {
  if (session == nullptr) {
    cf_zc::set_error("cf_zc_requeue: null session");
    return -1;
  }
  try {
    std::lock_guard<std::mutex> ownership_lock(
        session->impl.buffer_ownership_mu);
    session->impl.camera->requeue(buffer_index);
    return 0;
  } catch (const std::exception& error) {
    cf_zc::set_error(error.what());
    return -1;
  }
}

int cf_zc_copy_camera_nv12(CfZcSession* session, uint32_t buffer_index,
                           void* dst, size_t dst_size) {
  if (session == nullptr || dst == nullptr) {
    cf_zc::set_error("cf_zc_copy_camera_nv12: null argument");
    return -1;
  }
  try {
    std::lock_guard<std::mutex> ownership_lock(
        session->impl.buffer_ownership_mu);
    session->impl.camera->copy_nv12(buffer_index, dst, dst_size);
    return 0;
  } catch (const std::exception& error) {
    cf_zc::set_error(error.what());
    return -1;
  }
}

int cf_zc_infer_detections(CfZcSession* session, uint32_t buffer_index,
                           float score_threshold, int animal_mode,
                           CfZcDetection* out, int max_out,
                           CfZcProcessResult* result) {
  if (session == nullptr || result == nullptr) {
    cf_zc::set_error("cf_zc_infer_detections: null argument");
    return -1;
  }
  if ((out == nullptr) != (max_out <= 0)) {
    cf_zc::set_error("cf_zc_infer_detections: out/max_out mismatch");
    return -1;
  }
  try {
    std::lock_guard<std::mutex> ownership_lock(
        session->impl.buffer_ownership_mu);
    session->impl.camera->require_dequeued(buffer_index);
    std::lock_guard<std::mutex> infer_lock(session->impl.infer_mu);
    session->impl.load_model();

    auto start = std::chrono::steady_clock::now();
    const IM_STATUS crop_status = imcrop(
        session->impl.camera->rga_buffer(buffer_index),
        session->impl.crop_rga, session->impl.crop_rect);
    const double rga_ms = cf_zc::elapsed_ms(start);
    if (crop_status != IM_STATUS_SUCCESS) {
      cf_zc::set_error(std::string("imcrop failed: ") +
                       imStrError(crop_status));
      return -1;
    }
    start = std::chrono::steady_clock::now();
    const double post_ms =
        session->impl.npu->run(score_threshold, animal_mode != 0);
    const double npu_and_post_ms = cf_zc::elapsed_ms(start);
    const double npu_ms = std::max(0.0, npu_and_post_ms - post_ms);

    const auto& detections = session->impl.npu->detections();
    int copied = 0;
    if (out != nullptr && max_out > 0) {
      copied = std::min(max_out, static_cast<int>(detections.size()));
      for (int index = 0; index < copied; ++index) {
        out[index].x1 = detections[static_cast<size_t>(index)].x1;
        out[index].y1 = detections[static_cast<size_t>(index)].y1;
        out[index].x2 = detections[static_cast<size_t>(index)].x2;
        out[index].y2 = detections[static_cast<size_t>(index)].y2;
        out[index].score = detections[static_cast<size_t>(index)].score;
        out[index].class_id = detections[static_cast<size_t>(index)].class_id;
      }
    }

    result->ok = 1;
    result->rga_ms = rga_ms;
    result->npu_ms = npu_ms;
    result->post_ms = post_ms;
    result->frame_seq = session->impl.frame_seq.load();
    result->num_detections = static_cast<int>(detections.size());
    if (out != nullptr && max_out > 0) {
      result->num_detections = copied;
    }
    return 0;
  } catch (const std::exception& error) {
    cf_zc::set_error(error.what());
    return -1;
  }
}

int cf_zc_infer(CfZcSession* session, uint32_t buffer_index,
                float score_threshold, int animal_mode,
                CfZcProcessResult* result) {
  return cf_zc_infer_detections(session, buffer_index, score_threshold,
                                animal_mode, nullptr, 0, result);
}

int cf_zc_detection_count(CfZcSession* session) {
  if (session == nullptr) return 0;
  std::lock_guard<std::mutex> infer_lock(session->impl.infer_mu);
  if (!session->impl.npu) return 0;
  return static_cast<int>(session->impl.npu->detections().size());
}

int cf_zc_copy_detections(CfZcSession* session, CfZcDetection* out,
                          int max_out) {
  if (session == nullptr || out == nullptr || max_out < 0) {
    cf_zc::set_error("cf_zc_copy_detections: invalid argument");
    return -1;
  }
  std::lock_guard<std::mutex> infer_lock(session->impl.infer_mu);
  if (!session->impl.npu) {
    cf_zc::set_error("cf_zc_copy_detections: RKNN model is unloaded");
    return -1;
  }
  const auto& detections = session->impl.npu->detections();
  const int count =
      std::min(max_out, static_cast<int>(detections.size()));
  for (int index = 0; index < count; ++index) {
    out[index].x1 = detections[static_cast<size_t>(index)].x1;
    out[index].y1 = detections[static_cast<size_t>(index)].y1;
    out[index].x2 = detections[static_cast<size_t>(index)].x2;
    out[index].y2 = detections[static_cast<size_t>(index)].y2;
    out[index].score = detections[static_cast<size_t>(index)].score;
    out[index].class_id = detections[static_cast<size_t>(index)].class_id;
  }
  return count;
}

int cf_zc_crop_offset(CfZcSession* session, int* out_x, int* out_y) {
  if (session == nullptr || out_x == nullptr || out_y == nullptr) {
    cf_zc::set_error("cf_zc_crop_offset: null argument");
    return -1;
  }
  *out_x = session->impl.crop_x;
  *out_y = session->impl.crop_y;
  return 0;
}

int cf_zc_validate_run(const char* device, const char* model_path,
                       int frames) {
  if (frames < 1) frames = 30;
  CfZcSession* session = cf_zc_open(device, model_path, 640, 480, 320, 320, -1,
                                    -1);
  if (session == nullptr) {
    std::cout << "{\"status\":\"fail\",\"error\":\"" << cf_zc_last_error()
              << "\"}" << std::endl;
    return 1;
  }

  const int fds_before = cf_zc::open_fd_count();
  std::vector<double> rga_times;
  std::vector<double> npu_times;
  std::vector<double> post_times;
  rga_times.reserve(static_cast<size_t>(frames));
  npu_times.reserve(static_cast<size_t>(frames));
  post_times.reserve(static_cast<size_t>(frames));

  CfZcFrame frame{};
  CfZcProcessResult result{};
  for (int index = 0; index < frames; ++index) {
    if (cf_zc_dequeue(session, &frame, 3000) != 0) {
      std::cout << "{\"status\":\"fail\",\"error\":\"" << cf_zc_last_error()
                << "\"}" << std::endl;
      cf_zc_close(session);
      return 1;
    }
    if (cf_zc_infer(session, frame.buffer_index, 0.5f, 0, &result) != 0) {
      std::cout << "{\"status\":\"fail\",\"error\":\"" << cf_zc_last_error()
                << "\"}" << std::endl;
      cf_zc_requeue(session, frame.buffer_index);
      cf_zc_close(session);
      return 1;
    }
    rga_times.push_back(result.rga_ms);
    npu_times.push_back(result.npu_ms);
    post_times.push_back(result.post_ms);
    cf_zc_requeue(session, frame.buffer_index);
  }

  const int fds_after = cf_zc::open_fd_count();
  cf_zc_close(session);
  std::cout << "{\"status\":\"pass\",\"path\":\"v4l2-expbuf->rga-fd->"
               "rknn-fd\",\"camera_cpu_mapped\":true,"
               "\"crop_va_mapped\":true,\"crop_cpu_access\":false,"
               "\"frames_ok\":"
            << frames << ",\"rga_ms_p50\":" << cf_zc::percentile50(rga_times)
            << ",\"npu_ms_p50\":" << cf_zc::percentile50(npu_times)
            << ",\"post_ms_p50\":" << cf_zc::percentile50(post_times)
            << ",\"fd_delta\":" << (fds_after - fds_before)
            << ",\"input_format\":\"rgb\"}" << std::endl;
  return 0;
}

}  // extern "C"
