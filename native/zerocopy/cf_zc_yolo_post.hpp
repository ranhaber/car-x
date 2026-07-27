#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace cf_zc {

struct Detection {
  float x1 = 0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  float score = 0;
  int32_t class_id = 0;
};

namespace {

constexpr int kCatCocoId = 17;
constexpr int kNumClasses = 80;
constexpr int kDflBins = 16;
constexpr float kNmsThreshold = 0.45f;

// YOLO contiguous COCO-80 index -> official COCO category id.
constexpr int32_t kYolo80ToCoco[80] = {
    1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 27, 28, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
    43, 44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
    63, 64, 65, 67, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86,
    87, 88, 89, 90,
};

constexpr int kAnimalClasses0Idx[] = {15, 16, 17, 18, 19, 21};

inline bool is_animal_class(int class_index) {
  for (int value : kAnimalClasses0Idx) {
    if (value == class_index) return true;
  }
  return false;
}

inline int32_t yolo80_to_coco(int class_index) {
  if (class_index < 0 || class_index >= kNumClasses) return class_index + 1;
  return kYolo80ToCoco[class_index];
}

inline float dequant(int8_t value, float scale, int32_t zero_point) {
  return (static_cast<float>(value) - static_cast<float>(zero_point)) * scale;
}

inline float iou_xyxy(const Detection& a, const Detection& b) {
  const float x1 = std::max(a.x1, b.x1);
  const float y1 = std::max(a.y1, b.y1);
  const float x2 = std::min(a.x2, b.x2);
  const float y2 = std::min(a.y2, b.y2);
  const float iw = std::max(0.0f, x2 - x1);
  const float ih = std::max(0.0f, y2 - y1);
  const float inter = iw * ih;
  if (inter <= 0.0f) return 0.0f;
  const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
  const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
  const float uni = area_a + area_b - inter;
  return uni > 0.0f ? inter / uni : 0.0f;
}

inline Detection decode_dfl_cell(const int8_t* box, float box_scale,
                                 int32_t box_zp, int cell, int grid_h,
                                 int grid_w, int input_w, int input_h) {
  const int plane = grid_h * grid_w;
  float decoded[4] = {0, 0, 0, 0};
  for (int side = 0; side < 4; ++side) {
    float logits[kDflBins];
    float max_logit = -std::numeric_limits<float>::infinity();
    for (int bin = 0; bin < kDflBins; ++bin) {
      const int8_t q = box[(side * kDflBins + bin) * plane + cell];
      logits[bin] = dequant(q, box_scale, box_zp);
      max_logit = std::max(max_logit, logits[bin]);
    }
    float sum = 0.0f;
    for (int bin = 0; bin < kDflBins; ++bin) {
      logits[bin] = std::exp(logits[bin] - max_logit);
      sum += logits[bin];
    }
    float expected = 0.0f;
    for (int bin = 0; bin < kDflBins; ++bin) {
      expected += (logits[bin] / sum) * static_cast<float>(bin);
    }
    decoded[side] = expected;
  }

  const float col = static_cast<float>(cell % grid_w);
  const float row = static_cast<float>(cell / grid_w);
  const float stride_x = static_cast<float>(input_w) / static_cast<float>(grid_w);
  const float stride_y = static_cast<float>(input_h) / static_cast<float>(grid_h);
  Detection det{};
  det.x1 = (col + 0.5f - decoded[0]) * stride_x;
  det.y1 = (row + 0.5f - decoded[1]) * stride_y;
  det.x2 = (col + 0.5f + decoded[2]) * stride_x;
  det.y2 = (row + 0.5f + decoded[3]) * stride_y;
  return det;
}

struct TensorView {
  const int8_t* data = nullptr;
  float scale = 1.0f;
  int32_t zero_point = 0;
  int channels = 0;
  int height = 0;
  int width = 0;
};

inline void decode_branch(const TensorView& box, const TensorView& score,
                          const TensorView& score_sum, int input_w, int input_h,
                          float score_threshold, bool animal_mode,
                          std::vector<Detection>* out) {
  const int grid_h = box.height;
  const int grid_w = box.width;
  const int plane = grid_h * grid_w;
  for (int cell = 0; cell < plane; ++cell) {
    const float sum = dequant(score_sum.data[cell], score_sum.scale,
                              score_sum.zero_point);
    if (sum < score_threshold) continue;

    int best_class = 0;
    float best_score = -std::numeric_limits<float>::infinity();
    for (int cls = 0; cls < kNumClasses; ++cls) {
      const float value = dequant(score.data[cls * plane + cell], score.scale,
                                  score.zero_point);
      if (value > best_score) {
        best_score = value;
        best_class = cls;
      }
    }
    if (best_score < score_threshold) continue;

    int32_t coco_id = yolo80_to_coco(best_class);
    if (animal_mode && is_animal_class(best_class)) {
      coco_id = kCatCocoId;
    }
    if (coco_id != kCatCocoId) continue;

    Detection det =
        decode_dfl_cell(box.data, box.scale, box.zero_point, cell, grid_h,
                        grid_w, input_w, input_h);
    det.score = best_score;
    det.class_id = coco_id;

    // Clamp to model input space (no letterbox on this path).
    det.x1 = std::max(0.0f, std::min(det.x1, static_cast<float>(input_w)));
    det.y1 = std::max(0.0f, std::min(det.y1, static_cast<float>(input_h)));
    det.x2 = std::max(0.0f, std::min(det.x2, static_cast<float>(input_w)));
    det.y2 = std::max(0.0f, std::min(det.y2, static_cast<float>(input_h)));
    if (det.x2 > det.x1 && det.y2 > det.y1) {
      out->push_back(det);
    }
  }
}

inline void nms_inplace(std::vector<Detection>* detections) {
  std::sort(detections->begin(), detections->end(),
            [](const Detection& a, const Detection& b) {
              return a.score > b.score;
            });
  std::vector<Detection> kept;
  kept.reserve(detections->size());
  std::vector<char> suppressed(detections->size(), 0);
  for (size_t i = 0; i < detections->size(); ++i) {
    if (suppressed[i]) continue;
    kept.push_back((*detections)[i]);
    for (size_t j = i + 1; j < detections->size(); ++j) {
      if (suppressed[j]) continue;
      if ((*detections)[i].class_id != (*detections)[j].class_id) continue;
      if (iou_xyxy((*detections)[i], (*detections)[j]) > kNmsThreshold) {
        suppressed[j] = 1;
      }
    }
  }
  *detections = std::move(kept);
}

}  // namespace

// Decode the 9-tensor YOLOv8 model-zoo head from RKNN INT8 output pointers.
// Outputs are expected in NCHW order: box/score/score_sum × 3 scales.
inline std::vector<Detection> decode_yolov8_int8(
    const int8_t* const* tensors, const float* scales, const int32_t* zero_points,
    const int* heights, const int* widths, int input_w, int input_h,
    float score_threshold, bool animal_mode) {
  std::vector<Detection> detections;
  detections.reserve(64);
  for (int branch = 0; branch < 3; ++branch) {
    TensorView box{tensors[branch * 3], scales[branch * 3],
                   zero_points[branch * 3], 64, heights[branch * 3],
                   widths[branch * 3]};
    TensorView score{tensors[branch * 3 + 1], scales[branch * 3 + 1],
                     zero_points[branch * 3 + 1], 80, heights[branch * 3 + 1],
                     widths[branch * 3 + 1]};
    TensorView score_sum{tensors[branch * 3 + 2], scales[branch * 3 + 2],
                         zero_points[branch * 3 + 2], 1,
                         heights[branch * 3 + 2], widths[branch * 3 + 2]};
    decode_branch(box, score, score_sum, input_w, input_h, score_threshold,
                  animal_mode, &detections);
  }
  nms_inplace(&detections);
  return detections;
}

}  // namespace cf_zc
