// Thin CLI over libcat_follow_zerocopy (same JSON gate as the monolithic validator).
#include "../native/zerocopy/cf_zc.h"

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  const std::string device = argc > 1 ? argv[1] : "/dev/video11";
  const std::string model =
      argc > 2 ? argv[2] : "models/yolov8n_coco_320_rk3576_int8.rknn";
  const int frames = argc > 3 ? std::max(1, std::atoi(argv[3])) : 30;
  return cf_zc_validate_run(device.c_str(), model.c_str(), frames);
}
