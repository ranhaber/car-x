#pragma once

#include <dirent.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace cf_zc {

inline void checked_ioctl(int fd, unsigned long request, void* arg,
                          const char* name) {
  if (ioctl(fd, request, arg) < 0) {
    throw std::runtime_error(std::string(name) + ": " + std::strerror(errno));
  }
}

inline double elapsed_ms(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

inline double percentile50(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  return values.size() % 2 ? values[middle]
                           : (values[middle - 1] + values[middle]) / 2.0;
}

inline int open_fd_count() {
  DIR* directory = opendir("/proc/self/fd");
  if (!directory) return -1;
  int count = 0;
  while (const dirent* entry = readdir(directory)) {
    if (entry->d_name[0] != '.') ++count;
  }
  closedir(directory);
  return count - 1;
}

}  // namespace cf_zc
