#pragma once
#ifndef COMMON_LIB_INCLUDE_TIME_H_
#define COMMON_LIB_INCLUDE_TIME_H_

#include <string>

#include <ros/ros.h>
#include <sys/time.h>

namespace common {
class Clock {
 public:
  Clock() {}

  /// @brief start clock timer
  void start() {
    gettimeofday(&real_time_start_, NULL);
    cpu_start_ = clock();
  }

  void takeTime() {
    struct timeval end;
    gettimeofday(&end, NULL);
    cpu_time_ms_ = static_cast<double>(clock() - cpu_start_) / CLOCKS_PER_SEC * kSecondsToMiliseconds;

    uint64_t seconds, useconds;

    seconds = end.tv_sec - real_time_start_.tv_sec;
    useconds = end.tv_usec - real_time_start_.tv_usec;
    real_time_ms_ = (seconds * kSecondsToMiliseconds + useconds * kMicrosecondsToMiliseconds);
  }

  /// @brief Return elapsed physical time
  /// @return
  double getRealTime() { return real_time_ms_; }

  /// @brief Return elapsed CPU time
  /// @return
  double getCPUTime() { return cpu_time_ms_; }

  double takeRealTime() {
    takeTime();
    return getRealTime();
  }

 private:
  struct timeval real_time_start_;
  double real_time_ms_, cpu_time_ms_;
  clock_t cpu_start_;

  static constexpr double kSecondsToMiliseconds = 1000.0;
  static constexpr double kMicrosecondsToMiliseconds = 0.001;
};
}  // namespace common

#endif
