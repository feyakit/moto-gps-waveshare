#pragma once

#include <cstdint>

#include "esp_err.h"

class MotionHeadingSensor {
 public:
  // `accel_mps2` is the raw gravity-inclusive acceleration magnitude.  The
  // heading path ignores it; the parked/idle screen policy uses it to tell a
  // running engine (chassis vibration) from a switched-off bike.
  using SampleCallback = void (*)(float heading_rate_dps,
                                  float accel_mps2,
                                  std::uint64_t sample_ms,
                                  void* context);

  MotionHeadingSensor() = default;
  MotionHeadingSensor(const MotionHeadingSensor&) = delete;
  MotionHeadingSensor& operator=(const MotionHeadingSensor&) = delete;

  // Starts one low-priority 125 Hz sensor task. Failure is non-fatal: phone
  // course navigation continues to work without the local latency filler.
  esp_err_t start(SampleCallback callback, void* context);

 private:
  static void task_entry(void* context);
  void run();

  SampleCallback callback_ = nullptr;
  void* callback_context_ = nullptr;
  bool started_ = false;
};
