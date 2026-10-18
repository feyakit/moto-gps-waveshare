#pragma once

#include <cmath>
#include <cstdint>

/**
 * Decides when the round display may stop drawing because the bike has been
 * parked, and when it must light up again.
 *
 * The board has no ignition line, so "parked" is inferred from the QMI8658
 * accelerometer: a running engine keeps the whole chassis buzzing well above
 * the noise floor of a stationary sensor.  That is deliberately different from
 * "not moving", which would blank the screen at every long traffic light.
 *
 * Pure C++ with no ESP-IDF or LVGL dependency so the timing rules are unit
 * tested on the host.
 */
class MotionIdleDetector {
 public:
  struct Config {
    /// Deviation from 1 g that counts as motion.  Idle engine vibration on the
    /// handlebar mount sits above this; a parked bike reads below it.
    float motion_g = 0.035F;
    /// How long the bike must stay quiet before the screen is blanked.
    std::uint64_t parked_hold_ms = 45'000;
    /// While a route is being followed the screen is a navigation instrument,
    /// so a stop at a traffic light must never blank it.  Only a genuinely
    /// finished ride (engine off) reaches this longer hold.
    std::uint64_t parked_hold_with_route_ms = 180'000;
    /// Speed that proves the bike is moving even on a perfectly smooth
    /// surface, in metres per second.
    float moving_speed_mps = 0.6F;
  };

  enum class Action {
    None,
    /// The screen should be blanked now.
    Sleep,
    /// The screen should be lit now.
    Wake,
  };

  MotionIdleDetector() = default;
  explicit MotionIdleDetector(Config config) : config_(config) {}

  /// Call at sensor rate.  `route_active` is true while the display is showing
  /// an active navigation.
  Action update(float accel_mps2, std::uint64_t now_ms, bool route_active) {
    if (!std::isfinite(accel_mps2)) {
      return Action::None;
    }
    ensure_primed(now_ms);
    if (std::fabs(accel_mps2 - kGravityMps2) >= config_.motion_g) {
      return note_motion(now_ms);
    }
    if (sleeping_) {
      return Action::None;
    }
    const std::uint64_t hold = route_active
                                   ? config_.parked_hold_with_route_ms
                                   : config_.parked_hold_ms;
    if (now_ms >= last_motion_ms_ && now_ms - last_motion_ms_ >= hold) {
      sleeping_ = true;
      return Action::Sleep;
    }
    return Action::None;
  }

  /// Phone-reported speed, which proves motion even when the chassis is too
  /// smooth for the accelerometer threshold.
  Action note_speed(float speed_mps, std::uint64_t now_ms) {
    if (!std::isfinite(speed_mps) || speed_mps < config_.moving_speed_mps) {
      return Action::None;
    }
    ensure_primed(now_ms);
    return note_motion(now_ms);
  }

  /// A touch or button press is a deliberate request to see the screen.
  Action note_activity(std::uint64_t now_ms) {
    ensure_primed(now_ms);
    return note_motion(now_ms);
  }

  /// Restarts the quiet timer without waking, used when the renderer starts.
  void prime(std::uint64_t now_ms) {
    last_motion_ms_ = now_ms;
    sleeping_ = false;
    primed_ = true;
  }

  [[nodiscard]] bool primed() const { return primed_; }
  [[nodiscard]] bool sleeping() const { return sleeping_; }
  [[nodiscard]] std::uint64_t last_motion_ms() const { return last_motion_ms_; }
  [[nodiscard]] const Config& config() const { return config_; }

 private:
  static constexpr float kGravityMps2 = 9.80665F;

  /// The first real sample defines "quiet since now" instead of "quiet since
  /// boot", so a board powered on in a parked garage blanks after the hold.
  void ensure_primed(std::uint64_t now_ms) {
    if (!primed_) {
      prime(now_ms);
    }
  }

  Action note_motion(std::uint64_t now_ms) {
    last_motion_ms_ = now_ms;
    if (sleeping_) {
      sleeping_ = false;
      return Action::Wake;
    }
    return Action::None;
  }

  Config config_{};
  std::uint64_t last_motion_ms_ = 0;
  bool primed_ = false;
  bool sleeping_ = false;
};
