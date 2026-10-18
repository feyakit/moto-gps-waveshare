// Host tests for the parked/idle screen policy.  The firmware cannot show a
// rider when a stop at a traffic light blanks the navigation page, so these
// cases pin the exact timing rules instead.
#include "motion_idle_detector.hpp"

#include <iostream>

namespace {

int failures = 0;

#define CHECK(condition)                                                   \
  do {                                                                     \
    if (!(condition)) {                                                    \
      std::cerr << __FILE__ << ':' << __LINE__                             \
                << " CHECK failed: " #condition << '\n';                   \
      ++failures;                                                          \
    }                                                                      \
  } while (false)

using Action = MotionIdleDetector::Action;
constexpr float kStill = 9.81F;
// Engine running: the chassis buzzes, so the magnitude moves off 1 g.
constexpr float kRunning = 10.05F;

MotionIdleDetector::Config tight_config() {
  MotionIdleDetector::Config config;
  config.parked_hold_ms = 1'000;
  config.parked_hold_with_route_ms = 3'000;
  return config;
}

void test_parked_bike_sleeps_after_the_hold() {
  MotionIdleDetector detector(tight_config());
  detector.prime(0);
  CHECK(detector.update(kStill, 900, false) == Action::None);
  CHECK(!detector.sleeping());
  CHECK(detector.update(kStill, 1'000, false) == Action::Sleep);
  CHECK(detector.sleeping());
  // Staying parked must not keep reporting transitions.
  CHECK(detector.update(kStill, 2'000, false) == Action::None);
}

void test_running_engine_never_sleeps() {
  MotionIdleDetector detector(tight_config());
  detector.prime(0);
  for (std::uint64_t now = 100; now <= 20'000; now += 100) {
    CHECK(detector.update(kRunning, now, true) == Action::None);
  }
  CHECK(!detector.sleeping());
}

void test_route_active_uses_the_longer_hold() {
  MotionIdleDetector detector(tight_config());
  detector.prime(0);
  // A long traffic light with a route on screen must keep the instrument lit.
  CHECK(detector.update(kStill, 2'999, true) == Action::None);
  CHECK(!detector.sleeping());
  CHECK(detector.update(kStill, 3'000, true) == Action::Sleep);
}

void test_motion_wakes_the_screen() {
  MotionIdleDetector detector(tight_config());
  detector.prime(0);
  CHECK(detector.update(kStill, 1'000, false) == Action::Sleep);
  CHECK(detector.update(kRunning, 1'100, false) == Action::Wake);
  CHECK(!detector.sleeping());
  // The quiet timer restarted, so it cannot sleep again immediately.
  CHECK(detector.update(kStill, 1'500, false) == Action::None);
}

void test_touch_wakes_and_restarts_the_timer() {
  MotionIdleDetector detector(tight_config());
  detector.prime(0);
  CHECK(detector.update(kStill, 1'000, false) == Action::Sleep);
  CHECK(detector.note_activity(1'200) == Action::Wake);
  CHECK(!detector.sleeping());
  CHECK(detector.update(kStill, 2'000, false) == Action::None);
  CHECK(detector.update(kStill, 2'200, false) == Action::Sleep);
}

void test_reported_speed_counts_as_motion() {
  MotionIdleDetector detector(tight_config());
  detector.prime(0);
  // A perfectly smooth surface plus a phone-reported 20 km/h still means the
  // rider is riding, so the screen must stay on.
  CHECK(detector.note_speed(5.5F, 900) == Action::None);
  CHECK(detector.update(kStill, 1'500, true) == Action::None);
  CHECK(!detector.sleeping());
}

void test_zero_and_invalid_inputs_do_not_wake() {
  MotionIdleDetector detector(tight_config());
  detector.prime(0);
  CHECK(detector.update(kStill, 1'000, false) == Action::Sleep);
  CHECK(detector.note_speed(0.0F, 1'100) == Action::None);
  CHECK(detector.note_speed(NAN, 1'150) == Action::None);
  CHECK(detector.update(NAN, 1'200, false) == Action::None);
  CHECK(detector.sleeping());
}

void test_clock_going_backwards_does_not_sleep() {
  MotionIdleDetector detector(tight_config());
  detector.prime(5'000);
  // A restarted millisecond counter must not be read as "quiet for ages".
  CHECK(detector.update(kStill, 10, false) == Action::None);
  CHECK(!detector.sleeping());
}

void test_first_sample_primes_the_quiet_timer() {
  MotionIdleDetector detector(tight_config());
  CHECK(!detector.primed());
  // A board switched on in a parked garage must not blank instantly just
  // because the millisecond counter is already past the hold.
  CHECK(detector.update(kStill, 1'000'000, false) == Action::None);
  CHECK(detector.primed());
  CHECK(detector.update(kStill, 1'000'500, false) == Action::None);
  CHECK(detector.update(kStill, 1'001'000, false) == Action::Sleep);
}

void test_prime_forgets_a_sleep() {
  MotionIdleDetector detector(tight_config());
  detector.prime(0);
  CHECK(detector.update(kStill, 1'000, false) == Action::Sleep);
  detector.prime(1'100);
  CHECK(!detector.sleeping());
}

}  // namespace

int main() {
  test_parked_bike_sleeps_after_the_hold();
  test_running_engine_never_sleeps();
  test_route_active_uses_the_longer_hold();
  test_motion_wakes_the_screen();
  test_touch_wakes_and_restarts_the_timer();
  test_reported_speed_counts_as_motion();
  test_zero_and_invalid_inputs_do_not_wake();
  test_clock_going_backwards_does_not_sleep();
  test_first_sample_primes_the_quiet_timer();
  test_prime_forgets_a_sleep();

  if (failures != 0) {
    std::cerr << failures << " motion idle detector checks failed\n";
    return 1;
  }
  std::cout << "motion idle detector checks passed\n";
  return 0;
}
