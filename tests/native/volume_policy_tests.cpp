#include "moto/audio/volume_policy.hpp"

#include <iostream>

namespace {

using moto::audio::clamp_percent;
using moto::audio::kVolumeLevels;
using moto::audio::next_volume_level;
using moto::audio::volume_down;
using moto::audio::volume_up;

int failures = 0;

#define CHECK(condition)                                                      \
  do {                                                                        \
    if (!(condition)) {                                                       \
      std::cerr << "CHECK failed: " #condition " at line " << __LINE__       \
                << "\n";                                                      \
      ++failures;                                                             \
    }                                                                         \
  } while (false)

void test_full_scale_is_the_first_step() {
  CHECK(kVolumeLevels[0] == 100);
  CHECK(clamp_percent(150) == 100);
  CHECK(clamp_percent(0) == 0);
  CHECK(clamp_percent(65) == 65);
}

void test_cycling_walks_down_then_wraps() {
  CHECK(next_volume_level(100) == 70);
  CHECK(next_volume_level(70) == 40);
  CHECK(next_volume_level(40) == 0);
  // Silence wraps back to full scale, so one press always gets sound back.
  CHECK(next_volume_level(0) == 100);
}

void test_a_stored_value_between_steps_moves_to_the_next_lower_one() {
  // A value from an older build must never make the panel louder on the first
  // press: it drops to the nearest step below it.
  CHECK(next_volume_level(85) == 70);
  CHECK(next_volume_level(55) == 40);
  CHECK(next_volume_level(10) == 0);
  CHECK(next_volume_level(1) == 0);
}

void test_out_of_range_input_is_clamped_first() {
  CHECK(next_volume_level(200) == 70);
}

void test_repeated_presses_return_to_the_start() {
  std::uint8_t level = 100;
  for (int i = 0; i < 4; ++i) {
    level = next_volume_level(level);
  }
  CHECK(level == 100);
}

void test_two_keys_step_and_saturate() {
  CHECK(volume_up(0) == 40);
  CHECK(volume_up(40) == 70);
  CHECK(volume_up(70) == 100);
  // Full scale is a hard stop, not a wrap into silence.
  CHECK(volume_up(100) == 100);

  CHECK(volume_down(100) == 70);
  CHECK(volume_down(70) == 40);
  CHECK(volume_down(40) == 0);
  CHECK(volume_down(0) == 0);
}

void test_two_keys_start_from_a_stored_value_between_steps() {
  CHECK(volume_up(55) == 70);
  CHECK(volume_down(55) == 40);
  CHECK(volume_up(85) == 100);
  CHECK(volume_down(85) == 70);
}

}  // namespace

int main() {
  test_full_scale_is_the_first_step();
  test_cycling_walks_down_then_wraps();
  test_a_stored_value_between_steps_moves_to_the_next_lower_one();
  test_out_of_range_input_is_clamped_first();
  test_repeated_presses_return_to_the_start();
  test_two_keys_step_and_saturate();
  test_two_keys_start_from_a_stored_value_between_steps();

  if (failures != 0) {
    std::cerr << failures << " volume policy checks failed\n";
    return 1;
  }
  std::cout << "Volume policy checks passed\n";
  return 0;
}
