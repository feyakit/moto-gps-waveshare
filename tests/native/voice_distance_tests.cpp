#include "moto/audio/voice_distance.hpp"

#include <iostream>

namespace {
using moto::audio::distance_phrase;
int failures = 0;
#define CHECK(c)                                                              \
  do {                                                                        \
    if (!(c)) {                                                               \
      std::cerr << "CHECK failed: " #c " line " << __LINE__ << "\n";          \
      ++failures;                                                             \
    }                                                                         \
  } while (false)

void test_close_maneuvers_get_no_distance() {
  CHECK(distance_phrase(0) == MOTO_VOICE_NONE);
  CHECK(distance_phrase(79) == MOTO_VOICE_NONE);
}

void test_each_bucket_rounds_down() {
  CHECK(distance_phrase(80) == MOTO_VOICE_DIST_100M);
  CHECK(distance_phrase(149) == MOTO_VOICE_DIST_100M);
  CHECK(distance_phrase(150) == MOTO_VOICE_DIST_200M);
  CHECK(distance_phrase(249) == MOTO_VOICE_DIST_200M);
  CHECK(distance_phrase(250) == MOTO_VOICE_DIST_300M);
  CHECK(distance_phrase(399) == MOTO_VOICE_DIST_300M);
  CHECK(distance_phrase(400) == MOTO_VOICE_DIST_500M);
  CHECK(distance_phrase(649) == MOTO_VOICE_DIST_500M);
  CHECK(distance_phrase(650) == MOTO_VOICE_DIST_800M);
  CHECK(distance_phrase(899) == MOTO_VOICE_DIST_800M);
  CHECK(distance_phrase(900) == MOTO_VOICE_DIST_1KM);
  CHECK(distance_phrase(5000) == MOTO_VOICE_DIST_1KM);
}

void test_the_spoken_value_never_overshoots_the_real_one() {
  // "一百米" may not be spoken for anything beyond 100 m.
  CHECK(distance_phrase(149) != MOTO_VOICE_DIST_200M);
  // And the kilometre claim only appears once a kilometre is plausible.
  CHECK(distance_phrase(899) != MOTO_VOICE_DIST_1KM);
}

}  // namespace

int main() {
  test_close_maneuvers_get_no_distance();
  test_each_bucket_rounds_down();
  test_the_spoken_value_never_overshoots_the_real_one();
  if (failures != 0) {
    std::cerr << failures << " voice distance checks failed\n";
    return 1;
  }
  std::cout << "Voice distance checks passed\n";
  return 0;
}
