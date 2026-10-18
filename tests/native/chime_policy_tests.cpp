// Host tests for the chime policy.  These pin the rules that decide when the
// panel is allowed to make a sound: a rider gets one cue per turn, one buzz per
// speeding episode, and never a cue for a notification iOS marked silent.
#include "moto/audio/chime_policy.hpp"

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

using moto::audio::ChimeEvent;
using moto::audio::ChimeInputs;
using moto::audio::ChimePolicy;
using moto::nav::ManeuverType;

/// 60 km/h, the speed most of the arithmetic below assumes.
constexpr double kRiding = 16.6667;
/// 72 km/h, clear of a 60 km/h limit.
constexpr double kSpeeding = 20.0;

ChimeInputs turn_inputs(std::uint32_t maneuver_id, ManeuverType maneuver,
                        std::uint32_t distance_m, double speed_mps) {
  ChimeInputs inputs;
  inputs.navigating = true;
  inputs.has_maneuver = true;
  inputs.maneuver = maneuver;
  inputs.maneuver_id = maneuver_id;
  inputs.distance_to_maneuver_m = distance_m;
  inputs.speed_mps = speed_mps;
  return inputs;
}

void test_turn_is_announced_twice_then_stays_quiet() {
  ChimePolicy policy;
  // At 60 km/h the lead times are 200 m and 67 m.
  CHECK(policy.update(turn_inputs(7, ManeuverType::Left, 300, kRiding)).empty());
  CHECK(policy.update(turn_inputs(7, ManeuverType::Left, 199, kRiding)).tones[0] ==
        MOTO_TONE_TURN_AHEAD);
  CHECK(policy.update(turn_inputs(7, ManeuverType::Left, 150, kRiding)).empty());
  CHECK(policy.update(turn_inputs(7, ManeuverType::Left, 66, kRiding)).tones[0] ==
        MOTO_TONE_TURN_LEFT);
  // Approaching the same turn never repeats itself.
  CHECK(policy.update(turn_inputs(7, ManeuverType::Left, 60, kRiding)).empty());
  CHECK(policy.update(turn_inputs(7, ManeuverType::Left, 10, kRiding)).empty());

  // The next maneuver is a new cue, not a repeat.
  CHECK(policy.update(turn_inputs(8, ManeuverType::Right, 400, kRiding)).empty());
  CHECK(policy.update(turn_inputs(8, ManeuverType::Right, 199, kRiding)).tones[0] ==
        MOTO_TONE_TURN_AHEAD);
  CHECK(policy.update(turn_inputs(8, ManeuverType::Right, 66, kRiding)).tones[0] ==
        MOTO_TONE_TURN_RIGHT);
}

// The whole point of lead time: a crawling rider next to the turn does not want
// a cue hundreds of metres back.
void test_slow_turn_is_cued_by_distance() {
  ChimePolicy policy;
  // 10.8 km/h: 36 m and 12 m.
  CHECK(policy.update(turn_inputs(3, ManeuverType::Left, 100, 3.0)).empty());
  CHECK(policy.update(turn_inputs(3, ManeuverType::Left, 36, 3.0)).tones[0] ==
        MOTO_TONE_TURN_AHEAD);
  CHECK(policy.update(turn_inputs(3, ManeuverType::Left, 12, 3.0)).tones[0] ==
        MOTO_TONE_TURN_LEFT);
}

// A maneuver that appears already close gets the real cue, not a chirp now and
// the cue a second later.
void test_close_maneuver_skips_the_chirp() {
  ChimePolicy policy;
  const ChimeEvent event =
      policy.update(turn_inputs(11, ManeuverType::Right, 40, kRiding));
  CHECK(event.count == 1);
  CHECK(event.tones[0] == MOTO_TONE_TURN_RIGHT);
  CHECK(policy.update(turn_inputs(11, ManeuverType::Right, 20, kRiding)).empty());
}

void test_direction_is_carried_by_pitch() {
  const ManeuverType leftish[] = {ManeuverType::SlightLeft, ManeuverType::Left,
                                  ManeuverType::SharpLeft,
                                  ManeuverType::UTurnLeft};
  const ManeuverType rightish[] = {ManeuverType::SlightRight,
                                   ManeuverType::Right,
                                   ManeuverType::SharpRight,
                                   ManeuverType::UTurnRight};
  for (const ManeuverType maneuver : leftish) {
    ChimePolicy policy;
    CHECK(policy.update(turn_inputs(1, maneuver, 30, kRiding)).tones[0] ==
          MOTO_TONE_TURN_LEFT);
  }
  for (const ManeuverType maneuver : rightish) {
    ChimePolicy policy;
    CHECK(policy.update(turn_inputs(1, maneuver, 30, kRiding)).tones[0] ==
          MOTO_TONE_TURN_RIGHT);
  }

  ChimePolicy roundabout;
  CHECK(roundabout.update(turn_inputs(1, ManeuverType::Roundabout, 30, kRiding))
            .tones[0] == MOTO_TONE_MANEUVER);
  ChimePolicy arrive;
  CHECK(arrive.update(turn_inputs(1, ManeuverType::Arrive, 30, kRiding))
            .tones[0] == MOTO_TONE_ARRIVE);
}

// Without maneuver ids the only evidence that a turn was passed is the distance
// to the next one jumping back up.
void test_passing_a_turn_without_ids_rearms() {
  ChimePolicy policy;
  CHECK(policy.update(turn_inputs(0, ManeuverType::Left, 190, kRiding)).tones[0] ==
        MOTO_TONE_TURN_AHEAD);
  CHECK(policy.update(turn_inputs(0, ManeuverType::Left, 30, kRiding)).tones[0] ==
        MOTO_TONE_TURN_LEFT);
  CHECK(policy.update(turn_inputs(0, ManeuverType::Left, 5, kRiding)).empty());

  // The rider is handed the next turn, well ahead.
  CHECK(policy.update(turn_inputs(0, ManeuverType::Left, 300, kRiding)).empty());
  CHECK(policy.update(turn_inputs(0, ManeuverType::Left, 190, kRiding)).tones[0] ==
        MOTO_TONE_TURN_AHEAD);
}

void test_speed_limit_buzzes_once_per_episode() {
  ChimePolicy policy;
  ChimeInputs inputs = turn_inputs(0, ManeuverType::Unknown, 0, kRiding);
  inputs.has_maneuver = false;
  inputs.speed_limit_kph = 60;

  CHECK(policy.update(inputs).empty());  // exactly at the limit

  inputs.speed_mps = kSpeeding;
  CHECK(policy.update(inputs).tones[0] == MOTO_TONE_SPEED_LIMIT);
  CHECK(policy.update(inputs).empty());  // still over: no nagging
  inputs.speed_mps = 19.0;               // 68 km/h, still over
  CHECK(policy.update(inputs).empty());

  inputs.speed_mps = 17.0;  // 61 km/h, back near the limit
  CHECK(policy.update(inputs).empty());
  inputs.speed_mps = kSpeeding;
  CHECK(policy.update(inputs).tones[0] == MOTO_TONE_SPEED_LIMIT);
}

void test_unknown_limit_stays_quiet() {
  ChimePolicy policy;
  ChimeInputs inputs = turn_inputs(0, ManeuverType::Unknown, 0, kSpeeding);
  inputs.has_maneuver = false;
  inputs.speed_limit_kph = 0;
  CHECK(policy.update(inputs).empty());

  // A road that stops reporting its limit also clears the latch, so the next
  // posted road can buzz again.
  inputs.speed_limit_kph = 50;
  CHECK(policy.update(inputs).tones[0] == MOTO_TONE_SPEED_LIMIT);
  inputs.speed_limit_kph = 0;
  CHECK(policy.update(inputs).empty());
  inputs.speed_limit_kph = 50;
  CHECK(policy.update(inputs).tones[0] == MOTO_TONE_SPEED_LIMIT);
}

void test_no_guidance_means_no_sounds() {
  ChimePolicy policy;
  ChimeInputs inputs = turn_inputs(5, ManeuverType::Left, 10, kSpeeding);
  inputs.speed_limit_kph = 30;
  inputs.navigating = false;
  CHECK(policy.update(inputs).empty());
  CHECK(policy.update(inputs).empty());

  // Guidance coming back is a fresh cue, not a continuation.
  inputs.navigating = true;
  const ChimeEvent event = policy.update(inputs);
  CHECK(event.count == 2);
  CHECK(event.tones[0] == MOTO_TONE_TURN_LEFT);
  CHECK(event.tones[1] == MOTO_TONE_SPEED_LIMIT);
}

void test_turn_and_limit_can_land_together() {
  ChimePolicy policy;
  ChimeInputs inputs = turn_inputs(2, ManeuverType::Right, 70, kSpeeding);
  inputs.speed_limit_kph = 60;
  const ChimeEvent event = policy.update(inputs);
  CHECK(event.count == 2);
  CHECK(event.tones[0] == MOTO_TONE_TURN_RIGHT);
  CHECK(event.tones[1] == MOTO_TONE_SPEED_LIMIT);
}

void test_notification_tones_respect_silence() {
  ChimePolicy policy;
  CHECK(policy.on_notification(false, false) == MOTO_TONE_NOTIFICATION);
  CHECK(policy.on_notification(true, false) == MOTO_TONE_CALL);
  CHECK(policy.on_notification(false, true) == MOTO_TONE_NONE);
  CHECK(policy.on_notification(true, true) == MOTO_TONE_NONE);
}

void test_reset_reannounces() {
  ChimePolicy policy;
  CHECK(policy.update(turn_inputs(4, ManeuverType::Left, 30, kRiding)).tones[0] ==
        MOTO_TONE_TURN_LEFT);
  CHECK(policy.update(turn_inputs(4, ManeuverType::Left, 20, kRiding)).empty());

  policy.reset();
  CHECK(policy.update(turn_inputs(4, ManeuverType::Left, 20, kRiding)).tones[0] ==
        MOTO_TONE_TURN_LEFT);
}

}  // namespace

int main() {
  test_turn_is_announced_twice_then_stays_quiet();
  test_slow_turn_is_cued_by_distance();
  test_close_maneuver_skips_the_chirp();
  test_direction_is_carried_by_pitch();
  test_passing_a_turn_without_ids_rearms();
  test_speed_limit_buzzes_once_per_episode();
  test_unknown_limit_stays_quiet();
  test_no_guidance_means_no_sounds();
  test_turn_and_limit_can_land_together();
  test_notification_tones_respect_silence();
  test_reset_reannounces();

  if (failures != 0) {
    std::cerr << failures << " chime policy checks failed\n";
    return 1;
  }
  std::cout << "chime policy checks passed\n";
  return 0;
}
