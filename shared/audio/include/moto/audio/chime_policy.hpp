#pragma once

#include <cstdint>

#include "moto_audio_tones.h"
#include "port_api/nav_types.hpp"

namespace moto::audio {

/// Everything the chime policy looks at, all of it already computed for the
/// instrument pages, so the policy never parses a route or talks to a phone.
struct ChimeInputs {
  /// A route is being guided and the fix is usable.  A parked board with a
  /// route still loaded must stay quiet.
  bool navigating = false;
  bool has_maneuver = false;
  nav::ManeuverType maneuver = nav::ManeuverType::Unknown;
  /// Identity of the next maneuver, so one turn is only announced once.
  /// Zero when the phone does not provide an id.
  std::uint32_t maneuver_id = 0;
  std::uint32_t distance_to_maneuver_m = 0;
  double speed_mps = 0.0;
  /// Zero means the provider has no trustworthy limit for this road.
  std::uint16_t speed_limit_kph = 0;
};

/// Up to two tones, because a turn and the speed limit can fall on the same
/// update and dropping one would lose information.  The board plays them in
/// order.
struct ChimeEvent {
  static constexpr std::uint8_t kMaxTones = 2;
  /// Each slot is either a synthesized tone or a spoken phrase; `voices` holds
  /// the phrase for the matching slot and MOTO_VOICE_NONE means "play the tone
  /// instead". A turn gets spoken words, everything else stays a sound cue so
  /// the rider is not talked at.
  moto_tone_t tones[kMaxTones] = {MOTO_TONE_NONE, MOTO_TONE_NONE};
  moto_voice_phrase_t voices[kMaxTones] = {MOTO_VOICE_NONE, MOTO_VOICE_NONE};
  /// Optional clip played immediately before `voices[i]`: the distance prefix
  /// of a spoken turn ("三百米" then "前方左转"). MOTO_VOICE_NONE means none.
  moto_voice_phrase_t prefixes[kMaxTones] = {MOTO_VOICE_NONE, MOTO_VOICE_NONE};
  std::uint8_t count = 0;

  [[nodiscard]] bool empty() const { return count == 0; }
};

/// Turns navigation state into a small number of short sounds.
///
/// The rules are deliberately about *lead time* rather than a fixed distance:
/// a turn 300 m away is seconds of warning at 100 km/h and half a minute at
/// 20 km/h.  Distances are still capped so a cue never fires absurdly early on
/// a highway, and the floor speed keeps a stopped or crawling rider from
/// getting a cue the moment the next maneuver appears.
class ChimePolicy {
 public:
  /// Announce a turn twice: first a soft chirp, then the turn itself.
  static constexpr double kStageOneSeconds = 12.0;
  static constexpr std::uint32_t kStageOneMaxM = 500;
  static constexpr double kStageTwoSeconds = 4.0;
  static constexpr std::uint32_t kStageTwoMaxM = 150;
  /// Lead-time maths uses at least this speed, about 18 km/h.
  static constexpr double kMinSpeedMps = 5.0;
  /// A maneuver seen this much farther away than the last update is the next
  /// one (or a reroute), not the one already announced.
  static constexpr std::uint32_t kRearmJumpM = 40;
  /// Speeding has to be clear of the limit, and has to come back near it,
  /// before the buzz can repeat.
  static constexpr double kLimitOverKph = 5.0;
  static constexpr double kLimitClearKph = 2.0;

  /// Called once per navigation update, one per second from the phone.
  ChimeEvent update(const ChimeInputs& inputs);

  /// Called when ANCS reports a notification.  A silent notification stays
  /// silent: iOS already asked for that, and the panel need not be lit either.
  [[nodiscard]] moto_tone_t on_notification(bool incoming_call,
                                            bool silent) const;

  /// Forget everything, for a new link or a carousel demo restart.
  void reset();

 private:
  bool starts_new_maneuver(const ChimeInputs& inputs);

  bool has_last_maneuver_ = false;
  std::uint32_t last_maneuver_id_ = 0;
  std::uint32_t last_distance_m_ = 0;
  bool announced_far_ = false;
  bool announced_near_ = false;
  bool over_limit_ = false;
};

}  // namespace moto::audio
