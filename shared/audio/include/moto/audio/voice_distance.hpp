#pragma once

#include <cstdint>

#include "moto_audio_tones.h"

namespace moto::audio {

/// Picks the spoken distance prefix for a maneuver that is `meters` away.
///
/// The clips are a fixed set, so a real distance snaps to the nearest value
/// that is not misleading. Rounding down is deliberate: telling a rider "五百米"
/// when the turn is 420 m away makes them expect the turn too early, while
/// "三百米" at 420 m leaves a small buffer they will not notice.
///
/// Returns MOTO_VOICE_NONE below the first step: a card that appears already
/// close gets the action alone rather than a distance that is about to change.
constexpr moto_voice_phrase_t distance_phrase(std::uint32_t meters) noexcept {
  if (meters < 80U) return MOTO_VOICE_NONE;
  if (meters < 150U) return MOTO_VOICE_DIST_100M;
  if (meters < 250U) return MOTO_VOICE_DIST_200M;
  if (meters < 400U) return MOTO_VOICE_DIST_300M;
  if (meters < 650U) return MOTO_VOICE_DIST_500M;
  if (meters < 900U) return MOTO_VOICE_DIST_800M;
  return MOTO_VOICE_DIST_1KM;
}

}  // namespace moto::audio
