#pragma once

#include <cstddef>
#include <cstdint>

namespace moto::audio {

/// Speaker levels the board's key cycles through, loudest first.
///
/// Four steps rather than a continuous scale: the rider changes this while
/// riding, so each press has to land on a predictable, audible difference.
/// Zero is silence, and it is reachable on purpose -- a notification ding at
/// full scale next to an ear is worse than no ding at all.
inline constexpr std::uint8_t kVolumeLevels[] = {100, 70, 40, 0};
inline constexpr std::size_t kVolumeLevelCount =
    sizeof(kVolumeLevels) / sizeof(kVolumeLevels[0]);

constexpr std::uint8_t clamp_percent(std::uint8_t percent) noexcept {
  return percent > 100U ? 100U : percent;
}

/// The level one key press lands on.
///
/// Cycles downward and wraps to full scale. A stored value that is not one of
/// the steps (an older build, a hand-edited setting) still behaves: it moves
/// to the next lower step, so the first press can never make the panel louder
/// than the rider expects.
/// One step louder. Saturates at full scale instead of wrapping: with two
/// dedicated keys the rider expects a limit they can feel, not a surprise
/// jump back to silence.
constexpr std::uint8_t volume_up(std::uint8_t current) noexcept {
  const std::uint8_t clamped = clamp_percent(current);
  std::uint8_t best = kVolumeLevels[0];
  for (std::size_t i = 0; i < kVolumeLevelCount; ++i) {
    if (kVolumeLevels[i] > clamped &&
        (best <= clamped || kVolumeLevels[i] < best)) {
      best = kVolumeLevels[i];
    }
  }
  return best > clamped ? best : clamped;
}

/// One step quieter, saturating at silence.
constexpr std::uint8_t volume_down(std::uint8_t current) noexcept {
  const std::uint8_t clamped = clamp_percent(current);
  std::uint8_t best = kVolumeLevels[kVolumeLevelCount - 1];
  for (std::size_t i = 0; i < kVolumeLevelCount; ++i) {
    if (kVolumeLevels[i] < clamped &&
        (best >= clamped || kVolumeLevels[i] > best)) {
      best = kVolumeLevels[i];
    }
  }
  return best < clamped ? best : clamped;
}

/// Single-key cycle used when only one key is wired.
constexpr std::uint8_t next_volume_level(std::uint8_t current) noexcept {
  const std::uint8_t clamped = clamp_percent(current);
  for (std::size_t i = 0; i < kVolumeLevelCount; ++i) {
    if (kVolumeLevels[i] < clamped) {
      return kVolumeLevels[i];
    }
  }
  return kVolumeLevels[0];
}

}  // namespace moto::audio
