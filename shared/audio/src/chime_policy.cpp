#include "moto/audio/chime_policy.hpp"

#include "moto/audio/voice_distance.hpp"

#include <algorithm>

namespace moto::audio {
namespace {

/// Direction is carried by pitch, not by counting beeps: one cue to learn, and
/// it stays readable with a helmet on.
moto_tone_t arrival_tone(nav::ManeuverType maneuver) {
  switch (maneuver) {
    case nav::ManeuverType::SlightLeft:
    case nav::ManeuverType::Left:
    case nav::ManeuverType::SharpLeft:
    case nav::ManeuverType::UTurnLeft:
      return MOTO_TONE_TURN_LEFT;
    case nav::ManeuverType::SlightRight:
    case nav::ManeuverType::Right:
    case nav::ManeuverType::SharpRight:
    case nav::ManeuverType::UTurnRight:
      return MOTO_TONE_TURN_RIGHT;
    case nav::ManeuverType::Arrive:
      return MOTO_TONE_ARRIVE;
    default:
      // Roundabout, exit, continue and anything unknown: worth a sound, but
      // the screen arrow is what says which way.
      return MOTO_TONE_MANEUVER;
  }
}

void push(ChimeEvent& event, moto_tone_t tone, moto_voice_phrase_t voice,
          moto_voice_phrase_t prefix = MOTO_VOICE_NONE);

void push(ChimeEvent& event, moto_tone_t tone) {
  push(event, tone, MOTO_VOICE_NONE);
}

void push(ChimeEvent& event, moto_tone_t tone, moto_voice_phrase_t voice,
          moto_voice_phrase_t prefix) {
  if ((tone == MOTO_TONE_NONE && voice == MOTO_VOICE_NONE) ||
      event.count >= ChimeEvent::kMaxTones) {
    return;
  }
  event.tones[event.count] = tone;
  event.voices[event.count] = voice;
  event.prefixes[event.count] = prefix;
  ++event.count;
}

/// The spoken phrase for a maneuver, or MOTO_VOICE_NONE to keep the beep.
///
/// Only turns are spoken. A rider needs to know *which way* without looking,
/// which a pitch difference conveys badly; distance and road name would need
/// streamed speech the link cannot carry. Everything else stays a cue so the
/// panel never becomes a chatty passenger.
moto_voice_phrase_t arrival_voice(nav::ManeuverType maneuver) {
  switch (maneuver) {
    case nav::ManeuverType::SlightLeft:
    case nav::ManeuverType::Left:
    case nav::ManeuverType::SharpLeft:
    case nav::ManeuverType::UTurnLeft:
      return MOTO_VOICE_TURN_LEFT;
    case nav::ManeuverType::SlightRight:
    case nav::ManeuverType::Right:
    case nav::ManeuverType::SharpRight:
    case nav::ManeuverType::UTurnRight:
      return MOTO_VOICE_TURN_RIGHT;
    default:
      return MOTO_VOICE_NONE;
  }
}

}  // namespace

void ChimePolicy::reset() { *this = ChimePolicy{}; }

moto_tone_t ChimePolicy::on_notification(bool incoming_call,
                                        bool silent) const {
  if (silent) {
    return MOTO_TONE_NONE;
  }
  return incoming_call ? MOTO_TONE_CALL : MOTO_TONE_NOTIFICATION;
}

bool ChimePolicy::starts_new_maneuver(const ChimeInputs& inputs) {
  if (!has_last_maneuver_) {
    return true;
  }
  if (inputs.maneuver_id != 0 && inputs.maneuver_id != last_maneuver_id_) {
    return true;
  }
  // Without ids, passing a turn and being handed the next one shows up as the
  // distance jumping back up.  A reroute looks the same and deserves the cue
  // again, because the rider is looking at a new route either way.
  return inputs.distance_to_maneuver_m > last_distance_m_ + kRearmJumpM;
}

ChimeEvent ChimePolicy::update(const ChimeInputs& inputs) {
  ChimeEvent event;

  if (!inputs.navigating) {
    // No route, no guidance: silence, and forget where we were.
    has_last_maneuver_ = false;
    last_maneuver_id_ = 0;
    last_distance_m_ = 0;
    announced_far_ = false;
    announced_near_ = false;
    over_limit_ = false;
    return event;
  }

  if (inputs.has_maneuver) {
    if (starts_new_maneuver(inputs)) {
      announced_far_ = false;
      announced_near_ = false;
    }

    const double speed =
        inputs.speed_mps > kMinSpeedMps ? inputs.speed_mps : kMinSpeedMps;
    const double far_m =
        std::min(static_cast<double>(kStageOneMaxM), kStageOneSeconds * speed);
    const double near_m =
        std::min(static_cast<double>(kStageTwoMaxM), kStageTwoSeconds * speed);
    const double distance = static_cast<double>(inputs.distance_to_maneuver_m);

    if (!announced_near_ && distance <= near_m) {
      // Already close when the maneuver first appeared: the soft chirp would
      // only delay the cue that matters.
      announced_far_ = true;
      announced_near_ = true;
      push(event, arrival_tone(inputs.maneuver),
           arrival_voice(inputs.maneuver),
           distance_phrase(inputs.distance_to_maneuver_m));
    } else if (!announced_far_ && distance <= far_m) {
      announced_far_ = true;
      push(event, MOTO_TONE_TURN_AHEAD);
    }

    has_last_maneuver_ = true;
    last_maneuver_id_ = inputs.maneuver_id;
    last_distance_m_ = inputs.distance_to_maneuver_m;
  } else {
    has_last_maneuver_ = false;
    last_maneuver_id_ = 0;
    last_distance_m_ = 0;
    announced_far_ = false;
    announced_near_ = false;
  }

  if (inputs.speed_limit_kph > 0) {
    const double speed_kph = inputs.speed_mps * 3.6;
    const double limit = static_cast<double>(inputs.speed_limit_kph);
    if (speed_kph > limit + kLimitOverKph) {
      if (!over_limit_) {
        over_limit_ = true;
        push(event, MOTO_TONE_SPEED_LIMIT);
      }
    } else if (speed_kph <= limit + kLimitClearKph) {
      // Back near the limit re-arms the buzz, so one episode of speeding gets
      // one sound rather than a nagging one per update.
      over_limit_ = false;
    }
  } else {
    over_limit_ = false;
  }

  return event;
}

}  // namespace moto::audio
