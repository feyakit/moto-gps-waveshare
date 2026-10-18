#include "board_port.h"

#include "bsp/esp32_s3_touch_amoled_2_16.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "moto/audio/volume_policy.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace {

constexpr char kTag[] = "moto_audio";

// bsp_audio_init(NULL) configures the I2S port for 22050 Hz mono 16-bit, and
// the codec is opened to match.  Short cues do not need more.
constexpr std::uint32_t kSampleRate = 22050;
constexpr std::uint8_t kBitsPerSample = 16;
constexpr std::uint8_t kChannelCount = 1;
// The codec output is at full scale: a bike at speed with a helmet is a much
// noisier place than a desk, and the panel has to be audible over wind. The
// digital samples stay well clear of clipping (kAmplitude), so this only lifts
// the analogue gain rather than making the waveform harsher.
// Boot default; the rider changes it with the user key and it is persisted by
// the application layer, which owns NVS.
constexpr int kVolumePercent = 100;
constexpr int kAmplitude = 8192;
constexpr int kSamplesPerChunk = 256;
// Attack and release in milliseconds: a tone that starts or stops mid-slope
// clicks through the amplifier.
constexpr int kRampMs = 4;
constexpr int kEnvelopeScale = 1024;
// An idle class-D amplifier hisses and draws current for nothing, so the codec
// and the amplifier are closed again once the cues stop.
constexpr int kIdleCloseMs = 2000;
constexpr std::size_t kQueueDepth = 4;
// Audible even on a muted panel: the limit cue is how a rider learns the press
// registered when the level is already at an extreme.
constexpr int kVolumeLimitAudible = 70;

// The previous level is restored after a forced cue, so the limit thud does
// not leave the panel louder than the rider asked for.
int s_volume_before_forced = -1;

constexpr int kTableSize = 64;
const std::int16_t kSine[kTableSize] = {
         0,   3212,   6393,   9512,  12539,  15446,  18204,  20787,
     23170,  25329,  27245,  28898,  30273,  31356,  32137,  32609,
     32767,  32609,  32137,  31356,  30273,  28898,  27245,  25329,
     23170,  20787,  18204,  15446,  12539,   9512,   6393,   3212,
         0,  -3212,  -6393,  -9512, -12539, -15446, -18204, -20787,
    -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
    -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329,
    -23170, -20787, -18204, -15446, -12539,  -9512,  -6393,  -3212,
};

/// A tone is a list of segments; frequency 0 is a pause.
struct Segment {
  std::uint16_t freq_hz;
  std::uint16_t ms;
};

// The waveform behind each id belongs to the board, so a future revision with
// a different speaker can retune the sounds without touching the shared policy.
// Direction is carried by pitch: low is left, high is right.
const Segment kTurnAhead[] = {{1400, 70}};
const Segment kTurnLeft[] = {{900, 90}, {0, 60}, {900, 90}};
const Segment kTurnRight[] = {{1800, 90}, {0, 60}, {1800, 90}};
const Segment kManeuver[] = {{1200, 80}, {0, 60}, {1200, 80}};
const Segment kArrive[] = {{1200, 110}, {0, 40}, {1600, 110}, {0, 40}, {2000, 160}};
const Segment kSpeedLimit[] = {{440, 320}};
const Segment kNotification[] = {{2000, 90}, {0, 40}, {2600, 130}};
const Segment kCall[] = {{1400, 130}, {0, 80}, {1750, 130}, {0, 80}, {2200, 220}};
// Rising pair: unmistakably "louder". The old shared ding could not tell the
// rider which direction a press had gone, which read as keys that did nothing.
const Segment kVolumeUp[] = {{1200, 70}, {0, 40}, {1800, 110}};
// Falling pair: unmistakably "quieter".
const Segment kVolumeDown[] = {{1800, 70}, {0, 40}, {1200, 110}};
// One low thud: already at the end of the range. Deliberately different from
// both the rising and the falling pair -- the old flat double beep at 900 Hz
// sat between them and was easily mistaken for "one more step down".
const Segment kVolumeLimit[] = {{320, 180}};

QueueHandle_t s_queue = nullptr;
esp_codec_dev_handle_t s_speaker = nullptr;
bool s_open = false;
// Written by the key task, read when the codec is opened and whenever a cue is
// played, so it has to be lock free.
std::atomic<int> s_volume{kVolumePercent};
std::mutex s_start_mutex;

bool pattern_for(moto_tone_t tone, const Segment** segments,
                 std::size_t* count) {
  *segments = nullptr;
  *count = 0;
  switch (tone) {
      case MOTO_TONE_TURN_AHEAD:
        *segments = kTurnAhead;
        *count = sizeof(kTurnAhead) / sizeof(kTurnAhead[0]);
        return true;
      case MOTO_TONE_TURN_LEFT:
        *segments = kTurnLeft;
        *count = sizeof(kTurnLeft) / sizeof(kTurnLeft[0]);
        return true;
      case MOTO_TONE_TURN_RIGHT:
        *segments = kTurnRight;
        *count = sizeof(kTurnRight) / sizeof(kTurnRight[0]);
        return true;
      case MOTO_TONE_MANEUVER:
        *segments = kManeuver;
        *count = sizeof(kManeuver) / sizeof(kManeuver[0]);
        return true;
      case MOTO_TONE_ARRIVE:
        *segments = kArrive;
        *count = sizeof(kArrive) / sizeof(kArrive[0]);
        return true;
      case MOTO_TONE_SPEED_LIMIT:
        *segments = kSpeedLimit;
        *count = sizeof(kSpeedLimit) / sizeof(kSpeedLimit[0]);
        return true;
      case MOTO_TONE_NOTIFICATION:
        *segments = kNotification;
        *count = sizeof(kNotification) / sizeof(kNotification[0]);
        return true;
      case MOTO_TONE_CALL:
        *segments = kCall;
        *count = sizeof(kCall) / sizeof(kCall[0]);
        return true;
      case MOTO_TONE_VOLUME_UP:
        *segments = kVolumeUp;
        *count = sizeof(kVolumeUp) / sizeof(kVolumeUp[0]);
        return true;
      case MOTO_TONE_VOLUME_DOWN:
        *segments = kVolumeDown;
        *count = sizeof(kVolumeDown) / sizeof(kVolumeDown[0]);
        return true;
      case MOTO_TONE_VOLUME_LIMIT:
        *segments = kVolumeLimit;
        *count = sizeof(kVolumeLimit) / sizeof(kVolumeLimit[0]);
        return true;
    default:
      return false;
  }
}

/// 0..kEnvelopeScale, ramped at both ends of the segment.
int envelope(int index, int total, int ramp) {
  if (ramp <= 0 || total <= 2 * ramp) {
    return kEnvelopeScale;
  }
  if (index < ramp) {
    return kEnvelopeScale * index / ramp;
  }
  if (index >= total - ramp) {
    return kEnvelopeScale * (total - 1 - index) / ramp;
  }
  return kEnvelopeScale;
}

bool ensure_speaker_open() {
  if (s_speaker == nullptr) {
    // Also configures I2S and the power-amplifier pin (GPIO46).
    s_speaker = bsp_audio_codec_speaker_init();
    if (s_speaker == nullptr) {
      ESP_LOGW(kTag, "ES8311 speaker is unavailable; cues stay silent");
      return false;
    }
    esp_codec_dev_set_out_vol(s_speaker, s_volume.load());
  }
  if (!s_open) {
    esp_codec_dev_sample_info_t format = {};
    format.bits_per_sample = kBitsPerSample;
    format.channel = kChannelCount;
    format.sample_rate = kSampleRate;
    if (esp_codec_dev_open(s_speaker, &format) != ESP_CODEC_DEV_OK) {
      ESP_LOGW(kTag, "could not open the speaker path");
      return false;
    }
    // Reopening the path after the idle close resets the codec's gain, and
    // s_speaker stays non-null across that cycle, so the level has to be
    // applied here rather than only at first init.
    esp_codec_dev_set_out_vol(s_speaker, s_volume.load());
    s_open = true;
  }
  return true;
}

void close_speaker() {
  if (s_open && s_speaker != nullptr) {
    // Closing also releases the power amplifier.
    esp_codec_dev_close(s_speaker);
    s_open = false;
  }
}

void play_pattern(const Segment* segments, std::size_t count) {
  std::int16_t chunk[kSamplesPerChunk];
  // One phase across the whole tone, so a pause does not restart the wave with
  // a step and a click.
  std::uint32_t phase = 0;
  for (std::size_t index = 0; index < count; ++index) {
    const Segment& segment = segments[index];
    const int total =
        static_cast<int>(kSampleRate) * segment.ms / 1000;
    const int ramp = static_cast<int>(kSampleRate) * kRampMs / 1000;
    const std::uint32_t step = segment.freq_hz == 0
        ? 0U
        : static_cast<std::uint32_t>(
              static_cast<std::uint64_t>(segment.freq_hz) * kTableSize *
              65536ULL / kSampleRate);
    int produced = 0;
    while (produced < total) {
      const int limit = std::min(kSamplesPerChunk, total - produced);
      for (int i = 0; i < limit; ++i) {
        std::int16_t sample = 0;
        if (step != 0U) {
          const int scaled =
              static_cast<int>(kSine[(phase >> 16) & (kTableSize - 1)]) *
              kAmplitude / 32767;
          sample = static_cast<std::int16_t>(
              scaled * envelope(produced + i, total, ramp) / kEnvelopeScale);
          phase += step;
        }
        chunk[i] = sample;
      }
      if (esp_codec_dev_write(s_speaker, chunk,
                              limit * static_cast<int>(sizeof(std::int16_t))) !=
          ESP_CODEC_DEV_OK) {
        ESP_LOGW(kTag, "speaker write failed; dropping the rest of the cue");
        return;
      }
      produced += limit;
    }
  }
}

/// Plays a flash-resident spoken phrase, resampling 16 kHz PCM to the codec's
/// 22050 Hz. Linear interpolation is plenty for speech on this speaker and
/// costs nothing measurable at 24 samples per output sample.
void play_voice(moto_voice_phrase_t phrase) {
  const std::uint8_t* data = nullptr;
  std::uint32_t bytes = 0;
  if (!moto_voice_phrase_data(phrase, &data, &bytes) || data == nullptr ||
      bytes < 2) {
    return;
  }
  if (!ensure_speaker_open()) {
    return;
  }
  const auto* samples = reinterpret_cast<const std::int16_t*>(data);
  const std::uint32_t sample_count = bytes / 2;
  // Fixed point: source position advances by 16000/22050 per output sample.
  constexpr std::uint32_t kStep = 65536U * 16000U / 22050U;
  std::uint32_t position = 0;
  std::int16_t chunk[kSamplesPerChunk];
  while (position / 65536U < sample_count) {
    std::uint32_t index = position / 65536U;
    const std::uint32_t limit = std::min<std::uint32_t>(
        kSamplesPerChunk, sample_count - index);
    for (std::uint32_t i = 0; i < limit; ++i) {
      const std::uint32_t whole = position / 65536U;
      const std::uint32_t frac = position % 65536U;
      if (whole + 1U >= sample_count) {
        chunk[i] = 0;
      } else {
        const std::int32_t a = samples[whole];
        const std::int32_t b = samples[whole + 1U];
        chunk[i] = static_cast<std::int16_t>(
            a + ((b - a) * static_cast<std::int32_t>(frac)) / 65536);
      }
      position += kStep;
    }
    if (esp_codec_dev_write(s_speaker, chunk,
                            static_cast<int>(limit * sizeof(std::int16_t))) !=
        ESP_CODEC_DEV_OK) {
      ESP_LOGW(kTag, "speaker write failed; dropping the rest of the phrase");
      return;
    }
  }
}

/// Puts the rider's level back after a forced-audible cue.
void restore_volume() {
  s_volume.store(s_volume_before_forced >= 0 ? s_volume_before_forced
                                             : kVolumePercent);
  s_volume_before_forced = -1;
  if (s_speaker != nullptr && s_open) {
    esp_codec_dev_set_out_vol(s_speaker, s_volume.load());
  }
}

void audio_task(void*) {
  moto_audio_cue_t cue = {};
  while (true) {
    if (xQueueReceive(s_queue, &cue, pdMS_TO_TICKS(kIdleCloseMs)) != pdTRUE) {
      close_speaker();
      continue;
    }
    // The "at the limit" thud is the one cue that must survive a muted panel:
    // it is the only thing that tells the rider their press was understood,
    // and by definition it fires when the level is at an extreme.
    const bool forced = cue.tone == MOTO_TONE_VOLUME_LIMIT;
    if (forced) {
      s_volume_before_forced = s_volume.load();
      s_volume.store(kVolumeLimitAudible);
    }
    if (cue.voice != MOTO_VOICE_NONE) {
      play_voice(cue.voice);
      if (forced) restore_volume();
      continue;
    }
    const Segment* segments = nullptr;
    std::size_t count = 0;
    if (pattern_for(cue.tone, &segments, &count) && ensure_speaker_open()) {
      play_pattern(segments, count);
    }
    if (forced) restore_volume();
  }
}

}  // namespace

esp_err_t board_port_play_tone(moto_tone_t tone) {
  if (tone == MOTO_TONE_NONE) {
    return ESP_OK;
  }
  {
    const std::lock_guard<std::mutex> lock(s_start_mutex);
    if (s_queue == nullptr) {
      s_queue = xQueueCreate(kQueueDepth, sizeof(moto_audio_cue_t));
      if (s_queue == nullptr) {
        return ESP_ERR_NO_MEM;
      }
      if (xTaskCreate(audio_task, "moto_audio", 4096, nullptr, 2, nullptr) !=
          pdPASS) {
        vQueueDelete(s_queue);
        s_queue = nullptr;
        return ESP_FAIL;
      }
    }
  }
  // Never waits: a cue that arrives late is worse than one that is dropped.
  // The board plays tones in order and a burst cannot outrun this queue.
  const moto_audio_cue_t cue = {tone, MOTO_VOICE_NONE};
  return xQueueSend(s_queue, &cue, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t board_port_set_volume(uint8_t percent) {
  const int level = moto::audio::clamp_percent(percent);
  s_volume.store(level);
  if (s_speaker == nullptr || !s_open) {
    // Nothing is playing yet; the level is applied when the codec opens.
    return ESP_OK;
  }
  return esp_codec_dev_set_out_vol(s_speaker, level) == ESP_CODEC_DEV_OK
             ? ESP_OK
             : ESP_FAIL;
}

esp_err_t board_port_play_voice(moto_voice_phrase_t phrase) {
  if (phrase == MOTO_VOICE_NONE) {
    return ESP_OK;
  }
  {
    const std::lock_guard<std::mutex> lock(s_start_mutex);
    if (s_queue == nullptr) {
      s_queue = xQueueCreate(kQueueDepth, sizeof(moto_audio_cue_t));
      if (s_queue == nullptr) {
        return ESP_ERR_NO_MEM;
      }
      if (xTaskCreate(audio_task, "moto_audio", 4096, nullptr, 2, nullptr) !=
          pdPASS) {
        vQueueDelete(s_queue);
        s_queue = nullptr;
        return ESP_FAIL;
      }
    }
  }
  const moto_audio_cue_t cue = {MOTO_TONE_NONE, phrase};
  return xQueueSend(s_queue, &cue, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}
