#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The short sounds the round panel can play through the onboard speaker.
 *
 * The shared chime policy decides *when* one of these is worth playing; the
 * waveform behind each id belongs to the board, so a future revision with a
 * different speaker can retune the sounds without touching the decision logic.
 *
 * Kept as a C header because the board HAL is C, and the policy module is C++.
 */
typedef enum {
    MOTO_TONE_NONE = 0,
    MOTO_TONE_TURN_AHEAD,   /**< Soft chirp: a turn is coming up. */
    MOTO_TONE_TURN_LEFT,    /**< Low double beep. */
    MOTO_TONE_TURN_RIGHT,   /**< High double beep. */
    MOTO_TONE_MANEUVER,     /**< Roundabout, exit, U-turn: neutral double beep. */
    MOTO_TONE_ARRIVE,       /**< Rising triad when the destination is reached. */
    MOTO_TONE_SPEED_LIMIT,  /**< Low buzz: over the posted limit. */
    MOTO_TONE_NOTIFICATION, /**< One short ding. */
    MOTO_TONE_CALL,         /**< Ring pattern for an incoming call. */
    /* Volume keys. Each direction has its own shape so the rider can hear
       which way a press went without looking at the panel. */
    MOTO_TONE_VOLUME_UP,    /**< Rising pair. */
    MOTO_TONE_VOLUME_DOWN,  /**< Falling pair. */
    MOTO_TONE_VOLUME_LIMIT, /**< Flat pair: already at the end of the range. */
} moto_tone_t;

/**
 * Pre-rendered spoken phrases stored in flash.
 *
 * Streamed text-to-speech cannot work here: the BLE link carries about
 * 100 kbit/s while 16 kHz speech needs 256 kbit/s, so a sentence would break
 * up exactly when the rider needs it. These clips are produced ahead of time
 * by scripts/generate_voice_phrases.sh and play from flash on the millisecond
 * the cue fires, with or without the phone connected.
 *
 * The order must match the phrase list in that script.
 */
typedef enum {
    MOTO_VOICE_NONE = 0,
    MOTO_VOICE_TURN_LEFT,    /**< 前方左转 */
    MOTO_VOICE_TURN_RIGHT,   /**< 前方右转 */
    MOTO_VOICE_UTURN,        /**< 前方掉头 */
    MOTO_VOICE_ROUNDABOUT,   /**< 前方环岛 */
    MOTO_VOICE_ARRIVE,       /**< 已到达目的地 */
    MOTO_VOICE_CONGESTION,   /**< 前方拥堵 */
    MOTO_VOICE_SLOW_TRAFFIC, /**< 前方缓行 */
    MOTO_VOICE_OFF_ROUTE,    /**< 已偏航，正在重新规划 */
    MOTO_VOICE_SLOW_DOWN,    /**< 请减速 */
    MOTO_VOICE_CAUTION,      /**< 注意安全 */
} moto_voice_phrase_t;

/**
 * One queued sound: exactly one of the two fields is meaningful.
 *
 * A cue carries either a synthesized tone or a spoken phrase, never both, so
 * the audio task can decide between them with a single branch.
 */
typedef struct {
    moto_tone_t tone;
    moto_voice_phrase_t voice;
} moto_audio_cue_t;

/**
 * Looks up the PCM data for one phrase.
 *
 * The audio is 16 kHz mono signed 16-bit little-endian, already trimmed and
 * normalised; the board resamples it to the codec's rate. Returns false for an
 * unknown id so the caller can fall back to the synthesized cue.
 *
 * Defined in the generated shared/audio/src/moto_voice_phrases.c.
 */
bool moto_voice_phrase_data(moto_voice_phrase_t phrase, const uint8_t** data,
                            uint32_t* length);

#ifdef __cplusplus
}  /* extern "C" */
#endif
