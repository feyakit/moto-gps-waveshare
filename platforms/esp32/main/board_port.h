#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "lvgl.h"
#include "moto_audio_tones.h"
#include "moto_nav_ui.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
  MOTO_DISPLAY_WIDTH = MOTO_UI_CANVAS_WIDTH,
  MOTO_DISPLAY_HEIGHT = MOTO_UI_CANVAS_HEIGHT,
  MOTO_DISPLAY_BITS_PER_PIXEL = 16,
};

/**
 * Initialize only board/platform facilities required by the shared UI.
 *
 * A real implementation must:
 * - initialize LVGL exactly once;
 * - create one display matching MOTO_UI_CANVAS_WIDTH x
 *   MOTO_UI_CANVAS_HEIGHT using LV_COLOR_FORMAT_RGB565;
 * - attach the physical panel flush callback and compatible draw buffers;
 * - provide LVGL's millisecond tick;
 * - run lv_timer_handler from one board-owned FreeRTOS task; and
 * - serialize all other LVGL access through board_port_lock/unlock.
 *
 * It must not contain route parsing, navigation, rerouting, or UI layout logic.
 */
esp_err_t board_port_init(void);

/** Return the initialized LVGL display, or NULL before successful init. */
lv_display_t *board_port_get_display(void);

/**
 * Reveal the AMOLED after the caller has drawn the first intentional frame.
 * board_port_init keeps the panel dark so the LVGL default white screen can
 * never appear between panel reset and the boot animation.
 */
esp_err_t board_port_reveal_display(void);

/**
 * Lock LVGL for calls made outside the board-owned LVGL task.
 * timeout_ms == UINT32_MAX means wait indefinitely.
 */
bool board_port_lock(uint32_t timeout_ms);

/** Release a lock previously acquired with board_port_lock. */
void board_port_unlock(void);

/** GPIO18 is low while the board's active-low user key is physically held. */
/**
 * Set the AMOLED brightness, 0..100 percent. Zero is used by the parked/idle
 * screen policy: the panel stops emitting light while LVGL, BLE and the IMU
 * keep running, so the first touch or bump can light it again instantly.
 */
esp_err_t board_port_set_brightness(uint8_t percent);

/**
 * Queue one of the short cues defined in moto_audio_tones.h for the onboard
 * codec and speaker.
 *
 * Never blocks: the board's audio task plays queued tones in order, so this is
 * safe to call from the BLE receive callback or the LVGL task.  An id with no
 * waveform, or a board whose codec fails to start, stays silent.
 */
esp_err_t board_port_play_tone(moto_tone_t tone);

/// Sets the speaker level, 0..100. Safe before the codec ever opens: the value
/// is remembered and applied to the next playback. Never blocks.
esp_err_t board_port_set_volume(uint8_t percent);

/// Queues a flash-resident spoken phrase. Same contract as
/// board_port_play_tone: never blocks, and a cue that cannot be queued is
/// dropped rather than played late.
esp_err_t board_port_play_voice(moto_voice_phrase_t phrase);

bool board_port_power_button_pressed(void);

/// BOOT key, wired to GPIO0 with a pull-up. Active low.
///
/// GPIO0 is also a boot strapping pin: holding this key while the chip resets
/// selects the serial downloader instead of the application, so it may only be
/// read once the firmware is already running.
bool board_port_boot_button_pressed(void);

/// Reads and clears the AXP2101 interrupt-status registers (0x48..0x4A).
///
/// The PWR key is wired to the PMIC rather than to a GPIO, so this is the only
/// way a press reaches the firmware. Returns true when at least one bit was
/// set; `status` receives the three raw registers.
bool board_port_pmic_take_irq(uint8_t status[3]);

/// Reads the board RTC. False when the part is absent or the time was never
/// set, in which case the clock page must fall back to the phone.
bool board_port_rtc_read(int* year, int* month, int* day, int* hour,
                         int* minute, int* second, int* weekday);

/// Corrects the board RTC from a phone timestamp. The platform decides whether
/// the difference is large enough to be worth writing.
void board_port_rtc_sync(int year, int month, int day, int hour, int minute,
                         int second, int weekday);

/**
 * Request AXP2101 software power-off. Keep a deep-sleep fallback in the caller
 * for the USB-powered case until all three supply combinations are bench-tested.
 */
esp_err_t board_port_power_off(void);

#ifdef __cplusplus
}
#endif
