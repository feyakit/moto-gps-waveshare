#include <cstdint>

#include "ble_nav_transport_nimble.h"
#include "board_port.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "moto_nav_presenter.hpp"
#include "moto_nav_ui.h"
#include "moto/audio/volume_policy.hpp"
#include "motion_heading_sensor.h"
#include "nvs.h"
#include "phone_nav_bridge.h"

namespace {
constexpr char kTag[] = "moto_gps";
constexpr std::uint64_t kPowerHoldMs = 3'000;
// Anything shorter than this is contact bounce, not the rider asking for a
// different speaker level.
constexpr std::uint64_t kShortPressMinMs = 60;
constexpr char kVolumeNvsNamespace[] = "moto";
constexpr char kVolumeNvsKey[] = "volume";
constexpr std::uint8_t kVolumeDefault = 100;
// Panel brightness while the rider is looking at the screen. Zero is reserved
// for the parked/idle policy in PhoneNavBridge.
constexpr std::uint8_t kScreenOnBrightness = 100;
// Night is a glare problem, not a power problem: the panel has to stay
// readable without destroying the rider's dark adaptation.
constexpr std::uint8_t kNightBrightness = 35;
constexpr std::uint32_t kStatusPollMs = 5'000;

PhoneNavBridge* g_bridge = nullptr;
std::uint8_t g_brightness = kScreenOnBrightness;

moto::ble::AckStatus receive_phone_message(
    const moto::ble::ReassembledMessage& message, void* context) {
  return static_cast<PhoneNavBridge*>(context)->on_message(message);
}

void update_phone_link(bool active, void* context) {
  static_cast<PhoneNavBridge*>(context)->on_link_state(active);
}

void update_motion_heading(float heading_rate_dps, float accel_mps2,
                           std::uint64_t sample_ms, void* context) {
  auto* bridge = static_cast<PhoneNavBridge*>(context);
  bridge->on_imu_sample(heading_rate_dps, sample_ms);
  bridge->on_motion_sample(accel_mps2, sample_ms);
}

// Called on the NimBLE host task for every notification the phone publishes to
// the ANCS session.
void update_notification(const moto::ancs::NotificationSource& source,
                         const moto::ancs::NotificationAttributes& attributes,
                         void* context) {
  static_cast<PhoneNavBridge*>(context)->on_notification(source, attributes);
}

// Driven by PhoneNavBridge when the parked/idle policy blanks or lights the
// panel. Called from the IMU, BLE or LVGL thread, so it must not take the LVGL
// lock: esp_lcd queues the brightness command on its own bus lock.
void screen_power_changed(bool on, void*) {
  const std::uint8_t target =
      on ? (g_bridge != nullptr && g_bridge->is_night_time() ? kNightBrightness
                                                             : kScreenOnBrightness)
         : 0;
  g_brightness = target;
  const esp_err_t result = board_port_set_brightness(target);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "AMOLED brightness change failed: %s",
             esp_err_to_name(result));
  }
}

/// The bridge decides which cue a situation deserves; the board owns the
/// waveform.  Called from the BLE task inside present_navigation(), so it must
/// only queue the tone.
void play_tone(moto_tone_t tone, void*) {
  const esp_err_t result = board_port_play_tone(tone);
  if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
    ESP_LOGW(kTag, "cue %d could not be queued: %s", static_cast<int>(tone),
             esp_err_to_name(result));
  }
}

/// The phone pushed a time that the RTC disagrees with; correct the chip.
/// Called on the BLE task, so it must stay short: one I2C transaction.
void rtc_write_back(const moto::rtc_clock::Time& time, void*) {
  board_port_rtc_sync(time.year, time.month, time.day, time.hour, time.minute,
                      time.second, time.weekday);
}

/// Seed the clock page from the board RTC so it shows the right time before
/// any phone connects.
void seed_clock(PhoneNavBridge& bridge) {
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  int weekday = 0;
  if (!board_port_rtc_read(&year, &month, &day, &hour, &minute, &second,
                           &weekday)) {
    ESP_LOGI(kTag, "RTC has no usable time yet; clock follows the phone");
    return;
  }
  moto::rtc_clock::Time time{};
  time.year = static_cast<std::uint16_t>(year);
  time.month = static_cast<std::uint8_t>(month);
  time.day = static_cast<std::uint8_t>(day);
  time.hour = static_cast<std::uint8_t>(hour);
  time.minute = static_cast<std::uint8_t>(minute);
  time.second = static_cast<std::uint8_t>(second);
  time.weekday = static_cast<std::uint8_t>(weekday);
  bridge.seed_clock_from_rtc(time);
  ESP_LOGI(kTag, "clock seeded from RTC: %02d:%02d", hour, minute);
}

/// Spoken navigation phrases come from flash, so this only queues one.
void play_voice(moto_voice_phrase_t phrase, void*) {
  const esp_err_t result = board_port_play_voice(phrase);
  if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
    ESP_LOGW(kTag, "phrase %d could not be queued: %s", static_cast<int>(phrase),
             esp_err_to_name(result));
  }
}

void demo_tick_task(void* context) {
  auto* bridge = static_cast<PhoneNavBridge*>(context);
  while (true) {
    const auto now_ms = static_cast<std::uint64_t>(esp_timer_get_time()) /
                        1'000U;
    bridge->update_demo(now_ms);
    bridge->tick_clock();
    // Match the display and route interpolation at 40 Hz. Keeping all three
    // clocks phase-compatible avoids periodic long frame gaps.
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

/// Speaker level the key cycles through. Loaded once at boot so a press never
/// has to touch flash.
std::uint8_t g_volume = kVolumeDefault;

std::uint8_t load_volume() {
  nvs_handle_t handle = 0;
  if (nvs_open(kVolumeNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
    return kVolumeDefault;
  }
  std::uint8_t value = kVolumeDefault;
  const esp_err_t result = nvs_get_u8(handle, kVolumeNvsKey, &value);
  nvs_close(handle);
  return result == ESP_OK ? moto::audio::clamp_percent(value) : kVolumeDefault;
}

void save_volume(std::uint8_t value) {
  nvs_handle_t handle = 0;
  if (nvs_open(kVolumeNvsNamespace, NVS_READWRITE, &handle) != ESP_OK) {
    return;
  }
  nvs_set_u8(handle, kVolumeNvsKey, value);
  nvs_commit(handle);
  nvs_close(handle);
}

/// Applies and persists one new speaker level, with a cue as feedback.
/// `direction` picks the cue; it is +1 for a louder press, -1 for quieter.
/// A press that saturates still answers with the limit cue, because silence
/// there is indistinguishable from a key that failed to register.
void apply_volume(std::uint8_t level, int direction, const char* source) {
  if (level == g_volume) {
    ESP_LOGI(kTag, "volume stays %u%% (%s, at limit)",
             static_cast<unsigned>(level), source);
    board_port_play_tone(MOTO_TONE_VOLUME_LIMIT);
    return;
  }
  g_volume = level;
  board_port_set_volume(level);
  save_volume(level);
  ESP_LOGI(kTag, "volume -> %u%% (%s)", static_cast<unsigned>(level), source);
  board_port_play_tone(direction > 0 ? MOTO_TONE_VOLUME_UP
                                     : MOTO_TONE_VOLUME_DOWN);
}


/// Watches the PMIC's interrupt-status registers so a PWR-key press can be
/// told apart from a battery or charger event.
///
/// The PWR key is wired to the AXP2101, not to a GPIO, so the firmware cannot
/// read it directly; these three registers are the only way the press reaches
/// the ESP32. Reading is non-destructive and the same task clears what it
/// reports, so each press is logged once.
/// BOOT key: volume down. Reads GPIO0 only while the firmware is running --
/// holding it during reset would select the serial downloader instead.
void boot_key_task(void*) {
  bool pressed = false;
  std::uint64_t pressed_since_ms = 0;
  while (true) {
    const std::uint64_t now_ms =
        static_cast<std::uint64_t>(esp_timer_get_time()) / 1'000U;
    const bool down = board_port_boot_button_pressed();
    if (down && !pressed) {
      pressed = true;
      pressed_since_ms = now_ms;
      ESP_LOGI(kTag, "BOOT key pressed");
    } else if (!down && pressed) {
      pressed = false;
      const std::uint64_t held_ms = now_ms - pressed_since_ms;
      if (held_ms >= kShortPressMinMs && held_ms < kPowerHoldMs) {
        apply_volume(moto::audio::volume_down(g_volume), -1, "BOOT key");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void pmu_key_task(void*) {
  std::uint64_t last_press_ms = 0;
  while (true) {
    std::uint8_t status[3] = {};
    if (board_port_pmic_take_irq(status)) {
      // 0x49 bits 0 and 3 are what a PWRON press latches on this board; other
      // bits there belong to the charger and must not change the volume.
      constexpr std::uint8_t kPwrKeyMask = 0x09;
      const std::uint64_t now_ms =
          static_cast<std::uint64_t>(esp_timer_get_time()) / 1'000U;
      if ((status[1] & kPwrKeyMask) != 0) {
        // The PWR key stays the power key: it is reported here only so a field
        // problem can tell a PMU event from a real key press.
        if (now_ms - last_press_ms >= 250) {
          last_press_ms = now_ms;
          ESP_LOGI(kTag, "PMU PWR key press");
        }
      } else {
        ESP_LOGI(kTag, "PMU event (not the key): 48=%02x 49=%02x 4A=%02x",
                 status[0], status[1], status[2]);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/// Battery, link quality and day/night dimming, all on one slow timer.
///
/// They share a task because none of them is urgent and each costs one I2C or
/// HCI round trip; keeping them together also means the panel is never asked
/// to repaint for two of them in the same frame.
struct StatusContext {
  PhoneNavBridge* bridge;
  BleNavTransport* transport;
};

void status_task(void* context) {
  auto* ctx = static_cast<StatusContext*>(context);
  bool low_warned = false;
  bool battery_logged = false;
  std::uint8_t last_bars = 0xFFU;
  while (true) {
    moto_battery_t battery{};
    if (board_port_battery(&battery)) {
      // 0xFF tells the UI "no pack fitted", which is a different statement
      // from "0% left" on a bench supply.
      ctx->bridge->set_battery(battery.present != 0U ? battery.percent : 0xFFU,
                               battery.charging != 0U);
      if (!battery_logged) {
        battery_logged = true;
        ESP_LOGI(kTag, "battery: %u%% %umV charging=%u present=%u",
                 static_cast<unsigned>(battery.percent),
                 static_cast<unsigned>(battery.millivolts),
                 static_cast<unsigned>(battery.charging),
                 static_cast<unsigned>(battery.present));
      }
      const bool low = battery.present != 0U && battery.charging == 0U &&
                       battery.percent <= 15U;
      if (low && !low_warned) {
        low_warned = true;
        ESP_LOGW(kTag, "battery low: %u%%", battery.percent);
        board_port_play_tone(MOTO_TONE_SPEED_LIMIT);
      } else if (!low) {
        low_warned = false;
      }
    }

    // 0 dBm means no link; bars are coarse on purpose so they do not flicker
    // while the phone moves around inside the jacket.
    const int rssi = ctx->transport->rssi();
    const std::uint8_t bars =
        rssi == 0 ? 0U
                  : rssi >= -60 ? 3U : rssi >= -75 ? 2U : 1U;
    if (bars != last_bars) {
      last_bars = bars;
      ESP_LOGI(kTag, "link bars -> %u (rssi %d dBm)",
               static_cast<unsigned>(bars), rssi);
    }
    ctx->bridge->set_link_quality(bars);

    if (!ctx->bridge->screen_off()) {
      const std::uint8_t target = ctx->bridge->is_night_time()
                                      ? kNightBrightness
                                      : kScreenOnBrightness;
      if (target != g_brightness) {
        g_brightness = target;
        board_port_set_brightness(target);
        ESP_LOGI(kTag, "brightness -> %u%%", static_cast<unsigned>(target));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(kStatusPollMs));
  }
}

void power_button_task(void* context) {
  auto* bridge = static_cast<PhoneNavBridge*>(context);
  std::uint64_t pressed_since_ms = 0;
  // GPIO18 is a separate active-low user key, not the AXP2101 power-on input.
  ESP_LOGI(kTag, "GPIO18 user-key hold detector ready");
  while (true) {
    const std::uint64_t now_ms =
        static_cast<std::uint64_t>(esp_timer_get_time()) / 1'000U;
    if (!board_port_power_button_pressed()) {
      if (pressed_since_ms != 0) {
        const std::uint64_t held_ms = now_ms - pressed_since_ms;
        pressed_since_ms = 0;
        // A short press is the volume key: the rider cannot look down and drag
        // a slider, and a four-step cycle is predictable without feedback
        // beyond one cue.
        if (held_ms >= kShortPressMinMs && held_ms < kPowerHoldMs) {
          apply_volume(moto::audio::volume_up(g_volume), +1, "user key");
        }
      }
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (pressed_since_ms == 0) {
      pressed_since_ms = now_ms;
      // The key is also the wake gesture: a short press on a blanked panel
      // lights it again, and a held press shuts down over a visible screen.
      if (bridge != nullptr) {
        bridge->note_user_activity();
      }
      ESP_LOGI(kTag, "PWR press started");
    } else if (now_ms - pressed_since_ms >= kPowerHoldMs) {
      ESP_LOGI(kTag, "PWR held for 3 seconds; requesting shutdown");
      if (board_port_lock(UINT32_MAX)) {
        moto_nav_ui_show_power_off_screen();
        board_port_unlock();
      }
      vTaskDelay(pdMS_TO_TICKS(300));
      const esp_err_t result = board_port_power_off();
      if (result != ESP_OK) {
        ESP_LOGE(kTag, "AXP2101 software power-off failed: %s",
                 esp_err_to_name(result));
      }

      // If USB power keeps the board alive after PMIC shutdown, sleep after
      // releasing GPIO18 and wake when the active-low key is pressed again.
      vTaskDelay(pdMS_TO_TICKS(500));
      while (board_port_power_button_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(
          1ULL << 18, ESP_EXT1_WAKEUP_ANY_LOW));
      esp_deep_sleep_start();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
} // namespace

extern "C" void app_main(void) {
  static_assert(MOTO_DISPLAY_WIDTH == MOTO_UI_CANVAS_WIDTH &&
                    MOTO_DISPLAY_HEIGHT == MOTO_UI_CANVAS_HEIGHT,
                "The board and shared UI canvas must have one resolution");
  static_assert(LV_COLOR_DEPTH == MOTO_DISPLAY_BITS_PER_PIXEL,
                "Firmware and Web must both build the shared UI as RGB565");

  ESP_LOGI(kTag, "starting %dx%d RGB565 firmware target",
           MOTO_DISPLAY_WIDTH, MOTO_DISPLAY_HEIGHT);

  const esp_err_t init_result = board_port_init();
  if (init_result != ESP_OK) {
    ESP_LOGE(kTag, "board initialization stopped before shared UI startup: %s",
             esp_err_to_name(init_result));
    vTaskDelete(nullptr);
    return;
  }

  lv_display_t *const display = board_port_get_display();
  if (display == nullptr) {
    ESP_LOGE(kTag, "board port returned no LVGL display");
    vTaskDelete(nullptr);
    return;
  }

  if (lv_display_get_horizontal_resolution(display) != MOTO_DISPLAY_WIDTH ||
      lv_display_get_vertical_resolution(display) != MOTO_DISPLAY_HEIGHT ||
      lv_display_get_color_format(display) != LV_COLOR_FORMAT_RGB565) {
    ESP_LOGE(kTag, "board display violates the shared RGB565 portability "
                   "contract");
    vTaskDelete(nullptr);
    return;
  }

  if (!board_port_lock(UINT32_MAX)) {
    ESP_LOGE(kTag, "could not acquire LVGL lock");
    vTaskDelete(nullptr);
    return;
  }

  moto_nav_ui_show_boot_screen();
  // Commit the deliberately black first animation frame while the physical
  // panel is still hidden, then reveal it.  This removes the white frame that
  // used to leak from LVGL's default startup screen.
  lv_refr_now(display);
  const esp_err_t reveal_result = board_port_reveal_display();
  if (reveal_result != ESP_OK) {
    ESP_LOGE(kTag, "could not reveal boot animation: %s",
             esp_err_to_name(reveal_result));
  }
  board_port_unlock();

  // Give the display worker enough time to commit the monochrome power-on
  // frame before constructing the full production UI.
  vTaskDelay(pdMS_TO_TICKS(1'250));

  if (!board_port_lock(UINT32_MAX)) {
    ESP_LOGE(kTag, "could not reacquire LVGL lock after boot screen");
    vTaskDelete(nullptr);
    return;
  }

  moto_nav_ui_create();
  static moto::nav::NavPresenter presenter;
  static PhoneNavBridge phone_bridge(presenter);
  static BleNavTransport transport;
  // NavSnapshot contains the complete bounded roads/buildings window and is
  // several kilobytes.  A temporary here inflates app_main's stack frame for
  // the entire display/BSP startup call chain, which can trip the FreeRTOS
  // main-task canary before this line is even reached.  Keep the immutable
  // initial state in static storage just like the bridge's retained snapshots.
  static const moto::nav::NavSnapshot initial_snapshot{};
  presenter.update(initial_snapshot);
  presenter.apply_to_lvgl();
  moto_nav_ui_set_music_page_enabled(0);
  phone_bridge.install_ui_callbacks();
  board_port_unlock();

  if (!phone_bridge.start_renderer()) {
    ESP_LOGE(kTag, "UI renderer startup failed; BLE was not started");
    vTaskDelete(nullptr);
    return;
  }

  phone_bridge.set_sender(BleNavTransport::send_from_bridge, &transport);
  transport.set_callbacks(receive_phone_message, update_phone_link,
                          &phone_bridge);
  const esp_err_t ble_result = transport.start();
  if (ble_result != ESP_OK) {
    ESP_LOGE(kTag, "BLE startup failed; display remains in offline mode: %s",
             esp_err_to_name(ble_result));
  }

  phone_bridge.set_screen_power_callback(screen_power_changed, nullptr);
  phone_bridge.set_tone_callback(play_tone, nullptr);
  phone_bridge.set_voice_callback(play_voice, nullptr);
  phone_bridge.set_rtc_hook(rtc_write_back, nullptr);
  // The Apple notification client rides along on the iPhone's own link, so
  // installing its callback is all the wiring it needs.
  transport.ancs_client().set_notification_callback(update_notification,
                                                    &phone_bridge);

  static MotionHeadingSensor motion_sensor;
  const esp_err_t motion_result = motion_sensor.start(update_motion_heading,
                                                      &phone_bridge);
  if (motion_result != ESP_OK) {
    ESP_LOGW(kTag, "QMI8658 heading assist could not start: %s",
             esp_err_to_name(motion_result));
  }

  // Demo generation fills the retained snapshot in place, but geometry and
  // LVGL projection still use deeper C++ call frames than a trivial task.
  if (xTaskCreate(demo_tick_task, "moto_demo", 6'144, &phone_bridge, 2,
                  nullptr) != pdPASS) {
    ESP_LOGW(kTag, "navigation demo task could not start");
  }
  g_volume = load_volume();
  board_port_set_volume(g_volume);
  ESP_LOGI(kTag, "speaker volume %u%%", static_cast<unsigned>(g_volume));

  if (xTaskCreate(power_button_task, "moto_power", 3'072, &phone_bridge, 3,
                  nullptr) != pdPASS) {
    ESP_LOGW(kTag, "PWR long-hold task could not start");
  }
  if (xTaskCreate(pmu_key_task, "moto_pmu_key", 2'048, nullptr, 3,
                  nullptr) != pdPASS) {
    ESP_LOGW(kTag, "PMU key task could not start");
  }
  if (xTaskCreate(boot_key_task, "moto_boot_key", 2'048, nullptr, 3,
                  nullptr) != pdPASS) {
    ESP_LOGW(kTag, "BOOT key task could not start");
  }

  g_bridge = &phone_bridge;
  static StatusContext status_context{&phone_bridge, &transport};
  if (xTaskCreate(status_task, "moto_status", 3'072, &status_context, 2,
                  nullptr) != pdPASS) {
    ESP_LOGW(kTag, "status task could not start");
  }

  // Last, so the page is painted before the first minute tick.
  seed_clock(phone_bridge);

  ESP_LOGI(kTag,
           "iPhone BLE + QMI heading -> NavPresenter -> shared LVGL running");
  vTaskDelete(nullptr);
}
