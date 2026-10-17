#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>

#include "ancs_protocol.hpp"
#include "moto/ble_protocol/ble_protocol.hpp"
#include "moto/audio/chime_policy.hpp"
#include "moto_nav_presenter.hpp"
#include "motion_heading_fusion.hpp"
#include "motion_idle_detector.hpp"
#include "rtc_clock.hpp"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

class PhoneNavBridge {
 public:
  using SendCallback = bool (*)(const moto::ble::Message& message,
                                std::uint8_t frame_flags,
                                void* context);
  /// Reports a blank/lit panel request.  Called on the thread that detected
  /// the transition, never with the bridge lock held.
  using ScreenPowerCallback = void (*)(bool on, void* context);
  /// Reports a sound the panel should play.  See set_tone_callback.
  using ToneCallback = void (*)(moto_tone_t tone, void* context);
  /// Reports a spoken phrase for the board to play from flash. Same contract as
  /// ToneCallback: never with the bridge lock held, and must not block.
  using VoiceCallback = void (*)(moto_voice_phrase_t phrase, void* context);

  explicit PhoneNavBridge(moto::nav::NavPresenter& presenter);

  void set_sender(SendCallback callback, void* context) noexcept;
  void set_screen_power_callback(ScreenPowerCallback callback,
                                 void* context) noexcept;
  /// Reports a sound the panel should play, for the navigation cues and the
  /// notification ding.  Called on the thread that noticed the situation,
  /// never with the bridge lock held, and must not block: the board queues the
  /// tone and plays it on its own task.
  void set_tone_callback(ToneCallback callback, void* context) noexcept;
  void set_voice_callback(VoiceCallback callback, void* context) noexcept;
  /// Writes the board's RTC when the phone's time disagrees with it. Called
  /// outside the bridge lock, on the thread that received the push.
  using RtcWriteCallback = void (*)(const moto::rtc_clock::Time& time,
                                    void* context);
  void set_rtc_hook(RtcWriteCallback callback, void* context) noexcept;
  /// Seeds the clock page from the board RTC at boot, before any phone link.
  void seed_clock_from_rtc(const moto::rtc_clock::Time& time);
  /// Advances the clock page once a minute when the phone is not updating it.
  /// Cheap enough to call at the display refresh rate.
  void tick_clock();
  void install_ui_callbacks();
  // Starts the sole background owner of NavPresenter and all LVGL writes.
  // BLE/IMU callbacks only retain state and signal this low-priority task.
  bool start_renderer();
#ifndef ESP_PLATFORM
  // Deterministic pump for native tests. Production firmware always uses the
  // task created by start_renderer().
  void render_pending_for_test();
#endif
  void on_link_state(bool active);
  void on_imu_sample(float heading_rate_dps, std::uint64_t sample_ms);
  /// Raw accelerometer magnitude from the same 125 Hz task.  Kept separate
  /// from on_imu_sample so the yaw assist and the screen policy can evolve
  /// independently.
  void on_motion_sample(float accel_mps2, std::uint64_t sample_ms);
  /// Touch, page change or button press: the rider wants to see the screen.
  void note_user_activity();
  /// A notification read from the phone's ANCS session (BLE task).
  void on_notification(const moto::ancs::NotificationSource& source,
                       const moto::ancs::NotificationAttributes& attributes);
  [[nodiscard]] bool screen_off() const noexcept;
  void update_demo(std::uint64_t now_ms);
  moto::ble::AckStatus on_message(
      const moto::ble::ReassembledMessage& message);

 private:
  struct GeometryAssembly {
    bool active = false;
    bool complete = false;
    std::uint32_t route_token = 0;
    std::uint32_t route_generation = 0;
    std::uint16_t chunk_count = 0;
    std::uint16_t next_chunk = 0;
    std::uint16_t total_point_count = 0;
    std::uint16_t point_count = 0;
    moto::ble::GeoPointE6 origin_e6{};
    moto::nav::Gcj02Point origin{};
    std::array<moto::nav::Gcj02Point,
               moto::nav::kRouteViewPointCapacity>
        points{};
  };

  static void page_changed(moto_ui_page_t page, void* context);
  static void interaction(void* context);
  static void music_command(moto_music_command_t command, void* context);
  static void trip_reset(void* context);
  static void demo_changed(uint8_t enabled, void* context);

  void consume_navigation(const moto::ble::NavigationSnapshot& input);
  void note_phone_speed(float speed_mps);
  moto::ble::AckStatus consume_geometry(
      const moto::ble::RouteGeometry& input);
  void consume_traffic(const moto::ble::TrafficDeviation& input);
  void consume_media(const moto::ble::MediaState& input);
  void consume_clock(const moto::ble::ClockState& input);
  void consume_trip(const moto::ble::TripState& input);
  moto::ble::AckStatus consume_map_scene(const moto::ble::MapScene& input);
  // Requires state_mutex_. This helper never calls LVGL or the BLE sender.
  void apply_geometry_locked(moto::nav::NavSnapshot& target);
  moto::nav::NavSnapshot& phone_snapshot_locked() noexcept;
  void present_navigation();
  void present_motion();
  void request_render(std::uint32_t flags) noexcept;
  // Requires no lock. Evaluates the chime policy against the current snapshot
  // and hands any resulting tones to the board.
  void publish_chimes();
  void play_tone(moto_tone_t tone);
  void play_voice(moto_voice_phrase_t phrase);
  // Requires no lock. Applies a parked/idle transition, if any.
  void apply_screen_action(MotionIdleDetector::Action action);
  void render_pending();
#ifdef ESP_PLATFORM
  static void render_task(void* context);
  void run_renderer();
#endif
  void send_page_command(moto_ui_page_t page);
  void send_music_command(moto_music_command_t command);
  void send_trip_reset_command();
  void send_device_command(moto::ble::DeviceCommand command);
  void set_demo_active(bool active);
  // Fill an existing member in place. NavSnapshot contains the bounded map
  // window and is intentionally several kilobytes; returning one by value can
  // overflow the small FreeRTOS demo-task stack as road density grows.
  static void fill_demo_snapshot(moto::nav::NavSnapshot& output,
                                 std::uint64_t elapsed_ms);

  moto::nav::NavPresenter& presenter_;
  moto::nav::NavSnapshot snapshot_{};
  // Only the render task writes these retained copies. They let state_mutex_
  // be released before waiting for LVGL or running the relatively expensive
  // road/building projection.
  moto::nav::NavSnapshot render_snapshot_{};
  GeometryAssembly geometry_{};
  SendCallback sender_ = nullptr;
  void* sender_context_ = nullptr;
  ScreenPowerCallback screen_power_ = nullptr;
  void* screen_power_context_ = nullptr;
  ToneCallback tone_callback_ = nullptr;
  VoiceCallback voice_callback_ = nullptr;
  void* voice_context_ = nullptr;
  void* tone_context_ = nullptr;
  // Turns the navigation snapshot into a small number of short sounds.  Every
  // access happens under state_mutex_ with the callbacks kept short.
  moto::audio::ChimePolicy chime_policy_{};
  // Blanking the panel also stops LVGL work; the pending flags are retained so
  // the first frame after waking repaints everything.
  std::atomic<bool> screen_off_{false};
  MotionIdleDetector idle_detector_{};
  // Notifications are copied out of the BLE callback so the render task owns
  // the strings it hands to LVGL.
  struct NotificationRender {
    char category[24] = {};
    char app[moto::ancs::kAppLength] = {};
    char title[moto::ancs::kTitleLength] = {};
    char message[moto::ancs::kMessageLength] = {};
    std::uint32_t hold_ms = 0;
  };
  NotificationRender notification_state_{};
  bool has_notification_ = false;
  // UID of the card currently on screen, so a removal from the phone only
  // closes the matching notification and never a newer one.
  std::uint32_t notification_uid_ = 0;
  bool dismiss_notification_ = false;
  NotificationRender render_notification_{};
  bool render_notification_valid_ = false;
  bool render_dismiss_notification_ = false;
  // BLE decoding runs on moto_ble_rx while touch callbacks run on the LVGL
  // worker. Protect every access to the shared navigation/session/command
  // state, and release this mutex before calling LVGL or the BLE sender.
  mutable std::mutex state_mutex_;
  std::uint32_t phone_session_id_ = 0;
  std::uint16_t next_command_id_ = 1;
  bool link_active_ = false;
  moto_ui_phone_connection_t ui_phone_connection_ = MOTO_UI_PHONE_OFFLINE;
  moto::ble::MediaState media_state_{};
  moto::ble::MediaState render_media_state_{};
  moto_ui_clock_t clock_state_{};
  moto::rtc_clock::LocalClock local_clock_{};
  RtcWriteCallback rtc_hook_ = nullptr;
  void* rtc_hook_context_ = nullptr;
  moto_ui_clock_t render_clock_state_{};
  moto_ui_trip_t trip_state_{};
  moto_ui_trip_t render_trip_state_{};
  bool music_page_enabled_ = false;
  bool render_music_page_enabled_ = false;
  moto_ui_phone_connection_t render_phone_connection_ =
      MOTO_UI_PHONE_OFFLINE;
  bool render_demo_active_ = false;
  enum RenderFlag : std::uint32_t {
    RenderNavigation = 1U << 0U,
    RenderMotion = 1U << 1U,
    RenderMedia = 1U << 2U,
    RenderClock = 1U << 3U,
    RenderTrip = 1U << 4U,
    RenderNotification = 1U << 5U,
    // The phone removed the notification the panel is showing.
    RenderNotificationDismiss = 1U << 6U,
    RenderAll = RenderNavigation | RenderMotion | RenderMedia | RenderClock |
                RenderTrip | RenderNotification | RenderNotificationDismiss,
  };
  std::atomic<std::uint32_t> pending_render_flags_{0};
#ifdef ESP_PLATFORM
  std::atomic<TaskHandle_t> render_task_handle_{nullptr};
#endif
  MotionHeadingFusion heading_fusion_;
  std::uint64_t last_motion_present_ms_ = 0;
  moto::nav::NavSnapshot snapshot_before_demo_{};
  std::uint64_t demo_started_ms_ = 0;
  bool demo_active_ = false;
};
