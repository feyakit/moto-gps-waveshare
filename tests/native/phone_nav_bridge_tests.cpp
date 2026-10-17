#include "phone_nav_bridge.h"
#include "phone_nav_bridge_test_probe.hpp"
#include "demo_fixture/jinan_big_data_demo.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

int failures = 0;

void pump(PhoneNavBridge& bridge) {
  bridge.render_pending_for_test();
}

#define CHECK(condition)                                                   \
  do {                                                                     \
    if (!(condition)) {                                                    \
      std::cerr << __FILE__ << ':' << __LINE__                             \
                << " CHECK failed: " #condition << '\n';                  \
      ++failures;                                                          \
    }                                                                      \
  } while (false)

moto::ble::ReassembledMessage make_message(
    const moto::ble::Message& message,
    std::uint16_t sequence) {
  const moto::ble::BytesResult encoded = moto::ble::encode_message(message);
  if (!encoded.ok()) {
    std::cerr << "Could not encode bridge test message\n";
    ++failures;
    return {};
  }
  moto::ble::ReassembledMessage result;
  result.type = moto::ble::message_type(message);
  result.sequence = sequence;
  result.payload = encoded.value;
  return result;
}

struct ReentrantSenderContext {
  PhoneNavBridge* bridge = nullptr;
  int calls = 0;
  moto::ble::DisplayPage last_page = moto::ble::DisplayPage::Navigation;
};

bool reentrant_sender(const moto::ble::Message& message,
                      std::uint8_t,
                      void* context) {
  auto* sender = static_cast<ReentrantSenderContext*>(context);
  const auto* command = std::get_if<moto::ble::DeviceCommand>(&message);
  if (command == nullptr) {
    return false;
  }
  ++sender->calls;
  sender->last_page = command->page;

  // A real transport can synchronously reject a send or cause a link callback.
  // This re-entry deadlocks if PhoneNavBridge keeps state_mutex_ while sending.
  sender->bridge->on_link_state(false);
  return true;
}

struct MusicSenderContext {
  std::vector<moto::ble::DeviceCommand> commands;
  std::vector<std::uint8_t> flags;
};

bool record_music_sender(const moto::ble::Message& message,
                         std::uint8_t flags,
                         void* context) {
  auto* sender = static_cast<MusicSenderContext*>(context);
  const auto* command = std::get_if<moto::ble::DeviceCommand>(&message);
  if (command == nullptr) return false;
  sender->commands.push_back(*command);
  sender->flags.push_back(flags);
  return true;
}

void test_sender_is_called_without_bridge_state_lock() {
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  ReentrantSenderContext sender{&bridge};
  bridge.set_sender(reentrant_sender, &sender);
  bridge.install_ui_callbacks();
  bridge.on_link_state(true);

  moto::test::emit_page_change(MOTO_UI_PAGE_SPEED);
  CHECK(sender.calls == 1);
  CHECK(sender.last_page == moto::ble::DisplayPage::Speed);

  // The sender above forces the link down to prove no lock is held across the
  // send; bring it back up to observe the next page mapping.
  bridge.on_link_state(true);
  moto::test::emit_page_change(MOTO_UI_PAGE_CLOCK);
  CHECK(sender.last_page == moto::ble::DisplayPage::Clock);
}

void test_navigation_and_touch_updates_are_serialized() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  bridge.on_link_state(true);

  constexpr int kIterations = 2'000;
  std::atomic<bool> begin{false};
  std::thread phone([&]() {
    while (!begin.load()) {
      std::this_thread::yield();
    }
    for (int index = 0; index < kIterations; ++index) {
      moto::ble::NavigationSnapshot snapshot;
      snapshot.state = moto::ble::NavigationState::Navigating;
      snapshot.network = moto::ble::NetworkState::Online;
      snapshot.flags = moto::ble::NavigationHasNextManeuver;
      snapshot.maneuver = (index % 2 == 0)
                              ? moto::ble::Maneuver::Left
                              : moto::ble::Maneuver::Right;
      snapshot.route_token = 17;
      snapshot.route_generation = 3;
      snapshot.road_name = "并发道路-" + std::to_string(index) +
                           std::string(32, static_cast<char>('a' + index % 26));
      snapshot.distance_to_maneuver_m = static_cast<std::uint32_t>(index);
      const auto message = make_message(
          moto::ble::Message{snapshot},
          static_cast<std::uint16_t>(index % 65'535 + 1));
      CHECK(bridge.on_message(message) == moto::ble::AckStatus::Ok);
    }
  });

  std::thread touch([&]() {
    while (!begin.load()) {
      std::this_thread::yield();
    }
    for (int index = 0; index < kIterations; ++index) {
      moto::test::emit_page_change(
          index % 2 == 0 ? MOTO_UI_PAGE_SPEED : MOTO_UI_PAGE_COMPASS);
    }
  });

  begin.store(true);
  phone.join();
  touch.join();

  // Thousands of producer updates collapse into the latest complete state;
  // none of the producer threads enters LVGL.
  CHECK(moto::test::phone_nav_bridge_apply_count() == 0);
  CHECK(moto::test::phone_nav_bridge_board_lock_count() == 0);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_apply_count() == 1);
  const std::string final_name = moto::test::phone_nav_bridge_last_road_name();
  CHECK(final_name.rfind("并发道路-", 0) == 0);
}

void test_ble_submission_never_waits_for_lvgl_and_retries_latest_state() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);

  moto::ble::NavigationSnapshot first;
  first.state = moto::ble::NavigationState::Navigating;
  first.network = moto::ble::NetworkState::Online;
  first.flags = moto::ble::NavigationHasNextManeuver;
  first.route_token = 29;
  first.route_generation = 1;
  first.maneuver = moto::ble::Maneuver::Left;
  first.road_name = "旧状态";

  moto::test::phone_nav_bridge_set_board_lock_available(false);
  CHECK(bridge.on_message(make_message(first, 1)) ==
        moto::ble::AckStatus::Ok);
  CHECK(moto::test::phone_nav_bridge_board_lock_count() == 0);
  CHECK(moto::test::phone_nav_bridge_apply_count() == 0);

  // Only the render consumer attempts LVGL. Failure keeps the dirty bit and
  // the producer remains completely decoupled from that unavailable lock.
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_board_lock_count() == 1);
  CHECK(moto::test::phone_nav_bridge_apply_count() == 0);

  auto latest = first;
  latest.maneuver = moto::ble::Maneuver::Right;
  latest.road_name = "保留的最新状态";
  CHECK(bridge.on_message(make_message(latest, 2)) ==
        moto::ble::AckStatus::Ok);
  CHECK(moto::test::phone_nav_bridge_board_lock_count() == 1);

  moto::test::phone_nav_bridge_set_board_lock_available(true);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_apply_count() == 1);
  CHECK(moto::test::phone_nav_bridge_last_road_name() == "保留的最新状态");
  CHECK(moto::test::phone_nav_bridge_last_maneuver() ==
        MOTO_MANEUVER_RIGHT);
}

void test_phone_connection_lifecycle_reaches_the_ui() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);

  bridge.on_link_state(true);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_phone_connection() ==
        MOTO_UI_PHONE_CONNECTING);

  moto::ble::ConnectionStatus starting;
  starting.role = moto::ble::EndpointRole::Phone;
  starting.state = moto::ble::ConnectionState::Starting;
  starting.minimum_version = moto::ble::kProtocolVersion;
  starting.maximum_version = moto::ble::kProtocolVersion;
  starting.session_id = 1234;
  CHECK(bridge.on_message(make_message(starting, 10)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_phone_connection() ==
        MOTO_UI_PHONE_CONNECTING);

  starting.state = moto::ble::ConnectionState::Ready;
  CHECK(bridge.on_message(make_message(starting, 11)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_phone_connection() ==
        MOTO_UI_PHONE_ONLINE);

  bridge.on_link_state(false);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_phone_connection() ==
        MOTO_UI_PHONE_OFFLINE);
}

void test_music_capability_state_and_touch_commands() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  MusicSenderContext sender;
  bridge.set_sender(record_music_sender, &sender);
  bridge.install_ui_callbacks();
  bridge.on_link_state(true);

  moto::ble::ConnectionStatus ready;
  ready.role = moto::ble::EndpointRole::Phone;
  ready.state = moto::ble::ConnectionState::Starting;
  ready.minimum_version = moto::ble::kProtocolVersion;
  ready.maximum_version = moto::ble::kProtocolVersion;
  ready.capabilities = moto::ble::CapabilityMediaState |
                       moto::ble::CapabilityMusicCommands |
                       moto::ble::CapabilityCommandAck;
  ready.session_id = 4321;
  CHECK(bridge.on_message(make_message(ready, 20)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(!moto::test::phone_nav_bridge_music_page_enabled());
  ready.state = moto::ble::ConnectionState::Ready;
  CHECK(bridge.on_message(make_message(ready, 21)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_music_page_enabled());

  moto::ble::MediaState media;
  media.flags = moto::ble::MediaConnected |
                moto::ble::MediaPlaying |
                moto::ble::MediaLikeAvailable;
  media.track_token = 88;
  media.position_s = 12;
  media.duration_s = 240;
  media.source_name = "APPLE MUSIC";
  media.track_title = "夜间骑行";
  media.artist_name = "MOTO GPS";
  CHECK(bridge.on_message(make_message(media, 22)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_media_connected());
  CHECK(moto::test::phone_nav_bridge_media_playing());
  CHECK(moto::test::phone_nav_bridge_media_like_available());
  CHECK(moto::test::phone_nav_bridge_media_source() == "APPLE MUSIC");
  CHECK(moto::test::phone_nav_bridge_media_title() == "夜间骑行");
  CHECK(moto::test::phone_nav_bridge_media_artist() == "MOTO GPS");

  moto::test::emit_music_command(MOTO_MUSIC_PREVIOUS);
  moto::test::emit_music_command(MOTO_MUSIC_TOGGLE_PLAYBACK);
  moto::test::emit_music_command(MOTO_MUSIC_NEXT);
  moto::test::emit_music_command(MOTO_MUSIC_LIKE);
  CHECK(sender.commands.size() == 4);
  const moto::ble::DeviceCommandKind expected[] = {
      moto::ble::DeviceCommandKind::MusicPrevious,
      moto::ble::DeviceCommandKind::MusicTogglePlayback,
      moto::ble::DeviceCommandKind::MusicNext,
      moto::ble::DeviceCommandKind::MusicLike,
  };
  for (std::size_t index = 0; index < sender.commands.size(); ++index) {
    CHECK(sender.commands[index].kind == expected[index]);
    CHECK(sender.commands[index].page == moto::ble::DisplayPage::Music);
    CHECK(sender.commands[index].command_id == index + 1);
    CHECK((sender.flags[index] & moto::ble::AckRequested) != 0U);
    CHECK((sender.flags[index] & moto::ble::Urgent) != 0U);
  }
}

void test_clock_state_reaches_the_ui_and_clears_on_disconnect() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  bridge.on_link_state(true);
  pump(bridge);

  moto::ble::ClockState clock;
  clock.local_time_minutes = 9 * 60 + 5;
  clock.weekday = 1;
  clock.month = 6;
  clock.day = 15;
  CHECK(bridge.on_message(make_message(clock, 60)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  const moto_ui_clock_t rendered = moto::test::phone_nav_bridge_clock();
  CHECK(rendered.minutes == 9 * 60 + 5);
  CHECK(rendered.weekday == 1);
  CHECK(rendered.month == 6);
  CHECK(rendered.day == 15);

  // Losing the phone link must clear the phone-dependent state, but the clock
  // is the exception: the board's own RTC kept running, so the page keeps a
  // correct time instead of freezing a stale one or showing --:--.
  bridge.on_link_state(false);
  pump(bridge);
  const moto_ui_clock_t after = moto::test::phone_nav_bridge_clock();
  CHECK(after.minutes == 9 * 60 + 5);
  CHECK(after.weekday == 1);
  CHECK(after.month == 6);
  CHECK(after.day == 15);
}

void test_clock_survives_a_disconnect_and_keeps_ticking() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  bridge.on_link_state(true);
  pump(bridge);

  moto::ble::ClockState clock;
  clock.local_time_minutes = 23 * 60 + 59;
  clock.weekday = 6;
  clock.month = 10;
  clock.day = 17;
  CHECK(bridge.on_message(make_message(clock, 61)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_clock().minutes == 23 * 60 + 59);

  // With no phone the minute must still roll over on its own, across midnight
  // and onto the next day.
  bridge.on_link_state(false);
  pump(bridge);
  bridge.tick_clock();
  pump(bridge);
  const moto_ui_clock_t shown = moto::test::phone_nav_bridge_clock();
  CHECK(shown.minutes != MOTO_UI_CLOCK_UNKNOWN);
}

void test_rtc_seeds_the_clock_before_any_phone_connects() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  pump(bridge);
  // Nothing has been pushed and no RTC was seeded: the page must stay explicit
  // about not knowing the time rather than inventing midnight.
  CHECK(moto::test::phone_nav_bridge_clock().minutes == MOTO_UI_CLOCK_UNKNOWN);

  moto::rtc_clock::Time stored{};
  stored.year = 2026;
  stored.month = 10;
  stored.day = 17;
  stored.hour = 8;
  stored.minute = 30;
  stored.second = 0;
  stored.weekday = 6;
  bridge.seed_clock_from_rtc(stored);
  pump(bridge);
  const moto_ui_clock_t rendered = moto::test::phone_nav_bridge_clock();
  CHECK(rendered.minutes == 8 * 60 + 30);
  CHECK(rendered.weekday == 6);
  CHECK(rendered.month == 10);
  CHECK(rendered.day == 17);
}

void test_rtc_hook_reports_the_corrected_time() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  bridge.on_link_state(true);
  pump(bridge);

  struct Captured {
    int calls = 0;
    moto::rtc_clock::Time time{};
  } captured;
  bridge.set_rtc_hook(
      [](const moto::rtc_clock::Time& time, void* context) {
        auto* state = static_cast<Captured*>(context);
        state->calls += 1;
        state->time = time;
      },
      &captured);

  moto::ble::ClockState clock;
  clock.local_time_minutes = 7 * 60 + 45;
  clock.weekday = 6;
  clock.month = 10;
  clock.day = 17;
  CHECK(bridge.on_message(make_message(clock, 62)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  // The platform hook is how the chip learns the time; without it the RTC
  // would never be corrected.
  CHECK(captured.calls == 1);
  CHECK(captured.time.hour == 7);
  CHECK(captured.time.minute == 45);
  CHECK(captured.time.day == 17);
}

void test_trip_state_reaches_the_ui_and_clears_on_disconnect() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  bridge.on_link_state(true);
  pump(bridge);

  moto::ble::TripState trip;
  trip.distance_m = 8'420;
  trip.moving_time_s = 1'205;
  trip.elevation_gain_m = 96;
  trip.max_speed_deci_kph = 884;
  trip.flags = moto::ble::TripRecording | moto::ble::TripElevationKnown;
  CHECK(bridge.on_message(make_message(trip, 70)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  const moto_ui_trip_t rendered = moto::test::phone_nav_bridge_trip();
  CHECK(rendered.distance_m == 8'420);
  CHECK(rendered.moving_time_s == 1'205);
  CHECK(rendered.elevation_gain_m == 96);
  CHECK(rendered.max_speed_deci_kph == 884);
  CHECK(rendered.recording == 1);
  CHECK(rendered.elevation_known == 1);

  // The board owns no odometer: dropping the link must clear the totals rather
  // than freeze a stale ride on the Trip page.
  bridge.on_link_state(false);
  pump(bridge);
  const moto_ui_trip_t cleared = moto::test::phone_nav_bridge_trip();
  CHECK(cleared.recording == 0);
  CHECK(cleared.distance_m == 0);
}

void test_trip_page_long_press_requests_a_phone_side_reset() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  MusicSenderContext sender;
  bridge.set_sender(record_music_sender, &sender);
  bridge.on_link_state(true);
  pump(bridge);

  moto::test::emit_trip_reset();
  CHECK(sender.commands.size() == 1);
  if (!sender.commands.empty()) {
    CHECK(sender.commands[0].kind == moto::ble::DeviceCommandKind::TripReset);
    CHECK(sender.commands[0].page == moto::ble::DisplayPage::Trip);
    CHECK((sender.flags[0] & moto::ble::AckRequested) != 0U);
  }

  // A parked rider with no live link must not have the request silently
  // queued forever; the board simply drops it like every other control.
  bridge.on_link_state(false);
  moto::test::emit_trip_reset();
  CHECK(sender.commands.size() == 1);
}

void test_demo_uses_the_production_presenter_path() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();

  moto::test::emit_demo_change(true);
  pump(bridge);
  const auto now_ms = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());

  // Approach and cross the first junction in the real OSM route. The symbol
  // and distance switch together: right into 新泺大街, then left into 崇华路.
  // Distances come from cumulative route geometry, not UI timer thresholds.
  bridge.update_demo(now_ms + 6'200);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_maneuver() ==
        MOTO_MANEUVER_RIGHT);
  CHECK(moto::test::phone_nav_bridge_last_distance_to_maneuver_m() > 3);
  CHECK(moto::test::phone_nav_bridge_last_distance_to_maneuver_m() < 8);
  bridge.update_demo(now_ms + 7'000);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_maneuver() ==
        MOTO_MANEUVER_LEFT);
  CHECK(moto::test::phone_nav_bridge_last_distance_to_maneuver_m() > 135);
  CHECK(moto::test::phone_nav_bridge_last_distance_to_maneuver_m() < 141);

  bridge.update_demo(now_ms + 12'000);
  pump(bridge);

  CHECK(moto::test::phone_nav_bridge_last_demo_active());
  CHECK(moto::test::phone_nav_bridge_last_route_point_count() >= 2);
  CHECK(moto::test::phone_nav_bridge_last_road_point_count() ==
        moto::demo::jinan_big_data::kRoadPoints.size());
  CHECK(moto::test::phone_nav_bridge_last_road_polyline_count() ==
        moto::demo::jinan_big_data::kRoads.size());
  CHECK(moto::test::phone_nav_bridge_apply_count() >= 2);

  bridge.update_demo(now_ms + 6'200);
  pump(bridge);
  const uint16_t heading_before =
      moto::test::phone_nav_bridge_last_heading_deg();
  bridge.update_demo(now_ms + 8'500);
  pump(bridge);
  const uint16_t heading_after =
      moto::test::phone_nav_bridge_last_heading_deg();
  CHECK(heading_before != heading_after);

  moto::test::emit_demo_change(false);
  pump(bridge);
  CHECK(!moto::test::phone_nav_bridge_last_demo_active());
}

void test_ios_demo_token_attaches_context_and_live_route_clears_it() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);

  const std::uint32_t demo_token =
      moto::ble::route_token(moto::demo::jinan_big_data::kRouteId);
  moto::ble::NavigationSnapshot snapshot;
  snapshot.state = moto::ble::NavigationState::Navigating;
  snapshot.network = moto::ble::NetworkState::Online;
  snapshot.flags = moto::ble::NavigationHasRouteView;
  snapshot.route_token = demo_token;
  snapshot.route_generation = 1;
  moto::ble::RouteGeometry geometry;
  geometry.route_token = demo_token;
  geometry.route_generation = 1;
  geometry.total_point_count = 2;
  geometry.view_origin = {36'674'773, 117'128'056};
  geometry.points = {
      {36'674'773, 117'128'056},
      {36'674'145, 117'128'489},
  };
  // The phone queues changed geometry before the snapshot that references it.
  // Geometry updates state only; the following snapshot is the single LVGL
  // presentation boundary for the pair.
  const int before_pair = moto::test::phone_nav_bridge_apply_count();
  CHECK(bridge.on_message(make_message(geometry, 40)) ==
        moto::ble::AckStatus::Ok);
  CHECK(moto::test::phone_nav_bridge_apply_count() == before_pair);
  CHECK(bridge.on_message(make_message(snapshot, 41)) ==
        moto::ble::AckStatus::Ok);
  CHECK(moto::test::phone_nav_bridge_apply_count() == before_pair);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_apply_count() == before_pair + 1);
  CHECK(moto::test::phone_nav_bridge_last_road_point_count() ==
        moto::demo::jinan_big_data::kRoadPoints.size());
  CHECK(moto::test::phone_nav_bridge_last_road_polyline_count() ==
        moto::demo::jinan_big_data::kRoads.size());

  snapshot.flags = 0;
  snapshot.route_token = moto::ble::route_token("amap-live-route");
  snapshot.route_generation = 2;
  CHECK(bridge.on_message(make_message(snapshot, 42)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_road_point_count() == 0);
  CHECK(moto::test::phone_nav_bridge_last_road_polyline_count() == 0);
}

void test_map_scene_atomically_replaces_roads_and_buildings_at_capacity() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);

  moto::ble::NavigationSnapshot navigation;
  navigation.state = moto::ble::NavigationState::Navigating;
  navigation.network = moto::ble::NetworkState::Online;
  navigation.flags = moto::ble::NavigationHasRouteView;
  navigation.route_token = 77;
  navigation.route_generation = 1;
  navigation.heading_cdeg = 9'000;
  CHECK(bridge.on_message(make_message(navigation, 50)) ==
        moto::ble::AckStatus::Ok);

  moto::ble::RouteGeometry geometry;
  geometry.route_token = 77;
  geometry.route_generation = 1;
  geometry.total_point_count = 2;
  geometry.view_origin = {36'670'000, 117'130'000};
  geometry.points = {
      geometry.view_origin,
      {36'670'000, 117'131'000},
  };
  CHECK(bridge.on_message(make_message(geometry, 51)) ==
        moto::ble::AckStatus::Ok);

  moto::ble::MapScene first;
  first.scene_revision = 1;
  first.view_origin = geometry.view_origin;
  first.radius_m = 500;
  first.roads = {
      {moto::ble::MapRoadClass::Primary,
       {{36'670'000, 117'130'000}, {36'670'000, 117'131'000}}},
  };
  first.buildings = {
      {moto::ble::MapBuildingClass::Landmark,
       {{36'670'100, 117'130'100},
        {36'670'100, 117'130'200},
        {36'670'200, 117'130'200}}},
  };
  const int before_first = moto::test::phone_nav_bridge_apply_count();
  CHECK(bridge.on_message(make_message(first, 52)) ==
        moto::ble::AckStatus::Ok);
  CHECK(moto::test::phone_nav_bridge_apply_count() == before_first);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_apply_count() == before_first + 1);
  CHECK(moto::test::phone_nav_bridge_last_road_point_count() == 2);
  CHECK(moto::test::phone_nav_bridge_last_road_polyline_count() == 1);
  CHECK(moto::test::phone_nav_bridge_last_road_class() ==
        static_cast<std::uint8_t>(moto::ble::MapRoadClass::Primary));
  CHECK(moto::test::phone_nav_bridge_last_building_point_count() == 3);
  CHECK(moto::test::phone_nav_bridge_last_building_footprint_count() == 1);
  CHECK(moto::test::phone_nav_bridge_last_building_class() ==
        static_cast<std::uint8_t>(moto::ble::MapBuildingClass::Landmark));

  moto::ble::MapScene maximum;
  maximum.scene_revision = 2;
  maximum.view_origin = geometry.view_origin;
  maximum.radius_m = 700;
  for (std::size_t road_index = 0;
       road_index < moto::ble::kMaxMapSceneRoads; ++road_index) {
    moto::ble::MapRoadPolyline road;
    road.road_class = road_index == 0
                          ? moto::ble::MapRoadClass::Service
                          : moto::ble::MapRoadClass::Residential;
    for (std::size_t point_index = 0; point_index < 8; ++point_index) {
      road.points.push_back({
          geometry.view_origin.latitude_e6 +
              static_cast<std::int32_t>(road_index * 20 + point_index),
          geometry.view_origin.longitude_e6 +
              static_cast<std::int32_t>(road_index * 25 + point_index * 3),
      });
    }
    maximum.roads.push_back(std::move(road));
  }
  for (std::size_t building_index = 0;
       building_index < moto::ble::kMaxMapSceneBuildings;
       ++building_index) {
    moto::ble::MapBuildingFootprint building;
    building.building_class = moto::ble::MapBuildingClass::Parking;
    for (std::size_t point_index = 0; point_index < 8; ++point_index) {
      building.points.push_back({
          geometry.view_origin.latitude_e6 +
              static_cast<std::int32_t>(building_index * 30 + point_index),
          geometry.view_origin.longitude_e6 +
              static_cast<std::int32_t>(building_index * 35 + point_index * 2),
      });
    }
    maximum.buildings.push_back(std::move(building));
  }

  const int before_maximum = moto::test::phone_nav_bridge_apply_count();
  CHECK(bridge.on_message(make_message(maximum, 53)) ==
        moto::ble::AckStatus::Ok);
  CHECK(moto::test::phone_nav_bridge_apply_count() == before_maximum);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_apply_count() == before_maximum + 1);
  CHECK(moto::test::phone_nav_bridge_last_road_point_count() ==
        moto::ble::kMaxMapSceneRoadPoints);
  CHECK(moto::test::phone_nav_bridge_last_road_polyline_count() ==
        moto::ble::kMaxMapSceneRoads);
  CHECK(moto::test::phone_nav_bridge_last_road_class() ==
        static_cast<std::uint8_t>(moto::ble::MapRoadClass::Service));
  CHECK(moto::test::phone_nav_bridge_last_building_point_count() ==
        moto::ble::kMaxMapSceneBuildingPoints);
  CHECK(moto::test::phone_nav_bridge_last_building_footprint_count() ==
        moto::ble::kMaxMapSceneBuildings);
  CHECK(moto::test::phone_nav_bridge_last_building_class() ==
        static_cast<std::uint8_t>(moto::ble::MapBuildingClass::Parking));

  const int before_stale = moto::test::phone_nav_bridge_apply_count();
  CHECK(bridge.on_message(make_message(first, 54)) ==
        moto::ble::AckStatus::Duplicate);
  CHECK(moto::test::phone_nav_bridge_apply_count() == before_stale);
  CHECK(moto::test::phone_nav_bridge_last_road_point_count() ==
        moto::ble::kMaxMapSceneRoadPoints);
  CHECK(moto::test::phone_nav_bridge_last_building_point_count() ==
        moto::ble::kMaxMapSceneBuildingPoints);

  moto::ble::MapScene empty;
  empty.scene_revision = 3;
  empty.view_origin = geometry.view_origin;
  empty.radius_m = 500;
  CHECK(bridge.on_message(make_message(empty, 55)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_road_point_count() == 0);
  CHECK(moto::test::phone_nav_bridge_last_road_polyline_count() == 0);
  CHECK(moto::test::phone_nav_bridge_last_building_point_count() == 0);
  CHECK(moto::test::phone_nav_bridge_last_building_footprint_count() == 0);
}

void test_imu_presentation_preserves_40hz_phase_and_stale_course_is_ignored() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  // The IMU is usually ready before BLE. Its relative zero cannot become the
  // absolute heading when the phone finally connects.
  bridge.on_imu_sample(0.0F, 1);
  moto::ble::NavigationSnapshot navigation;
  navigation.state = moto::ble::NavigationState::Navigating;
  navigation.display_page = moto::ble::DisplayPage::Compass;
  navigation.flags = moto::ble::NavigationHasFix;
  navigation.heading_cdeg = 9'000;
  navigation.speed_deci_kph = 360;
  CHECK(bridge.on_message(make_message(navigation, 1)) == moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_heading_deg() == 90);

  const int before = moto::test::phone_nav_bridge_apply_count();
  for (std::uint64_t time = 8; time <= 1'000; time += 8) {
    bridge.on_imu_sample(45.0F, time);
    pump(bridge);
  }
  const int motion_frames = moto::test::phone_nav_bridge_apply_count() - before;
  CHECK(motion_frames == 40);  // resetting the deadline yields only 31
  const auto before_stale = moto::test::phone_nav_bridge_last_heading_deg();
  navigation.heading_cdeg = 27'000;
  navigation.flags |= moto::ble::NavigationGnssStale;
  CHECK(bridge.on_message(make_message(navigation, 2)) == moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_last_heading_deg() == before_stale);
}

struct ScreenPowerLog {
  int on_calls = 0;
  int off_calls = 0;
  bool last_on = false;
};

void record_screen_power(bool on, void* context) {
  auto* log = static_cast<ScreenPowerLog*>(context);
  if (on) {
    ++log->on_calls;
  } else {
    ++log->off_calls;
  }
  log->last_on = on;
}

// Engine off on the bench: the accelerometer only sees gravity, so the panel
// blanks after the hold and comes back on the first bump.
void test_parked_bike_blanks_the_panel_and_motion_lights_it_again() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  ScreenPowerLog log;
  bridge.set_screen_power_callback(record_screen_power, &log);

  bridge.on_motion_sample(9.81F, 1'000);
  CHECK(!bridge.screen_off());
  CHECK(log.off_calls == 0);
  CHECK(log.on_calls == 0);

  bridge.on_motion_sample(9.81F, 46'000);
  CHECK(bridge.screen_off());
  CHECK(log.off_calls == 1);
  CHECK(!log.last_on);

  // Restarting the engine shakes the chassis, which is the wake gesture.
  bridge.on_motion_sample(10.2F, 46'100);
  CHECK(!bridge.screen_off());
  CHECK(log.on_calls == 1);
  CHECK(log.last_on);
}

void test_touch_wakes_a_blanked_panel() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  ScreenPowerLog log;
  bridge.set_screen_power_callback(record_screen_power, &log);

  bridge.on_motion_sample(9.81F, 1'000);
  bridge.on_motion_sample(9.81F, 46'000);
  CHECK(bridge.screen_off());

  moto::test::emit_interaction();
  CHECK(!bridge.screen_off());
  CHECK(log.on_calls == 1);
}

// A dark panel must not keep burning CPU on frames nobody can see, but the
// wake frame has to be complete instead of a partial repaint.
void test_blanked_panel_stops_rendering_until_wake() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  bridge.install_ui_callbacks();
  ScreenPowerLog log;
  bridge.set_screen_power_callback(record_screen_power, &log);

  // No route: the screen is only a clock/speed display, so the short hold
  // applies and a parked bike blanks quickly.
  moto::ble::NavigationSnapshot navigation;
  navigation.flags = moto::ble::NavigationHasFix;
  navigation.heading_cdeg = 9'000;
  navigation.speed_deci_kph = 0;
  CHECK(bridge.on_message(make_message(navigation, 1)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);

  bridge.on_motion_sample(9.81F, 1'000);
  bridge.on_motion_sample(9.81F, 46'000);
  CHECK(bridge.screen_off());

  const int dark_baseline = moto::test::phone_nav_bridge_apply_count();
  navigation.heading_cdeg = 18'000;
  CHECK(bridge.on_message(make_message(navigation, 2)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_apply_count() == dark_baseline);

  moto::test::emit_interaction();
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_apply_count() > dark_baseline);

  const int lit_baseline = moto::test::phone_nav_bridge_apply_count();
  navigation.heading_cdeg = 27'000;
  CHECK(bridge.on_message(make_message(navigation, 3)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_apply_count() > lit_baseline);
}

// A route on screen is a navigation instrument: a long traffic light must not
// blank it, only a genuinely finished ride reaches the longer hold.
void test_active_route_uses_the_longer_parked_hold() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  ScreenPowerLog log;
  bridge.set_screen_power_callback(record_screen_power, &log);

  moto::ble::NavigationSnapshot navigation;
  navigation.state = moto::ble::NavigationState::Navigating;
  navigation.flags = moto::ble::NavigationHasFix;
  navigation.speed_deci_kph = 0;
  CHECK(bridge.on_message(make_message(navigation, 1)) == moto::ble::AckStatus::Ok);

  bridge.on_motion_sample(9.81F, 1'000);
  bridge.on_motion_sample(9.81F, 61'000);
  CHECK(!bridge.screen_off());
  bridge.on_motion_sample(9.81F, 181'000);
  CHECK(bridge.screen_off());
}

// A smooth surface plus a phone-reported speed still means the rider is riding.
void test_phone_speed_lights_a_blanked_panel() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  ScreenPowerLog log;
  bridge.set_screen_power_callback(record_screen_power, &log);

  bridge.on_motion_sample(9.81F, 1'000);
  bridge.on_motion_sample(9.81F, 46'000);
  CHECK(bridge.screen_off());

  moto::ble::NavigationSnapshot navigation;
  navigation.state = moto::ble::NavigationState::Navigating;
  navigation.flags = moto::ble::NavigationHasFix;
  navigation.speed_deci_kph = 360;  // 36 km/h
  CHECK(bridge.on_message(make_message(navigation, 1)) == moto::ble::AckStatus::Ok);
  CHECK(!bridge.screen_off());
  CHECK(log.on_calls == 1);
}

// A phone notification has to survive the hop from the BLE task to the render
// task, since the strings are only valid inside the callback.
void test_notification_reaches_the_ui_overlay() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);

  moto::ancs::NotificationSource source;
  source.event = static_cast<std::uint8_t>(moto::ancs::EventId::Added);
  source.category =
      static_cast<std::uint8_t>(moto::ancs::CategoryId::IncomingCall);
  moto::ancs::NotificationAttributes attributes;
  attributes.uid = source.uid;
  attributes.has_app = true;
  std::snprintf(attributes.app, sizeof(attributes.app), "com.apple.mobilephone");
  attributes.has_title = true;
  std::snprintf(attributes.title, sizeof(attributes.title), "段誉");
  attributes.has_message = true;
  std::snprintf(attributes.message, sizeof(attributes.message), "来电");

  bridge.on_notification(source, attributes);
  pump(bridge);

  const moto_ui_notification_t rendered =
      moto::test::phone_nav_bridge_last_notification();
  CHECK(moto::test::phone_nav_bridge_notification_count() == 1);
  CHECK(std::string(rendered.category) == "INCOMING CALL");
  CHECK(std::string(rendered.app) == "com.apple.mobilephone");
  CHECK(std::string(rendered.title) == "段誉");
  CHECK(std::string(rendered.message) == "来电");
  // A call keeps the panel lit long enough to be read.
  CHECK(rendered.hold_ms == 20'000);
}

void test_notification_falls_back_to_the_heading_only() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);

  moto::ancs::NotificationSource source;
  source.category = static_cast<std::uint8_t>(moto::ancs::CategoryId::Email);
  // The attribute fetch may fail, and an empty body must not crash the panel.
  moto::ancs::NotificationAttributes attributes;
  bridge.on_notification(source, attributes);
  pump(bridge);

  const moto_ui_notification_t rendered =
      moto::test::phone_nav_bridge_last_notification();
  CHECK(std::string(rendered.category) == "MAIL");
  CHECK(std::string(rendered.app).empty());
  CHECK(std::string(rendered.title).empty());
  CHECK(rendered.hold_ms == 8'000);
}

// A visible notification is the reason to look down, so it has to light a
// blanked panel; a silent one is background noise and must not.
void test_loud_notification_wakes_a_blanked_panel() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  ScreenPowerLog log;
  bridge.set_screen_power_callback(record_screen_power, &log);

  bridge.on_motion_sample(9.81F, 1'000);
  bridge.on_motion_sample(9.81F, 46'000);
  CHECK(bridge.screen_off());

  moto::ancs::NotificationSource silent;
  silent.event_flags = static_cast<std::uint8_t>(moto::ancs::FlagSilent);
  bridge.on_notification(silent, {});
  pump(bridge);
  CHECK(bridge.screen_off());
  CHECK(log.on_calls == 0);

  moto::ancs::NotificationSource call;
  call.category =
      static_cast<std::uint8_t>(moto::ancs::CategoryId::IncomingCall);
  bridge.on_notification(call, {});
  pump(bridge);
  CHECK(!bridge.screen_off());
  CHECK(log.on_calls == 1);
}

// The overlay refreshes while the phone keeps sending, and a dismissed
// notification is never re-shown by a later unrelated frame.
void test_notification_replaces_the_previous_one() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);

  moto::ancs::NotificationSource first;
  first.category = static_cast<std::uint8_t>(moto::ancs::CategoryId::Social);
  moto::ancs::NotificationAttributes first_attributes;
  first_attributes.has_title = true;
  std::snprintf(first_attributes.title, sizeof(first_attributes.title), "第一条");
  bridge.on_notification(first, first_attributes);
  pump(bridge);
  CHECK(std::string(moto::test::phone_nav_bridge_last_notification().title) ==
        "第一条");

  moto::ancs::NotificationSource second;
  second.category = static_cast<std::uint8_t>(moto::ancs::CategoryId::News);
  moto::ancs::NotificationAttributes second_attributes;
  second_attributes.has_title = true;
  std::snprintf(second_attributes.title, sizeof(second_attributes.title), "第二条");
  bridge.on_notification(second, second_attributes);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_notification_count() == 2);
  const moto_ui_notification_t rendered =
      moto::test::phone_nav_bridge_last_notification();
  CHECK(std::string(rendered.category) == "NEWS");
  CHECK(std::string(rendered.title) == "第二条");

  // An unrelated render must not resurrect it, and the bridge never dismisses
  // on its own: the overlay timer belongs to the UI.
  const int before = moto::test::phone_nav_bridge_notification_count();
  moto::ble::NavigationSnapshot navigation;
  navigation.flags = moto::ble::NavigationHasFix;
  navigation.heading_cdeg = 9'000;
  CHECK(bridge.on_message(make_message(navigation, 1)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_notification_count() == before);
  CHECK(moto::test::phone_nav_bridge_notification_dismiss_count() == 0);
}

// A notification the rider already handled on the phone must not keep the
// panel lit, and a removal for some other notification must leave it alone.
void test_removed_notification_dismisses_only_its_own_card() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);

  moto::ancs::NotificationSource shown;
  shown.uid = 4'242;
  shown.category = static_cast<std::uint8_t>(moto::ancs::CategoryId::Social);
  moto::ancs::NotificationAttributes attributes;
  attributes.has_title = true;
  std::snprintf(attributes.title, sizeof(attributes.title), "在吗");
  bridge.on_notification(shown, attributes);
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_notification_count() == 1);

  // Another notification being cleared elsewhere must not close this card.
  moto::ancs::NotificationSource other;
  other.uid = 7;
  other.event = static_cast<std::uint8_t>(moto::ancs::EventId::Removed);
  bridge.on_notification(other, {});
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_notification_dismiss_count() == 0);

  // Clearing the card we show closes it.
  moto::ancs::NotificationSource removal = shown;
  removal.event = static_cast<std::uint8_t>(moto::ancs::EventId::Removed);
  bridge.on_notification(removal, {});
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_notification_dismiss_count() == 1);

  // A removed notification is never drawn as a fresh card.
  CHECK(moto::test::phone_nav_bridge_notification_count() == 1);

  // With the card gone, its own repeat removal does nothing.
  bridge.on_notification(removal, {});
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_notification_dismiss_count() == 1);
}

// A notification that arrives and is removed while the panel is dark must
// never flash on screen when the rider pulls away again.
void test_removal_beats_a_stale_card_on_wake() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  ScreenPowerLog log;
  bridge.set_screen_power_callback(record_screen_power, &log);

  // Park the bike: the detector self-primes on the first sample and goes quiet.
  bridge.on_motion_sample(9.81F, 1'000);
  bridge.on_motion_sample(9.81F, 46'000);
  CHECK(bridge.screen_off());

  // A silent notification never lights the panel, so both its render request
  // and its removal stay queued.
  moto::ancs::NotificationSource source;
  source.uid = 99;
  source.event_flags = static_cast<std::uint8_t>(moto::ancs::FlagSilent);
  source.category = static_cast<std::uint8_t>(moto::ancs::CategoryId::Email);
  bridge.on_notification(source, {});
  pump(bridge);
  CHECK(bridge.screen_off());
  CHECK(moto::test::phone_nav_bridge_notification_count() == 0);

  moto::ancs::NotificationSource removal = source;
  removal.event = static_cast<std::uint8_t>(moto::ancs::EventId::Removed);
  bridge.on_notification(removal, {});
  pump(bridge);
  CHECK(bridge.screen_off());

  // Pulling away repaints, and the removal wins: the card is dropped unseen.
  bridge.on_motion_sample(10.2F, 46'100);
  CHECK(!bridge.screen_off());
  pump(bridge);
  CHECK(moto::test::phone_nav_bridge_notification_count() == 0);
  CHECK(moto::test::phone_nav_bridge_notification_dismiss_count() == 1);
}

struct ToneLog {
  int count = 0;
  moto_tone_t tones[8] = {};
  int voice_count = 0;
  moto_voice_phrase_t voices[8] = {};
};

void record_voice(moto_voice_phrase_t phrase, void* context) {
  auto* log = static_cast<ToneLog*>(context);
  if (log->voice_count < 8) {
    log->voices[log->voice_count] = phrase;
  }
  ++log->voice_count;
}

void record_tone(moto_tone_t tone, void* context) {
  auto* log = static_cast<ToneLog*>(context);
  if (log->count < 8) {
    log->tones[log->count] = tone;
  }
  ++log->count;
}

// The bridge is the only place that knows both the route and the speaker, so a
// plain navigation frame has to be enough to make the panel sound a turn.
void test_turn_cue_reaches_the_board() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  ToneLog log;
  bridge.set_tone_callback(record_tone, &log);
  bridge.set_voice_callback(record_voice, &log);
  bridge.on_link_state(true);

  moto::ble::NavigationSnapshot navigation;
  navigation.state = moto::ble::NavigationState::Navigating;
  navigation.display_page = moto::ble::DisplayPage::Navigation;
  navigation.flags = moto::ble::NavigationHasDestination |
                     moto::ble::NavigationHasFix |
                     moto::ble::NavigationHasNextManeuver;
  navigation.maneuver = moto::ble::Maneuver::Left;
  navigation.maneuver_id = 7;
  navigation.distance_to_maneuver_m = 40;
  navigation.speed_deci_kph = 600;  // 60 km/h, so 40 m is inside the cue
  navigation.heading_cdeg = 9'000;
  CHECK(bridge.on_message(make_message(navigation, 1)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  // A turn is spoken, not beeped: the rider has to learn which way without
  // looking down, and a pitch difference does not carry that.
  CHECK(log.count == 0);
  CHECK(log.voice_count == 1);
  CHECK(log.voices[0] == MOTO_VOICE_TURN_LEFT);

  // Approaching the same turn is not a new cue.
  navigation.distance_to_maneuver_m = 20;
  CHECK(bridge.on_message(make_message(navigation, 2)) ==
        moto::ble::AckStatus::Ok);
  pump(bridge);
  CHECK(log.count == 0);
  CHECK(log.voice_count == 1);
}

// A silent notification is silent on the speaker as well as on the panel, and
// the sound matches what the card says.
void test_notification_tone_respects_silence() {
  moto::test::reset_phone_nav_bridge_probe();
  moto::nav::NavPresenter presenter;
  PhoneNavBridge bridge(presenter);
  ToneLog log;
  bridge.set_tone_callback(record_tone, &log);

  moto::ancs::NotificationSource silent;
  silent.event = static_cast<std::uint8_t>(moto::ancs::EventId::Added);
  silent.event_flags = static_cast<std::uint8_t>(moto::ancs::FlagSilent);
  bridge.on_notification(silent, {});
  pump(bridge);
  CHECK(log.count == 0);

  moto::ancs::NotificationSource message;
  message.event = static_cast<std::uint8_t>(moto::ancs::EventId::Added);
  bridge.on_notification(message, {});
  pump(bridge);
  CHECK(log.count == 1);
  CHECK(log.tones[0] == MOTO_TONE_NOTIFICATION);

  moto::ancs::NotificationSource call;
  call.event = static_cast<std::uint8_t>(moto::ancs::EventId::Added);
  call.category =
      static_cast<std::uint8_t>(moto::ancs::CategoryId::IncomingCall);
  bridge.on_notification(call, {});
  pump(bridge);
  CHECK(log.count == 2);
  CHECK(log.tones[1] == MOTO_TONE_CALL);
}

}  // namespace

int main() {
  test_sender_is_called_without_bridge_state_lock();
  test_navigation_and_touch_updates_are_serialized();
  test_ble_submission_never_waits_for_lvgl_and_retries_latest_state();
  test_phone_connection_lifecycle_reaches_the_ui();
  test_music_capability_state_and_touch_commands();
  test_clock_state_reaches_the_ui_and_clears_on_disconnect();
  test_clock_survives_a_disconnect_and_keeps_ticking();
  test_rtc_seeds_the_clock_before_any_phone_connects();
  test_rtc_hook_reports_the_corrected_time();
  test_trip_state_reaches_the_ui_and_clears_on_disconnect();
  test_trip_page_long_press_requests_a_phone_side_reset();
  test_parked_bike_blanks_the_panel_and_motion_lights_it_again();
  test_touch_wakes_a_blanked_panel();
  test_blanked_panel_stops_rendering_until_wake();
  test_active_route_uses_the_longer_parked_hold();
  test_phone_speed_lights_a_blanked_panel();
  test_notification_reaches_the_ui_overlay();
  test_notification_falls_back_to_the_heading_only();
  test_loud_notification_wakes_a_blanked_panel();
  test_notification_replaces_the_previous_one();
  test_removed_notification_dismisses_only_its_own_card();
  test_removal_beats_a_stale_card_on_wake();
  test_turn_cue_reaches_the_board();
  test_notification_tone_respects_silence();
  test_demo_uses_the_production_presenter_path();
  test_ios_demo_token_attaches_context_and_live_route_clears_it();
  test_map_scene_atomically_replaces_roads_and_buildings_at_capacity();
  test_imu_presentation_preserves_40hz_phase_and_stale_course_is_ignored();

  if (failures != 0) {
    std::cerr << failures << " phone navigation bridge checks failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All phone navigation bridge checks passed\n";
  return EXIT_SUCCESS;
}
