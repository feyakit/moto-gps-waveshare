// Renders the real round-panel UI to PNG files on the host.
//
// This is a screenshot tool, not a test: it drives the same shared
// moto_nav_ui the firmware runs, through the same headless LVGL display the
// render tests use, and writes the resulting RGB565 framebuffer as a PNG. The
// images are therefore the actual layout, fonts and geometry, not a mock-up.
//
// Usage: nav_ui_screenshots <output-directory>

#include "moto_nav_ui.h"

#include <lvgl.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr int kWidth = MOTO_UI_CANVAS_WIDTH;
constexpr int kHeight = MOTO_UI_CANVAS_HEIGHT;
constexpr std::size_t kFrameBytes = static_cast<std::size_t>(kWidth) *
                                    static_cast<std::size_t>(kHeight) * 2U;

std::vector<std::uint8_t> frame;

void flush_cb(lv_display_t* display, const lv_area_t* area,
              std::uint8_t* px_map) {
  for (int y = area->y1; y <= area->y2; ++y) {
    const std::size_t row_bytes =
        static_cast<std::size_t>(area->x2 - area->x1 + 1) * 2U;
    const std::size_t src =
        static_cast<std::size_t>(y - area->y1) *
        static_cast<std::size_t>(area->x2 - area->x1 + 1) * 2U;
    const std::size_t dst =
        (static_cast<std::size_t>(y) * kWidth +
         static_cast<std::size_t>(area->x1)) * 2U;
    std::memcpy(frame.data() + dst, px_map + src, row_bytes);
  }
  lv_display_flush_ready(display);
}

void init_display() {
  lv_init();
  frame.assign(kFrameBytes, 0);

  static std::vector<std::uint8_t> draw_buffer;
  draw_buffer.resize(static_cast<std::size_t>(kWidth) * 64U * 2U);

  lv_display_t* display = lv_display_create(kWidth, kHeight);
  lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
  lv_display_set_flush_cb(display, flush_cb);
  lv_display_set_buffers(display, draw_buffer.data(), nullptr,
                         draw_buffer.size(),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
}

/// Writes the current frame as a 24-bit PNG.
bool write_png(const std::string& path) {
  const int channels = 3;
  std::vector<std::uint8_t> raw;
  raw.reserve(static_cast<std::size_t>(kHeight) * (1U + kWidth * channels));

  for (int y = 0; y < kHeight; ++y) {
    raw.push_back(0);  // filter: none
    for (int x = 0; x < kWidth; ++x) {
      const std::size_t offset =
          (static_cast<std::size_t>(y) * kWidth + x) * 2U;
      const std::uint16_t pixel =
          static_cast<std::uint16_t>(frame[offset] | (frame[offset + 1] << 8));
      const std::uint8_t r = static_cast<std::uint8_t>(((pixel >> 11) & 0x1F) * 255 / 31);
      const std::uint8_t g = static_cast<std::uint8_t>(((pixel >> 5) & 0x3F) * 255 / 63);
      const std::uint8_t b = static_cast<std::uint8_t>((pixel & 0x1F) * 255 / 31);
      raw.push_back(r);
      raw.push_back(g);
      raw.push_back(b);
    }
  }

  // zlib stream: 2-byte header, deflate block, Adler-32 trailer.
  const auto adler32 = [](const std::vector<std::uint8_t>& data) {
    std::uint32_t a = 1;
    std::uint32_t b = 0;
    for (std::uint8_t byte : data) {
      a = (a + byte) % 65521U;
      b = (b + a) % 65521U;
    }
    return (b << 16) | a;
  };

  // Store-mode deflate: no compression, which keeps this dependency-free.
  std::vector<std::uint8_t> zlib;
  zlib.push_back(0x78);
  zlib.push_back(0x01);
  std::size_t remaining = raw.size();
  std::size_t index = 0;
  while (remaining > 0) {
    const std::size_t chunk = remaining > 65535U ? 65535U : remaining;
    const bool last = chunk == remaining;
    zlib.push_back(last ? 1 : 0);
    zlib.push_back(static_cast<std::uint8_t>(chunk & 0xFF));
    zlib.push_back(static_cast<std::uint8_t>((chunk >> 8) & 0xFF));
    const std::uint16_t nlen = static_cast<std::uint16_t>(~chunk);
    zlib.push_back(static_cast<std::uint8_t>(nlen & 0xFF));
    zlib.push_back(static_cast<std::uint8_t>((nlen >> 8) & 0xFF));
    zlib.insert(zlib.end(), raw.begin() + static_cast<long>(index),
                raw.begin() + static_cast<long>(index + chunk));
    index += chunk;
    remaining -= chunk;
  }
  const std::uint32_t adler = adler32(raw);
  zlib.push_back(static_cast<std::uint8_t>((adler >> 24) & 0xFF));
  zlib.push_back(static_cast<std::uint8_t>((adler >> 16) & 0xFF));
  zlib.push_back(static_cast<std::uint8_t>((adler >> 8) & 0xFF));
  zlib.push_back(static_cast<std::uint8_t>(adler & 0xFF));

  const auto crc32 = [](const std::uint8_t* data, std::size_t length) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < length; ++i) {
      crc ^= data[i];
      for (int bit = 0; bit < 8; ++bit) {
        crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
      }
    }
    return crc ^ 0xFFFFFFFFU;
  };

  FILE* file = std::fopen(path.c_str(), "wb");
  if (file == nullptr) return false;

  const auto put32 = [&](std::uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) {
      std::fputc(static_cast<int>((value >> shift) & 0xFF), file);
    }
  };

  static const std::uint8_t kSignature[8] = {0x89, 'P', 'N', 'G', 0x0D,
                                             0x0A, 0x1A, 0x0A};
  std::fwrite(kSignature, 1, sizeof(kSignature), file);

  const auto chunk = [&](const char* type, const std::uint8_t* data,
                         std::size_t length) {
    put32(static_cast<std::uint32_t>(length));
    std::fwrite(type, 1, 4, file);
    if (length > 0) std::fwrite(data, 1, length, file);
    std::vector<std::uint8_t> crc_input;
    crc_input.insert(crc_input.end(), type, type + 4);
    if (length > 0) crc_input.insert(crc_input.end(), data, data + length);
    put32(crc32(crc_input.data(), crc_input.size()));
  };

  // IHDR layout: width (0..3), height (4..7), bit depth (8), colour type (9),
  // compression (10), filter (11), interlace (12). Writing the depth at byte 3
  // corrupted the width field and produced an unreadable PNG.
  std::uint8_t header[13] = {};
  header[0] = static_cast<std::uint8_t>((kWidth >> 24) & 0xFF);
  header[1] = static_cast<std::uint8_t>((kWidth >> 16) & 0xFF);
  header[2] = static_cast<std::uint8_t>((kWidth >> 8) & 0xFF);
  header[3] = static_cast<std::uint8_t>(kWidth & 0xFF);
  header[4] = static_cast<std::uint8_t>((kHeight >> 24) & 0xFF);
  header[5] = static_cast<std::uint8_t>((kHeight >> 16) & 0xFF);
  header[6] = static_cast<std::uint8_t>((kHeight >> 8) & 0xFF);
  header[7] = static_cast<std::uint8_t>(kHeight & 0xFF);
  header[8] = 8;  // bit depth
  header[9] = 2;  // colour type: truecolour
  chunk("IHDR", header, sizeof(header));
  chunk("IDAT", zlib.data(), zlib.size());
  chunk("IEND", nullptr, 0);
  std::fclose(file);
  return true;
}

void render() {
  // LVGL's timers and animations only advance when the tick source does; the
  // firmware's esp_timer does this in the background.
  for (int i = 0; i < 12; ++i) {
    lv_tick_inc(50);
    lv_timer_handler();
  }
}

moto_ui_state_t make_state(moto_ui_page_t page) {
  moto_ui_state_t state{};
  state.page = page;
  state.mode = MOTO_UI_NAVIGATING;
  state.online = 1;
  state.has_destination = 1;
  state.maneuver = MOTO_MANEUVER_LEFT;
  state.traffic = MOTO_TRAFFIC_CLEAR;
  state.distance_to_maneuver_m = 320;
  state.remaining_distance_m = 4820;
  state.remaining_time_s = 720;
  state.route_progress_percent = 34;
  state.speed_kph = 46;
  state.heading_deg = 72;
  state.gps_accuracy_m = 4;
  return state;
}

/// A short route plus surrounding grey streets, so the map pages show the real
/// heading-up projection instead of an empty disc.
void fill_geometry(moto_ui_state_t* state) {
  static const int route[][2] = {
      {240, 430}, {232, 372}, {236, 318}, {246, 262},
      {252, 206}, {240, 152}, {226, 106},
  };
  state->route_point_count = sizeof(route) / sizeof(route[0]);
  for (int i = 0; i < state->route_point_count; ++i) {
    state->route_points[i].x = static_cast<std::int16_t>(route[i][0]);
    state->route_points[i].y = static_cast<std::int16_t>(route[i][1]);
  }
  static const int roads[][2] = {
      {60, 300}, {140, 296}, {220, 300}, {300, 306}, {380, 312}, {430, 316},
      {150, 60}, {152, 140}, {148, 220}, {144, 300}, {140, 380}, {136, 430},
  };
  state->road_point_count = sizeof(roads) / sizeof(roads[0]);
  for (int i = 0; i < state->road_point_count; ++i) {
    state->road_points[i].x = static_cast<std::int16_t>(roads[i][0]);
    state->road_points[i].y = static_cast<std::int16_t>(roads[i][1]);
  }
  state->road_polyline_count = 1;
  state->road_polylines[0].first_point_index = 0;
  state->road_polylines[0].point_count =
      static_cast<std::uint8_t>(state->road_point_count);
  state->road_polylines[0].road_class = 2;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string output = argc > 1 ? argv[1] : ".";
  init_display();
  moto_nav_ui_create();
  moto_nav_ui_set_phone_connection(MOTO_UI_PHONE_ONLINE);
  moto_nav_ui_set_battery(82, 1);
  moto_nav_ui_set_link_quality(3);

  struct Shot {
    const char* name;
    moto_ui_page_t page;
  };
  const Shot shots[] = {
      {"01-navigation", MOTO_UI_PAGE_NAVIGATION},
      {"02-speed", MOTO_UI_PAGE_SPEED},
      {"03-trip", MOTO_UI_PAGE_TRIP},
      {"04-compass", MOTO_UI_PAGE_COMPASS},
      {"05-clock", MOTO_UI_PAGE_CLOCK},
      {"06-messages", MOTO_UI_PAGE_MESSAGES},
  };

  // Trip and clock need their own payloads; set once and reused per shot.
  moto_ui_trip_t trip{};
  trip.distance_m = 18400;
  trip.moving_time_s = 2460;
  trip.elevation_gain_m = 210;
  trip.max_speed_deci_kph = 812;
  trip.recording = 1;
  trip.elevation_known = 1;
  moto_nav_ui_set_trip(&trip);

  moto_ui_clock_t clock{};
  clock.minutes = 18 * 60 + 42;
  clock.weekday = 6;
  clock.month = 10;
  clock.day = 18;
  clock.second = 20;
  moto_nav_ui_set_clock(&clock);

  // The lifecycle veil covers the navigation page until the session reaches a
  // steady state, and it fades out rather than disappearing instantly. Push the
  // connection to online, then advance the animation clock far enough for every
  // fade to finish before the first frame is captured.
  moto_nav_ui_set_phone_connection(MOTO_UI_PHONE_ONLINE);
  for (int i = 0; i < 60; ++i) {
    lv_tick_inc(50);
    lv_timer_handler();
  }

  for (const Shot& shot : shots) {
    moto_ui_state_t state = make_state(shot.page);
    fill_geometry(&state);
    moto_nav_ui_set_state(&state);
    // Other pages are not covered by the veil, but the clock hands and the
    // fade animations still need the timer to run.
    for (int i = 0; i < 20; ++i) {
      lv_tick_inc(50);
      lv_timer_handler();
    }
    const std::string path = output + "/" + shot.name + ".png";
    if (!write_png(path)) {
      std::fprintf(stderr, "could not write %s\n", path.c_str());
      return 1;
    }
    std::printf("wrote %s\n", path.c_str());
  }

  // The notification card, shown over the navigation page.
  moto_ui_state_t nav = make_state(MOTO_UI_PAGE_NAVIGATION);
  fill_geometry(&nav);
  moto_nav_ui_set_state(&nav);
  moto_ui_notification_t card{};
  card.category = "SOCIAL";
  card.app = "微信";
  card.title = "初见";
  card.message = "晚上八点老地方，到了给我发消息";
  card.hold_ms = 20000;
  card.local_minutes = 18 * 60 + 42;
  moto_nav_ui_show_notification(&card);
  render();
  if (!write_png(output + "/07-notification.png")) {
    std::fprintf(stderr, "could not write notification\n");
    return 1;
  }
  std::printf("wrote 07-notification.png\n");

  return 0;
}
