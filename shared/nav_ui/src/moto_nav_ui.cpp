#include "moto_nav_ui.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>

#include "lvgl.h"

LV_FONT_DECLARE(moto_font_clock_96);
LV_FONT_DECLARE(moto_font_nav_16);
// Notification text is arbitrary user content, so these carry the GB2312
// level-1 Hanzi set. Instrument copy stays on Montserrat.
LV_FONT_DECLARE(moto_font_notify_16);
LV_FONT_DECLARE(moto_font_notify_20);
// The full-screen notification card has its own, much larger sizes: it is read
// at a glance at speed, not studied.
LV_FONT_DECLARE(moto_font_notify_28);
LV_FONT_DECLARE(moto_font_notify_34);

namespace {

constexpr lv_color_t kBlack = LV_COLOR_MAKE(0x05, 0x06, 0x07);
constexpr lv_color_t kWhite = LV_COLOR_MAKE(0xF3, 0xF4, 0xEF);
constexpr lv_color_t kIce = LV_COLOR_MAKE(0xB8, 0xED, 0xF5);
constexpr lv_color_t kGraphite = LV_COLOR_MAKE(0x30, 0x35, 0x39);
// Real surrounding roads need to remain legible on the AMOLED's true-black
// background.  LV_OPA_70 is 70/255 (not 70 percent), which made the previous
// road layer effectively disappear on the physical display.
constexpr lv_color_t kRoadGray = LV_COLOR_MAKE(0x42, 0x47, 0x4B);
constexpr lv_color_t kRoadMajor = LV_COLOR_MAKE(0x62, 0x69, 0x6D);
constexpr lv_color_t kRoadMinor = LV_COLOR_MAKE(0x2B, 0x30, 0x33);
constexpr lv_color_t kBuildingGray = LV_COLOR_MAKE(0x24, 0x29, 0x2C);
constexpr lv_color_t kBuildingLandmark = LV_COLOR_MAKE(0x38, 0x40, 0x44);
constexpr lv_color_t kQuiet = LV_COLOR_MAKE(0x78, 0x7E, 0x7F);
constexpr lv_color_t kSoft = LV_COLOR_MAKE(0xAE, 0xB2, 0xB0);
constexpr lv_color_t kAmber = LV_COLOR_MAKE(0xE6, 0xC8, 0x4F);
constexpr lv_color_t kRed = LV_COLOR_MAKE(0xFF, 0x4B, 0x43);
constexpr lv_color_t kGreen = LV_COLOR_MAKE(0x69, 0xD4, 0x94);
constexpr double kPi = 3.14159265358979323846;
constexpr int kCompassTickCount = 24;
constexpr int kSpeedTickCount = 18;
constexpr int kClockTickCount = 60;
// Trip page rows: moving time, average speed, maximum speed. The hero number
// above them is total distance.
constexpr int kTripRowMoving = 0;
constexpr int kTripRowAverage = 1;
constexpr int kTripRowMax = 2;
constexpr int kTripRowCount = 3;
constexpr int kDesignWidth = 360;
constexpr std::uint32_t kPageDotsVisibleMs = 5'000;
// Keep LVGL, the UI interpolation timer and IMU presentation on the same
// 40 Hz cadence. The previous 40/33 ms mismatch periodically produced a
// 66 ms visual gap even when both tasks were otherwise keeping up.
constexpr std::uint32_t kRouteMotionFrameMs = 25;
static_assert(LV_DEF_REFR_PERIOD == kRouteMotionFrameMs,
              "LVGL refresh and map motion must use the same cadence");
constexpr std::uint32_t kConnectionSuccessHoldMs = 920;

enum class LifecycleVisual : std::uint8_t {
    Hidden = 0,
    PhoneOffline,
    PhoneConnecting,
    Connected,
    Ready,
    Planning,
};

constexpr int px(int value) {
    return value >= 0
               ? (value * MOTO_UI_CANVAS_WIDTH + kDesignWidth / 2) /
                     kDesignWidth
               : -((-value * MOTO_UI_CANVAS_WIDTH + kDesignWidth / 2) /
                   kDesignWidth);
}

constexpr double px(double value) {
    return value * static_cast<double>(MOTO_UI_CANVAS_WIDTH) /
           static_cast<double>(kDesignWidth);
}

struct MapPolyline {
    const lv_point_precise_t *points = nullptr;
    std::uint16_t count = 0;
};

// Notification card geometry and the history ring. Declared before Ui because
// the history entries are sized from these.
// Design-space width for the card text. The round panel is 480 px across at
// its widest, but the chord shrinks fast away from the middle: at the top of
// the title band only ~416 px is visible, and near the bottom of the body only
// ~358 px. 260 design px scales to ~347 real px, which stays inside the glass
// at every row the card uses and still leaves a readable line length.
constexpr int kNotificationTextWidth = 260;
constexpr int kNotificationTitleHeight = 124;
constexpr int kNotificationMessageHeight = 150;
constexpr int kNotificationHistorySize = 8;
constexpr int kNotificationTextCapacity = 96;

struct Ui {
    lv_obj_t *screen = nullptr;
    lv_obj_t *pages[MOTO_UI_PAGE_COUNT]{};
    lv_obj_t *page_dots[MOTO_UI_PAGE_COUNT]{};
    lv_timer_t *page_dots_timer = nullptr;
    moto_ui_page_t page = MOTO_UI_PAGE_NAVIGATION;
    bool page_dots_visible = true;
    // True while the rider is on the local Messages page. The phone never
    // drives this page, so snapshot-driven page changes are ignored until the
    // rider swipes away.
    // True once the rider has swiped away from the navigation page. While it is
    // set, snapshot-driven page changes are ignored: navigation snapshots keep
    // arriving several times a second during a ride, and letting them choose
    // the page yanked the rider back to Navigation on every update -- the
    // visible "flash and snap back" when turning the wheel.
    bool user_selected_page = false;

    lv_obj_t *nav_map = nullptr;
    lv_obj_t *nav_buildings[MOTO_UI_BUILDING_FOOTPRINT_CAPACITY]{};
    MapPolyline nav_building_lines[MOTO_UI_BUILDING_FOOTPRINT_CAPACITY]{};
    lv_point_precise_t nav_building_points[
        MOTO_UI_BUILDING_POINT_CAPACITY +
        MOTO_UI_BUILDING_FOOTPRINT_CAPACITY]{};
    float nav_building_x[MOTO_UI_BUILDING_POINT_CAPACITY]{};
    float nav_building_y[MOTO_UI_BUILDING_POINT_CAPACITY]{};
    float nav_building_target_x[MOTO_UI_BUILDING_POINT_CAPACITY]{};
    float nav_building_target_y[MOTO_UI_BUILDING_POINT_CAPACITY]{};
    moto_ui_building_span_t
        nav_building_spans[MOTO_UI_BUILDING_FOOTPRINT_CAPACITY]{};
    std::uint8_t nav_building_point_count = 0;
    std::uint8_t nav_building_footprint_count = 0;
    std::uint32_t nav_building_scene_revision = 0;
    lv_obj_t *nav_roads[MOTO_UI_ROAD_POLYLINE_CAPACITY]{};
    MapPolyline nav_road_lines[MOTO_UI_ROAD_POLYLINE_CAPACITY]{};
    lv_point_precise_t nav_road_points[MOTO_UI_ROAD_POINT_CAPACITY]{};
    float nav_road_x[MOTO_UI_ROAD_POINT_CAPACITY]{};
    float nav_road_y[MOTO_UI_ROAD_POINT_CAPACITY]{};
    float nav_road_target_x[MOTO_UI_ROAD_POINT_CAPACITY]{};
    float nav_road_target_y[MOTO_UI_ROAD_POINT_CAPACITY]{};
    moto_ui_polyline_span_t
        nav_road_spans[MOTO_UI_ROAD_POLYLINE_CAPACITY]{};
    std::uint8_t nav_road_point_count = 0;
    std::uint8_t nav_road_polyline_count = 0;
    std::uint32_t nav_road_scene_revision = 0;
    lv_obj_t *nav_route_shadow = nullptr;
    lv_obj_t *nav_route = nullptr;
    MapPolyline nav_route_shadow_line{};
    MapPolyline nav_route_line{};
    lv_point_precise_t nav_route_points[MOTO_UI_ROUTE_POINT_CAPACITY]{};
    float nav_route_x[MOTO_UI_ROUTE_POINT_CAPACITY]{};
    float nav_route_y[MOTO_UI_ROUTE_POINT_CAPACITY]{};
    float nav_route_target_x[MOTO_UI_ROUTE_POINT_CAPACITY]{};
    float nav_route_target_y[MOTO_UI_ROUTE_POINT_CAPACITY]{};
    std::uint8_t nav_route_point_count = 0;
    std::uint8_t nav_route_target_count = 0;
    std::uint32_t nav_route_identity = 0;
    std::uint32_t nav_route_generation = 0;
    lv_timer_t *nav_route_motion_timer = nullptr;
    lv_obj_t *nav_marker = nullptr;
    lv_obj_t *nav_status = nullptr;
    lv_obj_t *status_battery_body = nullptr;
    lv_obj_t *status_battery_fill = nullptr;
    lv_obj_t *status_battery_cap = nullptr;
    lv_obj_t *status_battery_text = nullptr;
    lv_obj_t *status_bars[4]{};
    uint8_t battery_percent = MOTO_UI_BATTERY_UNKNOWN;
    uint8_t battery_charging = 0;
    uint8_t link_bars = 0;
    lv_obj_t *nav_distance = nullptr;
    lv_obj_t *nav_unit = nullptr;
    lv_obj_t *nav_maneuver = nullptr;
    moto_maneuver_t nav_maneuver_type = MOTO_MANEUVER_STRAIGHT;
    lv_obj_t *nav_limit = nullptr;
    lv_obj_t *nav_limit_value = nullptr;
    lv_obj_t *nav_progress = nullptr;

    // A full-screen empty-state surface replaces the old tiny
    // WAITING FOR PHONE / WAITING FOR ROUTE caption. It is kept inside the
    // navigation page so the speed and compass tools remain independently
    // usable while the phone is disconnected.
    lv_obj_t *nav_lifecycle = nullptr;
    lv_obj_t *nav_lifecycle_symbol = nullptr;
    lv_obj_t *nav_lifecycle_title = nullptr;
    lv_obj_t *nav_lifecycle_subtitle = nullptr;
    lv_obj_t *nav_lifecycle_kicker = nullptr;
    lv_timer_t *nav_lifecycle_timer = nullptr;

    // Phone notifications are drawn on a full-canvas overlay so they stay
    // readable over the map and never disturb the instrument layout.
    lv_obj_t *notification_overlay = nullptr;
    lv_obj_t *notification_category = nullptr;
    lv_obj_t *notification_badge = nullptr;
    lv_obj_t *notification_app = nullptr;
    lv_obj_t *notification_title = nullptr;
    lv_obj_t *notification_message = nullptr;
    lv_obj_t *notification_hint = nullptr;
    lv_timer_t *notification_timer = nullptr;
    std::uint8_t notification_unread = 0;
    // Ring of recent notifications, newest at notification_head.
    struct HistoryEntry {
        char app[kNotificationTextCapacity]{};
        char title[kNotificationTextCapacity]{};
        char message[kNotificationTextCapacity]{};
        uint16_t local_minutes = MOTO_UI_CLOCK_UNKNOWN;
    };
    HistoryEntry notification_history[kNotificationHistorySize]{};
    int notification_head = 0;
    int notification_total = 0;
    // History page rows: title line plus the app/time line under it.
    lv_obj_t *history_rows[kNotificationHistorySize]{};
    // Second line of each entry: the body, which is what makes the log useful
    // at a glance instead of a list of app names.
    lv_obj_t *history_bodies[kNotificationHistorySize]{};
    lv_obj_t *history_empty = nullptr;
    lv_obj_t *history_back = nullptr;
    lv_timer_t *nav_success_timer = nullptr;
    moto_ui_phone_connection_t phone_connection = MOTO_UI_PHONE_OFFLINE;
    LifecycleVisual lifecycle_visual = LifecycleVisual::Hidden;
    LifecycleVisual lifecycle_target = LifecycleVisual::Hidden;
    std::uint16_t lifecycle_phase_deg = 0;
    bool lifecycle_success_active = false;
    bool navigation_has_guidance = false;
    bool navigation_route_request_in_flight = false;

    lv_obj_t *speed_arc = nullptr;
    lv_obj_t *speed_value = nullptr;
    lv_obj_t *speed_ticks[kSpeedTickCount]{};
    lv_point_precise_t speed_tick_points[kSpeedTickCount][2]{};

    lv_obj_t *compass_heading = nullptr;
    lv_obj_t *compass_cardinal = nullptr;
    lv_obj_t *compass_speed = nullptr;
    lv_obj_t *compass_ticks[kCompassTickCount]{};
    lv_point_precise_t compass_tick_points[kCompassTickCount][2]{};
    lv_obj_t *compass_letters[4]{};

    lv_obj_t *clock_ticks[kClockTickCount]{};
    lv_point_precise_t clock_tick_points[kClockTickCount][2]{};
    lv_obj_t *clock_time = nullptr;
    lv_obj_t *clock_date = nullptr;
    // Sweeping hands, drawn as mutable lines so one tick only rewrites points.
    lv_obj_t *clock_hand_hour = nullptr;
    lv_obj_t *clock_hand_minute = nullptr;
    lv_obj_t *clock_hand_second = nullptr;
    lv_point_precise_t clock_hand_hour_points[2]{};
    lv_point_precise_t clock_hand_minute_points[2]{};
    lv_point_precise_t clock_hand_second_points[2]{};
    // Angles actually drawn, eased towards the true time so a tick sweeps
    // instead of teleporting. -1 means "not positioned yet".
    double clock_hand_hour_shown = -1.0;
    double clock_hand_minute_shown = -1.0;
    double clock_hand_second_shown = -1.0;
    // Where each sweep started and when, so the angle is a pure function of
    // the elapsed time.
    double clock_hand_hour_from = -1.0;
    double clock_hand_minute_from = -1.0;
    double clock_hand_second_from = -1.0;
    std::uint32_t clock_sweep_start_ms = 0;
    moto_ui_clock_t clock{};

    lv_obj_t *trip_distance = nullptr;
    lv_obj_t *trip_unit = nullptr;
    lv_obj_t *trip_row_labels[kTripRowCount]{};
    lv_obj_t *trip_row_values[kTripRowCount]{};
    lv_obj_t *trip_caption = nullptr;
    moto_ui_trip_t trip{};

    lv_obj_t *music_source = nullptr;
    lv_obj_t *music_title = nullptr;
    lv_obj_t *music_artist = nullptr;
    lv_obj_t *music_disc = nullptr;
    lv_obj_t *music_buttons[4]{};
    lv_obj_t *music_button_labels[4]{};
    moto_music_state_t music{};
    char music_source_text[32]{};
    char music_title_text[64]{};
    char music_artist_text[48]{};

    moto_music_command_callback_t music_callback = nullptr;
    void *music_callback_context = nullptr;
    moto_page_change_callback_t page_callback = nullptr;
    void *page_callback_context = nullptr;
    moto_demo_change_callback_t demo_callback = nullptr;
    void *demo_callback_context = nullptr;
    moto_trip_reset_callback_t trip_reset_callback = nullptr;
    void *trip_reset_callback_context = nullptr;
    moto_interaction_callback_t interaction_callback = nullptr;
    void *interaction_callback_context = nullptr;
    bool music_page_enabled = true;
    bool demo_active = false;
    bool reduce_motion = false;
} ui;

void reset_ui_state() {
    static_assert(std::is_trivially_copyable_v<Ui>,
                  "Ui must remain safe to reset without a stack temporary");

    // `ui = {}` materializes the complete aggregate as a temporary.  The
    // offline map buffers make Ui about 15 KiB, so that temporary can consume
    // nearly the whole ESP-IDF main-task stack before LVGL is called.  Clear
    // the retained instance in place and restore the two non-zero defaults.
    std::memset(&ui, 0, sizeof(ui));
    ui.page = MOTO_UI_PAGE_NAVIGATION;
    ui.page_dots_visible = true;
    ui.nav_maneuver_type = MOTO_MANEUVER_STRAIGHT;
    ui.phone_connection = MOTO_UI_PHONE_OFFLINE;
    ui.lifecycle_visual = LifecycleVisual::Hidden;
    ui.lifecycle_target = LifecycleVisual::Hidden;
    ui.music_page_enabled = true;
    ui.clock.minutes = MOTO_UI_CLOCK_UNKNOWN;
    // Force the first geometry payload to bind every mutable LVGL line even
    // when its protocol revision happens to start at zero.
    ui.nav_route_generation = ~std::uint32_t{0};
    ui.nav_route_identity = ~std::uint32_t{0};
    ui.nav_road_scene_revision = ~std::uint32_t{0};
    ui.nav_building_scene_revision = ~std::uint32_t{0};
}

lv_obj_t *make_layer(lv_obj_t *parent) {
    lv_obj_t *layer = lv_obj_create(parent);
    lv_obj_remove_style_all(layer);
    lv_obj_set_size(layer, MOTO_UI_CANVAS_WIDTH, MOTO_UI_CANVAS_HEIGHT);
    lv_obj_set_pos(layer, 0, 0);
    lv_obj_remove_flag(layer, LV_OBJ_FLAG_SCROLLABLE);
    return layer;
}

lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font,
                     lv_color_t color, const char *text) {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_label_set_text(label, text);
    return label;
}

void copy_text(char *destination, std::size_t capacity, const char *source,
               const char *fallback) {
    if(capacity == 0) return;
    const char *value = source != nullptr && source[0] != '\0' ? source : fallback;
    std::snprintf(destination, capacity, "%s", value != nullptr ? value : "");
}

lv_color_t traffic_color(moto_traffic_t traffic) {
    switch(traffic) {
        case MOTO_TRAFFIC_CLEAR: return kIce;
        case MOTO_TRAFFIC_SLOW: return kAmber;
        case MOTO_TRAFFIC_CONGESTED:
        case MOTO_TRAFFIC_SEVERE: return kRed;
        default: return kQuiet;
    }
}

const char *cardinal_name(std::uint16_t heading) {
    static const char *names[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    return names[((heading + 22U) / 45U) % 8U];
}

void draw_vehicle_marker(lv_event_t *event) {
    lv_obj_t *object = lv_event_get_target_obj(event);
    lv_area_t area;
    lv_obj_get_coords(object, &area);
    lv_layer_t *layer = lv_event_get_layer(event);

    lv_draw_triangle_dsc_t triangle;
    lv_draw_triangle_dsc_init(&triangle);
    triangle.color = kBlack;
    triangle.opa = LV_OPA_COVER;
    triangle.p[0] = {static_cast<lv_value_precise_t>(area.x1 + px(17)),
                     static_cast<lv_value_precise_t>(area.y1)};
    triangle.p[1] = {static_cast<lv_value_precise_t>(area.x1),
                     static_cast<lv_value_precise_t>(area.y1 + px(36))};
    triangle.p[2] = {static_cast<lv_value_precise_t>(area.x1 + px(34)),
                     static_cast<lv_value_precise_t>(area.y1 + px(36))};
    lv_draw_triangle(layer, &triangle);

    triangle.color = kWhite;
    triangle.p[0] = {static_cast<lv_value_precise_t>(area.x1 + px(17)),
                     static_cast<lv_value_precise_t>(area.y1 + px(5))};
    triangle.p[1] = {static_cast<lv_value_precise_t>(area.x1 + px(6)),
                     static_cast<lv_value_precise_t>(area.y1 + px(30))};
    triangle.p[2] = {static_cast<lv_value_precise_t>(area.x1 + px(28)),
                     static_cast<lv_value_precise_t>(area.y1 + px(30))};
    lv_draw_triangle(layer, &triangle);
}

bool page_available(int index) {
    if(index == MOTO_UI_PAGE_MUSIC) return ui.music_page_enabled;
    // The Messages page is always reachable. Hiding it until a notification
    // arrived made it impossible to tell "the page is missing" apart from
    // "nothing has arrived yet", and an empty log with an explicit hint is a
    // better answer than a page that silently is not there.
    return true;
}

void update_page_dots() {
    // Lay the row out from the real widths instead of a fixed pitch. The active
    // dot is 16 px wide and the rest are 5 px, so a fixed step left the gap
    // after the active dot at 6 px while every other gap was 17 px.
    // These are design-space values: everything below goes through px(), which
    // scales the 360 px design to the real canvas. Mixing a raw pixel centre in
    // here shifted the whole row to the right by a third of the screen.
    constexpr int kDotWidth = 5;
    constexpr int kActiveWidth = 16;
    constexpr int kGap = 11;
    constexpr int kRowCenter = kDesignWidth / 2;

    int total = 0;
    int visible = 0;
    for(int i = 0; i < MOTO_UI_PAGE_COUNT; ++i) {
        if(!page_available(i)) continue;
        total += i == static_cast<int>(ui.page) ? kActiveWidth : kDotWidth;
        ++visible;
    }
    if(visible > 1) total += kGap * (visible - 1);

    int x = kRowCenter - total / 2;
    for(int i = 0; i < MOTO_UI_PAGE_COUNT; ++i) {
        const bool active = i == static_cast<int>(ui.page);
        const int width = active ? kActiveWidth : kDotWidth;
        lv_obj_set_size(ui.page_dots[i], px(width), px(5));
        lv_obj_set_style_radius(ui.page_dots[i], px(3), 0);
        lv_obj_set_style_bg_color(ui.page_dots[i], active ? kWhite : kGraphite, 0);
        if(page_available(i)) {
            lv_obj_set_x(ui.page_dots[i], px(x));
            x += width + kGap;
        }
        if(ui.page_dots_visible && page_available(i)) {
            lv_obj_remove_flag(ui.page_dots[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(ui.page_dots[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void refresh_history_page();

void hide_page_dots(lv_timer_t *) {
    ui.page_dots_visible = false;
    update_page_dots();
    if(ui.page_dots_timer != nullptr) lv_timer_pause(ui.page_dots_timer);
}

void reveal_page_dots() {
    if(ui.screen == nullptr || ui.page_dots[0] == nullptr) return;
    ui.page_dots_visible = true;
    update_page_dots();
    if(ui.page_dots_timer != nullptr) {
        lv_timer_set_period(ui.page_dots_timer, kPageDotsVisibleMs);
        lv_timer_reset(ui.page_dots_timer);
        lv_timer_resume(ui.page_dots_timer);
    }
}

// Recent notifications kept for the history page. Eight is what the round
// screen can list legibly; older entries scroll out rather than being dropped
// silently, which is what the counter on the page shows.
constexpr std::uint32_t kNotificationHoldMs = 8'000;

void hide_notification_overlay() {
    if(ui.notification_overlay == nullptr) return;
    lv_obj_add_flag(ui.notification_overlay, LV_OBJ_FLAG_HIDDEN);
    if(ui.notification_timer != nullptr) lv_timer_pause(ui.notification_timer);
}

void notification_timeout(lv_timer_t *timer) {
    // The UI tree can be rebuilt while a timer is still alive.
    if(ui.notification_overlay == nullptr) {
        lv_timer_pause(timer);
        return;
    }
    hide_notification_overlay();
}

void notification_pressed(lv_event_t *) {
    if(ui.notification_overlay == nullptr) return;
    ui.notification_unread = 0;
    hide_notification_overlay();
}

void create_notification_overlay() {
    lv_obj_t *overlay = make_layer(ui.screen);
    ui.notification_overlay = overlay;
    lv_obj_set_style_bg_color(overlay, kBlack, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(overlay, notification_pressed, LV_EVENT_PRESSED,
                        nullptr);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);

    // Vertical rhythm for the card, top to bottom. Every band is sized for the
    // font it holds plus a gap, so nothing can collide when a long app name or
    // a wrapped title appears:
    //   44..66   category + unread badge   (16 px)
    //   74..104  app name                  (20 px)
    //  116..248  title, up to 2 lines      (34 px)
    //  258..410  body, up to 3 lines       (28 px)
    //   424..446  tap hint                 (16 px)
    ui.notification_category =
        make_label(overlay, &lv_font_montserrat_16, kIce, "NOTIFICATION");
    lv_obj_set_style_text_letter_space(ui.notification_category, px(4), 0);
    lv_obj_set_width(ui.notification_category, px(240));
    lv_obj_set_style_text_align(ui.notification_category, LV_TEXT_ALIGN_CENTER,
                                0);
    lv_obj_align(ui.notification_category, LV_ALIGN_TOP_MID, 0, px(44));

    ui.notification_badge = make_label(overlay, &lv_font_montserrat_16, kQuiet,
                                       "");
    lv_obj_align(ui.notification_badge, LV_ALIGN_TOP_RIGHT, -px(46), px(44));

    // The app line is the bundle identifier when iOS has no display name for
    // it, so it is centred and dotted rather than given a whole row.
    ui.notification_app = make_label(overlay, &moto_font_notify_20, kQuiet, "");
    lv_obj_set_width(ui.notification_app, px(300));
    lv_obj_set_height(ui.notification_app, px(30));
    lv_label_set_long_mode(ui.notification_app, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(ui.notification_app, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(ui.notification_app, LV_ALIGN_TOP_MID, 0, px(74));

    // Title and body flow in one column so their spacing follows the text
    // instead of a fixed band: a one-line title used to leave an 80 px gap
    // before the body started, which looked like a broken layout.
    lv_obj_t *text_column = lv_obj_create(overlay);
    lv_obj_remove_style_all(text_column);
    lv_obj_set_width(text_column, px(kNotificationTextWidth));
    lv_obj_set_height(text_column, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(text_column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(text_column, px(14), 0);
    lv_obj_set_style_pad_all(text_column, 0, 0);
    lv_obj_align(text_column, LV_ALIGN_TOP_MID, 0, px(112));
    lv_obj_clear_flag(text_column, LV_OBJ_FLAG_SCROLLABLE);

    ui.notification_title =
        make_label(text_column, &moto_font_notify_34, kWhite, "");
    lv_obj_set_width(ui.notification_title, lv_pct(100));
    lv_label_set_long_mode(ui.notification_title, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(ui.notification_title, LV_TEXT_ALIGN_CENTER,
                                0);

    // One size down from the title and in the softer grey: the two lines were
    // nearly identical before, so the header did not read as a header.
    ui.notification_message =
        make_label(text_column, &moto_font_notify_28, kSoft, "");
    lv_obj_set_width(ui.notification_message, lv_pct(100));
    lv_label_set_long_mode(ui.notification_message, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(ui.notification_message, LV_TEXT_ALIGN_CENTER,
                                0);

    ui.notification_hint = make_label(overlay, &lv_font_montserrat_16, kQuiet,
                                      "TAP TO CLOSE");
    lv_obj_set_style_text_letter_space(ui.notification_hint, px(2), 0);
    lv_obj_align(ui.notification_hint, LV_ALIGN_BOTTOM_MID, 0, -px(30));

    ui.notification_timer =
        lv_timer_create(notification_timeout, kNotificationHoldMs, nullptr);
    lv_timer_pause(ui.notification_timer);
}

void interaction_event(lv_event_t *) {
    reveal_page_dots();
    if(ui.interaction_callback != nullptr) {
        ui.interaction_callback(ui.interaction_callback_context);
    }
}

void install_interaction_wake(lv_obj_t *object) {
    if(object == nullptr) return;
    lv_obj_add_event_cb(object, interaction_event, LV_EVENT_PRESSED, nullptr);
    const std::uint32_t child_count = lv_obj_get_child_count(object);
    for(std::uint32_t index = 0; index < child_count; ++index) {
        install_interaction_wake(lv_obj_get_child(object,
                                                  static_cast<int32_t>(index)));
    }
}

void show_page(moto_ui_page_t page, bool reveal_on_same_page = false) {
    if(page < MOTO_UI_PAGE_NAVIGATION || page >= MOTO_UI_PAGE_COUNT) return;
    if(page == MOTO_UI_PAGE_MUSIC && !ui.music_page_enabled) {
        page = MOTO_UI_PAGE_NAVIGATION;
    }
    const bool changed = page != ui.page;
    ui.page = page;
    for(int i = 0; i < MOTO_UI_PAGE_COUNT; ++i) {
        if(i == static_cast<int>(page)) {
            lv_obj_remove_flag(ui.pages[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(ui.pages[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    // The log has no data of its own arriving: it only changes when a
    // notification lands, so the rows are (re)built whenever the page becomes
    // visible rather than waiting for a snapshot update.
    if(page == MOTO_UI_PAGE_MESSAGES) refresh_history_page();
    update_page_dots();
    // Navigation state arrives continuously. Only a real page transition or
    // user action may restart the five-second affordance timer.
    if(changed || reveal_on_same_page) reveal_page_dots();
}

/// Swipe detection from raw press/release points.
///
/// LVGL's gesture recogniser is compiled out (it forces LV_USE_FLOAT, which
/// breaks the integer map geometry), so the direction is derived here: a drag
/// that covers enough horizontal distance and ends with the finger still
/// moving sideways counts as a page swipe. Vertical drags are left alone so
/// the Messages list can scroll.
void swipe_event(lv_event_t *event) {
    lv_indev_t *indev = lv_indev_active();
    if(indev == nullptr) return;
    const lv_event_code_t code = lv_event_get_code(event);

    static lv_point_t start{};
    static lv_point_t last{};
    static bool tracking = false;

    lv_point_t point{};
    lv_indev_get_point(indev, &point);

    if(code == LV_EVENT_PRESSED) {
        start = point;
        last = point;
        tracking = true;
        return;
    }
    if(!tracking) return;
    if(code == LV_EVENT_PRESSING) {
        last = point;
        return;
    }
    if(code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) return;

    tracking = false;
    const int dx = point.x - start.x;
    const int dy = point.y - start.y;
    // Both thresholds are in real pixels: 18% of the width is a deliberate
    // swipe on a 480 px panel, and the horizontal travel must clearly beat the
    // vertical one so a diagonal scroll is not read as a page change.
    const int threshold = MOTO_UI_CANVAS_WIDTH / 5;
    if(std::abs(dx) < threshold || std::abs(dx) <= std::abs(dy)) return;

    const int step = dx < 0 ? 1 : MOTO_UI_PAGE_COUNT - 1;
    int next = (static_cast<int>(ui.page) + step) % MOTO_UI_PAGE_COUNT;
    for(int hop = 0; hop < MOTO_UI_PAGE_COUNT && !page_available(next); ++hop) {
        next = (next + step) % MOTO_UI_PAGE_COUNT;
    }
    const auto requested = static_cast<moto_ui_page_t>(next);
    if(ui.page_callback != nullptr) {
        ui.page_callback(requested, ui.page_callback_context);
    } else {
        show_page(requested, true);
    }
    reveal_page_dots();
}

void draw_map_line(lv_layer_t *layer, const lv_draw_line_dsc_t &line) {
    const lv_area_t original_clip = layer->_clip_area;
    lv_area_t visible{
        std::max(original_clip.x1, static_cast<int32_t>(
            std::min(line.p1.x, line.p2.x) - line.width)),
        std::max(original_clip.y1, static_cast<int32_t>(
            std::min(line.p1.y, line.p2.y) - line.width)),
        std::min(original_clip.x2, static_cast<int32_t>(
            std::max(line.p1.x, line.p2.x) + line.width)),
        std::min(original_clip.y2, static_cast<int32_t>(
            std::max(line.p1.y, line.p2.y) + line.width)),
    };
    if(visible.x1 > visible.x2 || visible.y1 > visible.y2) return;
    const float dx = static_cast<float>(line.p2.x - line.p1.x);
    const float dy = static_cast<float>(line.p2.y - line.p1.y);
    constexpr int32_t band_height = 32;
    if(std::abs(dy) <= band_height || std::abs(dx) <= 64.0F ||
       visible.y2 - visible.y1 < band_height) {
        lv_draw_line(layer, &line);
        return;
    }

    // LVGL masks every pixel in a diagonal's bounding rectangle, including
    // the empty space beside a long thin street. Narrow that rectangle per
    // non-overlapping band. Keep the original endpoints so antialiasing and
    // round caps remain identical, including where adjacent bands meet.
    const float slope = dx / dy;
    const float padding = static_cast<float>(line.width) + 2.0F;
    const float minimum_y = static_cast<float>(std::min(line.p1.y, line.p2.y));
    const float maximum_y = static_cast<float>(std::max(line.p1.y, line.p2.y));
    for(int32_t y = visible.y1; y <= visible.y2; y += band_height) {
        lv_area_t band = visible;
        band.y1 = y;
        band.y2 = std::min(y + band_height - 1, visible.y2);
        const float low = std::clamp(static_cast<float>(band.y1) - padding,
                                     minimum_y, maximum_y);
        const float high = std::clamp(static_cast<float>(band.y2) + padding,
                                      minimum_y, maximum_y);
        const float a = line.p1.x + slope * (low - line.p1.y);
        const float b = line.p1.x + slope * (high - line.p1.y);
        band.x1 = std::max(band.x1, static_cast<int32_t>(
            std::floor(std::min(a, b) - padding)));
        band.x2 = std::min(band.x2, static_cast<int32_t>(
            std::ceil(std::max(a, b) + padding)));
        if(band.x1 > band.x2) continue;
        // lv_draw_line copies this clip into its task before dispatching.
        layer->_clip_area = band;
        lv_draw_line(layer, &line);
    }
    layer->_clip_area = original_clip;
}

void draw_map_polyline(lv_event_t *event) {
    lv_obj_t *object = lv_event_get_target_obj(event);
    const auto code = lv_event_get_code(event);
    if(code == LV_EVENT_REFR_EXT_DRAW_SIZE) {
        auto *size = static_cast<int32_t *>(lv_event_get_param(event));
        *size = std::max(*size, lv_obj_get_style_line_width(object, LV_PART_MAIN));
        return;
    }
    if(code != LV_EVENT_DRAW_MAIN) return;
    const auto *polyline = static_cast<const MapPolyline *>(
        lv_event_get_user_data(event));
    if(polyline->points == nullptr || polyline->count < 2) return;
    lv_layer_t *layer = lv_event_get_layer(event);
    lv_area_t object_area;
    lv_obj_get_coords(object, &object_area);
    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.base.layer = layer;
    lv_obj_init_draw_line_dsc(object, LV_PART_MAIN, &line);
    for(std::uint16_t index = 1; index < polyline->count; ++index) {
        line.p1 = polyline->points[index - 1];
        line.p2 = polyline->points[index];
        line.p1.x += object_area.x1;
        line.p2.x += object_area.x1;
        line.p1.y += object_area.y1;
        line.p2.y += object_area.y1;
        draw_map_line(layer, line);
        line.round_start = 0;
    }
}

lv_obj_t *create_map_polyline(lv_obj_t *parent, MapPolyline &polyline) {
    lv_obj_t *object = lv_obj_create(parent);
    lv_obj_remove_style_all(object);
    lv_obj_remove_flag(object, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(object, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_user_data(object, &polyline);
    lv_obj_add_event_cb(object, draw_map_polyline, LV_EVENT_ALL, &polyline);
    return object;
}

void set_map_polyline_points(lv_obj_t *object, const lv_point_precise_t *points,
                            std::uint16_t count) {
    auto *polyline = static_cast<MapPolyline *>(lv_obj_get_user_data(object));
    polyline->points = points;
    polyline->count = count;
    lv_obj_invalidate(object);
}

bool apply_route_geometry_frame(bool rebind_lines = true) {
    if(ui.nav_route_point_count < 2) return false;
    bool pixels_changed = false;
    for(std::uint8_t i = 0; i < ui.nav_route_point_count; ++i) {
        const auto x = static_cast<lv_value_precise_t>(
            std::lround(ui.nav_route_x[i]));
        const auto y = static_cast<lv_value_precise_t>(
            std::lround(ui.nav_route_y[i]));
        pixels_changed = pixels_changed || ui.nav_route_points[i].x != x ||
                         ui.nav_route_points[i].y != y;
        ui.nav_route_points[i] = {x, y};
    }
    if(rebind_lines) {
        set_map_polyline_points(ui.nav_route_shadow, ui.nav_route_points,
                                   ui.nav_route_point_count);
        set_map_polyline_points(ui.nav_route, ui.nav_route_points,
                                   ui.nav_route_point_count);
    }
    return pixels_changed || rebind_lines;
}

bool apply_road_geometry_frame(bool rebind_lines = true) {
    if(ui.nav_road_point_count < 2 || ui.nav_road_polyline_count == 0) {
        return false;
    }
    bool pixels_changed = false;
    for(std::uint8_t i = 0; i < ui.nav_road_point_count; ++i) {
        const auto x = static_cast<lv_value_precise_t>(
            std::lround(ui.nav_road_x[i]));
        const auto y = static_cast<lv_value_precise_t>(
            std::lround(ui.nav_road_y[i]));
        pixels_changed = pixels_changed || ui.nav_road_points[i].x != x ||
                         ui.nav_road_points[i].y != y;
        ui.nav_road_points[i] = {x, y};
    }
    if(rebind_lines) {
        for(std::uint8_t i = 0; i < ui.nav_road_polyline_count; ++i) {
            const moto_ui_polyline_span_t span = ui.nav_road_spans[i];
            set_map_polyline_points(
                ui.nav_roads[i],
                &ui.nav_road_points[span.first_point_index],
                span.point_count);
        }
    }
    return pixels_changed || rebind_lines;
}

bool apply_building_geometry_frame(bool rebind_lines = true) {
    if(ui.nav_building_point_count < 3 ||
       ui.nav_building_footprint_count == 0) return false;
    bool pixels_changed = false;
    std::size_t packed_index = 0;
    for(std::uint8_t footprint_index = 0;
        footprint_index < ui.nav_building_footprint_count;
        ++footprint_index) {
        const moto_ui_building_span_t span =
            ui.nav_building_spans[footprint_index];
        for(std::uint8_t point_index = 0; point_index < span.point_count;
            ++point_index) {
            const std::size_t source = span.first_point_index + point_index;
            const auto x = static_cast<lv_value_precise_t>(
                std::lround(ui.nav_building_x[source]));
            const auto y = static_cast<lv_value_precise_t>(
                std::lround(ui.nav_building_y[source]));
            pixels_changed = pixels_changed ||
                ui.nav_building_points[packed_index].x != x ||
                ui.nav_building_points[packed_index].y != y;
            ui.nav_building_points[packed_index++] = {x, y};
        }
        // BLE omits the duplicate closing point; LVGL needs it explicitly.
        const auto closing =
            ui.nav_building_points[packed_index - span.point_count];
        pixels_changed = pixels_changed ||
            ui.nav_building_points[packed_index].x != closing.x ||
            ui.nav_building_points[packed_index].y != closing.y;
        ui.nav_building_points[packed_index] = closing;
        if(rebind_lines) {
            set_map_polyline_points(
                ui.nav_buildings[footprint_index],
                &ui.nav_building_points[packed_index - span.point_count],
                span.point_count + 1U);
        }
        ++packed_index;
    }
    return pixels_changed || rebind_lines;
}

void route_motion_tick(lv_timer_t *) {
    if(ui.nav_route_target_count < 2 || ui.nav_route_point_count < 2) return;
    // 0.39 at 25 ms has approximately the same smoothing time constant as
    // the old 0.56 at 40 ms, but supplies smaller and more frequent steps.
    // Screen coordinates need subpixel precision, not geographic doubles.
    // Float uses the ESP32-S3 FPU for every point in this 40 Hz hot loop.
    const float blend = ui.reduce_motion ? 1.0F : 0.39F;
    bool route_moved = false;
    for(std::uint8_t i = 0; i < ui.nav_route_point_count; ++i) {
        const float dx = ui.nav_route_target_x[i] - ui.nav_route_x[i];
        const float dy = ui.nav_route_target_y[i] - ui.nav_route_y[i];
        if(std::abs(dx) < 0.08F && std::abs(dy) < 0.08F) {
            ui.nav_route_x[i] = ui.nav_route_target_x[i];
            ui.nav_route_y[i] = ui.nav_route_target_y[i];
            continue;
        }
        ui.nav_route_x[i] += dx * blend;
        ui.nav_route_y[i] += dy * blend;
        route_moved = true;
    }
    bool roads_moved = false;
    for(std::uint8_t i = 0; i < ui.nav_road_point_count; ++i) {
        const float dx = ui.nav_road_target_x[i] - ui.nav_road_x[i];
        const float dy = ui.nav_road_target_y[i] - ui.nav_road_y[i];
        if(std::abs(dx) < 0.08F && std::abs(dy) < 0.08F) {
            ui.nav_road_x[i] = ui.nav_road_target_x[i];
            ui.nav_road_y[i] = ui.nav_road_target_y[i];
            continue;
        }
        ui.nav_road_x[i] += dx * blend;
        ui.nav_road_y[i] += dy * blend;
        roads_moved = true;
    }
    bool buildings_moved = false;
    for(std::uint8_t i = 0; i < ui.nav_building_point_count; ++i) {
        const float dx = ui.nav_building_target_x[i] - ui.nav_building_x[i];
        const float dy = ui.nav_building_target_y[i] - ui.nav_building_y[i];
        if(std::abs(dx) < 0.08F && std::abs(dy) < 0.08F) {
            ui.nav_building_x[i] = ui.nav_building_target_x[i];
            ui.nav_building_y[i] = ui.nav_building_target_y[i];
            continue;
        }
        ui.nav_building_x[i] += dx * blend;
        ui.nav_building_y[i] += dy * blend;
        buildings_moved = true;
    }
    // The line objects retain pointers to the mutable point arrays. Ordinary
    // animation therefore only changes those arrays and invalidates the one
    // common map layer. Rebinding every road/building used to invalidate the
    // same 480x232 region dozens of times per frame.
    bool pixels_changed = false;
    if(buildings_moved) {
        pixels_changed = apply_building_geometry_frame(false) || pixels_changed;
    }
    if(roads_moved) {
        pixels_changed = apply_road_geometry_frame(false) || pixels_changed;
    }
    if(route_moved) {
        pixels_changed = apply_route_geometry_frame(false) || pixels_changed;
    }
    if(pixels_changed) lv_obj_invalidate(ui.nav_map);
}

void update_road_geometry(const moto_ui_state_t *state) {
    const std::uint8_t point_count = std::min<std::uint8_t>(
        state->road_point_count, MOTO_UI_ROAD_POINT_CAPACITY);
    const std::uint8_t polyline_count = std::min<std::uint8_t>(
        state->road_polyline_count, MOTO_UI_ROAD_POLYLINE_CAPACITY);
    if(point_count < 2 || polyline_count == 0) {
        ui.nav_road_point_count = 0;
        ui.nav_road_polyline_count = 0;
        ui.nav_road_scene_revision = state->map_scene_revision;
        for(lv_obj_t *road : ui.nav_roads) {
            lv_obj_add_flag(road, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    bool spans_changed =
        ui.nav_road_scene_revision != state->map_scene_revision ||
        ui.nav_road_point_count != point_count ||
        ui.nav_road_polyline_count != polyline_count;
    for(std::uint8_t i = 0; i < point_count; ++i) {
        ui.nav_road_target_x[i] = state->road_points[i].x;
        ui.nav_road_target_y[i] = state->road_points[i].y;
    }
    for(std::uint8_t i = 0; i < polyline_count; ++i) {
        const moto_ui_polyline_span_t next = state->road_polylines[i];
        spans_changed = spans_changed ||
                        ui.nav_road_spans[i].first_point_index !=
                            next.first_point_index ||
                        ui.nav_road_spans[i].point_count != next.point_count ||
                        ui.nav_road_spans[i].road_class != next.road_class;
        ui.nav_road_spans[i] = next;
    }

    // A wide point can move more than 96 px during a perfectly ordinary fast
    // yaw. Treating screen-space distance as a route replacement made the
    // whole map jump. Only topology/revision changes snap; motion interpolates.
    const bool snap = spans_changed || ui.reduce_motion;
    ui.nav_road_point_count = point_count;
    ui.nav_road_polyline_count = polyline_count;
    ui.nav_road_scene_revision = state->map_scene_revision;
    if(snap) {
        for(std::uint8_t i = 0; i < point_count; ++i) {
            ui.nav_road_x[i] = ui.nav_road_target_x[i];
            ui.nav_road_y[i] = ui.nav_road_target_y[i];
        }
    }
    if(spans_changed) {
        for(std::uint8_t i = 0; i < MOTO_UI_ROAD_POLYLINE_CAPACITY; ++i) {
            if(i < polyline_count) {
                const std::uint8_t road_class = ui.nav_road_spans[i].road_class;
                const bool major = road_class <= 2U;
                const bool service = road_class >= 4U;
                lv_obj_set_style_line_width(
                    ui.nav_roads[i], px(major ? 4 : (service ? 2 : 3)), 0);
                lv_obj_set_style_line_color(
                    ui.nav_roads[i],
                    major ? kRoadMajor : (service ? kRoadMinor : kRoadGray), 0);
                lv_obj_remove_flag(ui.nav_roads[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(ui.nav_roads[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    if(snap) apply_road_geometry_frame();
}


void update_building_geometry(const moto_ui_state_t *state) {
    const std::uint8_t point_count = std::min<std::uint8_t>(
        state->building_point_count, MOTO_UI_BUILDING_POINT_CAPACITY);
    const std::uint8_t footprint_count = std::min<std::uint8_t>(
        state->building_footprint_count,
        MOTO_UI_BUILDING_FOOTPRINT_CAPACITY);
    if(point_count < 3 || footprint_count == 0) {
        ui.nav_building_point_count = 0;
        ui.nav_building_footprint_count = 0;
        ui.nav_building_scene_revision = state->map_scene_revision;
        for(lv_obj_t *building : ui.nav_buildings) {
            lv_obj_add_flag(building, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    bool spans_changed =
        ui.nav_building_scene_revision != state->map_scene_revision ||
        ui.nav_building_point_count != point_count ||
        ui.nav_building_footprint_count != footprint_count;
    for(std::uint8_t i = 0; i < point_count; ++i) {
        ui.nav_building_target_x[i] = state->building_points[i].x;
        ui.nav_building_target_y[i] = state->building_points[i].y;
    }
    for(std::uint8_t i = 0; i < footprint_count; ++i) {
        const moto_ui_building_span_t next = state->building_footprints[i];
        spans_changed = spans_changed ||
                        ui.nav_building_spans[i].first_point_index !=
                            next.first_point_index ||
                        ui.nav_building_spans[i].point_count !=
                            next.point_count ||
                        ui.nav_building_spans[i].building_class !=
                            next.building_class;
        ui.nav_building_spans[i] = next;
    }

    const bool snap = spans_changed || ui.reduce_motion;
    ui.nav_building_point_count = point_count;
    ui.nav_building_footprint_count = footprint_count;
    ui.nav_building_scene_revision = state->map_scene_revision;
    if(snap) {
        for(std::uint8_t i = 0; i < point_count; ++i) {
            ui.nav_building_x[i] = ui.nav_building_target_x[i];
            ui.nav_building_y[i] = ui.nav_building_target_y[i];
        }
    }
    if(spans_changed) {
        for(std::uint8_t i = 0;
            i < MOTO_UI_BUILDING_FOOTPRINT_CAPACITY; ++i) {
            if(i < footprint_count) {
                const bool landmark =
                    ui.nav_building_spans[i].building_class == 1U;
                lv_obj_set_style_line_color(
                    ui.nav_buildings[i],
                    landmark ? kBuildingLandmark : kBuildingGray, 0);
                lv_obj_set_style_line_width(
                    ui.nav_buildings[i], px(landmark ? 2 : 1), 0);
                lv_obj_remove_flag(ui.nav_buildings[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(ui.nav_buildings[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    if(snap) apply_building_geometry_frame();
}

void update_route_geometry(const moto_ui_state_t *state) {
    const std::uint8_t count = std::min<std::uint8_t>(
        state->route_point_count, MOTO_UI_ROUTE_POINT_CAPACITY);
    if(count < 2) {
        ui.nav_route_point_count = 0;
        ui.nav_route_target_count = 0;
        ui.nav_route_identity = state->route_identity;
        ui.nav_route_generation = state->route_generation;
        lv_obj_add_flag(ui.nav_route_shadow, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui.nav_route, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui.nav_marker, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    for(std::uint8_t i = 0; i < count; ++i) {
        ui.nav_route_target_x[i] = state->route_points[i].x;
        ui.nav_route_target_y[i] = state->route_points[i].y;
    }

    // A new route or a wholesale reroute should appear immediately. Ordinary
    // GNSS/IMU updates are blended at 40 Hz so the road glides under the fixed
    // rider marker instead of jumping from one phone fix to the next.
    const bool snap = ui.nav_route_identity != state->route_identity ||
                      ui.nav_route_generation != state->route_generation ||
                      ui.nav_route_point_count != count || ui.reduce_motion;
    ui.nav_route_target_count = count;
    ui.nav_route_point_count = count;
    ui.nav_route_identity = state->route_identity;
    ui.nav_route_generation = state->route_generation;
    if(snap) {
        for(std::uint8_t i = 0; i < count; ++i) {
            ui.nav_route_x[i] = ui.nav_route_target_x[i];
            ui.nav_route_y[i] = ui.nav_route_target_y[i];
        }
    }

    lv_obj_remove_flag(ui.nav_route_shadow, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(ui.nav_route, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(ui.nav_marker, LV_OBJ_FLAG_HIDDEN);
    if(snap) apply_route_geometry_frame();
}

lv_point_precise_t arrow_base(const lv_point_precise_t& previous,
                              const lv_point_precise_t& tip,
                              double head_length = px(24.0)) {
    const double dx = static_cast<double>(tip.x - previous.x);
    const double dy = static_cast<double>(tip.y - previous.y);
    const double length = std::max(1.0, std::hypot(dx, dy));
    const double ux = dx / length;
    const double uy = dy / length;
    return {
        static_cast<lv_value_precise_t>(tip.x - ux * head_length),
        static_cast<lv_value_precise_t>(tip.y - uy * head_length),
    };
}

void draw_arrowhead(lv_layer_t *layer, const lv_area_t& area,
                    const lv_point_precise_t& previous,
                    const lv_point_precise_t& tip) {
    const double dx = static_cast<double>(tip.x - previous.x);
    const double dy = static_cast<double>(tip.y - previous.y);
    const double length = std::max(1.0, std::hypot(dx, dy));
    const double ux = dx / length;
    const double uy = dy / length;
    const auto base = arrow_base(previous, tip);
    const double wing = px(18.0);

    lv_draw_triangle_dsc_t triangle;
    lv_draw_triangle_dsc_init(&triangle);
    triangle.color = kWhite;
    triangle.opa = LV_OPA_COVER;
    triangle.p[0] = {
        static_cast<lv_value_precise_t>(area.x1 + tip.x),
        static_cast<lv_value_precise_t>(area.y1 + tip.y),
    };
    triangle.p[1] = {
        static_cast<lv_value_precise_t>(area.x1 + base.x - uy * wing),
        static_cast<lv_value_precise_t>(area.y1 + base.y + ux * wing),
    };
    triangle.p[2] = {
        static_cast<lv_value_precise_t>(area.x1 + base.x + uy * wing),
        static_cast<lv_value_precise_t>(area.y1 + base.y - ux * wing),
    };
    lv_draw_triangle(layer, &triangle);
}

void draw_maneuver_icon(lv_event_t *event) {
    lv_obj_t *object = lv_event_get_target_obj(event);
    lv_area_t area;
    lv_obj_get_coords(object, &area);
    lv_layer_t *layer = lv_event_get_layer(event);

    lv_point_precise_t points[8]{};
    int count = 0;
    switch(ui.nav_maneuver_type) {
        case MOTO_MANEUVER_RIGHT:
            points[0] = {px(34), px(102)};
            points[1] = {px(34), px(63)};
            points[2] = {px(55), px(42)};
            points[3] = {px(105), px(42)};
            count = 4;
            break;
        case MOTO_MANEUVER_LEFT:
            points[0] = {px(82), px(102)};
            points[1] = {px(82), px(63)};
            points[2] = {px(61), px(42)};
            points[3] = {px(11), px(42)};
            count = 4;
            break;
        case MOTO_MANEUVER_SLIGHT_RIGHT:
            points[0] = {px(36), px(102)};
            points[1] = {px(36), px(72)};
            points[2] = {px(96), px(12)};
            count = 3;
            break;
        case MOTO_MANEUVER_SLIGHT_LEFT:
            points[0] = {px(80), px(102)};
            points[1] = {px(80), px(72)};
            points[2] = {px(20), px(12)};
            count = 3;
            break;
        case MOTO_MANEUVER_UTURN:
            points[0] = {px(88), px(102)};
            points[1] = {px(88), px(50)};
            points[2] = {px(82), px(33)};
            points[3] = {px(69), px(23)};
            points[4] = {px(51), px(20)};
            points[5] = {px(34), px(27)};
            points[6] = {px(25), px(43)};
            points[7] = {px(25), px(84)};
            count = 8;
            break;
        case MOTO_MANEUVER_ROUNDABOUT: {
            lv_draw_line_dsc_t stem;
            lv_draw_line_dsc_init(&stem);
            stem.color = kWhite;
            stem.width = px(11);
            stem.round_start = 1;
            stem.round_end = 1;
            stem.p1 = {static_cast<lv_value_precise_t>(area.x1 + px(58)),
                       static_cast<lv_value_precise_t>(area.y1 + px(101))};
            stem.p2 = {static_cast<lv_value_precise_t>(area.x1 + px(58)),
                       static_cast<lv_value_precise_t>(area.y1 + px(79))};
            lv_draw_line(layer, &stem);

            lv_draw_arc_dsc_t circle;
            lv_draw_arc_dsc_init(&circle);
            circle.color = kWhite;
            circle.width = px(11);
            circle.rounded = 1;
            circle.center = {
                static_cast<int32_t>(area.x1 + px(58)),
                static_cast<int32_t>(area.y1 + px(53)),
            };
            circle.radius = px(27);
            circle.start_angle = 86;
            circle.end_angle = 326;
            lv_draw_arc(layer, &circle);

            const lv_point_precise_t previous = {px(69), px(24)};
            const lv_point_precise_t tip = {px(86), px(36)};
            draw_arrowhead(layer, area, previous, tip);
            return;
        }
        case MOTO_MANEUVER_ARRIVE: {
            lv_draw_line_dsc_t pole;
            lv_draw_line_dsc_init(&pole);
            pole.color = kWhite;
            pole.width = px(11);
            pole.round_start = 1;
            pole.round_end = 1;
            pole.p1 = {static_cast<lv_value_precise_t>(area.x1 + px(42)),
                       static_cast<lv_value_precise_t>(area.y1 + px(101))};
            pole.p2 = {static_cast<lv_value_precise_t>(area.x1 + px(42)),
                       static_cast<lv_value_precise_t>(area.y1 + px(17))};
            lv_draw_line(layer, &pole);

            lv_draw_triangle_dsc_t flag;
            lv_draw_triangle_dsc_init(&flag);
            flag.color = kWhite;
            flag.opa = LV_OPA_COVER;
            flag.p[0] = {
                static_cast<lv_value_precise_t>(area.x1 + px(46)),
                static_cast<lv_value_precise_t>(area.y1 + px(20)),
            };
            flag.p[1] = {
                static_cast<lv_value_precise_t>(area.x1 + px(102)),
                static_cast<lv_value_precise_t>(area.y1 + px(39)),
            };
            flag.p[2] = {
                static_cast<lv_value_precise_t>(area.x1 + px(46)),
                static_cast<lv_value_precise_t>(area.y1 + px(58)),
            };
            lv_draw_triangle(layer, &flag);
            return;
        }
        case MOTO_MANEUVER_STRAIGHT:
        default:
            points[0] = {px(58), px(103)};
            points[1] = {px(58), px(10)};
            count = 2;
            break;
    }

    lv_point_precise_t absolute[8]{};
    const lv_point_precise_t base = arrow_base(points[count - 2],
                                                points[count - 1]);
    for(int i = 0; i < count; ++i) {
        const lv_point_precise_t point = i == count - 1 ? base : points[i];
        absolute[i] = {
            static_cast<lv_value_precise_t>(area.x1 + point.x),
            static_cast<lv_value_precise_t>(area.y1 + point.y),
        };
    }
    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.color = kWhite;
    line.width = px(12);
    line.round_start = 1;
    line.round_end = 1;
    line.points = absolute;
    line.point_cnt = count;
    lv_draw_line(layer, &line);
    draw_arrowhead(layer, area, points[count - 2], points[count - 1]);
}

void update_maneuver(moto_maneuver_t maneuver) {
    if(ui.nav_maneuver_type == maneuver) return;
    ui.nav_maneuver_type = maneuver;
    lv_obj_invalidate(ui.nav_maneuver);
}

void draw_segment(lv_layer_t *layer, lv_color_t color, int width,
                  lv_opa_t opacity, const lv_point_precise_t *points,
                  int count) {
    lv_draw_line_dsc_t line;
    lv_draw_line_dsc_init(&line);
    line.color = color;
    line.width = width;
    line.opa = opacity;
    line.round_start = 1;
    line.round_end = 1;
    // LVGL's descriptor predates const-correct polyline inputs; the draw task
    // only reads the points for the duration of this call.
    line.points = const_cast<lv_point_precise_t *>(points);
    line.point_cnt = count;
    lv_draw_line(layer, &line);
}

void draw_disc(lv_layer_t *layer, int x, int y, int diameter,
               lv_color_t color, lv_opa_t opacity) {
    lv_draw_rect_dsc_t disc;
    lv_draw_rect_dsc_init(&disc);
    disc.radius = LV_RADIUS_CIRCLE;
    disc.bg_color = color;
    disc.bg_opa = opacity;
    disc.border_opa = LV_OPA_TRANSP;
    const lv_area_t area = {
        x - diameter / 2,
        y - diameter / 2,
        x + diameter / 2,
        y + diameter / 2,
    };
    lv_draw_rect(layer, &disc, &area);
}

void draw_phone_body(lv_layer_t *layer, const lv_area_t& area,
                     lv_color_t color, lv_opa_t opacity) {
    const lv_area_t phone = {
        area.x1 + px(49),
        area.y1 + px(25),
        area.x1 + px(101),
        area.y1 + px(121),
    };
    lv_draw_rect_dsc_t frame;
    lv_draw_rect_dsc_init(&frame);
    frame.radius = px(13);
    frame.bg_opa = LV_OPA_TRANSP;
    frame.border_color = color;
    frame.border_width = px(4);
    frame.border_opa = opacity;
    lv_draw_rect(layer, &frame, &phone);

    const lv_point_precise_t receiver[] = {
        {static_cast<lv_value_precise_t>(area.x1 + px(67)),
         static_cast<lv_value_precise_t>(area.y1 + px(36))},
        {static_cast<lv_value_precise_t>(area.x1 + px(83)),
         static_cast<lv_value_precise_t>(area.y1 + px(36))},
    };
    const lv_point_precise_t home[] = {
        {static_cast<lv_value_precise_t>(area.x1 + px(68)),
         static_cast<lv_value_precise_t>(area.y1 + px(110))},
        {static_cast<lv_value_precise_t>(area.x1 + px(82)),
         static_cast<lv_value_precise_t>(area.y1 + px(110))},
    };
    draw_segment(layer, color, px(3), opacity, receiver, 2);
    draw_segment(layer, color, px(3), opacity, home, 2);
}

void draw_connection_symbol(lv_event_t *event) {
    lv_obj_t *object = lv_event_get_target_obj(event);
    lv_area_t area;
    lv_obj_get_coords(object, &area);
    lv_layer_t *layer = lv_event_get_layer(event);
    const int center_x = area.x1 + px(75);
    const int center_y = area.y1 + px(73);
    const double phase = ui.lifecycle_phase_deg * kPi / 180.0;

    if(ui.lifecycle_visual == LifecycleVisual::Connected) {
        const int pulse = static_cast<int>(px(67) +
            std::lround((std::sin(phase) + 1.0) * px(3.0)));
        lv_draw_arc_dsc_t halo;
        lv_draw_arc_dsc_init(&halo);
        halo.color = kGreen;
        halo.width = px(3);
        halo.opa = LV_OPA_50;
        halo.rounded = 1;
        halo.center = {center_x, center_y};
        halo.radius = pulse;
        halo.start_angle = 0;
        halo.end_angle = 360;
        lv_draw_arc(layer, &halo);
        draw_disc(layer, center_x, center_y, px(92), kGreen, LV_OPA_COVER);
        const lv_point_precise_t check[] = {
            {static_cast<lv_value_precise_t>(area.x1 + px(53)),
             static_cast<lv_value_precise_t>(area.y1 + px(74))},
            {static_cast<lv_value_precise_t>(area.x1 + px(69)),
             static_cast<lv_value_precise_t>(area.y1 + px(89))},
            {static_cast<lv_value_precise_t>(area.x1 + px(99)),
             static_cast<lv_value_precise_t>(area.y1 + px(55))},
        };
        draw_segment(layer, kBlack, px(9), LV_OPA_COVER, check, 3);
        return;
    }

    if(ui.lifecycle_visual == LifecycleVisual::Ready ||
       ui.lifecycle_visual == LifecycleVisual::Planning) {
        const lv_point_precise_t route[] = {
            {static_cast<lv_value_precise_t>(area.x1 + px(34)),
             static_cast<lv_value_precise_t>(area.y1 + px(113))},
            {static_cast<lv_value_precise_t>(area.x1 + px(50)),
             static_cast<lv_value_precise_t>(area.y1 + px(91))},
            {static_cast<lv_value_precise_t>(area.x1 + px(80)),
             static_cast<lv_value_precise_t>(area.y1 + px(98))},
            {static_cast<lv_value_precise_t>(area.x1 + px(111)),
             static_cast<lv_value_precise_t>(area.y1 + px(48))},
        };
        draw_segment(layer, kGraphite, px(12), LV_OPA_COVER, route, 4);
        draw_segment(layer, kWhite, px(5), LV_OPA_COVER, route, 4);
        draw_disc(layer, route[0].x, route[0].y, px(16), kWhite,
                  LV_OPA_COVER);

        lv_draw_arc_dsc_t pin;
        lv_draw_arc_dsc_init(&pin);
        pin.color = kWhite;
        pin.width = px(5);
        pin.opa = LV_OPA_COVER;
        pin.rounded = 1;
        pin.center = {route[3].x, route[3].y};
        pin.radius = px(13);
        pin.start_angle = 0;
        pin.end_angle = 360;
        lv_draw_arc(layer, &pin);
        draw_disc(layer, route[3].x, route[3].y, px(7), kWhite,
                  LV_OPA_COVER);

        if(ui.lifecycle_visual == LifecycleVisual::Planning) {
            // A single travelling bead communicates progress without a
            // processor-heavy full-screen animation.
            const double t =
                static_cast<double>(ui.lifecycle_phase_deg % 120U) / 120.0;
            const int segment = std::min(2, static_cast<int>(t * 3.0));
            const double local = t * 3.0 - segment;
            const int x = static_cast<int>(std::lround(
                route[segment].x +
                (route[segment + 1].x - route[segment].x) * local));
            const int y = static_cast<int>(std::lround(
                route[segment].y +
                (route[segment + 1].y - route[segment].y) * local));
            draw_disc(layer, x, y, px(12), kWhite, LV_OPA_COVER);
        } else {
            const lv_opa_t ready_opa = static_cast<lv_opa_t>(
                150 + std::lround((std::sin(phase) + 1.0) * 40.0));
            draw_disc(layer, route[0].x, route[0].y, px(25), kWhite,
                      ready_opa);
        }
        return;
    }

    draw_phone_body(layer, area, kWhite, LV_OPA_COVER);
    const bool connecting =
        ui.lifecycle_visual == LifecycleVisual::PhoneConnecting;
    const int speed = connecting ? 2 : 1;
    for(int index = 0; index < 3; ++index) {
        lv_draw_arc_dsc_t scan;
        lv_draw_arc_dsc_init(&scan);
        scan.color = index == 0 ? kWhite : kQuiet;
        scan.width = px(index == 0 ? 4 : 3);
        scan.opa = static_cast<lv_opa_t>(220 - index * 55);
        scan.rounded = 1;
        scan.center = {center_x, center_y};
        scan.radius = px(66 - index * 7);
        const int start =
            (static_cast<int>(ui.lifecycle_phase_deg) * speed + index * 112) %
            360;
        scan.start_angle = start;
        scan.end_angle = start + (connecting ? 54 : 34);
        lv_draw_arc(layer, &scan);
    }
    const int orbit_x = static_cast<int>(
        std::lround(center_x + std::sin(phase * speed) * px(66.0)));
    const int orbit_y = static_cast<int>(
        std::lround(center_y - std::cos(phase * speed) * px(66.0)));
    draw_disc(layer, orbit_x, orbit_y, px(connecting ? 11 : 9), kWhite,
              LV_OPA_COVER);
}

void set_layer_opacity(void *object, int32_t opacity) {
    lv_obj_set_style_opa(static_cast<lv_obj_t *>(object), opacity, 0);
}

void apply_lifecycle_content(LifecycleVisual visual) {
    ui.lifecycle_visual = visual;
    const char *kicker = "MOTO GPS / PHONE LINK";
    const char *title = "";
    const char *subtitle = "";
    lv_color_t title_color = kWhite;
    switch(visual) {
        case LifecycleVisual::PhoneOffline:
            title = "CONNECT PHONE";
            subtitle = "请打开手机应用";
            break;
        case LifecycleVisual::PhoneConnecting:
            title = "CONNECTING";
            subtitle = "正在建立连接";
            break;
        case LifecycleVisual::Connected:
            title = "CONNECTED";
            subtitle = "连接成功";
            title_color = kGreen;
            break;
        case LifecycleVisual::Ready:
            kicker = "MOTO GPS / READY";
            title = "READY TO RIDE";
            subtitle = "请在手机选择目的地";
            break;
        case LifecycleVisual::Planning:
            kicker = "MOTO GPS / ROUTE";
            title = "BUILDING ROUTE";
            subtitle = "正在规划路线";
            break;
        case LifecycleVisual::Hidden:
            break;
    }
    lv_label_set_text(ui.nav_lifecycle_kicker, kicker);
    lv_label_set_text(ui.nav_lifecycle_title, title);
    lv_label_set_text(ui.nav_lifecycle_subtitle, subtitle);
    lv_obj_set_style_text_color(ui.nav_lifecycle_title, title_color, 0);
    lv_obj_set_style_text_color(ui.nav_lifecycle_subtitle,
                                visual == LifecycleVisual::Connected
                                    ? kGreen : kSoft,
                                0);
    lv_obj_invalidate(ui.nav_lifecycle_symbol);
}

void lifecycle_fade_in();

void lifecycle_fade_out_complete(lv_anim_t *) {
    if(ui.nav_lifecycle == nullptr) return;
    if(ui.lifecycle_target == LifecycleVisual::Hidden) {
        ui.lifecycle_visual = LifecycleVisual::Hidden;
        lv_obj_add_flag(ui.nav_lifecycle, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_opa(ui.nav_lifecycle, LV_OPA_COVER, 0);
        return;
    }
    apply_lifecycle_content(ui.lifecycle_target);
    lifecycle_fade_in();
}

void lifecycle_fade_in() {
    lv_obj_remove_flag(ui.nav_lifecycle, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_opa(ui.nav_lifecycle, LV_OPA_TRANSP, 0);
    lv_anim_t fade;
    lv_anim_init(&fade);
    lv_anim_set_var(&fade, ui.nav_lifecycle);
    lv_anim_set_exec_cb(&fade, set_layer_opacity);
    lv_anim_set_values(&fade, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_duration(&fade, 220);
    lv_anim_set_path_cb(&fade, lv_anim_path_ease_out);
    lv_anim_start(&fade);
}

void transition_lifecycle(LifecycleVisual next) {
    if(ui.nav_lifecycle == nullptr) return;
    if(next == ui.lifecycle_target &&
       (next == ui.lifecycle_visual ||
        lv_obj_has_flag(ui.nav_lifecycle, LV_OBJ_FLAG_HIDDEN))) {
        return;
    }
    ui.lifecycle_target = next;
    lv_anim_delete(ui.nav_lifecycle, set_layer_opacity);
    if(ui.reduce_motion) {
        if(next == LifecycleVisual::Hidden) {
            ui.lifecycle_visual = LifecycleVisual::Hidden;
            lv_obj_add_flag(ui.nav_lifecycle, LV_OBJ_FLAG_HIDDEN);
        } else {
            apply_lifecycle_content(next);
            lv_obj_remove_flag(ui.nav_lifecycle, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_opa(ui.nav_lifecycle, LV_OPA_COVER, 0);
        }
        return;
    }
    if(ui.lifecycle_visual == LifecycleVisual::Hidden ||
       lv_obj_has_flag(ui.nav_lifecycle, LV_OBJ_FLAG_HIDDEN)) {
        if(next == LifecycleVisual::Hidden) return;
        apply_lifecycle_content(next);
        lifecycle_fade_in();
        return;
    }

    lv_anim_t fade;
    lv_anim_init(&fade);
    lv_anim_set_var(&fade, ui.nav_lifecycle);
    lv_anim_set_exec_cb(&fade, set_layer_opacity);
    lv_anim_set_values(&fade,
                       lv_obj_get_style_opa(ui.nav_lifecycle, LV_PART_MAIN),
                       LV_OPA_TRANSP);
    lv_anim_set_duration(&fade, 140);
    lv_anim_set_path_cb(&fade, lv_anim_path_ease_in);
    lv_anim_set_completed_cb(&fade, lifecycle_fade_out_complete);
    lv_anim_start(&fade);
}

LifecycleVisual desired_lifecycle_visual() {
    if(ui.demo_active) {
        return LifecycleVisual::Hidden;
    }
    if(ui.phone_connection == MOTO_UI_PHONE_OFFLINE) {
        return LifecycleVisual::PhoneOffline;
    }
    if(ui.phone_connection == MOTO_UI_PHONE_CONNECTING) {
        return LifecycleVisual::PhoneConnecting;
    }
    if(ui.lifecycle_success_active) {
        return LifecycleVisual::Connected;
    }
    if(ui.navigation_has_guidance) {
        return LifecycleVisual::Hidden;
    }
    if(ui.navigation_route_request_in_flight) {
        return LifecycleVisual::Planning;
    }
    return LifecycleVisual::Ready;
}

void refresh_lifecycle() {
    transition_lifecycle(desired_lifecycle_visual());
}

void lifecycle_tick(lv_timer_t *) {
    if(ui.reduce_motion || ui.nav_lifecycle_symbol == nullptr ||
       ui.lifecycle_visual == LifecycleVisual::Hidden) {
        return;
    }
    const std::uint16_t step =
        ui.lifecycle_visual == LifecycleVisual::PhoneConnecting ? 9U : 5U;
    ui.lifecycle_phase_deg =
        static_cast<std::uint16_t>((ui.lifecycle_phase_deg + step) % 360U);
    lv_obj_invalidate(ui.nav_lifecycle_symbol);
}

void connection_success_timeout(lv_timer_t *timer) {
    lv_timer_pause(timer);
    ui.lifecycle_success_active = false;
    refresh_lifecycle();
}

void set_navigation_guidance_visible(bool visible) {
    lv_obj_t *objects[] = {
        ui.nav_maneuver,
        ui.nav_distance,
        ui.nav_unit,
        ui.nav_progress,
    };
    for(lv_obj_t *object : objects) {
        if(visible) {
            lv_obj_remove_flag(object, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(object, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if(!visible) lv_obj_add_flag(ui.nav_limit, LV_OBJ_FLAG_HIDDEN);
}

void update_nav_status(const moto_ui_state_t *state) {
    const char *text = "";
    lv_color_t color = kQuiet;
    if(ui.demo_active) {
        lv_label_set_text(ui.nav_status, "DEMO RIDE");
        lv_obj_set_style_text_color(ui.nav_status, kWhite, 0);
        return;
    }
    if(state->mode != MOTO_UI_ARRIVED && state->route_point_count < 2) {
        // Empty navigation states are rendered by the full lifecycle surface,
        // never by a tiny diagnostic caption floating in a black screen.
        lv_label_set_text(ui.nav_status, "");
        return;
    }
    switch(state->mode) {
        case MOTO_UI_ACQUIRING_FIX: text = "ACQUIRING GPS"; color = kAmber; break;
        case MOTO_UI_REROUTING: text = "REROUTING"; color = kAmber; break;
        case MOTO_UI_OFFLINE: text = "ROUTE CACHED"; color = kQuiet; break;
        case MOTO_UI_ARRIVED: text = "ARRIVED"; color = kGreen; break;
        case MOTO_UI_NAVIGATING:
            if(state->online == 0) text = "NO HOTSPOT";
            break;
    }
    lv_label_set_text(ui.nav_status, text);
    lv_obj_set_style_text_color(ui.nav_status, color, 0);
}

void update_navigation(const moto_ui_state_t *state) {
    update_building_geometry(state);
    update_road_geometry(state);
    update_route_geometry(state);
    update_maneuver(state->maneuver);
    update_nav_status(state);

    // Never suggest "go straight for 0 m" while there is no route. Waiting is
    // a connection state, not actionable navigation guidance.
    const bool has_guidance = ui.demo_active ||
                              state->mode == MOTO_UI_ARRIVED ||
                              state->route_point_count >= 2;
    ui.navigation_has_guidance = has_guidance;
    ui.navigation_route_request_in_flight =
        state->route_request_in_flight != 0;
    set_navigation_guidance_visible(has_guidance);
    refresh_lifecycle();
    if(!has_guidance) return;

    char value[16];
    const char *unit = "m";
    if(state->distance_to_maneuver_m >= 1000) {
        std::snprintf(value, sizeof(value), "%.1f",
                      state->distance_to_maneuver_m / 1000.0);
        unit = "km";
    } else {
        std::snprintf(value, sizeof(value), "%u",
                      static_cast<unsigned>(state->distance_to_maneuver_m));
    }
    if(state->mode == MOTO_UI_ARRIVED) std::snprintf(value, sizeof(value), "0");
    lv_label_set_text(ui.nav_distance, value);
    lv_label_set_text(ui.nav_unit, unit);
    // This number is never the total trip distance. Its physical attachment to
    // the maneuver glyph makes that meaning clear without an explanatory label.
    lv_obj_align(ui.nav_distance, LV_ALIGN_TOP_LEFT, px(151), px(244));
    lv_obj_align_to(ui.nav_unit, ui.nav_distance, LV_ALIGN_OUT_RIGHT_BOTTOM,
                    px(6), px(-7));

    lv_arc_set_value(ui.nav_progress,
                     std::min<int>(100, state->route_progress_percent));
    lv_obj_set_style_arc_color(ui.nav_progress, traffic_color(state->traffic),
                               LV_PART_INDICATOR);

    if(state->speed_limit_kph > 0) {
        char limit[8];
        std::snprintf(limit, sizeof(limit), "%u",
                      static_cast<unsigned>(state->speed_limit_kph));
        lv_label_set_text(ui.nav_limit_value, limit);
        lv_obj_remove_flag(ui.nav_limit, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(ui.nav_limit, LV_OBJ_FLAG_HIDDEN);
    }
}

void update_speedometer(const moto_ui_state_t *state) {
    char value[8];
    std::snprintf(value, sizeof(value), "%u",
                  static_cast<unsigned>(state->speed_kph));
    lv_label_set_text(ui.speed_value, value);
    lv_obj_align(ui.speed_value, LV_ALIGN_CENTER, 0, px(-10));
    lv_arc_set_value(ui.speed_arc, std::min<int>(160, state->speed_kph));
    for(int i = 0; i < kSpeedTickCount; ++i) {
        const bool active = state->speed_kph >= static_cast<unsigned>(i * 10);
        lv_obj_set_style_line_color(ui.speed_ticks[i], active ? kWhite : kGraphite, 0);
    }
}

void update_compass(const moto_ui_state_t *state) {
    char heading[12];
    std::snprintf(heading, sizeof(heading), "%03u°",
                  static_cast<unsigned>(state->heading_deg));
    lv_label_set_text(ui.compass_heading, heading);
    lv_label_set_text(ui.compass_cardinal, cardinal_name(state->heading_deg));
    char speed[20];
    std::snprintf(speed, sizeof(speed), "%u km/h",
                  static_cast<unsigned>(state->speed_kph));
    lv_label_set_text(ui.compass_speed, speed);

    const double heading_rad = state->heading_deg * kPi / 180.0;
    for(int i = 0; i < kCompassTickCount; ++i) {
        const double angle = i * 15.0 * kPi / 180.0 - heading_rad;
        const double inner = px(i % 3 == 0 ? 138.0 : 145.0);
        ui.compass_tick_points[i][0] = {
            static_cast<lv_value_precise_t>(px(180.0) + std::sin(angle) * px(154.0)),
            static_cast<lv_value_precise_t>(px(180.0) - std::cos(angle) * px(154.0)),
        };
        ui.compass_tick_points[i][1] = {
            static_cast<lv_value_precise_t>(px(180.0) + std::sin(angle) * inner),
            static_cast<lv_value_precise_t>(px(180.0) - std::cos(angle) * inner),
        };
        lv_line_set_points_mutable(ui.compass_ticks[i],
                                   ui.compass_tick_points[i], 2);
        lv_obj_set_style_line_color(ui.compass_ticks[i], i == 0 ? kAmber : kQuiet, 0);
    }

    static const double bearings[4] = {0.0, 90.0, 180.0, 270.0};
    static const char *letters[4] = {"N", "E", "S", "W"};
    for(int i = 0; i < 4; ++i) {
        const double angle = bearings[i] * kPi / 180.0 - heading_rad;
        const int x = static_cast<int>(px(180.0) + std::sin(angle) * px(112.0));
        const int y = static_cast<int>(px(180.0) - std::cos(angle) * px(112.0));
        lv_label_set_text(ui.compass_letters[i], letters[i]);
        lv_obj_set_pos(ui.compass_letters[i], x - px(14), y - px(10));
        lv_obj_set_style_text_color(ui.compass_letters[i], i == 0 ? kAmber : kQuiet, 0);
    }
}

void update_music_view() {
    lv_label_set_text(ui.music_source, ui.music.connected ? ui.music_source_text : "PHONE NOT CONNECTED");
    lv_label_set_text(ui.music_title, ui.music_title_text);
    lv_label_set_text(ui.music_artist, ui.music_artist_text);
    lv_label_set_text(ui.music_button_labels[1], ui.music.playing ? "II" : ">");
    lv_label_set_text(ui.music_button_labels[3], ui.music.liked ? "SENT" : "LIKE");
    if(ui.music.like_available) {
        lv_obj_remove_flag(ui.music_buttons[3], LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(ui.music_buttons[3], LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_set_style_border_color(ui.music_disc,
                                  ui.music.playing ? kIce : kGraphite, 0);
}

void music_button_event(lv_event_t *event) {
    const auto command = static_cast<moto_music_command_t>(
        reinterpret_cast<std::intptr_t>(lv_event_get_user_data(event)));
    if(command == MOTO_MUSIC_TOGGLE_PLAYBACK) {
        ui.music.playing = ui.music.playing == 0;
    } else if(command == MOTO_MUSIC_LIKE && ui.music.like_available) {
        ui.music.liked = 1;
    }
    update_music_view();
    if(ui.music_callback != nullptr) ui.music_callback(command, ui.music_callback_context);
}

void create_page_dots() {
    for(int i = 0; i < MOTO_UI_PAGE_COUNT; ++i) {
        ui.page_dots[i] = lv_obj_create(ui.screen);
        lv_obj_remove_style_all(ui.page_dots[i]);
        lv_obj_set_size(ui.page_dots[i], px(5), px(5));
        lv_obj_set_y(ui.page_dots[i], px(337));
        lv_obj_set_style_bg_opa(ui.page_dots[i], LV_OPA_COVER, 0);
        lv_obj_set_style_radius(ui.page_dots[i], px(3), 0);
        lv_obj_clear_flag(ui.page_dots[i], LV_OBJ_FLAG_CLICKABLE);
    }
    ui.page_dots_timer = lv_timer_create(hide_page_dots,
                                         kPageDotsVisibleMs, nullptr);
    update_page_dots();
    reveal_page_dots();
}

void update_status_row() {
    const bool known = ui.battery_percent != MOTO_UI_BATTERY_UNKNOWN;
    if(ui.status_battery_text != nullptr) {
        char text[8];
        if(known) {
            lv_snprintf(text, sizeof(text), "%u",
                        static_cast<unsigned>(ui.battery_percent));
        } else {
            lv_snprintf(text, sizeof(text), "--");
        }
        lv_label_set_text(ui.status_battery_text, text);
        // The number sits inside the icon, so its colour has to follow what is
        // behind it: black once the white fill reaches the middle, white while
        // the pack is still mostly empty.
        const bool over_fill = known && ui.battery_percent >= 55;
        lv_obj_set_style_text_color(ui.status_battery_text,
                                    over_fill ? kBlack : kWhite, 0);
        // Re-centre inside the icon. The label is a child of the battery body,
        // so this centres on the pack itself and not on the whole page (which
        // is what an earlier revision did, pushing the digits off the icon).
        lv_obj_center(ui.status_battery_text);
    }
    if(ui.status_battery_body != nullptr) {
        if(known) {
            lv_obj_remove_flag(ui.status_battery_body, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(ui.status_battery_cap, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(ui.status_battery_fill, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(ui.status_battery_body, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui.status_battery_cap, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui.status_battery_fill, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if(ui.status_battery_fill != nullptr && known) {
        const int inner = px(34);
        int width = inner * static_cast<int>(ui.battery_percent) / 100;
        if(ui.battery_percent > 0 && width < px(3)) width = px(3);
        lv_obj_set_width(ui.status_battery_fill, width);
        lv_obj_set_style_bg_color(
            ui.status_battery_fill, ui.battery_percent <= 20 ? kAmber : kWhite,
            0);
    }
    if(ui.status_battery_cap != nullptr) {
        // A lit cap is the only charging hint that fits in this much space.
        lv_obj_set_style_bg_color(
            ui.status_battery_cap,
            ui.battery_charging != 0 ? kWhite : kSoft, 0);
    }
    for(int i = 0; i < 4; ++i) {
        if(ui.status_bars[i] == nullptr) continue;
        lv_obj_set_style_bg_color(ui.status_bars[i],
                                  i < ui.link_bars ? kWhite : kGraphite, 0);
    }
}

/// Top strip of the navigation page: signal at the far left, battery at the
/// far right, both inside the circular safe area.
///
/// The panel is round, so the usable width shrinks quickly towards y = 0: at
/// the height used here the corners are already cut away, which is why the two
/// clusters stop short of the real screen edges.
void create_status_row() {
    lv_obj_t *page = ui.pages[MOTO_UI_PAGE_NAVIGATION];

    static const int bar_heights[4] = {7, 11, 15, 19};
    for(int i = 0; i < 4; ++i) {
        lv_obj_t *bar = lv_obj_create(page);
        lv_obj_remove_style_all(bar);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(bar, px(5), px(bar_heights[i]));
        lv_obj_set_style_radius(bar, px(2), 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(bar, kGraphite, 0);
        // Bottom-aligned so the four bars grow upwards from one baseline.
        lv_obj_align(bar, LV_ALIGN_TOP_LEFT, px(24 + i * 8),
                     px(41 - bar_heights[i]));
        ui.status_bars[i] = bar;
    }

    // Compact pack: the previous 46x22 icon crowded the top edge of a round
    // panel, where the usable width shrinks fastest.
    ui.status_battery_body = lv_obj_create(page);
    lv_obj_remove_style_all(ui.status_battery_body);
    lv_obj_clear_flag(ui.status_battery_body, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(ui.status_battery_body, px(38), px(17));
    lv_obj_set_style_radius(ui.status_battery_body, px(4), 0);
    lv_obj_set_style_border_width(ui.status_battery_body, px(2), 0);
    lv_obj_set_style_border_color(ui.status_battery_body, kSoft, 0);
    lv_obj_set_style_bg_opa(ui.status_battery_body, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(ui.status_battery_body, 0, 0);
    lv_obj_align(ui.status_battery_body, LV_ALIGN_TOP_RIGHT, -px(24), px(20));

    ui.status_battery_fill = lv_obj_create(ui.status_battery_body);
    lv_obj_remove_style_all(ui.status_battery_fill);
    lv_obj_clear_flag(ui.status_battery_fill, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(ui.status_battery_fill, px(34), px(13));
    lv_obj_set_pos(ui.status_battery_fill, 0, 0);
    lv_obj_set_style_radius(ui.status_battery_fill, px(2), 0);
    lv_obj_set_style_bg_opa(ui.status_battery_fill, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(ui.status_battery_fill, kWhite, 0);

    ui.status_battery_cap = lv_obj_create(page);
    lv_obj_remove_style_all(ui.status_battery_cap);
    lv_obj_clear_flag(ui.status_battery_cap, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(ui.status_battery_cap, px(3), px(7));
    lv_obj_set_style_radius(ui.status_battery_cap, px(2), 0);
    lv_obj_set_style_bg_opa(ui.status_battery_cap, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(ui.status_battery_cap, kSoft, 0);
    lv_obj_align(ui.status_battery_cap, LV_ALIGN_TOP_RIGHT, -px(20), px(25));

    // Child of the pack, so the digits are centred on the icon and move with it.
    ui.status_battery_text =
        make_label(ui.status_battery_body, &lv_font_montserrat_14, kWhite, "--");
    lv_obj_set_style_text_align(ui.status_battery_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(ui.status_battery_text);

    update_status_row();
}

void create_navigation_page() {
    lv_obj_t *page = ui.pages[MOTO_UI_PAGE_NAVIGATION];
    ui.nav_map = make_layer(page);
    // The map is the topmost hit target across most of the navigation page.
    // Forward swipe gestures to the page/screen. Local demo activation is not
    // bound to a long press: a glove, mount or palm must never replace active
    // phone guidance with the built-in fixture while riding.
    lv_obj_add_flag(ui.nav_map, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(ui.nav_map, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_set_height(ui.nav_map, px(232));

    // Real building footprints are quiet closed outlines beneath both roads
    // and the selected route. No inferred rectangles or decorative fill is
    // generated on-device.
    for(std::uint8_t i = 0;
        i < MOTO_UI_BUILDING_FOOTPRINT_CAPACITY; ++i) {
        ui.nav_buildings[i] = create_map_polyline(ui.nav_map,
                                                  ui.nav_building_lines[i]);
        lv_obj_set_size(ui.nav_buildings[i], MOTO_UI_CANVAS_WIDTH, px(232));
        lv_obj_set_style_line_width(ui.nav_buildings[i], px(1), 0);
        lv_obj_set_style_line_color(ui.nav_buildings[i], kBuildingGray, 0);
        lv_obj_set_style_line_opa(ui.nav_buildings[i], LV_OPA_COVER, 0);
        lv_obj_set_style_line_rounded(ui.nav_buildings[i], false, 0);
        lv_obj_add_flag(ui.nav_buildings[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Quiet, real street context sits behind the selected route. Each road is
    // its own LVGL line so disconnected streets are never joined by a fake
    // diagonal. The bundled Jinan fixture uses all eight bounded slots; a
    // future online provider must simplify its response to the same limit.
    for(std::uint8_t i = 0; i < MOTO_UI_ROAD_POLYLINE_CAPACITY; ++i) {
        ui.nav_roads[i] = create_map_polyline(ui.nav_map, ui.nav_road_lines[i]);
        lv_obj_set_size(ui.nav_roads[i], MOTO_UI_CANVAS_WIDTH, px(232));
        lv_obj_set_style_line_width(ui.nav_roads[i], px(3), 0);
        lv_obj_set_style_line_color(ui.nav_roads[i], kRoadGray, 0);
        lv_obj_set_style_line_opa(ui.nav_roads[i], LV_OPA_COVER, 0);
        lv_obj_set_style_line_rounded(ui.nav_roads[i], true, 0);
        lv_obj_add_flag(ui.nav_roads[i], LV_OBJ_FLAG_HIDDEN);
    }

    ui.nav_route_shadow = create_map_polyline(ui.nav_map, ui.nav_route_shadow_line);
    lv_obj_set_size(ui.nav_route_shadow, MOTO_UI_CANVAS_WIDTH, px(232));
    lv_obj_set_style_line_width(ui.nav_route_shadow, px(13), 0);
    lv_obj_set_style_line_color(ui.nav_route_shadow, kGraphite, 0);
    lv_obj_set_style_line_rounded(ui.nav_route_shadow, true, 0);
    ui.nav_route = create_map_polyline(ui.nav_map, ui.nav_route_line);
    lv_obj_set_size(ui.nav_route, MOTO_UI_CANVAS_WIDTH, px(232));
    lv_obj_set_style_line_width(ui.nav_route, px(6), 0);
    lv_obj_set_style_line_color(ui.nav_route, kWhite, 0);
    lv_obj_set_style_line_rounded(ui.nav_route, true, 0);
    ui.nav_route_motion_timer = lv_timer_create(route_motion_tick,
                                                 kRouteMotionFrameMs,
                                                 nullptr);

    ui.nav_marker = lv_obj_create(ui.nav_map);
    lv_obj_remove_style_all(ui.nav_marker);
    lv_obj_set_size(ui.nav_marker, px(35), px(37));
    lv_obj_set_pos(ui.nav_marker, px(163), px(177));
    lv_obj_clear_flag(ui.nav_marker, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui.nav_marker, draw_vehicle_marker,
                        LV_EVENT_DRAW_MAIN_END, nullptr);

    ui.nav_status = make_label(page, &lv_font_montserrat_16, kAmber, "");
    lv_obj_set_width(ui.nav_status, px(220));
    lv_obj_set_style_text_align(ui.nav_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(ui.nav_status, LV_ALIGN_TOP_MID, 0, px(48));

    // The next action is the navigation page's single visual hero. The short
    // road window above is deliberately quieter and contains no fake side
    // streets; this standard symbol must be understood in one glance.
    ui.nav_maneuver = lv_obj_create(page);
    lv_obj_remove_style_all(ui.nav_maneuver);
    lv_obj_set_size(ui.nav_maneuver, px(116), px(108));
    lv_obj_set_pos(ui.nav_maneuver, px(29), px(220));
    lv_obj_clear_flag(ui.nav_maneuver, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui.nav_maneuver, draw_maneuver_icon,
                        LV_EVENT_DRAW_MAIN_END, nullptr);

    ui.nav_distance = make_label(page, &lv_font_montserrat_48, kWhite, "300");
    ui.nav_unit = make_label(page, &lv_font_montserrat_20, kQuiet, "m");

    ui.nav_limit = lv_obj_create(page);
    lv_obj_set_size(ui.nav_limit, px(61), px(61));
    // Keep the optional demo/SDK-provided value inside the circular safe area.
    // Real AMap Web-Service snapshots use zero and remain hidden because that
    // API does not return a road speed limit.
    lv_obj_set_pos(ui.nav_limit, px(248), px(238));
    lv_obj_set_style_radius(ui.nav_limit, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(ui.nav_limit, kWhite, 0);
    lv_obj_set_style_bg_opa(ui.nav_limit, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(ui.nav_limit, kRed, 0);
    lv_obj_set_style_border_width(ui.nav_limit, px(6), 0);
    lv_obj_set_style_pad_all(ui.nav_limit, 0, 0);
    lv_obj_clear_flag(ui.nav_limit, LV_OBJ_FLAG_CLICKABLE);
    ui.nav_limit_value = make_label(ui.nav_limit, &lv_font_montserrat_28, kBlack, "70");
    lv_obj_center(ui.nav_limit_value);

    ui.nav_progress = lv_arc_create(page);
    lv_obj_set_size(ui.nav_progress, px(316), px(316));
    lv_obj_center(ui.nav_progress);
    lv_arc_set_rotation(ui.nav_progress, 48);
    lv_arc_set_bg_angles(ui.nav_progress, 0, 84);
    lv_arc_set_range(ui.nav_progress, 0, 100);
    lv_arc_set_value(ui.nav_progress, 24);
    lv_obj_remove_style(ui.nav_progress, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(ui.nav_progress, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(ui.nav_progress, px(3), LV_PART_MAIN);
    lv_obj_set_style_arc_color(ui.nav_progress, kGraphite, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ui.nav_progress, px(4), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(ui.nav_progress, kIce, LV_PART_INDICATOR);
    lv_obj_move_to_index(ui.nav_progress, 0);

    // Connection lifecycle overlay. The composition follows the same circular
    // safe area as the navigation instrument and intentionally uses one hero
    // symbol instead of a dashboard of technical status labels.
    ui.nav_lifecycle = make_layer(page);
    lv_obj_set_style_bg_color(ui.nav_lifecycle, kBlack, 0);
    lv_obj_set_style_bg_opa(ui.nav_lifecycle, LV_OPA_COVER, 0);
    lv_obj_add_flag(ui.nav_lifecycle, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(ui.nav_lifecycle, LV_OBJ_FLAG_GESTURE_BUBBLE);

    ui.nav_lifecycle_kicker = make_label(
        ui.nav_lifecycle, &lv_font_montserrat_16, kQuiet,
        "MOTO GPS / PHONE LINK");
    lv_obj_set_style_text_letter_space(ui.nav_lifecycle_kicker, px(2), 0);
    lv_obj_align(ui.nav_lifecycle_kicker, LV_ALIGN_TOP_MID, 0, px(39));

    ui.nav_lifecycle_symbol = lv_obj_create(ui.nav_lifecycle);
    lv_obj_remove_style_all(ui.nav_lifecycle_symbol);
    lv_obj_set_size(ui.nav_lifecycle_symbol, px(150), px(150));
    lv_obj_align(ui.nav_lifecycle_symbol, LV_ALIGN_TOP_MID, 0, px(61));
    lv_obj_clear_flag(ui.nav_lifecycle_symbol, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui.nav_lifecycle_symbol, draw_connection_symbol,
                        LV_EVENT_DRAW_MAIN_END, nullptr);

    ui.nav_lifecycle_title = make_label(
        ui.nav_lifecycle, &lv_font_montserrat_28, kWhite, "CONNECT PHONE");
    lv_obj_set_width(ui.nav_lifecycle_title, px(310));
    lv_obj_set_style_text_align(ui.nav_lifecycle_title,
                                LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_letter_space(ui.nav_lifecycle_title, px(2), 0);
    lv_obj_align(ui.nav_lifecycle_title, LV_ALIGN_TOP_MID, 0, px(231));

    ui.nav_lifecycle_subtitle = make_label(
        ui.nav_lifecycle, &moto_font_nav_16, kSoft, "请打开手机应用");
    lv_obj_set_width(ui.nav_lifecycle_subtitle, px(280));
    lv_obj_set_style_text_align(ui.nav_lifecycle_subtitle,
                                LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(ui.nav_lifecycle_subtitle, LV_ALIGN_TOP_MID, 0, px(278));

    ui.nav_lifecycle_timer = lv_timer_create(lifecycle_tick, 40, nullptr);
    ui.nav_success_timer = lv_timer_create(connection_success_timeout,
                                            kConnectionSuccessHoldMs,
                                            nullptr);
    lv_timer_pause(ui.nav_success_timer);
    apply_lifecycle_content(LifecycleVisual::PhoneOffline);
    ui.lifecycle_target = LifecycleVisual::PhoneOffline;
    create_status_row();
}

void create_speed_page() {
    lv_obj_t *page = ui.pages[MOTO_UI_PAGE_SPEED];
    lv_obj_t *title = make_label(page, &lv_font_montserrat_16, kQuiet, "SPEED");
    lv_obj_set_style_text_letter_space(title, px(4), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, px(48));

    ui.speed_arc = lv_arc_create(page);
    lv_obj_set_size(ui.speed_arc, px(312), px(312));
    lv_obj_center(ui.speed_arc);
    lv_arc_set_rotation(ui.speed_arc, 138);
    lv_arc_set_bg_angles(ui.speed_arc, 0, 264);
    lv_arc_set_range(ui.speed_arc, 0, 160);
    lv_obj_remove_style(ui.speed_arc, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(ui.speed_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(ui.speed_arc, px(5), LV_PART_MAIN);
    lv_obj_set_style_arc_color(ui.speed_arc, kGraphite, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ui.speed_arc, px(7), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(ui.speed_arc, kIce, LV_PART_INDICATOR);

    for(int i = 0; i < kSpeedTickCount; ++i) {
        const double angle = (138.0 + i * (264.0 / (kSpeedTickCount - 1))) * kPi / 180.0;
        const double inner = px(i % 2 == 0 ? 132.0 : 136.0);
        ui.speed_tick_points[i][0] = {
            static_cast<lv_value_precise_t>(px(180.0) + std::cos(angle) * px(142.0)),
            static_cast<lv_value_precise_t>(px(180.0) + std::sin(angle) * px(142.0)),
        };
        ui.speed_tick_points[i][1] = {
            static_cast<lv_value_precise_t>(px(180.0) + std::cos(angle) * inner),
            static_cast<lv_value_precise_t>(px(180.0) + std::sin(angle) * inner),
        };
        ui.speed_ticks[i] = lv_line_create(page);
        lv_obj_set_size(ui.speed_ticks[i], MOTO_UI_CANVAS_WIDTH,
                        MOTO_UI_CANVAS_HEIGHT);
        lv_line_set_points_mutable(ui.speed_ticks[i], ui.speed_tick_points[i], 2);
        lv_obj_set_style_line_width(ui.speed_ticks[i], px(i % 2 == 0 ? 3 : 2), 0);
        lv_obj_set_style_line_color(ui.speed_ticks[i], kGraphite, 0);
    }
    ui.speed_value = make_label(page, &lv_font_montserrat_48, kWhite, "72");
    lv_obj_align(ui.speed_value, LV_ALIGN_CENTER, 0, px(-10));
    lv_obj_t *unit = make_label(page, &lv_font_montserrat_20, kQuiet, "km/h");
    lv_obj_set_style_text_letter_space(unit, px(2), 0);
    lv_obj_align(unit, LV_ALIGN_CENTER, 0, px(45));
    lv_obj_t *caption = make_label(page, &lv_font_montserrat_16, kQuiet, "LIVE SPEED");
    lv_obj_set_style_text_letter_space(caption, px(2), 0);
    lv_obj_align(caption, LV_ALIGN_BOTTOM_MID, 0, px(-49));
}

void create_compass_page() {
    lv_obj_t *page = ui.pages[MOTO_UI_PAGE_COMPASS];
    for(int i = 0; i < kCompassTickCount; ++i) {
        ui.compass_ticks[i] = lv_line_create(page);
        lv_obj_set_size(ui.compass_ticks[i], MOTO_UI_CANVAS_WIDTH,
                        MOTO_UI_CANVAS_HEIGHT);
        lv_obj_set_style_line_width(ui.compass_ticks[i], px(i % 3 == 0 ? 3 : 2), 0);
        lv_obj_set_style_line_color(ui.compass_ticks[i], kQuiet, 0);
    }
    for(int i = 0; i < 4; ++i) {
        ui.compass_letters[i] = make_label(page, &lv_font_montserrat_16, kQuiet, "N");
        lv_obj_set_size(ui.compass_letters[i], px(28), px(22));
        lv_obj_set_style_text_align(ui.compass_letters[i], LV_TEXT_ALIGN_CENTER, 0);
    }
    lv_obj_t *index = lv_obj_create(page);
    lv_obj_remove_style_all(index);
    lv_obj_set_size(index, px(9), px(9));
    lv_obj_set_pos(index, px(176), px(18));
    lv_obj_set_style_bg_color(index, kRed, 0);
    lv_obj_set_style_bg_opa(index, LV_OPA_COVER, 0);
    lv_obj_set_style_transform_rotation(index, 450, 0);

    ui.compass_heading = make_label(page, &lv_font_montserrat_48, kAmber, "359°");
    lv_obj_align(ui.compass_heading, LV_ALIGN_CENTER, 0, px(-37));
    ui.compass_cardinal = make_label(page, &lv_font_montserrat_20, kAmber, "N");
    lv_obj_align(ui.compass_cardinal, LV_ALIGN_CENTER, 0, px(10));
    ui.compass_speed = make_label(page, &lv_font_montserrat_20, kWhite, "72 km/h");
    lv_obj_set_style_text_letter_space(ui.compass_speed, px(1), 0);
    lv_obj_align(ui.compass_speed, LV_ALIGN_CENTER, 0, px(57));
}

/// Points one hand at `angle_deg` clockwise from twelve o'clock.
///
/// The hands are drawn from the dial centre outwards. `length` and the widths
/// are design-space values, converted through px() together with the centre so
/// the hands keep their proportions on any panel size.
/// Angle the hand should show `elapsed_ms` after `from_angle` became the target.
///
/// Driven by elapsed time rather than by a per-frame factor: the render rate is
/// not fixed (the bridge renders on demand), so a frame-counted ease would land
/// on a different angle each run and break the render tests' determinism. With
/// time as the input, the same timestamp always yields the same angle.
///
/// The move is a smoothstep over kSweepMs; at the end the hand sits exactly on
/// the target, and the shortest-arc wrap keeps 355 -> 5 deg from spinning the
/// long way round.
constexpr double kSweepMs = 420.0;

double sweep_angle(double from_angle, double target, double elapsed_ms) {
    if(from_angle < 0.0) return target;  // first placement: no sweep-in
    const double delta = std::fmod(target - from_angle + 540.0, 360.0) - 180.0;
    double t = elapsed_ms / kSweepMs;
    if(t >= 1.0) return target;
    if(t <= 0.0) return from_angle;
    // Smoothstep: zero velocity at both ends, so the start and stop read as
    // deliberate rather than as a snap.
    const double eased = t * t * (3.0 - 2.0 * t);
    return from_angle + delta * eased;
}

/// Points one hand at `angle_deg` clockwise from twelve o'clock.
///
/// The hands are drawn from the dial centre outwards. `length` and the widths
/// are design-space values, converted through px() together with the centre so
/// the hands keep their proportions on any panel size.
void aim_clock_hand(lv_obj_t *hand, lv_point_precise_t points[2],
                    double angle_deg, double length) {
    if(hand == nullptr) return;
    const double centre = px(180.0);
    const double radians = angle_deg * kPi / 180.0;
    points[0] = {
        static_cast<lv_value_precise_t>(centre),
        static_cast<lv_value_precise_t>(centre),
    };
    points[1] = {
        static_cast<lv_value_precise_t>(centre + std::sin(radians) * px(length)),
        static_cast<lv_value_precise_t>(centre - std::cos(radians) * px(length)),
    };
    // The points array is mutable and already registered, so only invalidate.
    lv_obj_invalidate(hand);
}

/// Moves the three hands to the current time. Cheap: no layout, no text.
void update_clock_hands() {
    if(ui.clock.minutes == MOTO_UI_CLOCK_UNKNOWN ||
       ui.clock.minutes >= 24U * 60U) {
        // No trustworthy time: park every hand at twelve rather than showing a
        // confident-looking wrong hour.
        aim_clock_hand(ui.clock_hand_hour, ui.clock_hand_hour_points,
                   ui.clock_hand_hour_shown, 78.0);
    aim_clock_hand(ui.clock_hand_minute, ui.clock_hand_minute_points,
                   ui.clock_hand_minute_shown, 112.0);
    aim_clock_hand(ui.clock_hand_second, ui.clock_hand_second_points,
                   ui.clock_hand_second_shown, 126.0);
        return;
    }
    const unsigned hour24 = ui.clock.minutes / 60U;
    const unsigned minute = ui.clock.minutes % 60U;
    const unsigned second = ui.clock.second % 60U;
    // The hour hand creeps forward with the minutes, the way a real watch does,
    // instead of jumping a whole hour at the top of the hour.
    const double hour_angle =
        (hour24 % 12U) * 30.0 + minute * 0.5 + second * (0.5 / 60.0);
    const double minute_angle = minute * 6.0 + second * 0.1;
    const double second_angle = second * 6.0;

    // Start a new sweep only when the target angle actually moved; a redraw
    // for any other reason (a page change, a repaint) must not restart it.
    const bool retarget = ui.clock_hand_second_shown < 0.0 ||
                          std::abs(ui.clock_hand_second_shown - second_angle) > 0.01;
    if(retarget) {
        ui.clock_hand_hour_from = ui.clock_hand_hour_shown;
        ui.clock_hand_minute_from = ui.clock_hand_minute_shown;
        ui.clock_hand_second_from = ui.clock_hand_second_shown;
        ui.clock_sweep_start_ms = lv_tick_get();
    }
    const double elapsed = static_cast<double>(lv_tick_get() - ui.clock_sweep_start_ms);
    ui.clock_hand_hour_shown =
        sweep_angle(ui.clock_hand_hour_from, hour_angle, elapsed);
    ui.clock_hand_minute_shown =
        sweep_angle(ui.clock_hand_minute_from, minute_angle, elapsed);
    ui.clock_hand_second_shown =
        sweep_angle(ui.clock_hand_second_from, second_angle, elapsed);

    aim_clock_hand(ui.clock_hand_hour, ui.clock_hand_hour_points,
                   ui.clock_hand_hour_shown, 78.0);
    aim_clock_hand(ui.clock_hand_minute, ui.clock_hand_minute_points,
                   ui.clock_hand_minute_shown, 112.0);
    aim_clock_hand(ui.clock_hand_second, ui.clock_hand_second_points,
                   ui.clock_hand_second_shown, 126.0);
}

void update_clock_view() {
    if(ui.clock_time == nullptr) return;
    if(ui.clock.minutes == MOTO_UI_CLOCK_UNKNOWN ||
       ui.clock.minutes >= 24U * 60U) {
        lv_label_set_text(ui.clock_time, "--:--");
    } else {
        char value[8];
        std::snprintf(value, sizeof(value), "%02u:%02u",
                      static_cast<unsigned>(ui.clock.minutes / 60U),
                      static_cast<unsigned>(ui.clock.minutes % 60U));
        lv_label_set_text(ui.clock_time, value);
    }

    static const char *weekdays[] = {
        "", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};
    static const char *months[] = {
        "", "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
        "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
    char date[24] = "";
    const bool has_weekday = ui.clock.weekday >= 1 && ui.clock.weekday <= 7;
    if(ui.clock.month >= 1 && ui.clock.month <= 12 &&
       ui.clock.day >= 1 && ui.clock.day <= 31) {
        if(has_weekday) {
            std::snprintf(date, sizeof(date), "%s %02u %s",
                          weekdays[ui.clock.weekday],
                          static_cast<unsigned>(ui.clock.day),
                          months[ui.clock.month]);
        } else {
            std::snprintf(date, sizeof(date), "%02u %s",
                          static_cast<unsigned>(ui.clock.day),
                          months[ui.clock.month]);
        }
    } else if(has_weekday) {
        std::snprintf(date, sizeof(date), "%s", weekdays[ui.clock.weekday]);
    }
    lv_label_set_text(ui.clock_date, date);
    update_clock_hands();
}

void create_clock_page() {
    lv_obj_t *page = ui.pages[MOTO_UI_PAGE_CLOCK];
    lv_obj_t *title = make_label(page, &lv_font_montserrat_16, kQuiet, "CLOCK");
    lv_obj_set_style_text_letter_space(title, px(4), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, px(48));

    // A 60-tick watch dial keeps this page in the same visual family as the
    // speed arc and the compass rose. The radii match the compass ring so the
    // two instrument pages are interchangeable. Minute ticks are short; every
    // fifth tick is longer, brighter and slightly thicker.
    for(int i = 0; i < kClockTickCount; ++i) {
        const bool major = (i % 5) == 0;
        const double angle = i * 6.0 * kPi / 180.0;
        const double outer = px(154.0);
        const double inner = px(major ? 138.0 : 145.0);
        ui.clock_tick_points[i][0] = {
            static_cast<lv_value_precise_t>(px(180.0) + std::sin(angle) * outer),
            static_cast<lv_value_precise_t>(px(180.0) - std::cos(angle) * outer),
        };
        ui.clock_tick_points[i][1] = {
            static_cast<lv_value_precise_t>(px(180.0) + std::sin(angle) * inner),
            static_cast<lv_value_precise_t>(px(180.0) - std::cos(angle) * inner),
        };
        ui.clock_ticks[i] = lv_line_create(page);
        lv_obj_set_size(ui.clock_ticks[i], MOTO_UI_CANVAS_WIDTH,
                        MOTO_UI_CANVAS_HEIGHT);
        lv_line_set_points_mutable(ui.clock_ticks[i],
                                   ui.clock_tick_points[i], 2);
        lv_obj_set_style_line_width(ui.clock_ticks[i], px(major ? 3 : 2), 0);
        lv_obj_set_style_line_color(ui.clock_ticks[i],
                                    major ? kSoft : kGraphite, 0);
    }

    // Hands first, text after: LVGL draws children in creation order, so this
    // puts the readout above the hands without relying on move_foreground.
    // Lines rather than images, so the geometry scales with the design width
    // and no asset has to be versioned. Lengths follow a watch: hour shortest,
    // minute longer, second longest and thinnest.
    struct HandSpec {
        lv_obj_t **hand;
        lv_point_precise_t *points;
        double length;
        int width;
        lv_color_t color;
    };
    const HandSpec hands[3] = {
        {&ui.clock_hand_hour, ui.clock_hand_hour_points, 78.0, 7, kWhite},
        {&ui.clock_hand_minute, ui.clock_hand_minute_points, 112.0, 5, kWhite},
        {&ui.clock_hand_second, ui.clock_hand_second_points, 126.0, 2, kIce},
    };
    for(const HandSpec &spec : hands) {
        *spec.hand = lv_line_create(page);
        lv_obj_set_size(*spec.hand, MOTO_UI_CANVAS_WIDTH,
                        MOTO_UI_CANVAS_HEIGHT);
        lv_obj_set_style_line_rounded(*spec.hand, true, 0);
        lv_obj_set_style_line_width(*spec.hand, px(spec.width), 0);
        lv_obj_set_style_line_color(*spec.hand, spec.color, 0);
        lv_line_set_points_mutable(*spec.hand, spec.points, 2);
        lv_obj_clear_flag(*spec.hand, LV_OBJ_FLAG_CLICKABLE);
    }

    ui.clock_time = make_label(page, &moto_font_clock_96, kWhite, "--:--");
    lv_obj_align(ui.clock_time, LV_ALIGN_CENTER, 0, px(-12));
    ui.clock_date = make_label(page, &lv_font_montserrat_20, kQuiet, "");
    lv_obj_set_style_text_letter_space(ui.clock_date, px(2), 0);
    lv_obj_align(ui.clock_date, LV_ALIGN_CENTER, 0, px(45));
    lv_obj_t *caption = make_label(page, &lv_font_montserrat_16, kQuiet,
                                   "LOCAL TIME");
    lv_obj_set_style_text_letter_space(caption, px(2), 0);
    lv_obj_align(caption, LV_ALIGN_BOTTOM_MID, 0, px(-49));

    // The readout is the primary information on this page, so it sits above
    // the hands. Order alone is not enough: a label's background is transparent
    // by default, which let the hands show through the gaps between digits and
    // read as if they were drawn on top. An opaque backing plate makes the
    // layering real.
    const auto make_plate = [](lv_obj_t *label, int pad_h, int pad_v) {
        if(label == nullptr) return;
        lv_obj_set_style_bg_color(label, kBlack, 0);
        lv_obj_set_style_bg_opa(label, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(label, px(6), 0);
        lv_obj_set_style_pad_hor(label, px(pad_h), 0);
        lv_obj_set_style_pad_ver(label, px(pad_v), 0);
    };
    make_plate(ui.clock_time, 6, 2);
    make_plate(ui.clock_date, 10, 4);
    // The dial title and the caption sit where the hands reach, so they need a
    // plate too: the second hand was slicing through "CLOCK".
    // The title plate needs real vertical padding: the second hand runs from
    // the centre out past y = 48 and would otherwise show through above and
    // below the words.
    make_plate(title, 14, 5);
    make_plate(caption, 14, 5);
    // Raised in ascending importance so the big readout ends up topmost.
    lv_obj_move_foreground(title);
    lv_obj_move_foreground(caption);
    lv_obj_move_foreground(ui.clock_date);
    lv_obj_move_foreground(ui.clock_time);

    update_clock_view();
}

void update_trip_view() {
    if(ui.trip_distance == nullptr) return;
    if(ui.trip.recording == 0) {
        lv_label_set_text(ui.trip_distance, "--");
        for(int i = 0; i < kTripRowCount; ++i) {
            lv_label_set_text(ui.trip_row_values[i], "--");
        }
        lv_label_set_text(ui.trip_caption, "NO TRIP");
        return;
    }

    char value[20];
    const double kilometres = static_cast<double>(ui.trip.distance_m) / 1000.0;
    // One decimal below 100 km keeps a short ride readable; above that the
    // tenth is noise on a glance-only instrument.
    std::snprintf(value, sizeof(value), kilometres < 100.0 ? "%.1f" : "%.0f",
                  kilometres);
    lv_label_set_text(ui.trip_distance, value);

    const unsigned total_seconds =
        static_cast<unsigned>(ui.trip.moving_time_s);
    const unsigned hours = total_seconds / 3600U;
    const unsigned minutes = (total_seconds / 60U) % 60U;
    const unsigned seconds = total_seconds % 60U;
    if(hours > 0U) {
        std::snprintf(value, sizeof(value), "%u:%02u:%02u", hours, minutes,
                      seconds);
    } else {
        std::snprintf(value, sizeof(value), "%u:%02u", minutes, seconds);
    }
    lv_label_set_text(ui.trip_row_values[kTripRowMoving], value);

    // Average speed is derived on the board so the phone only ever sends
    // totals. Moving time already excludes stops, so the division is the
    // rider-facing average rather than an elapsed-time average.
    const double average_kph =
        ui.trip.moving_time_s > 0
            ? static_cast<double>(ui.trip.distance_m) /
                  static_cast<double>(ui.trip.moving_time_s) * 3.6
            : 0.0;
    std::snprintf(value, sizeof(value), "%.1f", average_kph);
    lv_label_set_text(ui.trip_row_values[kTripRowAverage], value);

    std::snprintf(value, sizeof(value), "%.1f",
                  static_cast<double>(ui.trip.max_speed_deci_kph) / 10.0);
    lv_label_set_text(ui.trip_row_values[kTripRowMax], value);

    if(ui.trip.elevation_known != 0) {
        char caption[32];
        std::snprintf(caption, sizeof(caption), "CLIMB +%u M",
                      static_cast<unsigned>(ui.trip.elevation_gain_m));
        lv_label_set_text(ui.trip_caption, caption);
    } else {
        lv_label_set_text(ui.trip_caption, "CLIMB --");
    }
}

void trip_page_event(lv_event_t *event) {
    if(lv_event_get_code(event) != LV_EVENT_LONG_PRESSED) return;
    if(ui.trip_reset_callback != nullptr) {
        ui.trip_reset_callback(ui.trip_reset_callback_context);
    }
}

void create_trip_page() {
    lv_obj_t *page = ui.pages[MOTO_UI_PAGE_TRIP];
    // A trip computer is only useful if the rider can zero it without walking
    // back to the phone, so the whole page is the reset target.
    lv_obj_add_event_cb(page, trip_page_event, LV_EVENT_LONG_PRESSED, nullptr);
    lv_obj_t *title = make_label(page, &lv_font_montserrat_16, kQuiet, "TRIP");
    lv_obj_set_style_text_letter_space(title, px(4), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, px(48));

    // Total distance is the number a rider checks most often, so it is the
    // hero; the three instrument rows below carry the derived statistics.
    ui.trip_distance = make_label(page, &lv_font_montserrat_48, kWhite, "--");
    lv_obj_align(ui.trip_distance, LV_ALIGN_CENTER, 0, px(-62));
    ui.trip_unit = make_label(page, &lv_font_montserrat_16, kQuiet, "KM");
    lv_obj_set_style_text_letter_space(ui.trip_unit, px(3), 0);
    lv_obj_align(ui.trip_unit, LV_ALIGN_CENTER, 0, px(-20));

    static const char *row_labels[kTripRowCount] = {
        "MOVING", "AVG KM/H", "MAX KM/H"};
    static const int row_offsets[kTripRowCount] = {18, 54, 90};
    for(int i = 0; i < kTripRowCount; ++i) {
        ui.trip_row_labels[i] =
            make_label(page, &lv_font_montserrat_16, kQuiet, row_labels[i]);
        lv_obj_set_style_text_letter_space(ui.trip_row_labels[i], px(2), 0);
        lv_obj_align(ui.trip_row_labels[i], LV_ALIGN_LEFT_MID, px(96),
                     px(row_offsets[i]));
        ui.trip_row_values[i] =
            make_label(page, &lv_font_montserrat_20, kWhite, "--");
        lv_obj_align(ui.trip_row_values[i], LV_ALIGN_RIGHT_MID, px(-96),
                     px(row_offsets[i]));
    }
    ui.trip_caption = make_label(page, &lv_font_montserrat_16, kQuiet,
                                 "NO TRIP");
    lv_obj_set_style_text_letter_space(ui.trip_caption, px(2), 0);
    lv_obj_align(ui.trip_caption, LV_ALIGN_BOTTOM_MID, 0, px(-49));
    update_trip_view();
}

/// Copies one notification into the history ring, newest at the head.
void push_notification_history(const char *app, const char *title,
                               const char *message, uint16_t local_minutes) {
    const int slot = ui.notification_head;
    copy_text(ui.notification_history[slot].app,
              sizeof(ui.notification_history[slot].app), app, "");
    copy_text(ui.notification_history[slot].title,
              sizeof(ui.notification_history[slot].title), title, "");
    copy_text(ui.notification_history[slot].message,
              sizeof(ui.notification_history[slot].message), message, "");
    ui.notification_history[slot].local_minutes = local_minutes;
    ui.notification_head = (slot + 1) % kNotificationHistorySize;
    if(ui.notification_total < kNotificationHistorySize) ++ui.notification_total;
}

void refresh_history_page() {
    if(ui.history_rows[0] == nullptr) return;
    for(int i = 0; i < kNotificationHistorySize; ++i) {
        if(ui.history_rows[i] == nullptr) continue;
        lv_obj_t *body = ui.history_bodies[i];
        if(i >= ui.notification_total) {
            lv_obj_add_flag(ui.history_rows[i], LV_OBJ_FLAG_HIDDEN);
            if(body != nullptr) lv_obj_add_flag(body, LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        // Walk backwards from the head so row 0 is always the newest.
        const int index =
            (ui.notification_head - 1 - i + kNotificationHistorySize * 2) %
            kNotificationHistorySize;
        const auto &entry = ui.notification_history[index];

        // Line 1: "12:34  微信". The time is what tells a rider how stale the
        // entry is; the app name says where it came from.
        char heading[64];
        if(entry.local_minutes != MOTO_UI_CLOCK_UNKNOWN &&
           entry.local_minutes < 24U * 60U) {
            const char *app = entry.app[0] != '\0' ? entry.app : "通知";
            lv_snprintf(heading, sizeof(heading), "%02u:%02u  %s",
                        static_cast<unsigned>(entry.local_minutes / 60U),
                        static_cast<unsigned>(entry.local_minutes % 60U), app);
        } else {
            lv_snprintf(heading, sizeof(heading), "%s",
                        entry.app[0] != '\0' ? entry.app : "通知");
        }
        lv_label_set_text(ui.history_rows[i], heading);
        lv_obj_remove_flag(ui.history_rows[i], LV_OBJ_FLAG_HIDDEN);

        // Line 2: title, then the body if it says something new. A short
        // notification often repeats the title in the body.
        if(body == nullptr) continue;
        char text[192];
        const bool has_title = entry.title[0] != '\0';
        const bool has_body = entry.message[0] != '\0';
        if(has_title && has_body && std::strcmp(entry.title, entry.message) != 0) {
            lv_snprintf(text, sizeof(text), "%s：%s", entry.title,
                        entry.message);
        } else if(has_title) {
            lv_snprintf(text, sizeof(text), "%s", entry.title);
        } else if(has_body) {
            lv_snprintf(text, sizeof(text), "%s", entry.message);
        } else {
            lv_snprintf(text, sizeof(text), "(无内容)");
        }
        lv_label_set_text(body, text);
        lv_obj_remove_flag(body, LV_OBJ_FLAG_HIDDEN);
        // Let the column recompute its scroll extent after the text changes,
        // otherwise a long body would be clipped until the next layout pass.
        lv_obj_update_layout(body);
    }
    if(ui.history_empty != nullptr) {
        if(ui.notification_total == 0) {
            lv_obj_remove_flag(ui.history_empty, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(ui.history_empty, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void history_back_event(lv_event_t *) {
    // Leaving explicitly also clears the hold, so the phone's snapshot can
    // drive the page again.
    if(ui.page_callback != nullptr) {
        ui.page_callback(MOTO_UI_PAGE_NAVIGATION, ui.page_callback_context);
    } else {
        ui.user_selected_page = false;
        show_page(MOTO_UI_PAGE_NAVIGATION, true);
    }
}

void create_messages_page() {
    lv_obj_t *page = ui.pages[MOTO_UI_PAGE_MESSAGES];
    lv_obj_t *title =
        make_label(page, &lv_font_montserrat_16, kQuiet, "MESSAGES");
    lv_obj_set_style_text_letter_space(title, px(4), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, px(34));

    ui.history_empty = make_label(page, &moto_font_notify_20, kQuiet,
                                  "NO MESSAGES YET");
    lv_obj_center(ui.history_empty);

    // A visible way out. The scroll container covers most of this page, and a
    // gesture that lands on it must not be the only route back.
    ui.history_back = make_label(page, &lv_font_montserrat_16, kQuiet,
                                 "TAP TO RETURN");
    lv_obj_set_style_text_letter_space(ui.history_back, px(2), 0);
    lv_obj_align(ui.history_back, LV_ALIGN_BOTTOM_MID, 0, -px(30));
    lv_obj_add_flag(ui.history_back, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ui.history_back, history_back_event,
                        LV_EVENT_PRESSED, nullptr);

    // A scrolling column rather than a fixed list: a message can be long, and
    // the entries are read by swiping, not by fitting everything at once.
    // The container is inset from the panel edges so the round bezel never
    // clips a line, and its scroll is vertical only so it cannot steal the
    // horizontal page swipe.
    lv_obj_t *column = lv_obj_create(page);
    lv_obj_remove_style_all(column);
    lv_obj_set_size(column, px(330), px(330));
    lv_obj_align(column, LV_ALIGN_CENTER, 0, px(24));
    lv_obj_set_style_bg_opa(column, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(column, px(6), 0);
    lv_obj_set_style_pad_row(column, px(10), 0);
    lv_obj_set_flex_flow(column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(column, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(column, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_width(column, px(4), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(column, kGraphite, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(column, LV_OPA_COVER, LV_PART_SCROLLBAR);
    lv_obj_add_flag(column, LV_OBJ_FLAG_SCROLLABLE);
    // The scroll container covers most of this page, so it decides what a drag
    // means. It scrolls vertically and must never claim the horizontal swipe:
    // the screen-level handler is what changes pages. Gesture bubbling lets the
    // unclaimed gesture reach it, and EVENT_BUBBLE keeps a tap on a card from
    // being swallowed here instead of reaching the explicit return control.
    lv_obj_add_flag(column, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_flag(column, LV_OBJ_FLAG_EVENT_BUBBLE);

    for(int i = 0; i < kNotificationHistorySize; ++i) {
        // One card per entry: dim heading line and the message under it.
        lv_obj_t *card = lv_obj_create(column);
        lv_obj_remove_style_all(card);
        lv_obj_set_width(card, lv_pct(100));
        lv_obj_set_height(card, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, px(3), 0);

        lv_obj_t *heading = make_label(card, &moto_font_notify_16, kQuiet, "");
        lv_obj_set_width(heading, lv_pct(100));
        lv_obj_set_style_text_align(heading, LV_TEXT_ALIGN_LEFT, 0);
        lv_label_set_long_mode(heading, LV_LABEL_LONG_DOT);
        lv_obj_add_flag(heading, LV_OBJ_FLAG_HIDDEN);
        ui.history_rows[i] = heading;

        lv_obj_t *body = make_label(card, &moto_font_notify_28, kWhite, "");
        lv_obj_set_width(body, lv_pct(100));
        // Wrap, not truncate: the whole point of the page is reading the text.
        lv_label_set_long_mode(body, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(body, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_add_flag(body, LV_OBJ_FLAG_HIDDEN);
        ui.history_bodies[i] = body;
    }
    refresh_history_page();
}

void create_music_page() {
    lv_obj_t *page = ui.pages[MOTO_UI_PAGE_MUSIC];
    // Song titles and artist names are arbitrary user content, so these three
    // labels use the bundled Hanzi-capable font. Montserrat has no CJK glyphs
    // at all: a Chinese title showed up as a row of boxes.
    ui.music_source = make_label(page, &moto_font_notify_16, kQuiet, "APPLE MUSIC");
    lv_obj_set_style_text_letter_space(ui.music_source, px(2), 0);
    lv_obj_align(ui.music_source, LV_ALIGN_TOP_MID, 0, px(37));

    ui.music_disc = lv_obj_create(page);
    lv_obj_set_size(ui.music_disc, px(104), px(104));
    lv_obj_align(ui.music_disc, LV_ALIGN_TOP_MID, 0, px(71));
    lv_obj_set_style_radius(ui.music_disc, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(ui.music_disc, kGraphite, 0);
    lv_obj_set_style_bg_opa(ui.music_disc, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ui.music_disc, px(5), 0);
    lv_obj_set_style_border_color(ui.music_disc, kIce, 0);
    lv_obj_set_style_pad_all(ui.music_disc, 0, 0);
    lv_obj_t *core = lv_obj_create(ui.music_disc);
    lv_obj_set_size(core, px(30), px(30));
    lv_obj_center(core);
    lv_obj_set_style_radius(core, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(core, kBlack, 0);
    lv_obj_set_style_bg_opa(core, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(core, 0, 0);

    ui.music_title = make_label(page, &moto_font_notify_20, kWhite, "NIGHT RIDE");
    lv_obj_set_width(ui.music_title, px(260));
    lv_obj_set_style_text_align(ui.music_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(ui.music_title, LV_LABEL_LONG_DOT);
    lv_obj_align(ui.music_title, LV_ALIGN_TOP_MID, 0, px(190));
    ui.music_artist = make_label(page, &moto_font_notify_16, kQuiet, "PHONE NOW PLAYING");
    lv_obj_set_width(ui.music_artist, px(260));
    lv_obj_set_style_text_align(ui.music_artist, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(ui.music_artist, LV_LABEL_LONG_DOT);
    lv_obj_align(ui.music_artist, LV_ALIGN_TOP_MID, 0, px(216));

    static const char *labels[4] = {"<<", ">", ">>", "LIKE"};
    static const int positions[4][3] = {
        {72, 254, 54}, {153, 247, 62}, {234, 254, 54}, {147, 312, 66},
    };
    for(int i = 0; i < 4; ++i) {
        ui.music_buttons[i] = lv_button_create(page);
        lv_obj_set_size(ui.music_buttons[i], px(positions[i][2]),
                        px(i == 3 ? 28 : positions[i][2]));
        lv_obj_set_pos(ui.music_buttons[i], px(positions[i][0]),
                       px(positions[i][1]));
        lv_obj_set_style_radius(ui.music_buttons[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(ui.music_buttons[i], i == 1 ? kWhite : kGraphite, 0);
        lv_obj_set_style_bg_opa(ui.music_buttons[i], LV_OPA_COVER, 0);
        lv_obj_set_style_shadow_width(ui.music_buttons[i], 0, 0);
        lv_obj_set_style_border_width(ui.music_buttons[i], 0, 0);
        // Pin the pressed appearance to the idle appearance. The LVGL default
        // theme darkens and grows pressed buttons (recolor 35% + 3 px grow);
        // on the ESP32 PPA render path that pre-composited recolor painted as
        // a hard-edged yellow-green block over the round button. Identical
        // pressed styles remove the visual state difference by construction.
        lv_obj_set_style_bg_color(ui.music_buttons[i], i == 1 ? kWhite : kGraphite,
                                  LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(ui.music_buttons[i], LV_OPA_COVER,
                                LV_STATE_PRESSED);
        lv_obj_set_style_radius(ui.music_buttons[i], LV_RADIUS_CIRCLE,
                                LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(ui.music_buttons[i], 0, LV_STATE_PRESSED);
        lv_obj_set_style_border_width(ui.music_buttons[i], 0, LV_STATE_PRESSED);
        lv_obj_set_style_recolor_opa(ui.music_buttons[i], LV_OPA_TRANSP,
                                     LV_STATE_PRESSED);
        lv_obj_set_style_transform_width(ui.music_buttons[i], 0,
                                         LV_STATE_PRESSED);
        lv_obj_set_style_transform_height(ui.music_buttons[i], 0,
                                          LV_STATE_PRESSED);
        lv_obj_add_flag(ui.music_buttons[i], LV_OBJ_FLAG_GESTURE_BUBBLE);
        lv_obj_add_event_cb(
            ui.music_buttons[i], music_button_event, LV_EVENT_CLICKED,
            reinterpret_cast<void *>(static_cast<std::intptr_t>(i)));
        ui.music_button_labels[i] = make_label(
            ui.music_buttons[i], i == 3 ? &lv_font_montserrat_16 : &lv_font_montserrat_20,
            i == 1 ? kBlack : kWhite, labels[i]);
        lv_obj_center(ui.music_button_labels[i]);
    }
}

void set_boot_content_opacity(void *object, int32_t opacity) {
    lv_obj_set_style_opa(static_cast<lv_obj_t *>(object), opacity, 0);
}

}  // namespace

extern "C" void moto_nav_ui_show_boot_screen(void) {
    /* This entry point is also used while the full UI is live. Delete every
       timer that retains widget pointers before reset_ui_state() and
       lv_obj_clean() invalidate the tree. */
    if(ui.page_dots_timer != nullptr) {
        lv_timer_delete(ui.page_dots_timer);
        ui.page_dots_timer = nullptr;
    }
    if(ui.nav_route_motion_timer != nullptr) {
        lv_timer_delete(ui.nav_route_motion_timer);
        ui.nav_route_motion_timer = nullptr;
    }
    if(ui.nav_lifecycle_timer != nullptr) {
        lv_timer_delete(ui.nav_lifecycle_timer);
        ui.nav_lifecycle_timer = nullptr;
    }
    if(ui.nav_success_timer != nullptr) {
        lv_timer_delete(ui.nav_success_timer);
        ui.nav_success_timer = nullptr;
    }
    reset_ui_state();
    ui.screen = lv_screen_active();
    lv_obj_clean(ui.screen);
    lv_obj_remove_flag(ui.screen, LV_OBJ_FLAG_SCROLLABLE);
    const lv_color_t boot_black = LV_COLOR_MAKE(0x00, 0x00, 0x00);
    const lv_color_t boot_white = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF);
    lv_obj_set_style_bg_color(ui.screen, boot_black, 0);
    lv_obj_set_style_bg_opa(ui.screen, LV_OPA_COVER, 0);

    // Keep the AMOLED background truly black and animate only the white mark.
    // A single opacity animation with a delayed reverse is cheaper than
    // per-letter animation and fits inside the existing 1.25 s boot cadence.
    lv_obj_t *content = make_layer(ui.screen);
    lv_obj_set_style_opa(content, LV_OPA_TRANSP, 0);

    lv_obj_t *moto = make_label(content, &lv_font_montserrat_48,
                                boot_white, "MOTO");
    lv_obj_set_style_text_letter_space(moto, px(4), 0);
    lv_obj_set_style_text_outline_stroke_color(moto, boot_white, 0);
    lv_obj_set_style_text_outline_stroke_width(moto, px(2), 0);
    lv_obj_set_style_text_outline_stroke_opa(moto, LV_OPA_COVER, 0);
    lv_obj_set_style_transform_scale_x(moto, 340, 0);
    lv_obj_set_style_transform_scale_y(moto, 340, 0);
    lv_obj_align(moto, LV_ALIGN_CENTER, 0, px(-37));

    lv_obj_t *gps = make_label(content, &lv_font_montserrat_48,
                               boot_white, "GPS");
    lv_obj_set_style_text_letter_space(gps, px(10), 0);
    lv_obj_set_style_text_outline_stroke_color(gps, boot_white, 0);
    lv_obj_set_style_text_outline_stroke_width(gps, px(2), 0);
    lv_obj_set_style_text_outline_stroke_opa(gps, LV_OPA_COVER, 0);
    lv_obj_set_style_transform_scale_x(gps, 340, 0);
    lv_obj_set_style_transform_scale_y(gps, 340, 0);
    lv_obj_align(gps, LV_ALIGN_CENTER, 0, px(35));

    // One deliberate signature: a road-like diagonal cut through two solid
    // white velocity bars. No grey, colour or decorative loading spinner.
    for(int i = 0; i < 2; ++i) {
        lv_obj_t *bar = lv_obj_create(content);
        lv_obj_remove_style_all(bar);
        lv_obj_set_size(bar, px(i == 0 ? 56 : 38), px(5));
        lv_obj_set_pos(bar, px(i == 0 ? 73 : 250), px(i == 0 ? 259 : 94));
        lv_obj_set_style_bg_color(bar, boot_white, 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_set_style_transform_rotation(bar, -120, 0);
    }

    lv_anim_t fade;
    lv_anim_init(&fade);
    lv_anim_set_var(&fade, content);
    lv_anim_set_exec_cb(&fade, set_boot_content_opacity);
    lv_anim_set_values(&fade, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_delay(&fade, 30);
    lv_anim_set_duration(&fade, 230);
    lv_anim_set_reverse_delay(&fade, 650);
    lv_anim_set_reverse_duration(&fade, 260);
    lv_anim_set_path_cb(&fade, lv_anim_path_ease_in_out);
    lv_anim_start(&fade);
}

extern "C" void moto_nav_ui_show_power_off_screen(void) {
    if(ui.screen == nullptr) return;
    if(ui.page_dots_timer != nullptr) {
        lv_timer_delete(ui.page_dots_timer);
        ui.page_dots_timer = nullptr;
    }
    if(ui.nav_route_motion_timer != nullptr) {
        lv_timer_delete(ui.nav_route_motion_timer);
        ui.nav_route_motion_timer = nullptr;
    }
    if(ui.nav_lifecycle_timer != nullptr) {
        lv_timer_delete(ui.nav_lifecycle_timer);
        ui.nav_lifecycle_timer = nullptr;
    }
    if(ui.nav_success_timer != nullptr) {
        lv_timer_delete(ui.nav_success_timer);
        ui.nav_success_timer = nullptr;
    }

    lv_obj_clean(ui.screen);
    lv_obj_set_style_bg_color(ui.screen, LV_COLOR_MAKE(0x00, 0x00, 0x00), 0);
    lv_obj_set_style_bg_opa(ui.screen, LV_OPA_COVER, 0);

    lv_obj_t *brand = make_label(ui.screen, &lv_font_montserrat_48,
                                 LV_COLOR_MAKE(0xFF, 0xFF, 0xFF), "MOTO");
    lv_obj_set_style_text_letter_space(brand, px(4), 0);
    lv_obj_set_style_text_outline_stroke_color(
        brand, LV_COLOR_MAKE(0xFF, 0xFF, 0xFF), 0);
    lv_obj_set_style_text_outline_stroke_width(brand, px(2), 0);
    lv_obj_set_style_text_outline_stroke_opa(brand, LV_OPA_COVER, 0);
    lv_obj_align(brand, LV_ALIGN_CENTER, 0, px(-18));

    lv_obj_t *status = make_label(ui.screen, &lv_font_montserrat_16,
                                  LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
                                  "POWER OFF");
    lv_obj_set_style_text_letter_space(status, px(4), 0);
    lv_obj_align(status, LV_ALIGN_CENTER, 0, px(35));
}

extern "C" void moto_nav_ui_create(void) {
    reset_ui_state();
    ui.screen = lv_screen_active();
    lv_obj_clean(ui.screen);
    lv_obj_remove_flag(ui.screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui.screen, kBlack, 0);
    lv_obj_set_style_bg_opa(ui.screen, LV_OPA_COVER, 0);
    // Press/release travel instead of LV_EVENT_GESTURE: see swipe_event.
    lv_obj_add_event_cb(ui.screen, swipe_event, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(ui.screen, swipe_event, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(ui.screen, swipe_event, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(ui.screen, swipe_event, LV_EVENT_PRESS_LOST, nullptr);
    for(int i = 0; i < MOTO_UI_PAGE_COUNT; ++i) {
        ui.pages[i] = make_layer(ui.screen);
        // Both flags: swipe detection listens for press/release on the screen,
        // so a touch that lands on a child of a page has to bubble up or the
        // whole gesture is swallowed by whichever widget was hit.
        lv_obj_add_flag(ui.pages[i], LV_OBJ_FLAG_GESTURE_BUBBLE);
        lv_obj_add_flag(ui.pages[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    }
    create_navigation_page();
    create_speed_page();
    create_trip_page();
    create_compass_page();
    create_clock_page();
    create_music_page();
    create_messages_page();
    create_page_dots();
    create_notification_overlay();
    // Press events are delivered to the topmost object under the finger, not
    // necessarily to its page. Register once across the finished tree so any
    // touch reliably wakes the dots, including the map and music controls.
    install_interaction_wake(ui.screen);

    moto_music_state_t initial_music = {
        "APPLE MUSIC", "NIGHT RIDE", "PHONE NOW PLAYING", 1, 1, 0, 0,
    };
    moto_nav_ui_set_music_state(&initial_music);

    moto_ui_state_t initial{};
    initial.mode = MOTO_UI_ACQUIRING_FIX;
    initial.page = MOTO_UI_PAGE_NAVIGATION;
    initial.maneuver = MOTO_MANEUVER_STRAIGHT;
    initial.traffic = MOTO_TRAFFIC_UNKNOWN;
    moto_nav_ui_set_state(&initial);
    show_page(MOTO_UI_PAGE_NAVIGATION, true);
}

extern "C" void moto_nav_ui_set_state(const moto_ui_state_t *state) {
    if(state == nullptr || ui.screen == nullptr) return;
    // The Messages page is local to the panel and has no counterpart in the
    // snapshot the phone drives. Navigation updates arrive several times per
    // second, so without this the rider would be pulled off the page on the
    // next update; any touch on another page clears the hold.
    if(!ui.user_selected_page) {
        show_page(state->page);
    } else if(state->page != ui.page) {
        // DIAG: the snapshot is asking for a different page while the rider
        // holds one. If this fires during navigation, the hold is working and
        // something else is repainting.
        lv_log("HOLD page=%d want=%d\n", (int)ui.page, (int)state->page);
    }
    // Hidden pages do not need to be invalidated. Page changes immediately
    // apply a fresh snapshot through PhoneNavBridge, so this keeps every page
    // correct while avoiding three full page redraws per navigation update.
    switch(ui.page) {
        case MOTO_UI_PAGE_NAVIGATION: update_navigation(state); break;
        case MOTO_UI_PAGE_SPEED: update_speedometer(state); break;
        case MOTO_UI_PAGE_COMPASS: update_compass(state); break;
        case MOTO_UI_PAGE_MESSAGES: refresh_history_page(); break;
        case MOTO_UI_PAGE_CLOCK:
        case MOTO_UI_PAGE_TRIP:
        case MOTO_UI_PAGE_MUSIC:
        case MOTO_UI_PAGE_COUNT: break;
    }
}

extern "C" void moto_nav_ui_set_motion_state(const moto_ui_state_t *state) {
    if(state == nullptr || ui.screen == nullptr || state->page != ui.page) return;
    // QMI8658 samples arrive at high frequency. Only the route polyline or
    // compass rose actually changes with yaw; labels, arcs and hidden pages
    // remain untouched so LVGL submits a small, bounded dirty region.
    if(ui.page == MOTO_UI_PAGE_NAVIGATION) {
        update_building_geometry(state);
        update_road_geometry(state);
        update_route_geometry(state);
    } else if(ui.page == MOTO_UI_PAGE_COMPASS) {
        update_compass(state);
    }
}

extern "C" void moto_nav_ui_set_phone_connection(
    moto_ui_phone_connection_t connection) {
    if(ui.screen == nullptr || connection < MOTO_UI_PHONE_OFFLINE ||
       connection > MOTO_UI_PHONE_ONLINE) {
        return;
    }
    if(ui.phone_connection == connection) return;
    const bool became_online =
        connection == MOTO_UI_PHONE_ONLINE &&
        ui.phone_connection != MOTO_UI_PHONE_ONLINE;
    ui.phone_connection = connection;
    if(became_online) {
        ui.lifecycle_success_active = true;
        if(ui.nav_success_timer != nullptr) {
            lv_timer_set_period(ui.nav_success_timer,
                                kConnectionSuccessHoldMs);
            lv_timer_reset(ui.nav_success_timer);
            lv_timer_resume(ui.nav_success_timer);
        }
    } else if(connection != MOTO_UI_PHONE_ONLINE) {
        ui.lifecycle_success_active = false;
        if(ui.nav_success_timer != nullptr) {
            lv_timer_pause(ui.nav_success_timer);
        }
    }
    refresh_lifecycle();
}

extern "C" void moto_nav_ui_set_reduce_motion(uint8_t reduce_motion) {
    ui.reduce_motion = reduce_motion != 0;
    if(ui.nav_lifecycle_timer != nullptr) {
        if(ui.reduce_motion) {
            lv_timer_pause(ui.nav_lifecycle_timer);
        } else {
            lv_timer_resume(ui.nav_lifecycle_timer);
        }
    }
    refresh_lifecycle();
}

extern "C" void moto_nav_ui_set_page(moto_ui_page_t page) {
    if(ui.screen == nullptr) return;
    // The hold is owned by the gesture handler, not by this API: programmatic
    // page changes (a demo, a test, the phone asking for a page) must keep
    // working exactly as before.
    show_page(page, true);
}

/// Called by the gesture handler when the rider swipes to another page.
extern "C" void moto_nav_ui_hold_selected_page(moto_ui_page_t page) {
    if(ui.screen == nullptr) return;
    ui.user_selected_page = page != MOTO_UI_PAGE_NAVIGATION;
    show_page(page, true);
}

extern "C" moto_ui_page_t moto_nav_ui_get_page(void) {
    return ui.page;
}

extern "C" void moto_nav_ui_set_page_change_callback(
    moto_page_change_callback_t callback, void *context) {
    ui.page_callback = callback;
    ui.page_callback_context = context;
}

extern "C" void moto_nav_ui_set_clock(const moto_ui_clock_t *clock) {
    if(clock == nullptr) return;
    ui.clock = *clock;
    if(ui.screen != nullptr) update_clock_view();
}

extern "C" void moto_nav_ui_set_trip(const moto_ui_trip_t *trip) {
    if(trip == nullptr) return;
    ui.trip = *trip;
    if(ui.screen != nullptr) update_trip_view();
}

extern "C" void moto_nav_ui_set_trip_reset_callback(
    moto_trip_reset_callback_t callback, void *context) {
    ui.trip_reset_callback = callback;
    ui.trip_reset_callback_context = context;
}

extern "C" void moto_nav_ui_set_interaction_callback(
    moto_interaction_callback_t callback, void *context) {
    ui.interaction_callback = callback;
    ui.interaction_callback_context = context;
}

extern "C" void moto_nav_ui_show_notification(
    const moto_ui_notification_t *notification) {
    if(notification == nullptr || ui.notification_overlay == nullptr) return;

    const char *title = notification->title != nullptr ? notification->title : "";
    const char *message =
        notification->message != nullptr ? notification->message : "";
    // Some apps send only a body; showing it as the headline keeps the panel
    // from looking broken instead of merely terse.
    if(title[0] == '\0' && message[0] != '\0') {
        title = message;
        message = "";
    }
    lv_label_set_text(ui.notification_category,
                      notification->category != nullptr &&
                              notification->category[0] != '\0'
                          ? notification->category
                          : "NOTIFICATION");
    lv_label_set_text(ui.notification_app,
                      notification->app != nullptr ? notification->app : "");
    lv_label_set_text(ui.notification_title, title);
    lv_label_set_text(ui.notification_message, message);

    const bool first_message = ui.notification_total == 0;
    push_notification_history(
        notification->app != nullptr ? notification->app : "", title, message,
        notification->local_minutes);
    if(ui.page == MOTO_UI_PAGE_MESSAGES) refresh_history_page();
    // The Messages page only becomes a swipe target once something has been
    // recorded, so the first card has to reveal its dot or the rider never
    // learns the page exists.
    if(first_message) {
        update_page_dots();
        reveal_page_dots();
    }
    if(ui.notification_unread < 99) ++ui.notification_unread;
    if(ui.notification_unread > 1) {
        char badge[16] = {};
        std::snprintf(badge, sizeof(badge), "%u NEW",
                      static_cast<unsigned>(ui.notification_unread));
        lv_label_set_text(ui.notification_badge, badge);
    } else {
        lv_label_set_text(ui.notification_badge, "");
    }

    lv_obj_remove_flag(ui.notification_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(ui.notification_overlay);
    if(ui.notification_timer != nullptr) {
        lv_timer_set_period(ui.notification_timer,
                            notification->hold_ms == 0
                                ? kNotificationHoldMs
                                : notification->hold_ms);
        lv_timer_reset(ui.notification_timer);
        lv_timer_resume(ui.notification_timer);
    }
}

extern "C" void moto_nav_ui_dismiss_notification(void) {
    ui.notification_unread = 0;
    hide_notification_overlay();
}

extern "C" void moto_nav_ui_set_battery(uint8_t percent, uint8_t charging) {
    ui.battery_percent = percent > 100 ? MOTO_UI_BATTERY_UNKNOWN : percent;
    ui.battery_charging = charging != 0 ? 1 : 0;
    if(ui.screen != nullptr) update_status_row();
}

extern "C" void moto_nav_ui_set_link_quality(uint8_t bars) {
    ui.link_bars = bars > 3 ? 3 : bars;
    if(ui.screen != nullptr) update_status_row();
}

extern "C" void moto_nav_ui_set_music_state(const moto_music_state_t *state) {
    if(state == nullptr || ui.screen == nullptr) return;

    // Compare before copying. The phone re-sends the media state on every
    // playback tick, and rewriting three labels plus their text buffers that
    // often invalidated the Music page for no visible change.
    const bool same =
        ui.music.connected == state->connected &&
        ui.music.playing == state->playing &&
        ui.music.like_available == state->like_available &&
        ui.music.liked == state->liked &&
        std::strcmp(ui.music_source_text,
                    state->source_name != nullptr ? state->source_name : "") == 0 &&
        std::strcmp(ui.music_title_text,
                    state->track_title != nullptr ? state->track_title : "") == 0 &&
        std::strcmp(ui.music_artist_text,
                    state->artist_name != nullptr ? state->artist_name : "") == 0;
    if(same) return;

    ui.music = *state;
    copy_text(ui.music_source_text, sizeof(ui.music_source_text),
              state->source_name, "PHONE MEDIA");
    copy_text(ui.music_title_text, sizeof(ui.music_title_text),
              state->track_title, "--");
    copy_text(ui.music_artist_text, sizeof(ui.music_artist_text),
              state->artist_name, "--");
    ui.music.source_name = ui.music_source_text;
    ui.music.track_title = ui.music_title_text;
    ui.music.artist_name = ui.music_artist_text;
    update_music_view();
}

extern "C" void moto_nav_ui_set_music_page_enabled(uint8_t enabled) {
    if(ui.screen == nullptr) return;
    const bool wanted = enabled != 0;
    // Media state arrives several times a second. Re-running the transition and
    // the dot layout on every message kept kicking the rider off the Music page
    // and repainting the dot row, which is what looked like a flicker.
    if(wanted == ui.music_page_enabled) return;
    ui.music_page_enabled = wanted;
    if(!wanted && ui.page == MOTO_UI_PAGE_MUSIC) {
        // Only leave Music when the session is really gone, and only if the
        // rider is standing on it.
        show_page(MOTO_UI_PAGE_NAVIGATION, true);
    }
    update_page_dots();
}

extern "C" void moto_nav_ui_set_music_command_callback(
    moto_music_command_callback_t callback, void *context) {
    ui.music_callback = callback;
    ui.music_callback_context = context;
}

extern "C" void moto_nav_ui_set_demo_active(uint8_t enabled) {
    ui.demo_active = enabled != 0;
    refresh_lifecycle();
}

extern "C" void moto_nav_ui_set_demo_change_callback(
    moto_demo_change_callback_t callback, void *context) {
    ui.demo_callback = callback;
    ui.demo_callback_context = context;
}
