#pragma once

#include <stdint.h>

#define MOTO_UI_ROUTE_POINT_CAPACITY 24
#define MOTO_UI_ROAD_POINT_CAPACITY 192
#define MOTO_UI_ROAD_POLYLINE_CAPACITY 24
#define MOTO_UI_BUILDING_POINT_CAPACITY 128
#define MOTO_UI_BUILDING_FOOTPRINT_CAPACITY 16
#define MOTO_UI_CANVAS_WIDTH 480
#define MOTO_UI_CANVAS_HEIGHT 480

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTO_UI_ACQUIRING_FIX = 0,
    MOTO_UI_NAVIGATING,
    MOTO_UI_REROUTING,
    MOTO_UI_OFFLINE,
    MOTO_UI_ARRIVED,
} moto_ui_mode_t;

/**
 * Phone transport lifecycle is deliberately independent from internet
 * availability.  A phone can remain paired while its mobile network is
 * offline, so the empty navigation page must never infer BLE state from the
 * route provider's online flag.
 */
typedef enum {
    MOTO_UI_PHONE_OFFLINE = 0,
    MOTO_UI_PHONE_CONNECTING,
    MOTO_UI_PHONE_ONLINE,
} moto_ui_phone_connection_t;

typedef enum {
    MOTO_MANEUVER_STRAIGHT = 0,
    MOTO_MANEUVER_LEFT,
    MOTO_MANEUVER_RIGHT,
    MOTO_MANEUVER_SLIGHT_LEFT,
    MOTO_MANEUVER_SLIGHT_RIGHT,
    MOTO_MANEUVER_UTURN,
    MOTO_MANEUVER_ROUNDABOUT,
    MOTO_MANEUVER_ARRIVE,
} moto_maneuver_t;

typedef enum {
    MOTO_TRAFFIC_UNKNOWN = 0,
    MOTO_TRAFFIC_CLEAR,
    MOTO_TRAFFIC_SLOW,
    MOTO_TRAFFIC_CONGESTED,
    MOTO_TRAFFIC_SEVERE,
} moto_traffic_t;

typedef enum {
    MOTO_UI_PAGE_NAVIGATION = 0,
    MOTO_UI_PAGE_SPEED,
    MOTO_UI_PAGE_TRIP,
    MOTO_UI_PAGE_COMPASS,
    MOTO_UI_PAGE_CLOCK,
    MOTO_UI_PAGE_MUSIC,
    /* Recent notifications, newest first. Only reachable after a card has
       arrived, so a rider never swipes onto an empty log. */
    MOTO_UI_PAGE_MESSAGES,
    MOTO_UI_PAGE_COUNT,
} moto_ui_page_t;

/* Minutes-since-midnight sentinel used when the phone has no trustworthy
   local wall-clock yet (for example while disconnected). */
#define MOTO_UI_CLOCK_UNKNOWN 0xFFFFU

/* Battery percentage sentinel: no battery fitted (bench USB supply) or the
   PMIC has not answered yet. */
#define MOTO_UI_BATTERY_UNKNOWN 0xFFU

typedef struct {
    /* Minutes since local midnight, 0..1439, or MOTO_UI_CLOCK_UNKNOWN. */
    uint16_t minutes;
    /* ISO-8601 weekday: 1 Monday .. 7 Sunday; 0 unknown. */
    uint8_t weekday;
    /* Local calendar month/day; 0 when unknown. */
    uint8_t month;
    uint8_t day;
} moto_ui_clock_t;

/**
 * Ride totals accumulated by the phone.  The board only renders them: the
 * moving-time and elevation rules live in the phone's trip computer, so a
 * firmware update cannot change what a past ride would have counted.
 */
typedef struct {
    uint32_t distance_m;
    uint32_t moving_time_s;
    uint32_t elevation_gain_m;
    /* Peak speed in tenths of km/h; 0 when nothing was recorded. */
    uint16_t max_speed_deci_kph;
    /* Nonzero while the phone is accumulating the current ride. */
    uint8_t recording;
    /* Nonzero when elevation_gain_m is trustworthy. */
    uint8_t elevation_known;
} moto_ui_trip_t;

typedef enum {
    MOTO_MUSIC_PREVIOUS = 0,
    MOTO_MUSIC_TOGGLE_PLAYBACK,
    MOTO_MUSIC_NEXT,
    MOTO_MUSIC_LIKE,
} moto_music_command_t;

typedef struct {
    /* Native pixel coordinates in the single 480 x 480 shared canvas. */
    int16_t x;
    int16_t y;
} moto_ui_point_t;

/**
 * A span inside moto_ui_state_t::road_points. Background roads are kept as
 * separate polylines so unrelated streets are never joined by a fake line.
 */
typedef struct {
    uint8_t first_point_index;
    uint8_t point_count;
    uint8_t road_class;
} moto_ui_polyline_span_t;

typedef struct {
    uint8_t first_point_index;
    uint8_t point_count;
    uint8_t building_class;
} moto_ui_building_span_t;

typedef struct {
    const char *source_name;
    const char *track_title;
    const char *artist_name;
    uint8_t connected;
    uint8_t playing;
    uint8_t like_available;
    uint8_t liked;
} moto_music_state_t;

typedef void (*moto_music_command_callback_t)(moto_music_command_t command,
                                               void *context);
typedef void (*moto_page_change_callback_t)(moto_ui_page_t page,
                                             void *context);
typedef void (*moto_demo_change_callback_t)(uint8_t enabled,
                                             void *context);
/* Both fire on a tap or long press that the rider made on the panel itself. */
typedef void (*moto_trip_reset_callback_t)(void *context);
typedef void (*moto_interaction_callback_t)(void *context);

/**
 * One phone notification, already decoded from ANCS.  Strings are UTF-8 and
 * point at the caller's storage: the UI copies what it needs before returning.
 */
typedef struct {
    const char *category;
    const char *app;
    const char *title;
    const char *message;
    /* How long the card stays up before it disappears on its own. */
    uint32_t hold_ms;
} moto_ui_notification_t;

typedef struct {
    moto_ui_page_t page;
    moto_ui_mode_t mode;
    moto_maneuver_t maneuver;
    moto_traffic_t traffic;
    uint32_t distance_to_maneuver_m;
    uint32_t remaining_distance_m;
    uint32_t remaining_time_s;
    uint8_t route_progress_percent;
    uint8_t gps_accuracy_m;
    uint8_t online;
    uint8_t has_destination;
    uint8_t route_request_in_flight;
    /* Geometry identity, used only to distinguish a reroute/map replacement
       from ordinary high-rate heading and position motion. */
    uint32_t route_identity;
    uint32_t route_generation;
    uint32_t map_scene_revision;
    uint16_t speed_kph;
    uint16_t speed_limit_kph;
    uint16_t heading_deg;
    uint8_t route_point_count;
    moto_ui_point_t route_points[MOTO_UI_ROUTE_POINT_CAPACITY];
    uint8_t road_point_count;
    moto_ui_point_t road_points[MOTO_UI_ROAD_POINT_CAPACITY];
    uint8_t road_polyline_count;
    moto_ui_polyline_span_t
        road_polylines[MOTO_UI_ROAD_POLYLINE_CAPACITY];
    uint8_t building_point_count;
    moto_ui_point_t building_points[MOTO_UI_BUILDING_POINT_CAPACITY];
    uint8_t building_footprint_count;
    moto_ui_building_span_t
        building_footprints[MOTO_UI_BUILDING_FOOTPRINT_CAPACITY];
    const char *road_name;
    const char *next_road_name;
} moto_ui_state_t;

void moto_nav_ui_create(void);
/** Draw the monochrome power-on wordmark before the full UI is created. */
void moto_nav_ui_show_boot_screen(void);
/** Draw the final monochrome frame before the PMIC removes power. */
void moto_nav_ui_show_power_off_screen(void);
void moto_nav_ui_set_state(const moto_ui_state_t *state);
/** Apply only high-rate route/heading motion to the currently visible page. */
void moto_nav_ui_set_motion_state(const moto_ui_state_t *state);
/** Drive the full-screen connection/ready lifecycle on the navigation page. */
void moto_nav_ui_set_phone_connection(
    moto_ui_phone_connection_t connection);
void moto_nav_ui_set_reduce_motion(uint8_t reduce_motion);
void moto_nav_ui_set_page(moto_ui_page_t page);
moto_ui_page_t moto_nav_ui_get_page(void);
void moto_nav_ui_set_page_change_callback(
    moto_page_change_callback_t callback,
    void *context);
/** Update only the Clock page; other pages keep their current content. */
void moto_nav_ui_set_clock(const moto_ui_clock_t *clock);
/** Update only the Trip page. */
void moto_nav_ui_set_trip(const moto_ui_trip_t *trip);
/** Long-pressing the Trip page asks the phone to start a new ride. */
void moto_nav_ui_set_trip_reset_callback(
    moto_trip_reset_callback_t callback,
    void *context);
/** Any touch, page change or key press, for the parked-screen policy. */
void moto_nav_ui_set_interaction_callback(
    moto_interaction_callback_t callback,
    void *context);
/**
 * Show a phone notification over the current page.  Repeating the call while
 * one is visible replaces the content and restarts the hold, and the overlay
 * disappears on its own after hold_ms or as soon as it is touched.
 */
void moto_nav_ui_show_notification(const moto_ui_notification_t *notification);
/** Hide the notification overlay early, if one is visible. */
void moto_nav_ui_dismiss_notification(void);
/**
 * Corner status row: pack level and link quality, both at a glance while
 * riding. `percent` is 0..100 or MOTO_UI_BATTERY_UNKNOWN; `charging` only
 * decorates the level, it never hides it. `bars` is 0..3, 0 meaning no link.
 */
void moto_nav_ui_set_battery(uint8_t percent, uint8_t charging);
void moto_nav_ui_set_link_quality(uint8_t bars);
void moto_nav_ui_set_music_state(const moto_music_state_t *state);
void moto_nav_ui_set_music_page_enabled(uint8_t enabled);
void moto_nav_ui_set_music_command_callback(
    moto_music_command_callback_t callback,
    void *context);
/** Mark the production navigation renderer as being fed by demo snapshots. */
void moto_nav_ui_set_demo_active(uint8_t enabled);
/** Long-pressing the navigation page toggles this test-build demo source. */
void moto_nav_ui_set_demo_change_callback(
    moto_demo_change_callback_t callback,
    void *context);

#ifdef __cplusplus
}
#endif
