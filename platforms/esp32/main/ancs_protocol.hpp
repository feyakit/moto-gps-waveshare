#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

// Payload codec for Apple's Notification Center Service (ANCS).
//
// ANCS is only reachable from the firmware side: iOS exposes the service as a
// GATT server on the link the phone itself opened, so the board is a GATT
// client (the "Notification Consumer") inside its existing peripheral role.
// Everything in this header is free of NimBLE and LVGL types so the byte
// layouts can be unit tested on the host, where a wrong length or a missing
// NUL termination would otherwise only show up as garbled text on the panel.
namespace moto::ancs {

// EventID values (ANCS specification, Table 2-2).
enum class EventId : std::uint8_t {
  Added = 0,
  Modified = 1,
  Removed = 2,
};

// EventFlags bits (Table 2-3).
enum EventFlag : std::uint8_t {
  FlagSilent = 1U << 0U,
  FlagImportant = 1U << 1U,
  FlagPreExisting = 1U << 2U,
  FlagPositiveAction = 1U << 3U,
  FlagNegativeAction = 1U << 4U,
};

// CategoryID values (Table 2-4).
enum class CategoryId : std::uint8_t {
  Other = 0,
  IncomingCall = 1,
  MissedCall = 2,
  Voicemail = 3,
  Social = 4,
  Schedule = 5,
  Email = 6,
  News = 7,
  HealthAndFitness = 8,
  BusinessAndFinance = 9,
  Location = 10,
  Entertainment = 11,
};

// CommandID values written to the Control Point characteristic.
enum class CommandId : std::uint8_t {
  GetNotificationAttributes = 0,
  GetAppAttributes = 1,
  PerformNotificationAction = 2,
};

// NotificationAttributeID values inside a GetNotificationAttributes response.
enum class AttributeId : std::uint8_t {
  AppIdentifier = 0,
  Title = 1,
  Subtitle = 2,
  Message = 3,
  MessageSize = 4,
  Date = 5,
  PositiveActionLabel = 6,
  NegativeActionLabel = 7,
};

enum class ActionId : std::uint8_t {
  Positive = 0,
  Negative = 1,
};

// Notification source notifications are always exactly these eight bytes.
constexpr std::size_t kNotificationSourceSize = 8;
constexpr std::size_t kUidSize = 4;

// Requested maximum lengths. iOS truncates with an ellipsis, which is what the
// round panel wants: a readable summary, never a wall of text.
constexpr std::uint16_t kRequestedTitleLength = 64;
constexpr std::uint16_t kRequestedMessageLength = 160;

// Storage sizes. One byte more than the request so a truncated payload still
// gets a terminator.
constexpr std::size_t kAppLength = 40;
constexpr std::size_t kTitleLength = 96;
constexpr std::size_t kMessageLength = 192;

struct NotificationSource {
  std::uint8_t event = 0;
  std::uint8_t event_flags = 0;
  std::uint8_t category = 0;
  std::uint8_t category_count = 0;
  std::uint32_t uid = 0;

  [[nodiscard]] bool is_added() const {
    return event == static_cast<std::uint8_t>(EventId::Added);
  }
  [[nodiscard]] bool is_removed() const {
    return event == static_cast<std::uint8_t>(EventId::Removed);
  }
  [[nodiscard]] bool is_important() const {
    return (event_flags & static_cast<std::uint8_t>(FlagImportant)) != 0;
  }
  [[nodiscard]] bool is_silent() const {
    return (event_flags & static_cast<std::uint8_t>(FlagSilent)) != 0;
  }
  /// iOS replays everything already sitting in Notification Center as soon as
  /// the ANCS subscription goes live, marking every one of them with
  /// `PreExisting`.  Those did not "just arrive": they must not wake the panel,
  /// chime, or trigger an attribute round trip, or a full Notification Center
  /// stampedes the ATT link the moment the rider starts the bike.
  [[nodiscard]] bool is_pre_existing() const {
    return (event_flags & static_cast<std::uint8_t>(FlagPreExisting)) != 0;
  }
  [[nodiscard]] bool is_incoming_call() const {
    return category == static_cast<std::uint8_t>(CategoryId::IncomingCall);
  }
};

struct NotificationAttributes {
  std::uint32_t uid = 0;
  bool has_app = false;
  bool has_title = false;
  bool has_message = false;
  char app[kAppLength] = {};
  char title[kTitleLength] = {};
  char message[kMessageLength] = {};
};

/// Notification Source payload: EventID, EventFlags, CategoryID,
/// CategoryCount, NotificationUID (all little endian where multi-byte).
inline bool parse_notification_source(const std::uint8_t* data, std::size_t len,
                                      NotificationSource& out) {
  if (data == nullptr || len < kNotificationSourceSize) {
    return false;
  }
  out.event = data[0];
  out.event_flags = data[1];
  out.category = data[2];
  out.category_count = data[3];
  out.uid = static_cast<std::uint32_t>(data[4]) |
      (static_cast<std::uint32_t>(data[5]) << 8U) |
      (static_cast<std::uint32_t>(data[6]) << 16U) |
      (static_cast<std::uint32_t>(data[7]) << 24U);
  return true;
}

namespace detail {

/// Copies a UTF-8 attribute value into fixed storage, dropping the trailing
/// NUL iOS sometimes appends and always terminating the result.
inline void copy_attribute(const std::uint8_t* value, std::size_t length,
                           char* destination, std::size_t capacity) {
  destination[0] = '\0';
  if (value == nullptr || length == 0 || capacity == 0) {
    return;
  }
  std::size_t usable = length;
  while (usable > 0 && value[usable - 1] == '\0') {
    --usable;
  }
  if (usable >= capacity) {
    usable = capacity - 1;
  }
  std::memcpy(destination, value, usable);
  destination[usable] = '\0';
}

}  // namespace detail

/// Data Source payload for GetNotificationAttributes: CommandID,
/// NotificationUID, then repeated { AttributeID, u16 length, value } records.
inline bool parse_attributes_response(const std::uint8_t* data, std::size_t len,
                                      NotificationAttributes& out) {
  if (data == nullptr || len < 1 + kUidSize) {
    return false;
  }
  if (data[0] != static_cast<std::uint8_t>(CommandId::GetNotificationAttributes)) {
    return false;
  }
  out.uid = static_cast<std::uint32_t>(data[1]) |
      (static_cast<std::uint32_t>(data[2]) << 8U) |
      (static_cast<std::uint32_t>(data[3]) << 16U) |
      (static_cast<std::uint32_t>(data[4]) << 24U);

  std::size_t offset = 1 + kUidSize;
  while (offset + 3 <= len) {
    const std::uint8_t attribute_id = data[offset];
    const std::size_t value_length =
        static_cast<std::size_t>(data[offset + 1]) |
        (static_cast<std::size_t>(data[offset + 2]) << 8U);
    offset += 3;
    if (offset + value_length > len) {
      // A truncated record cannot be trusted; keep what was already decoded.
      break;
    }
    const std::uint8_t* value = data + offset;
    switch (static_cast<AttributeId>(attribute_id)) {
      case AttributeId::AppIdentifier:
        detail::copy_attribute(value, value_length, out.app, sizeof(out.app));
        out.has_app = out.app[0] != '\0';
        break;
      case AttributeId::Title:
        detail::copy_attribute(value, value_length, out.title,
                               sizeof(out.title));
        out.has_title = out.title[0] != '\0';
        break;
      case AttributeId::Message:
        detail::copy_attribute(value, value_length, out.message,
                               sizeof(out.message));
        out.has_message = out.message[0] != '\0';
        break;
      default:
        break;
    }
    offset += value_length;
  }
  return true;
}

/// Control Point request for the customer-facing fields of one notification.
/// AppIdentifier needs no length parameter; Title and Message do.
inline std::size_t build_get_attributes_request(std::uint32_t uid,
                                                std::uint8_t* out,
                                                std::size_t capacity) {
  constexpr std::size_t kRequestSize = 1 + kUidSize + 1 + 3 + 3;
  if (out == nullptr || capacity < kRequestSize) {
    return 0;
  }
  std::size_t index = 0;
  out[index++] = static_cast<std::uint8_t>(CommandId::GetNotificationAttributes);
  out[index++] = static_cast<std::uint8_t>(uid & 0xFFU);
  out[index++] = static_cast<std::uint8_t>((uid >> 8U) & 0xFFU);
  out[index++] = static_cast<std::uint8_t>((uid >> 16U) & 0xFFU);
  out[index++] = static_cast<std::uint8_t>((uid >> 24U) & 0xFFU);
  out[index++] = static_cast<std::uint8_t>(AttributeId::AppIdentifier);

  out[index++] = static_cast<std::uint8_t>(AttributeId::Title);
  out[index++] = static_cast<std::uint8_t>(kRequestedTitleLength & 0xFFU);
  out[index++] = static_cast<std::uint8_t>(kRequestedTitleLength >> 8U);

  out[index++] = static_cast<std::uint8_t>(AttributeId::Message);
  out[index++] = static_cast<std::uint8_t>(kRequestedMessageLength & 0xFFU);
  out[index++] = static_cast<std::uint8_t>(kRequestedMessageLength >> 8U);
  return index;
}

/// Control Point request used to answer or dismiss a notification.
inline std::size_t build_perform_action(std::uint32_t uid, ActionId action,
                                        std::uint8_t* out,
                                        std::size_t capacity) {
  constexpr std::size_t kRequestSize = 1 + kUidSize + 1;
  if (out == nullptr || capacity < kRequestSize) {
    return 0;
  }
  std::size_t index = 0;
  out[index++] = static_cast<std::uint8_t>(CommandId::PerformNotificationAction);
  out[index++] = static_cast<std::uint8_t>(uid & 0xFFU);
  out[index++] = static_cast<std::uint8_t>((uid >> 8U) & 0xFFU);
  out[index++] = static_cast<std::uint8_t>((uid >> 16U) & 0xFFU);
  out[index++] = static_cast<std::uint8_t>((uid >> 24U) & 0xFFU);
  out[index++] = static_cast<std::uint8_t>(action);
  return index;
}

/// Short banner heading for the round panel.
inline const char* category_label(std::uint8_t category) {
  switch (static_cast<CategoryId>(category)) {
    case CategoryId::IncomingCall: return "INCOMING CALL";
    case CategoryId::MissedCall: return "MISSED CALL";
    case CategoryId::Voicemail: return "VOICEMAIL";
    case CategoryId::Social: return "SOCIAL";
    case CategoryId::Schedule: return "CALENDAR";
    case CategoryId::Email: return "MAIL";
    case CategoryId::News: return "NEWS";
    case CategoryId::HealthAndFitness: return "HEALTH";
    case CategoryId::BusinessAndFinance: return "FINANCE";
    case CategoryId::Location: return "LOCATION";
    case CategoryId::Entertainment: return "MEDIA";
    case CategoryId::Other: return "NOTIFICATION";
  }
  return "NOTIFICATION";
}

/// Friendly name for the bundle identifiers iOS actually sends.
///
/// ANCS returns the AppIdentifier (a reverse-DNS bundle id), not a display
/// name, so a card without this table reads "com.tencent.xin" where the rider
/// expects "微信". Unknown identifiers fall through unchanged, which is still
/// better than guessing.
inline const char* app_display_name(const char* bundle_id) {
  if (bundle_id == nullptr) return "";
  struct Entry {
    const char* id;
    const char* name;
  };
  static constexpr Entry kNames[] = {
      {"com.tencent.xin", "微信"},
      {"com.tencent.mqq", "QQ"},
      {"com.apple.MobileSMS", "信息"},
      {"com.apple.mobilephone", "电话"},
      {"com.apple.mobilemail", "邮件"},
      {"com.apple.mobilecal", "日历"},
      {"com.apple.reminders", "提醒事项"},
      {"com.apple.Preferences", "设置"},
      {"com.apple.AppStore", "App Store"},
      {"com.apple.Music", "音乐"},
      {"com.apple.Maps", "地图"},
      {"com.alipay.iphoneclient", "支付宝"},
      {"com.taobao.iphoneclient", "淘宝"},
      {"com.jd.iphone", "京东"},
      {"com.meituan.imeituan", "美团"},
      {"com.dianping.dpmobile", "大众点评"},
      {"com.tencent.wework", "企业微信"},
      {"com.sina.weibo", "微博"},
      {"com.zhihu.ios", "知乎"},
      {"com.netease.cloudmusic", "网易云音乐"},
      {"com.bilibili.finder", "哔哩哔哩"},
      {"com.ss.iphone.ugc.Aweme", "抖音"},
  };
  for (const auto& entry : kNames) {
    const char* a = entry.id;
    const char* b = bundle_id;
    while (*a != '\0' && *a == *b) {
      ++a;
      ++b;
    }
    if (*a == '\0' && *b == '\0') return entry.name;
  }
  return bundle_id;
}

/// An incoming call deserves a longer look than a chat message.
inline std::uint32_t banner_hold_ms(const NotificationSource& source) {
  if (source.is_incoming_call()) {
    return 20'000;
  }
  return source.is_important() ? 12'000 : 8'000;
}

/// Keeps at most one Control Point round trip on the air.
///
/// When an accessory subscribes, iOS replays the whole notification centre as a
/// burst of events, and a second ATT request that arrives while one is still in
/// flight is refused outright (NimBLE answers BLE_HS_EBUSY, which showed up on
/// hardware as a run of notification cards with no title or body).  The gate
/// fetches the newest notification of a burst once, and chains to a newer one
/// only after the answer it is waiting for has landed.
class AttributeFetchGate {
 public:
  enum class Action {
    None,     ///< Nothing to do: an answer is already on its way.
    Request,  ///< Send the attribute request for `uid`.
    Deliver,  ///< The answer for `uid` belongs to the newest notification.
  };

  struct Plan {
    Action action = Action::None;
    std::uint32_t uid = 0;
  };

  /// An answer that never arrives must not wedge the board for the rest of the
  /// ride; an ATT round trip takes tens of milliseconds.
  static constexpr std::uint64_t kTimeoutMs = 2'000;

  /// A Notification Source event arrived.
  Plan on_notification(std::uint32_t uid, std::uint64_t now_ms) {
    latest_uid_ = uid;
    has_latest_ = true;
    if (in_flight_ && now_ms - request_sent_ms_ < kTimeoutMs) {
      return {Action::None, 0};
    }
    // Either nothing is outstanding, or the earlier request was lost.
    in_flight_ = false;
    return {Action::Request, uid};
  }

  /// The request for `uid` went out.
  void on_request_sent(std::uint32_t uid, std::uint64_t now_ms) {
    in_flight_ = true;
    requested_uid_ = uid;
    request_sent_ms_ = now_ms;
  }

  /// The request could not be sent, so no answer will ever arrive.
  void on_request_failed() { in_flight_ = false; }

  /// The Data Source answered.  Pass `requested_uid()` when the payload could
  /// not be decoded, so the gate still learns that the round trip finished.
  Plan on_answer(std::uint32_t uid, std::uint64_t now_ms) {
    (void)now_ms;
    if (!in_flight_ || uid != requested_uid_) {
      return {Action::None, 0};
    }
    in_flight_ = false;
    if (has_latest_ && latest_uid_ != uid) {
      // The rider got something newer while this text was in flight.
      return {Action::Request, latest_uid_};
    }
    has_latest_ = false;
    return {Action::Deliver, uid};
  }

  bool in_flight() const { return in_flight_; }
  std::uint32_t requested_uid() const { return requested_uid_; }
  bool has_latest() const { return has_latest_; }
  std::uint32_t latest_uid() const { return latest_uid_; }
  void reset() { *this = AttributeFetchGate{}; }

 private:
  bool in_flight_ = false;
  std::uint32_t requested_uid_ = 0;
  std::uint64_t request_sent_ms_ = 0;
  bool has_latest_ = false;
  std::uint32_t latest_uid_ = 0;
};

}  // namespace moto::ancs
