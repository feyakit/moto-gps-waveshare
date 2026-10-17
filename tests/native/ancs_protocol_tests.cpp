// Host tests for the ANCS payload codec.  These pin the byte layouts that
// otherwise only fail on a real iPhone, where a wrong length shows up as
// truncated or garbled text on the round panel.
#include "ancs_protocol.hpp"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

#define CHECK(condition)                                                   \
  do {                                                                     \
    if (!(condition)) {                                                    \
      std::cerr << __FILE__ << ':' << __LINE__                             \
                << " CHECK failed: " #condition << '\n';                   \
      ++failures;                                                          \
    }                                                                      \
  } while (false)

using moto::ancs::ActionId;
using moto::ancs::AttributeFetchGate;
using moto::ancs::AttributeId;
using moto::ancs::CommandId;
using moto::ancs::NotificationAttributes;
using moto::ancs::NotificationSource;

using GateAction = moto::ancs::AttributeFetchGate::Action;

void test_notification_source_parses_the_documented_layout() {
  // EventID added, no flags, category incoming call, 1 active call,
  // UID 0x11223344.
  const std::uint8_t payload[8] = {0x00, 0x00, 0x01, 0x01,
                                   0x44, 0x33, 0x22, 0x11};
  NotificationSource source;
  CHECK(moto::ancs::parse_notification_source(payload, sizeof(payload), source));
  CHECK(source.is_added());
  CHECK(!source.is_removed());
  CHECK(source.is_incoming_call());
  CHECK(source.category_count == 1);
  CHECK(source.uid == 0x11223344U);
  CHECK(!source.is_important());
  CHECK(!source.is_silent());
}

void test_notification_source_flags_and_removal() {
  // Removed message, important + silent, category other.
  const std::uint8_t payload[8] = {0x02, 0x03, 0x00, 0x00,
                                   0x01, 0x00, 0x00, 0x00};
  NotificationSource source;
  CHECK(moto::ancs::parse_notification_source(payload, sizeof(payload), source));
  CHECK(source.is_removed());
  CHECK(!source.is_added());
  CHECK(source.is_important());
  CHECK(source.is_silent());
  CHECK(!source.is_incoming_call());
}

void test_notification_source_rejects_short_or_missing_payloads() {
  NotificationSource source;
  const std::uint8_t payload[7] = {0x00, 0x00, 0x01, 0x01, 0x44, 0x33, 0x22};
  CHECK(!moto::ancs::parse_notification_source(payload, sizeof(payload), source));
  CHECK(!moto::ancs::parse_notification_source(nullptr, 8, source));
  // Extra bytes are harmless: iOS may pad the notification.
  const std::uint8_t padded[12] = {0x00, 0x00, 0x06, 0x02, 0x44, 0x33, 0x22,
                                   0x11, 0xFF, 0xFF, 0xFF, 0xFF};
  CHECK(moto::ancs::parse_notification_source(padded, sizeof(padded), source));
  CHECK(source.uid == 0x11223344U);
}

void test_notification_source_marks_the_pairing_replay_as_pre_existing() {
  // PreExisting bit set, added, category social: this is the shape iOS uses
  // when it replays everything already in Notification Center right after the
  // ANCS subscription goes live.
  const std::uint8_t payload[8] = {0x00, 0x04, 0x04, 0x09,
                                   0x2A, 0x00, 0x00, 0x00};
  NotificationSource source;
  CHECK(moto::ancs::parse_notification_source(payload, sizeof(payload), source));
  CHECK(source.is_pre_existing());
  CHECK(source.is_added());
  // A genuinely new notification carries no such flag.
  const std::uint8_t live[8] = {0x00, 0x00, 0x04, 0x0A,
                                0x2B, 0x00, 0x00, 0x00};
  CHECK(moto::ancs::parse_notification_source(live, sizeof(live), source));
  CHECK(!source.is_pre_existing());
  // PreExisting combined with Silent must not be mistaken for a live silent one.
  const std::uint8_t both[8] = {0x00, 0x05, 0x06, 0x01,
                                0x2C, 0x00, 0x00, 0x00};
  CHECK(moto::ancs::parse_notification_source(both, sizeof(both), source));
  CHECK(source.is_pre_existing());
  CHECK(source.is_silent());
}

void test_attributes_response_decodes_the_requested_fields() {
  std::vector<std::uint8_t> payload;
  payload.push_back(static_cast<std::uint8_t>(
      CommandId::GetNotificationAttributes));
  payload.insert(payload.end(), {0x44, 0x33, 0x22, 0x11});
  const char* app = "com.apple.mail";
  payload.push_back(static_cast<std::uint8_t>(AttributeId::AppIdentifier));
  payload.push_back(static_cast<std::uint8_t>(std::strlen(app)));
  payload.push_back(0x00);
  payload.insert(payload.end(), app, app + std::strlen(app));
  const char* title = "Mom";
  payload.push_back(static_cast<std::uint8_t>(AttributeId::Title));
  payload.push_back(3);
  payload.push_back(0x00);
  payload.insert(payload.end(), title, title + 3);
  const char* message = "Dinner at 7?";
  payload.push_back(static_cast<std::uint8_t>(AttributeId::Message));
  payload.push_back(static_cast<std::uint8_t>(std::strlen(message)));
  payload.push_back(0x00);
  payload.insert(payload.end(), message, message + std::strlen(message));

  NotificationAttributes attributes;
  CHECK(moto::ancs::parse_attributes_response(payload.data(), payload.size(),
                                             attributes));
  CHECK(attributes.uid == 0x11223344U);
  CHECK(attributes.has_app);
  CHECK(std::string(attributes.app) == app);
  CHECK(attributes.has_title);
  CHECK(std::string(attributes.title) == "Mom");
  CHECK(attributes.has_message);
  CHECK(std::string(attributes.message) == message);
}

void test_attributes_response_strips_trailing_nul_and_truncates() {
  std::vector<std::uint8_t> payload;
  payload.push_back(static_cast<std::uint8_t>(
      CommandId::GetNotificationAttributes));
  payload.insert(payload.end(), {0x00, 0x00, 0x00, 0x01});
  // Title with an over-long value plus the NUL some apps append.
  const std::string long_title(400, 'x');
  payload.push_back(static_cast<std::uint8_t>(AttributeId::Title));
  payload.push_back(static_cast<std::uint8_t>(long_title.size() & 0xFFU));
  payload.push_back(static_cast<std::uint8_t>((long_title.size() >> 8U) & 0xFFU));
  payload.insert(payload.end(), long_title.begin(), long_title.end());
  payload.push_back(0);

  NotificationAttributes attributes;
  CHECK(moto::ancs::parse_attributes_response(payload.data(), payload.size(),
                                             attributes));
  CHECK(attributes.has_title);
  CHECK(std::strlen(attributes.title) == moto::ancs::kTitleLength - 1);
  CHECK(attributes.title[0] == 'x');
  CHECK(!attributes.has_message);
}

void test_attributes_response_keeps_earlier_fields_on_a_broken_record() {
  std::vector<std::uint8_t> payload;
  payload.push_back(static_cast<std::uint8_t>(
      CommandId::GetNotificationAttributes));
  payload.insert(payload.end(), {0x01, 0x00, 0x00, 0x00});
  const char* message = "Hi";
  payload.push_back(static_cast<std::uint8_t>(AttributeId::Message));
  payload.push_back(2);
  payload.push_back(0x00);
  payload.insert(payload.end(), message, message + 2);
  // A record that claims more bytes than remain must not be read past the end.
  payload.push_back(static_cast<std::uint8_t>(AttributeId::Title));
  payload.push_back(0x40);
  payload.push_back(0x00);
  payload.push_back('y');

  NotificationAttributes attributes;
  CHECK(moto::ancs::parse_attributes_response(payload.data(), payload.size(),
                                             attributes));
  CHECK(attributes.has_message);
  CHECK(std::string(attributes.message) == "Hi");
  CHECK(!attributes.has_title);
}

void test_attributes_response_rejects_other_commands_and_short_payloads() {
  NotificationAttributes attributes;
  const std::uint8_t app_attributes[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(!moto::ancs::parse_attributes_response(app_attributes,
                                               sizeof(app_attributes),
                                               attributes));
  const std::uint8_t too_short[3] = {0x00, 0x01, 0x02};
  CHECK(!moto::ancs::parse_attributes_response(too_short, sizeof(too_short),
                                               attributes));
  CHECK(!moto::ancs::parse_attributes_response(nullptr, 32, attributes));
}

void test_get_attributes_request_layout() {
  std::uint8_t request[32] = {};
  const std::size_t length =
      moto::ancs::build_get_attributes_request(0x11223344U, request,
                                               sizeof(request));
  CHECK(length == 12);
  const std::uint8_t expected[12] = {
      0x00, 0x44, 0x33, 0x22, 0x11, 0x00, 0x01, 0x40, 0x00, 0x03, 0xA0, 0x00};
  CHECK(std::memcmp(request, expected, sizeof(expected)) == 0);
  // Too small a buffer must not write anything.
  CHECK(moto::ancs::build_get_attributes_request(1, request, 4) == 0);
}

void test_perform_action_request_layout() {
  std::uint8_t request[16] = {};
  const std::size_t length =
      moto::ancs::build_perform_action(0x01020304U, ActionId::Negative, request,
                                       sizeof(request));
  CHECK(length == 6);
  const std::uint8_t expected[6] = {0x02, 0x04, 0x03, 0x02, 0x01, 0x01};
  CHECK(std::memcmp(request, expected, sizeof(expected)) == 0);
}

void test_category_labels_and_hold_times() {
  CHECK(std::string(moto::ancs::category_label(
            static_cast<std::uint8_t>(moto::ancs::CategoryId::IncomingCall))) ==
        "INCOMING CALL");
  CHECK(std::string(moto::ancs::category_label(
            static_cast<std::uint8_t>(moto::ancs::CategoryId::Email))) ==
        "MAIL");
  // Unknown categories fall back to a neutral heading.
  CHECK(std::string(moto::ancs::category_label(200)) == "NOTIFICATION");

  NotificationSource call;
  call.category = static_cast<std::uint8_t>(moto::ancs::CategoryId::IncomingCall);
  CHECK(moto::ancs::banner_hold_ms(call) == 20'000);
  NotificationSource important;
  important.event_flags = static_cast<std::uint8_t>(moto::ancs::FlagImportant);
  CHECK(moto::ancs::banner_hold_ms(important) == 12'000);
  NotificationSource quiet;
  CHECK(moto::ancs::banner_hold_ms(quiet) == 8'000);
}

// Subscription makes iOS replay the whole notification centre at once.  Only
// the first event may start a Control Point request, and only the newest one of
// the burst may be fetched; anything else is refused by the phone as EBUSY.
void test_attribute_fetch_gate_collapses_a_subscription_burst() {
  AttributeFetchGate gate;
  auto plan = gate.on_notification(1, 0);
  CHECK(plan.action == GateAction::Request);
  CHECK(plan.uid == 1);
  gate.on_request_sent(plan.uid, 0);

  for (std::uint32_t uid = 2; uid <= 6; ++uid) {
    plan = gate.on_notification(uid, uid * 10);
    CHECK(plan.action == GateAction::None);
  }
  CHECK(gate.in_flight());
  CHECK(gate.latest_uid() == 6);

  // The answer for the oldest event chains exactly one request for the newest.
  plan = gate.on_answer(1, 60);
  CHECK(plan.action == GateAction::Request);
  CHECK(plan.uid == 6);
  gate.on_request_sent(6, 60);

  plan = gate.on_answer(6, 90);
  CHECK(plan.action == GateAction::Deliver);
  CHECK(plan.uid == 6);
  CHECK(!gate.has_latest());
  CHECK(!gate.in_flight());
}

void test_attribute_fetch_gate_handles_one_notification_at_a_time() {
  AttributeFetchGate gate;
  auto plan = gate.on_notification(7, 1'000);
  CHECK(plan.action == GateAction::Request);
  gate.on_request_sent(7, 1'000);

  plan = gate.on_answer(7, 1'030);
  CHECK(plan.action == GateAction::Deliver);
  CHECK(plan.uid == 7);

  // Nothing is outstanding, so the next event is fetched straight away.
  plan = gate.on_notification(8, 5'000);
  CHECK(plan.action == GateAction::Request);
  CHECK(plan.uid == 8);
}

// A dropped answer must not wedge the board for the rest of the ride.
void test_attribute_fetch_gate_recovers_from_a_lost_answer() {
  AttributeFetchGate gate;
  auto plan = gate.on_notification(1, 0);
  CHECK(plan.action == GateAction::Request);
  gate.on_request_sent(1, 0);

  plan = gate.on_notification(2, AttributeFetchGate::kTimeoutMs + 1);
  CHECK(plan.action == GateAction::Request);
  CHECK(plan.uid == 2);
  gate.on_request_sent(2, AttributeFetchGate::kTimeoutMs + 1);

  // The abandoned request answers late and is ignored.
  plan = gate.on_answer(1, AttributeFetchGate::kTimeoutMs + 50);
  CHECK(plan.action == GateAction::None);
  CHECK(gate.in_flight());

  plan = gate.on_answer(2, AttributeFetchGate::kTimeoutMs + 80);
  CHECK(plan.action == GateAction::Deliver);
  CHECK(plan.uid == 2);
}

void test_attribute_fetch_gate_ignores_answers_it_did_not_ask_for() {
  AttributeFetchGate gate;
  auto plan = gate.on_notification(4, 0);
  gate.on_request_sent(4, 0);

  plan = gate.on_answer(99, 10);
  CHECK(plan.action == GateAction::None);
  CHECK(gate.in_flight());

  plan = gate.on_answer(4, 20);
  CHECK(plan.action == GateAction::Deliver);
}

void test_attribute_fetch_gate_release_paths() {
  // A Control Point write that never left frees the gate immediately.
  AttributeFetchGate gate;
  auto plan = gate.on_notification(5, 0);
  CHECK(plan.action == GateAction::Request);
  gate.on_request_failed();
  CHECK(!gate.in_flight());
  plan = gate.on_notification(6, 5);
  CHECK(plan.action == GateAction::Request);
  CHECK(plan.uid == 6);

  // An answer the caller could not decode still ends the round trip, because
  // the caller passes back the uid it asked for.
  AttributeFetchGate undecodable;
  plan = undecodable.on_notification(8, 0);
  undecodable.on_request_sent(plan.uid, 0);
  plan = undecodable.on_answer(undecodable.requested_uid(), 10);
  CHECK(plan.action == GateAction::Deliver);
  CHECK(plan.uid == 8);
  CHECK(!undecodable.in_flight());

  // A fresh link starts from nothing.
  undecodable.reset();
  CHECK(!undecodable.in_flight());
  CHECK(!undecodable.has_latest());
  plan = undecodable.on_notification(9, 0);
  CHECK(plan.action == GateAction::Request);
}

}  // namespace

int main() {
  test_notification_source_parses_the_documented_layout();
  test_notification_source_flags_and_removal();
  test_notification_source_rejects_short_or_missing_payloads();
  test_notification_source_marks_the_pairing_replay_as_pre_existing();
  test_attributes_response_decodes_the_requested_fields();
  test_attributes_response_strips_trailing_nul_and_truncates();
  test_attributes_response_keeps_earlier_fields_on_a_broken_record();
  test_attributes_response_rejects_other_commands_and_short_payloads();
  test_get_attributes_request_layout();
  test_perform_action_request_layout();
  test_category_labels_and_hold_times();
  test_attribute_fetch_gate_collapses_a_subscription_burst();
  test_attribute_fetch_gate_handles_one_notification_at_a_time();
  test_attribute_fetch_gate_recovers_from_a_lost_answer();
  test_attribute_fetch_gate_ignores_answers_it_did_not_ask_for();
  test_attribute_fetch_gate_release_paths();

  if (failures != 0) {
    std::cerr << failures << " ANCS protocol checks failed\n";
    return 1;
  }
  std::cout << "ANCS protocol checks passed\n";
  return 0;
}
