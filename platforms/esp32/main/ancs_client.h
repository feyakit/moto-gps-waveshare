#pragma once

#include <cstddef>
#include <cstdint>

#include "ancs_protocol.hpp"

// Forward declarations keep NimBLE out of this header: the transport and
// app_main only need the notification callback.
struct ble_gatt_error;
struct ble_gatt_svc;
struct ble_gatt_chr;
struct ble_gatt_dsc;

namespace moto::esp32 {

/// GATT client for Apple's Notification Center Service.
///
/// The board stays a BLE peripheral: iOS publishes the ANCS as a GATT server
/// on the link the phone itself opened, and this class performs the client
/// procedures over that same connection. A connection is probed once, and if
/// the ANCS is not published on it nothing else happens, so the navigation
/// link keeps working exactly as before on phones that do not offer it.
class AncsClient {
 public:
  using NotificationCallback = void (*)(
      const moto::ancs::NotificationSource& source,
      const moto::ancs::NotificationAttributes& attributes,
      void* context);

  static constexpr std::uint16_t kNoConnection = 0xFFFFU;

  void set_notification_callback(NotificationCallback callback,
                                 void* context) noexcept;

  // NimBLE GAP events, forwarded by BleNavTransport. All are delivered on the
  // NimBLE host task, which is also where the GATT client procedures run.
  void on_connect(std::uint16_t connection_handle);
  void on_disconnect(std::uint16_t connection_handle);
  void on_encryption_changed(std::uint16_t connection_handle, bool encrypted);
  void on_mtu_changed(std::uint16_t connection_handle, std::uint16_t mtu);
  void on_notification(std::uint16_t connection_handle,
                       std::uint16_t value_handle, const std::uint8_t* data,
                       std::uint16_t length);

  /// True once the Notification Source subscription is in place.
  [[nodiscard]] bool subscribed() const noexcept {
    return notification_source_subscribed_;
  }
  [[nodiscard]] std::uint16_t connection_handle() const noexcept {
    return connection_handle_;
  }
  /// Replayed Notification Center entries discarded since the link came up.
  /// Exposed so the log can show a burst was understood, not silently lost.
  [[nodiscard]] std::uint32_t pre_existing_dropped() const noexcept {
    return pre_existing_dropped_;
  }

 private:
  static int service_discovered(std::uint16_t connection_handle,
                                const struct ble_gatt_error* error,
                                const struct ble_gatt_svc* service,
                                void* argument);
  static int characteristic_discovered(std::uint16_t connection_handle,
                                       const struct ble_gatt_error* error,
                                       const struct ble_gatt_chr* characteristic,
                                       void* argument);
  static int descriptor_discovered(std::uint16_t connection_handle,
                                   const struct ble_gatt_error* error,
                                   std::uint16_t characteristic_value_handle,
                                   const struct ble_gatt_dsc* descriptor,
                                   void* argument);

  void start_probe();
  int subscribe_to(std::uint16_t connection_handle,
                   std::uint16_t characteristic_value_handle);
  void request_security();
  bool request_attributes(std::uint32_t uid);
  // Sends the Control Point request for `uid`, or shows the heading alone when
  // the request cannot go out at all.
  void fetch_attributes(std::uint32_t uid);
  void deliver(const moto::ancs::NotificationSource& source,
               const moto::ancs::NotificationAttributes& attributes);

  NotificationCallback callback_ = nullptr;
  void* callback_context_ = nullptr;

  std::uint16_t connection_handle_ = kNoConnection;
  std::uint16_t service_end_handle_ = 0;
  std::uint16_t notification_source_handle_ = 0;
  std::uint16_t control_point_handle_ = 0;
  std::uint16_t data_source_handle_ = 0;
  bool notification_source_subscribed_ = false;
  bool data_source_subscribed_ = false;
  bool probing_ = false;
  bool service_found_ = false;
  bool service_missing_ = false;
  bool security_requested_ = false;
  bool reprobed_after_encryption_ = false;
  std::uint16_t mtu_ = 23;

  // The Data Source answer to one attribute request can span several
  // notifications; it is only parsed once the last chunk arrived.
  static constexpr std::size_t kAssemblyCapacity = 512;
  std::uint8_t assembly_[kAssemblyCapacity] = {};
  std::size_t assembly_length_ = 0;

  moto::ancs::NotificationSource pending_source_{};
  bool has_pending_source_ = false;
  // Only one Control Point round trip is allowed on the air; a subscription
  // burst otherwise turns into a run of empty cards.
  moto::ancs::AttributeFetchGate fetch_gate_{};
  std::uint32_t pre_existing_dropped_ = 0;
};

}  // namespace moto::esp32
