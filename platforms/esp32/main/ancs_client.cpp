#include "ancs_client.h"

#ifdef ESP_PLATFORM

#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"

namespace moto::esp32 {
namespace {

constexpr char kTag[] = "moto_ancs";

std::uint64_t now_ms() {
  return static_cast<std::uint64_t>(esp_timer_get_time()) / 1'000U;
}

// ANCS 7905F431-B5CE-4E99-A40F-4B1E122D00D0 and its three characteristics, in
// the little-endian order BLE_UUID128_INIT expects.
const ble_uuid128_t kServiceUuid = BLE_UUID128_INIT(
    0xd0, 0x00, 0x2d, 0x12, 0x1e, 0x4b, 0x0f, 0xa4,
    0x99, 0x4e, 0xce, 0xb5, 0x31, 0xf4, 0x05, 0x79);
const ble_uuid128_t kNotificationSourceUuid = BLE_UUID128_INIT(
    0xbd, 0x1d, 0xa2, 0x99, 0xe6, 0x25, 0x58, 0x8c,
    0xd9, 0x42, 0x01, 0x63, 0x0d, 0x12, 0xbf, 0x9f);
const ble_uuid128_t kControlPointUuid = BLE_UUID128_INIT(
    0xd9, 0xd9, 0xaa, 0xfd, 0xbd, 0x9b, 0x21, 0x98,
    0xa8, 0x49, 0xe1, 0x45, 0xf3, 0xd8, 0xd1, 0x69);
const ble_uuid128_t kDataSourceUuid = BLE_UUID128_INIT(
    0xfb, 0x7b, 0x7c, 0xce, 0x6a, 0xb3, 0x44, 0xbe,
    0xb5, 0x4b, 0xd6, 0x24, 0xe9, 0xc6, 0xea, 0x22);

constexpr std::uint8_t kCccdEnableNotification[2] = {0x01, 0x00};

// The Client Characteristic Configuration descriptor, as a named constant:
// BLE_UUID16_DECLARE builds a compound literal that C++ cannot take the
// address of.
const ble_uuid16_t kCccdUuid = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);

// Apple tells accessories to wait for the Insufficient Authentication error
// before asking the user to pair, instead of prompting on every connection.
// GATT discovery reports the raw ATT status, while a GATT operation reports it
// wrapped in BLE_HS_ATT_ERR.
bool is_authorization_error(int status) {
  return status == BLE_ATT_ERR_INSUFFICIENT_AUTHEN ||
      status == BLE_ATT_ERR_INSUFFICIENT_AUTHOR ||
      status == BLE_ATT_ERR_INSUFFICIENT_KEY_SZ ||
      status == BLE_ATT_ERR_INSUFFICIENT_ENC;
}

bool is_authorization_host_error(int status) {
  return status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_AUTHEN) ||
      status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_AUTHOR) ||
      status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_KEY_SZ) ||
      status == BLE_HS_ATT_ERR(BLE_ATT_ERR_INSUFFICIENT_ENC);
}

}  // namespace

void AncsClient::set_notification_callback(NotificationCallback callback,
                                           void* context) noexcept {
  callback_ = callback;
  callback_context_ = context;
}

void AncsClient::on_connect(std::uint16_t connection_handle) {
  if (connection_handle_ != kNoConnection) {
    // The board only consumes notifications from one link; a second central is
    // used for something else (a debugging app, another phone).
    ESP_LOGI(kTag, "ANCS already following connection %u, ignoring %u",
             connection_handle_, connection_handle);
    return;
  }
  connection_handle_ = connection_handle;
  mtu_ = 23;
  start_probe();
}

void AncsClient::on_disconnect(std::uint16_t connection_handle) {
  if (connection_handle != connection_handle_) {
    return;
  }
  if (notification_source_subscribed_) {
    ESP_LOGI(kTag, "ANCS session ended");
  }
  connection_handle_ = kNoConnection;
  service_end_handle_ = 0;
  notification_source_handle_ = 0;
  control_point_handle_ = 0;
  data_source_handle_ = 0;
  notification_source_subscribed_ = false;
  data_source_subscribed_ = false;
  probing_ = false;
  service_found_ = false;
  service_missing_ = false;
  security_requested_ = false;
  reprobed_after_encryption_ = false;
  mtu_ = 23;
  assembly_length_ = 0;
  has_pending_source_ = false;
  fetch_gate_.reset();
  pre_existing_dropped_ = 0;
}

void AncsClient::on_encryption_changed(std::uint16_t connection_handle,
                                       bool encrypted) {
  if (connection_handle != connection_handle_ || !encrypted) {
    return;
  }
  // Notification data is only released to an authorized link, so a probe that
  // came back empty before pairing may simply have been too early.
  if ((security_requested_ || service_missing_) &&
      !reprobed_after_encryption_) {
    reprobed_after_encryption_ = true;
    security_requested_ = false;
    service_missing_ = false;
    probing_ = false;
    start_probe();
  }
}

void AncsClient::on_mtu_changed(std::uint16_t connection_handle,
                                std::uint16_t mtu) {
  if (connection_handle != connection_handle_ || mtu < 23) {
    return;
  }
  mtu_ = mtu;
}

void AncsClient::on_notification(std::uint16_t connection_handle,
                                 std::uint16_t value_handle,
                                 const std::uint8_t* data,
                                 std::uint16_t length) {
  if (connection_handle != connection_handle_ || data == nullptr ||
      length == 0) {
    return;
  }

  if (notification_source_handle_ != 0 &&
      value_handle == notification_source_handle_) {
    moto::ancs::NotificationSource source;
    if (!moto::ancs::parse_notification_source(data, length, source)) {
      ESP_LOGW(kTag, "short notification source payload (%u bytes)", length);
      return;
    }
    if (source.is_removed()) {
      if (has_pending_source_ && pending_source_.uid == source.uid) {
        has_pending_source_ = false;
      }
      // Forwarded so the panel can close the card the phone just removed.
      deliver(source, moto::ancs::NotificationAttributes{});
      return;
    }
    if (source.is_pre_existing()) {
      // Replay of the phone's existing Notification Center, not something that
      // just happened.  Counting them keeps the log honest without spending a
      // Control Point round trip (or waking the panel) on each one.
      ++pre_existing_dropped_;
      return;
    }
    ESP_LOGI(kTag, "notification event=%u category=%u count=%u uid=%lu",
             source.event, source.category, source.category_count,
             static_cast<unsigned long>(source.uid));
    pending_source_ = source;
    has_pending_source_ = true;
    const auto plan = fetch_gate_.on_notification(source.uid, now_ms());
    if (plan.action == moto::ancs::AttributeFetchGate::Action::Request) {
      fetch_attributes(plan.uid);
    }
    // Otherwise a request for an earlier notification of this burst is still
    // on the air; its answer chains to the newest one.
    return;
  }

  if (value_handle != data_source_handle_) {
    ESP_LOGI(kTag, "notification on unexpected handle %u (%u bytes)",
             value_handle, length);
    return;
  }

  if (data_source_handle_ != 0 && value_handle == data_source_handle_) {
    if (assembly_length_ + length > kAssemblyCapacity) {
      ESP_LOGW(kTag, "attribute response overflow; dropping");
      assembly_length_ = 0;
      return;
    }
    std::memcpy(assembly_ + assembly_length_, data, length);
    assembly_length_ += length;
    // iOS splits an attribute response that does not fit one notification; a
    // full-sized chunk means more is coming.
    const std::uint16_t full_chunk =
        mtu_ > 3 ? static_cast<std::uint16_t>(mtu_ - 3) : 20;
    if (length == full_chunk) {
      return;
    }
    moto::ancs::NotificationAttributes attributes;
    const std::uint16_t assembled = assembly_length_;
    const bool parsed = moto::ancs::parse_attributes_response(
        assembly_, assembly_length_, attributes);
    assembly_length_ = 0;
    // An undecodable payload is reported against the uid that was asked for, so
    // the gate still learns that the round trip finished and can chain on.
    const std::uint32_t answered =
        parsed ? attributes.uid : fetch_gate_.requested_uid();
    const auto plan = fetch_gate_.on_answer(answered, now_ms());
    if (plan.action == moto::ancs::AttributeFetchGate::Action::Request) {
      // Something newer arrived while this text was in flight.
      fetch_attributes(plan.uid);
      return;
    }
    if (!parsed || plan.action != moto::ancs::AttributeFetchGate::Action::Deliver) {
      ESP_LOGW(kTag, "unusable attribute response: %u bytes, parsed=%d",
               assembled, parsed ? 1 : 0);
      return;
    }
    if (!has_pending_source_ || attributes.uid != pending_source_.uid) {
      return;
    }
    ESP_LOGI(kTag,
             "attributes for uid=%lu: %u bytes, title=%u message=%u app=%u",
             static_cast<unsigned long>(attributes.uid), assembled,
             attributes.has_title ? 1U : 0U,
             attributes.has_message ? 1U : 0U, attributes.has_app ? 1U : 0U);
    deliver(pending_source_, attributes);
    has_pending_source_ = false;
    return;
  }
}

void AncsClient::start_probe() {
  if (connection_handle_ == kNoConnection || probing_ || service_missing_) {
    return;
  }
  probing_ = true;
  const int result = ble_gattc_disc_svc_by_uuid(connection_handle_,
                                                &kServiceUuid.u,
                                                service_discovered, this);
  if (result != 0) {
    probing_ = false;
    ESP_LOGW(kTag, "ANCS discovery could not start: rc=%d", result);
  }
}

int AncsClient::subscribe_to(std::uint16_t connection_handle,
                             std::uint16_t characteristic_value_handle) {
  // Notification value handles are only useful once their Client
  // Characteristic Configuration descriptor is written, so the descriptor
  // search runs before the subscription is considered live.
  const int result = ble_gattc_disc_all_dscs(connection_handle,
                                             characteristic_value_handle,
                                             service_end_handle_,
                                             descriptor_discovered, this);
  if (result != 0) {
    ESP_LOGW(kTag, "descriptor discovery failed: rc=%d", result);
  }
  return 0;
}

void AncsClient::request_security() {
  if (connection_handle_ == kNoConnection || security_requested_) {
    return;
  }
  security_requested_ = true;
  ESP_LOGI(kTag, "ANCS needs a paired link; requesting security");
  const int result = ble_gap_security_initiate(connection_handle_);
  if (result != 0) {
    ESP_LOGW(kTag, "security initiation failed: rc=%d", result);
  }
}

bool AncsClient::request_attributes(std::uint32_t uid) {
  if (control_point_handle_ == 0) {
    return false;
  }
  std::uint8_t request[32] = {};
  const std::size_t length =
      moto::ancs::build_get_attributes_request(uid, request, sizeof(request));
  if (length == 0) {
    return false;
  }
  const int result = ble_gattc_write_flat(
      connection_handle_, control_point_handle_, request,
      static_cast<std::uint16_t>(length), nullptr, nullptr);
  if (result != 0) {
    ESP_LOGW(kTag, "attribute request failed: rc=%d", result);
    // A protected control point answers Insufficient Authentication until the
    // link is bonded; the retry after ENC_CHANGE then repeats the whole probe.
    if (is_authorization_host_error(result)) {
      request_security();
    }
    return false;
  }
  return true;
}

void AncsClient::fetch_attributes(std::uint32_t uid) {
  if (request_attributes(uid)) {
    fetch_gate_.on_request_sent(uid, now_ms());
    return;
  }
  fetch_gate_.on_request_failed();
  // Without the Control Point the heading alone still tells the rider what
  // just arrived, which beats showing nothing at all.
  if (!has_pending_source_ || pending_source_.uid != uid) {
    return;
  }
  moto::ancs::NotificationAttributes attributes;
  attributes.uid = uid;
  deliver(pending_source_, attributes);
  has_pending_source_ = false;
}

void AncsClient::deliver(const moto::ancs::NotificationSource& source,
                         const moto::ancs::NotificationAttributes& attributes) {
  if (callback_ == nullptr) {
    return;
  }
  callback_(source, attributes, callback_context_);
}

int AncsClient::service_discovered(std::uint16_t connection_handle,
                                   const struct ble_gatt_error* error,
                                   const struct ble_gatt_svc* service,
                                   void* argument) {
  auto* self = static_cast<AncsClient*>(argument);
  if (self == nullptr || error == nullptr ||
      connection_handle != self->connection_handle_) {
    return 0;
  }
  if (error->status == BLE_HS_EDONE) {
    self->probing_ = false;
    if (!self->service_found_) {
      self->service_missing_ = true;
      ESP_LOGI(kTag, "ANCS is not published on this link; navigation is "
                     "unaffected");
    }
    return 0;
  }
  if (error->status != 0) {
    self->probing_ = false;
    if (is_authorization_error(error->status)) {
      self->request_security();
      return 0;
    }
    ESP_LOGW(kTag, "ANCS service discovery failed: status=%d",
             error->status);
    return 0;
  }
  if (service == nullptr) {
    return 0;
  }
  self->service_found_ = true;
  self->service_end_handle_ = service->end_handle;
  const int result = ble_gattc_disc_all_chrs(connection_handle,
                                             service->start_handle,
                                             service->end_handle,
                                             characteristic_discovered, self);
  if (result != 0) {
    self->probing_ = false;
    ESP_LOGW(kTag, "ANCS characteristic discovery failed: rc=%d", result);
  }
  return 0;
}

int AncsClient::characteristic_discovered(
    std::uint16_t connection_handle, const struct ble_gatt_error* error,
    const struct ble_gatt_chr* characteristic, void* argument) {
  auto* self = static_cast<AncsClient*>(argument);
  if (self == nullptr || error == nullptr ||
      connection_handle != self->connection_handle_) {
    return 0;
  }
  if (error->status == BLE_HS_EDONE) {
    self->probing_ = false;
    if (self->notification_source_handle_ == 0) {
      ESP_LOGW(kTag, "ANCS has no Notification Source characteristic");
      return 0;
    }
    // Notification Source is mandatory; the Control Point fetches the text and
    // the Data Source carries it, so a missing one of those is worth naming.
    ESP_LOGI(kTag, "ANCS handles: source=%u control_point=%u data_source=%u",
             self->notification_source_handle_, self->control_point_handle_,
             self->data_source_handle_);
    if (self->control_point_handle_ == 0) {
      ESP_LOGW(kTag, "ANCS has no Control Point: headings only");
    }
    if (self->data_source_handle_ == 0) {
      ESP_LOGI(kTag, "ANCS without a Data Source: headings only");
    }
    // Subscription is a GATT procedure of its own, so it starts only now that
    // the characteristic walk has finished; the data source follows the
    // notification source instead of racing it.
    return self->subscribe_to(connection_handle,
                              self->notification_source_handle_);
  }
  if (error->status != 0 || characteristic == nullptr) {
    if (error->status != 0) {
      ESP_LOGW(kTag, "ANCS characteristic discovery error: status=%d",
               error->status);
    }
    return 0;
  }

  const bool notifiable =
      (characteristic->properties & BLE_GATT_CHR_PROP_NOTIFY) != 0;
  if (notifiable &&
      ble_uuid_cmp(&characteristic->uuid.u, &kNotificationSourceUuid.u) == 0) {
    self->notification_source_handle_ = characteristic->val_handle;
    ESP_LOGI(kTag, "ANCS notification source at handle %u",
             characteristic->val_handle);
    return 0;
  }
  if (notifiable &&
      ble_uuid_cmp(&characteristic->uuid.u, &kDataSourceUuid.u) == 0) {
    self->data_source_handle_ = characteristic->val_handle;
    ESP_LOGI(kTag, "ANCS data source at handle %u",
             characteristic->val_handle);
    return 0;
  }
  const bool writable =
      (characteristic->properties & BLE_GATT_CHR_PROP_WRITE) != 0;
  if (writable &&
      ble_uuid_cmp(&characteristic->uuid.u, &kControlPointUuid.u) == 0) {
    self->control_point_handle_ = characteristic->val_handle;
  }
  return 0;
}

int AncsClient::descriptor_discovered(
    std::uint16_t connection_handle, const struct ble_gatt_error* error,
    std::uint16_t characteristic_value_handle,
    const struct ble_gatt_dsc* descriptor, void* argument) {
  auto* self = static_cast<AncsClient*>(argument);
  if (self == nullptr || error == nullptr ||
      connection_handle != self->connection_handle_) {
    return 0;
  }
  if (error->status != 0 || descriptor == nullptr) {
    if (error->status != 0 && error->status != BLE_HS_EDONE) {
      // A range that spans the next characteristic declaration ends in an
      // attribute NimBLE does not accept as a descriptor; the CCCD before it
      // has been handled already, so this is noise rather than a failure.
      ESP_LOGD(kTag, "ANCS descriptor range ended early: status=%d",
               error->status);
    }
    if (characteristic_value_handle == self->notification_source_handle_ &&
        self->data_source_handle_ != 0 && !self->data_source_subscribed_) {
      self->subscribe_to(connection_handle, self->data_source_handle_);
    }
    return 0;
  }
  if (ble_uuid_cmp(&descriptor->uuid.u, &kCccdUuid.u) != 0) {
    return 0;
  }
  const bool is_notification_source =
      characteristic_value_handle == self->notification_source_handle_;
  const int result = ble_gattc_write_flat(
      connection_handle, descriptor->handle, kCccdEnableNotification,
      sizeof(kCccdEnableNotification), nullptr, nullptr);
  if (result != 0) {
    ESP_LOGW(kTag, "could not enable notifications (handle=%u): rc=%d",
             characteristic_value_handle, result);
    // This is the first protected access on most links: Apple expects the
    // accessory to pair only after the panel actually needs the data, so the
    // failure is what triggers the bond rather than the connection.
    if (is_authorization_host_error(result)) {
      self->request_security();
    }
    return 0;
  }
  if (is_notification_source) {
    self->notification_source_subscribed_ = true;
    ESP_LOGI(kTag, "ANCS ready: notifications will be shown on the panel");
  } else {
    self->data_source_subscribed_ = true;
  }
  return 0;
}

}  // namespace moto::esp32

#endif  // ESP_PLATFORM
