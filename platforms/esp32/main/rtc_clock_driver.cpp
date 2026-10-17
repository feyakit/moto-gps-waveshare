#include "rtc_clock_driver.h"

#include <cstddef>
#include <cstdint>

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

namespace moto::esp32 {
namespace {

constexpr const char* kTag = "moto_rtc";

// The shared bus already carries the touch controller, the PMIC, the IMU and
// the audio codec; 100 kHz is what the BSP publishes for all of them.
constexpr std::uint32_t kI2cTimeoutMs = 100;

// The year this firmware was compiled in: __DATE__ is "Mmm dd yyyy", so the
// last four characters are the year. A two-digit RTC year is ambiguous on its
// own; this is the only century a board running this image can be in.
constexpr int parse_build_year() noexcept {
  const char* date = __DATE__;
  int year = 0;
  for (int i = 7; i < 11; ++i) {
    if (date[i] < '0' || date[i] > '9') {
      return 2000;
    }
    year = year * 10 + (date[i] - '0');
  }
  return year;
}
constexpr int kBuildYear = parse_build_year();
static_assert(kBuildYear >= 2000 && kBuildYear <= 2099,
              "the RTC calendar supports 2000..2099 only");

}  // namespace

bool RtcClock::begin() {
  auto bus = bsp_i2c_get_handle();
  if (bus == nullptr) {
    ESP_LOGW(kTag, "no shared I2C bus; RTC unavailable");
    return false;
  }
  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = moto::rtc_clock::kAddress;
  config.scl_speed_hz = 100'000;
  i2c_master_dev_handle_t device = nullptr;
  const esp_err_t added = i2c_master_bus_add_device(bus, &config, &device);
  if (added != ESP_OK) {
    ESP_LOGW(kTag, "PCF85063 registration failed: %s", esp_err_to_name(added));
    return false;
  }
  device_ = device;
  available_ = true;

  // A factory-fresh chip powers up with STOP set, which freezes the counters at
  // 00:00:00 with the integrity flag clear. Start it, then read back to prove
  // the chip is really there: a stuck or absent part answers with the same
  // bytes as a stopped one.
  moto::rtc_clock::Time probe{};
  if (!read(probe)) {
    // Either the oscillator never ran or the part is not answering; writing a
    // timestamp here would be inventing data, so only start the oscillator and
    // let the first phone push seed the actual time.
    if (!start_oscillator()) {
      ESP_LOGW(kTag, "PCF85063 did not answer; clock page stays phone-only");
      available_ = false;
      return false;
    }
    ESP_LOGI(kTag, "PCF85063 present but unset; waiting for the phone");
    return true;
  }
  ESP_LOGI(kTag, "PCF85063 running: %04u-%02u-%02u %02u:%02u:%02u",
           static_cast<unsigned>(probe.year),
           static_cast<unsigned>(probe.month), static_cast<unsigned>(probe.day),
           static_cast<unsigned>(probe.hour),
           static_cast<unsigned>(probe.minute),
           static_cast<unsigned>(probe.second));
  return true;
}

bool RtcClock::read(moto::rtc_clock::Time& out) const {
  if (!available_) {
    return false;
  }
  std::uint8_t regs[7] = {};
  if (!read_registers(static_cast<std::uint8_t>(moto::rtc_clock::Register::Seconds),
                      regs, sizeof(regs))) {
    return false;
  }
  // The chip stores a two-digit year only. Once a phone has written a time we
  // remember that year here, so a value written as 2026 is read back as 2026
  // instead of being mapped onto the century the board booted in.
  // Before any phone has written a time, the only century this firmware can
  // possibly be running in is the one it was built in. The chip's two-digit
  // year then lands in that century instead of a guessed 2000.
  const std::uint16_t hint =
      year_hint_ != 0 ? year_hint_ : static_cast<std::uint16_t>(kBuildYear);
  if (!moto::rtc_clock::decode_registers(regs, sizeof(regs), hint, out)) {
    return false;
  }
  year_hint_ = out.year;
  return true;
}

bool RtcClock::write(const moto::rtc_clock::Time& time) {
  if (!available_ || !time.is_valid()) {
    return false;
  }
  // Remember the century before the two-digit year is stored.
  year_hint_ = time.year;
  std::uint8_t regs[7] = {};
  moto::rtc_clock::encode_registers(time, regs);
  if (!write_registers(
          static_cast<std::uint8_t>(moto::rtc_clock::Register::Seconds), regs,
          sizeof(regs))) {
    return false;
  }
  // Clear the integrity flag so a later read trusts the value we just wrote.
  if (!write_register(static_cast<std::uint8_t>(moto::rtc_clock::Register::Seconds),
                      regs[0])) {
    return false;
  }
  return start_oscillator();
}

bool RtcClock::sync_from_phone(const moto::rtc_clock::Time& phone_time) {
  if (!available_ || !phone_time.is_valid()) {
    return false;
  }
  moto::rtc_clock::Time stored{};
  if (!read(stored)) {
    // Nothing trustworthy to compare against: seed it.
    return write(phone_time);
  }
  if (!moto::rtc_clock::needs_writeback(
          stored, phone_time, moto::rtc_clock::kWritebackToleranceSeconds)) {
    return false;
  }
  ESP_LOGI(kTag,
           "RTC drift corrected: %02u:%02u:%02u -> %02u:%02u:%02u",
           static_cast<unsigned>(stored.hour),
           static_cast<unsigned>(stored.minute),
           static_cast<unsigned>(stored.second),
           static_cast<unsigned>(phone_time.hour),
           static_cast<unsigned>(phone_time.minute),
           static_cast<unsigned>(phone_time.second));
  return write(phone_time);
}

bool RtcClock::start_oscillator() {
  std::uint8_t control1 = 0;
  if (!read_registers(
          static_cast<std::uint8_t>(moto::rtc_clock::Register::Control1),
          &control1, 1)) {
    return false;
  }
  const std::uint8_t wanted = static_cast<std::uint8_t>(
      (control1 & static_cast<std::uint8_t>(~moto::rtc_clock::kControl1Stop)) |
      moto::rtc_clock::kControl1CapSel12p5);
  if (wanted != control1 &&
      !write_register(
          static_cast<std::uint8_t>(moto::rtc_clock::Register::Control1),
          wanted)) {
    return false;
  }
  return write_register(
      static_cast<std::uint8_t>(moto::rtc_clock::Register::Control2),
      moto::rtc_clock::kControl2Default);
}

bool RtcClock::read_registers(std::uint8_t start, std::uint8_t* out,
                              std::size_t length) const {
  if (device_ == nullptr || out == nullptr || length == 0) {
    return false;
  }
  const esp_err_t result = i2c_master_transmit_receive(
      static_cast<i2c_master_dev_handle_t>(device_), &start, 1, out, length,
      kI2cTimeoutMs);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "read @0x%02X failed: %s", start, esp_err_to_name(result));
    return false;
  }
  return true;
}

bool RtcClock::write_register(std::uint8_t reg, std::uint8_t value) {
  return write_registers(reg, &value, 1);
}

bool RtcClock::write_registers(std::uint8_t start, const std::uint8_t* data,
                               std::size_t length) {
  if (device_ == nullptr || data == nullptr || length == 0) {
    return false;
  }
  std::uint8_t buffer[8] = {};
  if (length + 1 > sizeof(buffer)) {
    return false;
  }
  buffer[0] = start;
  for (std::size_t i = 0; i < length; ++i) {
    buffer[i + 1] = data[i];
  }
  const esp_err_t result = i2c_master_transmit(
      static_cast<i2c_master_dev_handle_t>(device_), buffer,
      static_cast<std::size_t>(length + 1), kI2cTimeoutMs);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "write @0x%02X failed: %s", start, esp_err_to_name(result));
    return false;
  }
  return true;
}

}  // namespace moto::esp32
