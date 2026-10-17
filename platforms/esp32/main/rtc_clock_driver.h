#pragma once

#include <cstddef>
#include <cstdint>

#include "rtc_clock.hpp"

namespace moto::esp32 {

// Thin I2C driver for the PCF85063ATL fitted to the Waveshare 2.16 board.
//
// The chip keeps the time across a phone disconnect, which is the whole point:
// the clock page used to fall back to "--:--" the moment the BLE link dropped.
// Only the parts of the register file this project needs are touched.
class RtcClock {
 public:
  /// Registers the RTC on the shared BSP I2C bus. Safe to call once during
  /// board initialization; returns false when the chip does not answer, and the
  /// panel then keeps relying on the phone alone.
  bool begin();

  [[nodiscard]] bool available() const noexcept { return available_; }

  /// Reads the stored time. False when the chip is absent, the time was never
  /// set (oscillator stopped) or the registers do not decode.
  bool read(moto::rtc_clock::Time& out) const;

  /// Writes a valid time and starts the oscillator.
  bool write(const moto::rtc_clock::Time& time);

  /// Seeds the chip from the phone push when the phone's time differs from the
  /// stored one by more than the writeback tolerance. Returns true when a write
  /// actually happened.
  bool sync_from_phone(const moto::rtc_clock::Time& phone_time);

 private:
  bool read_registers(std::uint8_t start, std::uint8_t* out,
                      std::size_t length) const;
  bool write_register(std::uint8_t reg, std::uint8_t value);
  bool write_registers(std::uint8_t start, const std::uint8_t* data,
                       std::size_t length);
  bool start_oscillator();

  bool available_ = false;
  // Two-digit year disambiguation: the last full year that was written.
  mutable std::uint16_t year_hint_ = 0;
  void* device_ = nullptr;  // i2c_master_dev_handle_t, kept opaque here.
};

}  // namespace moto::esp32
