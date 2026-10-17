#pragma once

#include <cstddef>
#include <cstdint>

// Local-time bookkeeping for the PCF85063ATL on the Waveshare 2.16 board.
//
// The panel's clock page has two sources of truth: the phone pushes
// ClockState while the link is up, and the board's own RTC keeps running when
// the phone is away. This header holds the arithmetic that decides what to
// show and what to write back, with no ESP-IDF and no I2C types, so the leap
// year rules and the "which source wins" policy can be tested on the host.
namespace moto::rtc_clock {

// PCF85063 register file, offsets from 0x00.
enum class Register : std::uint8_t {
  Control1 = 0x00,
  Control2 = 0x01,
  Seconds = 0x04,
  Minutes = 0x05,
  Hours = 0x06,
  Days = 0x07,
  Weekdays = 0x08,
  Months = 0x09,
  Years = 0x0A,
};

// 7-bit address 0x51 on the shared bus, matching the schematic.
constexpr std::uint8_t kAddress = 0x51;

// Control1.STOP clears the prescaler and stops the time circuit. It powers up
// set, so a factory-fresh board has a frozen RTC that reads 00:00:00 forever.
constexpr std::uint8_t kControl1Stop = 1U << 5U;
// Control1.CAP_SEL picks the crystal load capacitance; 12.5 pF matches the
// 32.768 kHz part fitted on this board.
constexpr std::uint8_t kControl1CapSel12p5 = 1U << 0U;
// Alarm interrupt stays off: GPIO13 is not consumed yet, so INT must stay
// quiet instead of waking on an alarm nobody configured.
constexpr std::uint8_t kControl2Default = 0x00;

constexpr bool is_leap_year(std::uint16_t year) noexcept {
  return (year % 4U == 0U && year % 100U != 0U) || (year % 400U == 0U);
}

constexpr std::uint8_t days_in_month(std::uint16_t year,
                                     std::uint8_t month) noexcept {
  if (month < 1U || month > 12U) {
    return 0;
  }
  constexpr std::uint8_t kMonth[12] = {31, 28, 31, 30, 31, 30,
                                       31, 31, 30, 31, 30, 31};
  const std::uint8_t base = kMonth[month - 1U];
  if (month == 2U && is_leap_year(year)) {
    return 29;
  }
  return base;
}

constexpr bool valid_date(std::uint16_t year, std::uint8_t month,
                          std::uint8_t day) noexcept {
  if (year < 2000U || year > 2099U) {
    return false;
  }
  const std::uint8_t limit = days_in_month(year, month);
  return limit != 0 && day >= 1U && day <= limit;
}

struct Time {
  std::uint16_t year = 0;   ///< Full year, e.g. 2026.
  std::uint8_t month = 0;   ///< 1..12, 0 when unknown.
  std::uint8_t day = 0;     ///< 1..31, 0 when unknown.
  std::uint8_t hour = 0;    ///< 0..23.
  std::uint8_t minute = 0;  ///< 0..59.
  std::uint8_t second = 0;  ///< 0..59.
  std::uint8_t weekday = 0; ///< ISO-8601: 1 Monday .. 7 Sunday; 0 unknown.

  [[nodiscard]] bool is_valid() const noexcept {
    return valid_date(year, month, day) && hour < 24U && minute < 60U &&
           second < 60U && weekday <= 7U;
  }
  [[nodiscard]] bool operator==(const Time& rhs) const noexcept {
    return year == rhs.year && month == rhs.month && day == rhs.day &&
           hour == rhs.hour && minute == rhs.minute && second == rhs.second &&
           weekday == rhs.weekday;
  }
  [[nodiscard]] bool operator!=(const Time& rhs) const noexcept {
    return !(*this == rhs);
  }
};

/// Days since 1970-01-01, so a day rollover needs no calendar special case.
constexpr std::int64_t days_from_civil(std::uint16_t year, std::uint8_t month,
                                       std::uint8_t day) noexcept {
  std::int64_t y = year;
  const std::int64_t m = month;
  // March-based year so February's extra day falls at the end.
  y -= m <= 2 ? 1 : 0;
  const std::int64_t era = (y >= 0 ? y : y - 399) / 400;
  const std::int64_t yoe = y - era * 400;
  const std::int64_t doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 +
                           static_cast<std::int64_t>(day) - 1;
  const std::int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

/// Inverse of days_from_civil.
constexpr Time civil_from_days(std::int64_t days) noexcept {
  days += 719468;
  const std::int64_t era = (days >= 0 ? days : days - 146096) / 146097;
  const std::int64_t doe = days - era * 146097;
  const std::int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const std::int64_t y = yoe + era * 400;
  const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const std::int64_t mp = (5 * doy + 2) / 153;
  const std::int64_t d = doy - (153 * mp + 2) / 5 + 1;
  const std::int64_t m = mp < 10 ? mp + 3 : mp - 9;
  Time out{};
  out.year = static_cast<std::uint16_t>(y + (m <= 2 ? 1 : 0));
  out.month = static_cast<std::uint8_t>(m);
  out.day = static_cast<std::uint8_t>(d);
  return out;
}

/// ISO-8601 weekday for a civil date: 1970-01-01 was a Thursday.
constexpr std::uint8_t iso_weekday(std::uint16_t year, std::uint8_t month,
                                   std::uint8_t day) noexcept {
  const std::int64_t days = days_from_civil(year, month, day);
  // 1970-01-01 is day 0 and a Thursday (4).
  std::int64_t wd = (days + 4) % 7;
  if (wd < 0) {
    wd += 7;
  }
  return static_cast<std::uint8_t>(wd == 0 ? 7 : wd);
}

constexpr std::uint32_t seconds_of_day(const Time& t) noexcept {
  return static_cast<std::uint32_t>(t.hour) * 3600U +
         static_cast<std::uint32_t>(t.minute) * 60U + t.second;
}

/// Adds whole seconds to a time, rolling over minutes, hours, days, months and
/// leap years. Returns the input unchanged when it is not a valid date.
inline Time advance_seconds(const Time& t, std::uint32_t seconds) noexcept {
  if (!valid_date(t.year, t.month, t.day)) {
    return t;
  }
  std::int64_t day_index = days_from_civil(t.year, t.month, t.day);
  std::int64_t total = static_cast<std::int64_t>(seconds_of_day(t)) +
                       static_cast<std::int64_t>(seconds);
  day_index += total / 86400;
  total %= 86400;
  Time out = civil_from_days(day_index);
  out.hour = static_cast<std::uint8_t>(total / 3600);
  out.minute = static_cast<std::uint8_t>((total % 3600) / 60);
  out.second = static_cast<std::uint8_t>(total % 60);
  out.weekday = iso_weekday(out.year, out.month, out.day);
  return out;
}

/// Minutes since local midnight, the form the clock page renders.
inline std::uint16_t minutes_of_day(const Time& t) noexcept {
  return static_cast<std::uint16_t>(t.hour * 60U + t.minute);
}

/// Builds a Time from a phone push. The phone only sends minutes, weekday and
/// month/day, so the year and the clock's seconds are kept from `base`.
inline Time from_phone(std::uint16_t minutes, std::uint8_t weekday,
                       std::uint8_t month, std::uint8_t day,
                       const Time& base) noexcept {
  Time out = base;
  if (minutes >= 24U * 60U || !valid_date(base.year, month, day)) {
    return out;
  }
  out.month = month;
  out.day = day;
  out.hour = static_cast<std::uint8_t>(minutes / 60U);
  out.minute = static_cast<std::uint8_t>(minutes % 60U);
  // The phone knows its own calendar, and it does not send the year, so the
  // board cannot second-guess its weekday. Trust a valid one; only derive it
  // when the push leaves it unknown.
  if (weekday >= 1U && weekday <= 7U) {
    out.weekday = weekday;
  } else {
    out.weekday = iso_weekday(base.year, month, day);
  }
  return out;
}

/// Decodes the seven BCD registers read from 0x04 onward.
/// `year` in the chip is 0..99 and is mapped onto the century of `hint`.
inline bool decode_registers(const std::uint8_t* regs, std::size_t length,
                             std::uint16_t year_hint, Time& out) noexcept {
  if (regs == nullptr || length < 7) {
    return false;
  }
  const auto bcd = [](std::uint8_t v, std::uint8_t& result) noexcept {
    const std::uint8_t tens = v >> 4U;
    const std::uint8_t ones = v & 0x0FU;
    if (tens > 9U || ones > 9U) {
      return false;
    }
    result = static_cast<std::uint8_t>(tens * 10U + ones);
    return true;
  };
  // Bit 7 of the seconds register is the "clock integrity" flag: it means the
  // oscillator stopped, so the stored time cannot be trusted.
  if ((regs[0] & 0x80U) != 0U) {
    return false;
  }
  std::uint8_t second = 0;
  std::uint8_t minute = 0;
  std::uint8_t hour = 0;
  std::uint8_t day = 0;
  std::uint8_t weekday = 0;
  std::uint8_t month = 0;
  std::uint8_t year = 0;
  if (!bcd(regs[0] & 0x7FU, second) || !bcd(regs[1] & 0x7FU, minute) ||
      !bcd(regs[2] & 0x3FU, hour) || !bcd(regs[3] & 0x3FU, day) ||
      !bcd(regs[4] & 0x07U, weekday) || !bcd(regs[5] & 0x1FU, month) ||
      !bcd(regs[6], year)) {
    return false;
  }
  // The chip stores 0 as Sunday; ISO-8601 uses 7.
  if (weekday == 0) {
    weekday = 7;
  }
  // The chip stores two digits only. The caller passes the full year this
  // board is actually living in, so the century follows that year rather than
  // being guessed from the digits.
  Time decoded{};
  decoded.year = static_cast<std::uint16_t>(
      (year_hint / 100U) * 100U + year);
  decoded.month = month;
  decoded.day = day;
  decoded.hour = hour;
  decoded.minute = minute;
  decoded.second = second;
  decoded.weekday = weekday;
  if (!decoded.is_valid()) {
    return false;
  }
  // The weekday comes straight from the chip: it was written together with the
  // rest of the time, and the two-digit year is not enough to recompute it.
  out = decoded;
  return true;
}

/// Encodes for a write starting at register 0x04.
inline void encode_registers(const Time& t, std::uint8_t* regs) noexcept {
  const auto to_bcd = [](std::uint8_t v) noexcept {
    return static_cast<std::uint8_t>(((v / 10U) << 4U) | (v % 10U));
  };
  regs[0] = to_bcd(t.second);
  regs[1] = to_bcd(t.minute);
  regs[2] = to_bcd(t.hour);
  regs[3] = to_bcd(t.day);
  // Holidays and other deliberate skips are exactly when a mismatch would be
  // written back, so always store the calendar value.
  regs[4] = to_bcd(t.weekday == 7U ? 0U : t.weekday);
  regs[5] = to_bcd(t.month);
  regs[6] = to_bcd(static_cast<std::uint8_t>(t.year % 100U));
}

/// True when the stored time and the phone's push differ by more than a
/// tolerance, which is when the board should correct the chip rather than let
/// the two drift apart over weeks.
inline bool needs_writeback(const Time& stored, const Time& phone,
                            std::uint32_t tolerance_seconds) noexcept {
  if (!stored.is_valid() || !phone.is_valid()) {
    return true;
  }
  const std::int64_t delta =
      days_from_civil(phone.year, phone.month, phone.day) * 86400 +
      static_cast<std::int64_t>(seconds_of_day(phone)) -
      (days_from_civil(stored.year, stored.month, stored.day) * 86400 +
       static_cast<std::int64_t>(seconds_of_day(stored)));
  const std::int64_t magnitude = delta < 0 ? -delta : delta;
  return magnitude > static_cast<std::int64_t>(tolerance_seconds);
}

// The clock page's sentinel for "no time known".
constexpr std::uint16_t kUnknownMinutes = 0xFFFFU;

/// Holds the time the clock page should show and keeps it ticking when the
/// phone is away. The phone's push is authoritative while the link is up; the
/// board's RTC carries on afterwards, and a push that disagrees by more than
/// the tolerance is written back to the chip.
class LocalClock {
 public:
  void set_enabled(bool enabled) noexcept { enabled_ = enabled; }
  [[nodiscard]] bool enabled() const noexcept { return enabled_; }

  /// Seeds or corrects the clock from a phone push. `elapsed_ms` is the board's
  /// monotonic time, used so the displayed minute advances on its own.
  void apply_phone(std::uint16_t minutes, std::uint8_t weekday,
                   std::uint8_t month, std::uint8_t day, std::uint64_t elapsed_ms) noexcept {
    if (!enabled_) {
      return;
    }
    // The first push has nothing to build on: the phone always sends a full
    // month/day, so that alone is enough to start the calendar.
    const Time seed = base_.is_valid() ? base_ : Time{2000U, 1U, 1U, 0U, 0U, 0U, 6U};
    base_ = from_phone(minutes, weekday, month, day, seed);
    if (base_.is_valid()) {
      anchored_ms_ = elapsed_ms;
    }
  }

  /// Loads the RTC value when nothing better is known, e.g. at boot or right
  /// after the link drops.
  void seed(const Time& time, std::uint64_t elapsed_ms) noexcept {
    if (!enabled_) {
      return;
    }
    base_ = time;
    anchored_ms_ = time.is_valid() ? elapsed_ms : 0;
  }

  [[nodiscard]] bool valid() const noexcept { return base_.is_valid(); }

  /// The time to display at monotonic `elapsed_ms`.
  [[nodiscard]] Time now(std::uint64_t elapsed_ms) const noexcept {
    if (!base_.is_valid()) {
      return Time{};
    }
    if (elapsed_ms <= anchored_ms_) {
      return base_;
    }
    const std::uint64_t seconds = (elapsed_ms - anchored_ms_) / 1000U;
    return advance_seconds(base_, static_cast<std::uint32_t>(seconds));
  }

  /// Minutes since local midnight for the clock page, or the UI's unknown
  /// marker when nothing has been set.
  [[nodiscard]] std::uint16_t minutes(std::uint64_t elapsed_ms) const noexcept {
    const Time t = now(elapsed_ms);
    return t.is_valid() ? minutes_of_day(t) : kUnknownMinutes;
  }

  void reset() noexcept {
    base_ = Time{};
    anchored_ms_ = 0;
  }

 private:
  Time base_{};
  std::uint64_t anchored_ms_ = 0;
  bool enabled_ = false;
};

// How far the RTC may drift before the phone's next push is allowed to correct
// it. Beyond this the two would visibly disagree on the clock page.
constexpr std::uint32_t kWritebackToleranceSeconds = 90;

}  // namespace moto::rtc_clock
