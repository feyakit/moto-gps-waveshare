#include "rtc_clock.hpp"

#include <iostream>

namespace {

using namespace moto::rtc_clock;

int failures = 0;

#define CHECK(condition)                                                      \
  do {                                                                        \
    if (!(condition)) {                                                       \
      std::cerr << "CHECK failed: " #condition " at line " << __LINE__       \
                << "\n";                                                      \
      ++failures;                                                             \
    }                                                                         \
  } while (false)

void test_leap_years_follow_the_gregorian_rule() {
  CHECK(is_leap_year(2024));
  CHECK(!is_leap_year(2025));
  CHECK(!is_leap_year(2100));
  CHECK(is_leap_year(2000));
  CHECK(days_in_month(2024, 2) == 29);
  CHECK(days_in_month(2025, 2) == 28);
  CHECK(days_in_month(2026, 1) == 31);
  CHECK(days_in_month(2026, 4) == 30);
  CHECK(days_in_month(2026, 13) == 0);
  CHECK(valid_date(2026, 10, 17));
  CHECK(!valid_date(2026, 2, 29));
  CHECK(valid_date(2024, 2, 29));
  CHECK(!valid_date(2026, 4, 31));
  CHECK(!valid_date(1999, 1, 1));
  CHECK(!valid_date(2100, 1, 1));
}

void test_iso_weekday_matches_known_dates() {
  // 1970-01-01 was a Thursday.
  CHECK(iso_weekday(1970, 1, 1) == 4);
  // 2026-10-17 is a Saturday.
  CHECK(iso_weekday(2026, 10, 17) == 6);
  // 2024-02-29 was a Thursday.
  CHECK(iso_weekday(2024, 2, 29) == 4);
  // 2000-01-01 was a Saturday.
  CHECK(iso_weekday(2000, 1, 1) == 6);
}

void test_advance_rolls_over_every_boundary() {
  Time t{};
  t.year = 2026;
  t.month = 10;
  t.day = 17;
  t.hour = 23;
  t.minute = 59;
  t.second = 50;
  t.weekday = iso_weekday(2026, 10, 17);

  const Time next = advance_seconds(t, 20);
  CHECK(next.day == 18);
  CHECK(next.hour == 0);
  CHECK(next.minute == 0);
  CHECK(next.second == 10);
  CHECK(next.weekday == 7);  // Sunday

  // Month rollover.
  Time end_of_month{};
  end_of_month.year = 2026;
  end_of_month.month = 10;
  end_of_month.day = 31;
  end_of_month.hour = 23;
  end_of_month.minute = 59;
  end_of_month.second = 59;
  const Time november = advance_seconds(end_of_month, 1);
  CHECK(november.month == 11);
  CHECK(november.day == 1);

  // Leap day: 2024-02-28 23:59:59 + 1s must be the 29th.
  Time feb{};
  feb.year = 2024;
  feb.month = 2;
  feb.day = 28;
  feb.hour = 23;
  feb.minute = 59;
  feb.second = 59;
  const Time leap = advance_seconds(feb, 1);
  CHECK(leap.month == 2);
  CHECK(leap.day == 29);

  // Non-leap year skips the 29th.
  feb.year = 2026;
  const Time march = advance_seconds(feb, 1);
  CHECK(march.month == 3);
  CHECK(march.day == 1);

  // Year rollover.
  Time end_of_year{};
  end_of_year.year = 2026;
  end_of_year.month = 12;
  end_of_year.day = 31;
  end_of_year.hour = 23;
  end_of_year.minute = 59;
  end_of_year.second = 59;
  const Time new_year = advance_seconds(end_of_year, 1);
  CHECK(new_year.year == 2027);
  CHECK(new_year.month == 1);
  CHECK(new_year.day == 1);

  // An invalid date is passed through untouched instead of producing garbage.
  Time broken{};
  broken.month = 13;
  CHECK(advance_seconds(broken, 5) == broken);
}

void test_encode_and_decode_round_trip() {
  Time t{};
  t.year = 2026;
  t.month = 10;
  t.day = 17;
  t.hour = 23;
  t.minute = 5;
  t.second = 7;
  t.weekday = 6;

  std::uint8_t regs[7] = {};
  encode_registers(t, regs);
  // BCD: 07 05 23 17 06 10 26 with Sunday stored as 0.
  CHECK(regs[0] == 0x07);
  CHECK(regs[1] == 0x05);
  CHECK(regs[2] == 0x23);
  CHECK(regs[3] == 0x17);
  CHECK(regs[4] == 0x06);
  CHECK(regs[5] == 0x10);
  CHECK(regs[6] == 0x26);

  Time decoded{};
  CHECK(decode_registers(regs, sizeof(regs), 2026, decoded));
  CHECK(decoded == t);

  // Sunday is stored as 0 and must come back as ISO 7.
  Time sunday = t;
  sunday.day = 18;
  sunday.weekday = 7;
  encode_registers(sunday, regs);
  CHECK(regs[4] == 0x00);
  CHECK(decode_registers(regs, sizeof(regs), 2026, decoded));
  CHECK(decoded.weekday == 7);
}

void test_decode_rejects_a_stopped_or_corrupt_clock() {
  Time decoded{};
  // Bit 7 of the seconds register is the integrity flag: the oscillator
  // stopped, so 00:00:00 is not midnight, it is "never set".
  const std::uint8_t stopped[7] = {0x80, 0x00, 0x00, 0x01, 0x04, 0x01, 0x26};
  CHECK(!decode_registers(stopped, sizeof(stopped), 2026, decoded));

  // A non-BCD nibble cannot be a real time.
  const std::uint8_t garbage[7] = {0x0A, 0x00, 0x00, 0x01, 0x04, 0x01, 0x26};
  CHECK(!decode_registers(garbage, sizeof(garbage), 2026, decoded));

  // 31 February is not a date the chip can be trusted about.
  const std::uint8_t impossible[7] = {0x00, 0x00, 0x12, 0x31, 0x02, 0x02, 0x26};
  CHECK(!decode_registers(impossible, sizeof(impossible), 2026, decoded));

  // Short reads are refused rather than read past the array.
  CHECK(!decode_registers(impossible, 6, 2026, decoded));
  CHECK(!decode_registers(nullptr, 7, 2026, decoded));
}

void test_decode_keeps_the_weekday_the_chip_stored() {
  // The chip says Monday (1) for 2026-10-17. The two-digit year cannot tell
  // which Saturday/Monday pair is meant, so the stored counter is the only
  // trustworthy value; the phone writes it together with the rest of the time.
  const std::uint8_t regs[7] = {0x00, 0x30, 0x09, 0x17, 0x01, 0x10, 0x26};
  Time decoded{};
  CHECK(decode_registers(regs, sizeof(regs), 2026, decoded));
  CHECK(decoded.weekday == 1);
}

void test_decode_maps_the_two_digit_year_onto_the_hint() {
  // Year byte 26 with a 2026 hint is 2026, not 2000 or 1926. The supported
  // calendar is 2000..2099, which is also the only range a hint may name.
  const std::uint8_t regs[7] = {0x00, 0x30, 0x09, 0x17, 0x06, 0x10, 0x26};
  Time decoded{};
  CHECK(decode_registers(regs, sizeof(regs), 2026, decoded));
  CHECK(decoded.year == 2026);
  // Any year inside the same century picks that century: 2099 still means the
  // 2000s, so byte 26 stays 2026.
  CHECK(decode_registers(regs, sizeof(regs), 2099, decoded));
  CHECK(decoded.year == 2026);
  // A hint from the previous century would put the result out of the supported
  // calendar, and is refused instead of silently misread.
  CHECK(!decode_registers(regs, sizeof(regs), 1999, decoded));
}

void test_phone_push_fills_the_gap_the_phone_does_not_send() {
  Time base{};
  base.year = 2026;
  base.month = 10;
  base.day = 17;
  base.hour = 12;
  base.minute = 0;
  base.second = 30;
  base.weekday = 6;

  // The phone sends only minutes, weekday and month/day.
  const Time pushed = from_phone(14U * 60U + 5U, 6, 10, 17, base);
  CHECK(pushed.year == 2026);
  CHECK(pushed.month == 10);
  CHECK(pushed.day == 17);
  CHECK(pushed.hour == 14);
  CHECK(pushed.minute == 5);
  // Seconds are not part of the protocol; the clock keeps its own.
  CHECK(pushed.second == 30);

  // The phone's weekday is authoritative: it knows its own calendar and does
  // not send the year, so the board cannot recompute it reliably.
  const Time monday = from_phone(600, 1, 10, 17, base);
  CHECK(monday.weekday == 1);
  // With no weekday from the phone the date still yields one.
  const Time derived = from_phone(600, 0, 10, 17, base);
  CHECK(derived.weekday == 6);

  // Out-of-range input leaves the base untouched.
  CHECK(from_phone(24U * 60U, 6, 10, 17, base) == base);
  CHECK(from_phone(600, 6, 2, 30, base) == base);
}

void test_writeback_only_when_the_sources_disagree() {
  Time stored{};
  stored.year = 2026;
  stored.month = 10;
  stored.day = 17;
  stored.hour = 12;
  stored.minute = 0;
  stored.second = 0;
  stored.weekday = 6;

  Time phone = stored;
  phone.second = 10;
  // Ten seconds apart: normal drift between the two clocks, no write needed.
  CHECK(!needs_writeback(stored, phone, kWritebackToleranceSeconds));

  phone.hour = 13;
  // An hour apart: the RTC is wrong and must be corrected.
  CHECK(needs_writeback(stored, phone, kWritebackToleranceSeconds));

  // An unreadable or never-set RTC must always be corrected.
  Time garbage{};
  CHECK(needs_writeback(garbage, phone, kWritebackToleranceSeconds));
}

void test_writeback_detects_a_day_of_drift() {
  Time stored{};
  stored.year = 2026;
  stored.month = 10;
  stored.day = 16;
  stored.hour = 12;
  stored.minute = 0;
  stored.weekday = 5;
  Time phone = stored;
  phone.day = 17;
  phone.weekday = 6;
  CHECK(needs_writeback(stored, phone, kWritebackToleranceSeconds));
}

}  // namespace

int main() {
  test_leap_years_follow_the_gregorian_rule();
  test_iso_weekday_matches_known_dates();
  test_advance_rolls_over_every_boundary();
  test_encode_and_decode_round_trip();
  test_decode_rejects_a_stopped_or_corrupt_clock();
  test_decode_keeps_the_weekday_the_chip_stored();
  test_decode_maps_the_two_digit_year_onto_the_hint();
  test_phone_push_fills_the_gap_the_phone_does_not_send();
  test_writeback_only_when_the_sources_disagree();
  test_writeback_detects_a_day_of_drift();

  if (failures != 0) {
    std::cerr << failures << " RTC clock checks failed\n";
    return 1;
  }
  std::cout << "RTC clock checks passed\n";
  return 0;
}
