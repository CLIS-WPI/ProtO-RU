// SPDX-FileCopyrightText: Copyright (C) 2026 National University of Singapore
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "../../../../apps/examples/ofh/ru_emulator_gps_slot_aligner.h"
#include "ocudu/ocudulog/ocudulog.h"
#include <gtest/gtest.h>

using namespace ocudu;

namespace {

/// Timing notifier spy counting the forwarded notifications.
class timing_notifier_spy : public ru_timing_notifier
{
public:
  unsigned nof_ttis       = 0;
  unsigned nof_half_slots = 0;
  unsigned nof_full_slots = 0;

  void on_tti_boundary(const tti_boundary_context&) override { ++nof_ttis; }
  void on_ul_half_slot_boundary(slot_point) override { ++nof_half_slots; }
  void on_ul_full_slot_boundary(slot_point) override { ++nof_full_slots; }
};

/// 30 kHz slot duration in nanoseconds.
constexpr uint64_t SLOT_NS = 500000ULL;

/// \brief Builds a TTI context for the given radio slot whose (synthetic) host clock runs with the given GPS slot
/// offset: radio slot \c radio_slot airs when the GPS slot count is \c radio_slot + \c gps_slot_offset.
///
/// The time point carries air time minus the downlink processing lead the lower PHY notification exhibits (the aligner
/// adds it back), plus the given jitter.
tti_boundary_context make_tti(uint64_t radio_slot, unsigned gps_slot_offset, int64_t jitter_ns = 0)
{
  const uint64_t gps_air_ns  = (radio_slot + gps_slot_offset) * SLOT_NS;
  const uint64_t unix_air_ns = gps_air_ns + gps_slot_alignment::UNIX_TO_GPS_SECONDS_OFFSET * 1000000000ULL;

  tti_boundary_context ctx;
  ctx.slot = slot_point_extended(subcarrier_spacing::kHz30, radio_slot);
  ctx.time_point =
      std::chrono::system_clock::time_point(std::chrono::duration_cast<std::chrono::system_clock::duration>(
                                                std::chrono::nanoseconds(unix_air_ns + jitter_ns)) -
                                            std::chrono::milliseconds(1));
  return ctx;
}

/// Consecutive 64-TTI windows the aligner needs to adopt a new offset at 30 kHz (3 s of agreement).
const unsigned CONFIRM = ru_emulator_gps_slot_aligner::nof_confirm_windows(subcarrier_spacing::kHz30);

/// Feeds \c nof_windows windows of 64 TTIs at the given offset.
void feed_windows(ru_emulator_gps_slot_aligner& aligner, uint64_t& radio_slot, unsigned offset, unsigned nof_windows)
{
  for (unsigned i = 0; i != 64 * nof_windows; ++i, ++radio_slot) {
    aligner.on_tti_boundary(make_tti(radio_slot, offset));
  }
}

} // namespace

TEST(ru_emulator_gps_slot_aligner_test, offset_is_measured_after_the_settle_period_and_reported_once)
{
  timing_notifier_spy          next;
  std::vector<unsigned>        reported;
  ru_emulator_gps_slot_aligner aligner(
      next, ocudulog::fetch_basic_logger("TEST"), [&reported](unsigned offset) { reported.push_back(offset); });

  const unsigned gps_slot_offset = 1234;

  // One second of settle TTIs (2000 at 30 kHz) plus the measurement window.
  uint64_t radio_slot = 0;
  for (; radio_slot != 2100; ++radio_slot) {
    aligner.on_tti_boundary(make_tti(radio_slot, gps_slot_offset));
  }

  ASSERT_EQ(reported.size(), 1);
  ASSERT_EQ(reported.front(), gps_slot_offset);
  // Every notification was forwarded, including the ones consumed for measurement.
  ASSERT_EQ(next.nof_ttis, 2100);
}

TEST(ru_emulator_gps_slot_aligner_test, sub_half_slot_jitter_does_not_change_the_measured_offset)
{
  timing_notifier_spy          next;
  std::vector<unsigned>        reported;
  ru_emulator_gps_slot_aligner aligner(
      next, ocudulog::fetch_basic_logger("TEST"), [&reported](unsigned offset) { reported.push_back(offset); });

  const unsigned gps_slot_offset = 77;

  // Notification times jitter by up to 200 us (less than half a 30 kHz slot) around the ideal instant.
  uint64_t radio_slot = 0;
  for (; radio_slot != 2100; ++radio_slot) {
    const int64_t jitter_ns = (radio_slot % 5) * 50000 - 100000;
    aligner.on_tti_boundary(make_tti(radio_slot, gps_slot_offset, jitter_ns));
  }

  ASSERT_EQ(reported.size(), 1);
  ASSERT_EQ(reported.front(), gps_slot_offset);
}

TEST(ru_emulator_gps_slot_aligner_test, sustained_drift_re_aligns_the_offset)
{
  timing_notifier_spy          next;
  std::vector<unsigned>        reported;
  ru_emulator_gps_slot_aligner aligner(
      next, ocudulog::fetch_basic_logger("TEST"), [&reported](unsigned offset) { reported.push_back(offset); });

  const unsigned base       = 500;
  uint64_t       radio_slot = 0;

  // Settle (2000 TTIs) plus one full measurement window (64) at the base offset: the offset is established once.
  for (unsigned i = 0; i != 2000 + 64; ++i, ++radio_slot) {
    aligner.on_tti_boundary(make_tti(radio_slot, base));
  }
  ASSERT_EQ(reported.size(), 1);
  ASSERT_EQ(reported.front(), base);

  // The radio clock drifted by a slot (offset base+1). One window short of the confirmation time nothing changes.
  ASSERT_EQ(CONFIRM, 94U);
  feed_windows(aligner, radio_slot, base + 1, CONFIRM - 1);
  ASSERT_EQ(reported.size(), 1);

  // The window completing REALIGN_CONFIRM_MS of agreement adopts the new offset, once.
  feed_windows(aligner, radio_slot, base + 1, 1);
  ASSERT_EQ(reported.size(), 2);
  ASSERT_EQ(reported.back(), base + 1);
  feed_windows(aligner, radio_slot, base + 1, 2 * CONFIRM);
  ASSERT_EQ(reported.size(), 2);
}

TEST(ru_emulator_gps_slot_aligner_test, agreement_one_window_short_then_back_does_not_re_align)
{
  timing_notifier_spy          next;
  std::vector<unsigned>        reported;
  ru_emulator_gps_slot_aligner aligner(
      next, ocudulog::fetch_basic_logger("TEST"), [&reported](unsigned offset) { reported.push_back(offset); });

  const unsigned base       = 500;
  uint64_t       radio_slot = 0;
  for (unsigned i = 0; i != 2000 + 64; ++i, ++radio_slot) {
    aligner.on_tti_boundary(make_tti(radio_slot, base));
  }

  // Almost 3 s at base+1, one window back at base, then almost 3 s at base+1 again: the window at base restarts the
  // count, so neither run reaches the confirmation.
  feed_windows(aligner, radio_slot, base + 1, CONFIRM - 1);
  feed_windows(aligner, radio_slot, base, 1);
  feed_windows(aligner, radio_slot, base + 1, CONFIRM - 1);
  ASSERT_EQ(reported.size(), 1);
  ASSERT_EQ(reported.front(), base);
}

TEST(ru_emulator_gps_slot_aligner_test, offset_on_a_slot_boundary_does_not_flip_flop)
{
  timing_notifier_spy          next;
  std::vector<unsigned>        reported;
  ru_emulator_gps_slot_aligner aligner(
      next, ocudulog::fetch_basic_logger("TEST"), [&reported](unsigned offset) { reported.push_back(offset); });

  const unsigned base       = 628;
  uint64_t       radio_slot = 0;
  for (unsigned i = 0; i != 2000 + 64; ++i, ++radio_slot) {
    aligner.on_tti_boundary(make_tti(radio_slot, base));
  }
  ASSERT_EQ(reported.size(), 1);

  // The drift has brought the timing to half a slot (250 us) past the base offset: with +/-50 us of host jitter every
  // measurement rounds to base or base+1 about equally, so the window medians alternate between the two (the bench
  // case: offset 627/628 flipping three times in 0.2 s). Two minutes of it must not re-align once.
  uint32_t lcg = 12345;
  unsigned nof_windows_at_next = 0;
  for (unsigned w = 0; w != 3750; ++w) {
    unsigned at_next = 0;
    for (unsigned i = 0; i != 64; ++i, ++radio_slot) {
      lcg                     = lcg * 1664525U + 1013904223U;
      const int64_t jitter_ns = 250000 + int64_t(lcg >> 8) % 100001 - 50000;
      at_next += jitter_ns >= 250000;
      aligner.on_tti_boundary(make_tti(radio_slot, base, jitter_ns));
    }
    nof_windows_at_next += at_next > 32;
  }
  // The input really alternated (both medians occurred often), and the offset never moved.
  ASSERT_GT(nof_windows_at_next, 1000U);
  ASSERT_LT(nof_windows_at_next, 2750U);
  ASSERT_EQ(reported.size(), 1);
  ASSERT_EQ(reported.front(), base);

  // Once the drift has carried the timing clearly past the boundary, the new offset is adopted exactly once.
  feed_windows(aligner, radio_slot, base + 1, CONFIRM);
  ASSERT_EQ(reported.size(), 2);
  ASSERT_EQ(reported.back(), base + 1);
}

TEST(ru_emulator_gps_slot_aligner_test, single_window_excursion_does_not_re_align)
{
  timing_notifier_spy          next;
  std::vector<unsigned>        reported;
  ru_emulator_gps_slot_aligner aligner(
      next, ocudulog::fetch_basic_logger("TEST"), [&reported](unsigned offset) { reported.push_back(offset); });

  const unsigned base       = 500;
  uint64_t       radio_slot = 0;

  for (unsigned i = 0; i != 2000 + 64; ++i, ++radio_slot) {
    aligner.on_tti_boundary(make_tti(radio_slot, base));
  }
  ASSERT_EQ(reported.size(), 1);

  // A single window straddles the next slot boundary (one-off excursion), then it returns: no re-alignment.
  for (unsigned i = 0; i != 64; ++i, ++radio_slot) {
    aligner.on_tti_boundary(make_tti(radio_slot, base + 1));
  }
  for (unsigned i = 0; i != 64; ++i, ++radio_slot) {
    aligner.on_tti_boundary(make_tti(radio_slot, base));
  }
  ASSERT_EQ(reported.size(), 1);
  ASSERT_EQ(reported.front(), base);
}

TEST(ru_emulator_gps_slot_aligner_test, other_boundaries_are_forwarded)
{
  timing_notifier_spy          next;
  ru_emulator_gps_slot_aligner aligner(next, ocudulog::fetch_basic_logger("TEST"), [](unsigned) {});

  slot_point slot(to_numerology_value(subcarrier_spacing::kHz30), 5);
  aligner.on_ul_half_slot_boundary(slot);
  aligner.on_ul_full_slot_boundary(slot);

  ASSERT_EQ(next.nof_half_slots, 1);
  ASSERT_EQ(next.nof_full_slots, 1);
}
