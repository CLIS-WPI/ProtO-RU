// SPDX-FileCopyrightText: Copyright (C) 2026 National University of Singapore
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ru_emulator_gps_slot_alignment.h"
#include "ocudu/ocudulog/logger.h"
#include "ocudu/ru/ru_timing_notifier.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <functional>

namespace ocudu {

/// \brief Timing-notifier decorator that measures and tracks the GPS slot offset at runtime.
///
/// Sits between the SDR Radio Unit and the TTI orchestrator. Every TTI notification carries the radio slot and the
/// host time the lower PHY stamped on it; pairing the two yields the offset between the radio slot numbering and the
/// GPS wire numbering (see \ref gps_slot_alignment) - the same measurement the original ProtO-RU took inside its
/// patched lower PHY at radio start. Because the pairing uses the host wall clock, it works with any radio clock
/// source (gpsdo, external or internal), as long as the host clock is GPS/NTP-accurate.
///
/// The first second of notifications is skipped (the downlink pipeline fill and the radio's deferred stream start make
/// them fire early); the offset is then established as the median of a measurement window and applied through the
/// callback. Measurement then continues for the lifetime of the run: with a disciplined radio clock (gpsdo, or a
/// shared external reference) the radio and host timelines do not drift, so the median never changes and the offset is
/// reported exactly once. With the radio on its free-running internal clock, the radio sample clock drifts against the
/// host/GPS timeline the offset is keyed to; the tracker re-applies the offset once the windowed median has agreed on a
/// new value for \ref REALIGN_CONFIRM_MS of consecutive windows, keeping the wire-slot labelling correct as the drift
/// accumulates. The confirmation spans seconds, not two windows: while the drift crosses a slot boundary, host-clock
/// jitter makes consecutive window medians alternate between the two neighbouring offsets for minutes, and a two-window
/// gate re-labelled the slots back and forth (observed on the bench: 3 flips in 0.2 s, and 55 in two minutes), each
/// flip dropping the attached UEs. A window back at the applied offset restarts the confirmation. Each re-alignment is a one-slot step, so it is a far smaller disturbance than the unbounded
/// labelling error a frozen offset would accrue - though a disciplined clock remains the only way to remove the drift
/// (and the carrier CFO it implies) entirely.
class ru_emulator_gps_slot_aligner : public ru_timing_notifier
{
public:
  ru_emulator_gps_slot_aligner(ru_timing_notifier&           next_,
                               ocudulog::basic_logger&       logger_,
                               std::function<void(unsigned)> on_offset_measured_) :
    next(next_), logger(logger_), on_offset_measured(std::move(on_offset_measured_))
  {
  }

  // See interface for documentation.
  void on_tti_boundary(const tti_boundary_context& context) override
  {
    measure(context);
    next.on_tti_boundary(context);
  }

  // See interface for documentation.
  void on_ul_half_slot_boundary(slot_point slot) override { next.on_ul_half_slot_boundary(slot); }

  // See interface for documentation.
  void on_ul_full_slot_boundary(slot_point slot) override { next.on_ul_full_slot_boundary(slot); }

private:
  /// \brief Lead between a TTI notification's time point and the notified slot's air time.
  ///
  /// The lower PHY throttles downlink baseband processing to stay one millisecond ahead of reception (see
  /// rx_to_tx_max_delay in the lower PHY factory), and stamps the notification with now + TTI advance while the slot
  /// airs at now + TTI advance + this lead. It is the same quantity the original ProtO-RU compensated with its
  /// rx_to_tx_max_delay calibration term.
  static constexpr std::chrono::milliseconds DL_PROCESSING_TO_AIR_LEAD{1};

  /// Number of TTI measurements each windowed median is taken over.
  static constexpr unsigned NOF_MEASUREMENTS = 64;

public:
  /// Time, in milliseconds, consecutive windows must agree on a new offset before it is adopted.
  static constexpr unsigned REALIGN_CONFIRM_MS = 3000;

  /// Number of consecutive windows that make up \ref REALIGN_CONFIRM_MS at the given subcarrier spacing.
  static constexpr unsigned nof_confirm_windows(subcarrier_spacing scs)
  {
    const unsigned slots = REALIGN_CONFIRM_MS * get_nof_slots_per_subframe(scs);
    return (slots + NOF_MEASUREMENTS - 1) / NOF_MEASUREMENTS;
  }

private:

  void measure(const tti_boundary_context& context)
  {
    const subcarrier_spacing scs = context.slot.scs();

    // Skip the first second of notifications.
    const uint64_t settle_ttis = uint64_t(1000U) * get_nof_slots_per_subframe(scs);
    if (nof_ttis++ < settle_ttis) {
      return;
    }

    const slot_point radio_slot(to_numerology_value(scs),
                                context.slot.count() % gps_slot_alignment::nof_slots_per_ofh_period(scs));
    samples[nof_samples++] =
        gps_slot_alignment::measure_gps_slot_offset(context.time_point + DL_PROCESSING_TO_AIR_LEAD, radio_slot);
    if (nof_samples != samples.size()) {
      return;
    }
    nof_samples = 0;

    // Windowed median rejects the host-clock jitter on individual measurements.
    std::array<unsigned, NOF_MEASUREMENTS> sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    const unsigned median = sorted[sorted.size() / 2];

    // Establish the offset on the first full window.
    if (!applied) {
      apply(median);
      logger.info("Aligned the radio slot numbering to the GPS wire numbering: slot offset '{}'. Tracking the radio "
                  "clock drift against the host clock; keep the host clock GPS/NTP-synchronized.",
                  median);
      return;
    }

    // Already aligned and the median has not moved: a pending candidate, if any, was jitter.
    if (median == applied_offset) {
      nof_pending_windows = 0;
      return;
    }

    // The median moved. Count consecutive windows agreeing on the same new value; adopt it only after
    // REALIGN_CONFIRM_MS of them, so the alternating medians of a boundary crossing never re-label the slots.
    if (nof_pending_windows != 0 && median == pending_offset) {
      ++nof_pending_windows;
    } else {
      pending_offset      = median;
      nof_pending_windows = 1;
    }
    if (nof_pending_windows < nof_confirm_windows(scs)) {
      return;
    }

    const unsigned previous = applied_offset;
    apply(median);
    ++nof_realignments;
    logger.info("Re-aligned the radio slot numbering to the GPS wire numbering: slot offset '{}' -> '{}' after {} "
                "consecutive windows ({} ms) agreed; re-alignment #{} (radio clock drift). Use a gpsdo or shared "
                "external reference to remove the drift.",
                previous,
                median,
                nof_pending_windows,
                nof_pending_windows * NOF_MEASUREMENTS / get_nof_slots_per_subframe(scs),
                nof_realignments);
    nof_pending_windows = 0;
  }

  void apply(unsigned offset)
  {
    applied        = true;
    applied_offset = offset;
    on_offset_measured(offset);
  }

  ru_timing_notifier&                    next;
  ocudulog::basic_logger&                logger;
  std::function<void(unsigned)>          on_offset_measured;
  std::array<unsigned, NOF_MEASUREMENTS> samples{};
  uint64_t                               nof_ttis    = 0;
  unsigned                               nof_samples = 0;
  /// Currently applied offset and whether one has been applied yet.
  bool     applied        = false;
  unsigned applied_offset = 0;
  /// Candidate offset and the number of consecutive windows that agreed on it (drift-tracking hysteresis).
  unsigned pending_offset      = 0;
  unsigned nof_pending_windows = 0;
  /// Number of re-alignments since the start, for the log.
  unsigned nof_realignments = 0;
};

} // namespace ocudu
