// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 National University of Singapore
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ocudulog/logger.h"
#include "ocudu/ofh/timing/ofh_ota_symbol_boundary_notifier_manager.h"
#include "ocudu/support/executors/task_executor.h"
#include <array>
#include <atomic>
#include <cstdint>

namespace ocudu {

/// Realtime worker that generates OTA symbol notifications.
class ru_emulator_timing_notifier : public ofh::ota_symbol_boundary_notifier_manager
{
  enum class worker_status { running, stop_requested, stopped };

  ocudulog::basic_logger&                         logger;
  std::vector<ofh::ota_symbol_boundary_notifier*> ota_notifiers;
  task_executor&                                  executor;
  subcarrier_spacing                              scs;
  const unsigned                                  nof_symbols_per_slot;
  const unsigned                                  nof_symbols_per_sec;
  const unsigned                                  nof_slots_per_hyper_system_frame;
  const std::chrono::duration<double, std::nano>  symbol_duration;
  const std::chrono::nanoseconds                  sleep_time;
  unsigned                                        previous_symb_index = 0;
  std::atomic<worker_status>                      status{worker_status::running};

  /// OPENRU_TIMING_ABS_SLEEP=1: one absolute CLOCK_REALTIME sleep to the next symbol boundary instead of
  /// sleep_time (symbol/15) polls. Default off (upstream behaviour).
  const bool abs_sleep;

  /// In-thread timing measurement (both modes): 1 us buckets, last bucket = overflow.
  static constexpr unsigned nof_hist_buckets = 256;
  struct histogram {
    std::array<uint64_t, nof_hist_buckets> buckets = {};
    uint64_t                               count   = 0;
    int64_t                                max_ns  = 0;
    uint64_t                               nof_outliers = 0;
    void                                   add(int64_t ns);
    /// Value in us below which the given fraction of samples lies (bucket upper edge).
    unsigned                               percentile_us(double fraction) const;
  };
  histogram work_hist;
  histogram overshoot_hist;
  int64_t   next_report_ns       = 0;
  unsigned  nof_outliers_logged  = 0;

  /// Records a sample; logs it once per report period (max 50) when above the outlier threshold.
  void record(histogram& hist, int64_t ns, const char* what);
  /// Logs and resets the period histograms every 60 s (called from the timing thread).
  void report_if_due(int64_t now_mono_ns, bool force);
  /// Sleeps until the next symbol boundary (absolute CLOCK_REALTIME) and records the overshoot.
  void sleep_until_next_symbol();

public:
  ru_emulator_timing_notifier(ocudulog::basic_logger& logger_, task_executor& executor_, subcarrier_spacing scs_);

  /// Starts operation of the timing notifier.
  void start();

  /// Stops operation of the timing notifier.
  void stop();

  /// See interface for documentation.
  void subscribe(span<ofh::ota_symbol_boundary_notifier*> notifiers) override;

private:
  /// Main timing loop.
  void timing_loop();

  /// Polls the system time checking for the start of a new OTA symbol.
  void poll();

  /// Notifies the given slot symbol point through the registered notifiers.
  void notify_slot_symbol_point(ofh::slot_symbol_point slot);
};

} // namespace ocudu
