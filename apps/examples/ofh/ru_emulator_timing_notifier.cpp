// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 National University of Singapore
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ru_emulator_timing_notifier.h"
#include "ocudu/ran/cyclic_prefix.h"
#include "fmt/format.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <future>
#include <thread>

using namespace ocudu;
using namespace ofh;

/// Difference between Unix seconds to GPS seconds.
/// GPS epoch: 1980.1.6 00:00:00 (UTC); Unix time epoch: 1970:1.1 00:00:00 UTC
/// Last leap second added in 31st Dec 2016 by IERS.
/// 1970:1.1 - 1980.1.6: 3657 days
/// 3657*24*3600=315 964 800 seconds (Unix seconds value at 1980.1.6 00:00:00 (UTC))
/// There are 18 leap seconds inserted after 1980.1.6 00:00:00 (UTC), which means GPS is 18 seconds larger.
static constexpr uint64_t UNIX_TO_GPS_SECONDS_OFFSET = 315964800ULL - 18ULL;

/// Offset for converting from UTC to GPS time including Alpha and Beta parameters.
static std::chrono::nanoseconds gps_offset;

namespace {

/// A GPS clock implementation.
struct gps_clock {
  using duration   = std::chrono::nanoseconds;
  using rep        = duration::rep;
  using period     = duration::period;
  using time_point = std::chrono::time_point<gps_clock>;
  // static constexpr bool is_steady = false;

  static time_point now()
  {
    ::timespec ts;
    ::clock_gettime(CLOCK_REALTIME, &ts);

    time_point now(std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec));

    return now - gps_offset;
  }
};

} // namespace

static int64_t clock_ns(clockid_t id)
{
  ::timespec ts;
  ::clock_gettime(id, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

/// Samples above this are logged individually (rate limited) and counted as outliers.
static constexpr int64_t TIMING_OUTLIER_NS     = 20000;
static constexpr int64_t TIMING_REPORT_NS      = 60LL * 1000000000LL;
static constexpr unsigned TIMING_MAX_OUTLIER_LOGS = 50;

/// Calculates the fractional part inside a second from the given time point.
static std::chrono::nanoseconds calculate_ns_fraction_from(gps_clock::time_point tp)
{
  auto tp_sec = std::chrono::time_point_cast<std::chrono::seconds>(tp);
  return tp - tp_sec;
}

/// Returns the symbol index inside a second.
static unsigned get_symbol_index(std::chrono::nanoseconds                 fractional_ns,
                                 std::chrono::duration<double, std::nano> symbol_duration)
{
  // Perform operation with enough precision to avoid rounding errors when the amount of fractional nanoseconds is big.
  return fractional_ns / symbol_duration;
}

/// Returns current system slot.
static inline uint64_t get_current_system_slot(gps_clock::time_point    tp,
                                               std::chrono::nanoseconds slot_duration,
                                               unsigned                 nof_slots_per_hyper_system_frame)
{
  // Get the time since the epoch.
  auto time_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return (time_since_epoch / slot_duration) % nof_slots_per_hyper_system_frame;
}

/// Returns the difference between cur and prev taking into account a potential wrap arounds of the values.
static unsigned circular_distance(unsigned cur, unsigned prev, unsigned size)
{
  return (cur >= prev) ? (cur - prev) : (size + cur - prev);
}

ru_emulator_timing_notifier::ru_emulator_timing_notifier(ocudulog::basic_logger& logger_,
                                                         ocudu::task_executor&   executor_,
                                                         subcarrier_spacing      scs_) :
  logger(logger_),
  executor(executor_),
  scs(scs_),
  nof_symbols_per_slot(get_nsymb_per_slot(cyclic_prefix::NORMAL)),
  nof_symbols_per_sec(nof_symbols_per_slot * get_nof_slots_per_subframe(scs) * NOF_SUBFRAMES_PER_FRAME * 100),
  nof_slots_per_hyper_system_frame(slot_point(scs, 0).nof_slots_per_hyper_system_frame()),
  symbol_duration(1e9 / nof_symbols_per_sec),
  sleep_time(std::chrono::duration_cast<std::chrono::nanoseconds>(symbol_duration) / 15),
  abs_sleep([] {
    const char* v = std::getenv("OPENRU_TIMING_ABS_SLEEP");
    return (v != nullptr) && (std::strcmp(v, "1") == 0);
  }())
{
  // The GPS time epoch starts on 1980.1.6 so make sure that the system time is set after this date.
  // For simplicity reasons, only allow dates after 1981.
  ::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  ::tm     utc_time;
  ::gmtime_r(&tt, &utc_time);
  int current_year = utc_time.tm_year + 1900;
  report_error_if_not(current_year >= 1981,
                      "The Open FrontHaul standard uses GPS time for synchronization. Make sure that system time is "
                      "set after the year 1981 since the GPS time epoch starts on 1980.1.6");

  gps_offset = std::chrono::seconds(UNIX_TO_GPS_SECONDS_OFFSET);
}

void ru_emulator_timing_notifier::start()
{
  logger.info("Starting the realtime timing notifier (sleep mode: {})",
              abs_sleep ? "absolute, once per symbol" : "polling, symbol/15");

  std::promise<void> p;
  std::future<void>  fut = p.get_future();

  if (!executor.defer([this, &p]() {
        status.store(worker_status::running, std::memory_order_relaxed);
        // Signal start() caller thread that the operation is complete.
        p.set_value();

        auto ns_fraction    = calculate_ns_fraction_from(gps_clock::now());
        previous_symb_index = get_symbol_index(ns_fraction, symbol_duration);
        timing_loop();
      })) {
    report_fatal_error("Unable to start the realtime timing worker");
  }

  // Block waiting for timing executor to start.
  fut.wait();

  logger.info("Started the realtime timing worker");
}

void ru_emulator_timing_notifier::stop()
{
  logger.info("Requesting stop of the realtime timing worker");
  status.store(worker_status::stop_requested, std::memory_order_relaxed);

  // Wait for the timing thread to stop - this line also introduces a happens-before relationship with the clear on
  // ota_notifier.
  while (status.load(std::memory_order_acquire) != worker_status::stopped) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Clear the subscribed notifiers.
  ota_notifiers.clear();

  logger.info("Stopped the realtime timing worker");
}

void ru_emulator_timing_notifier::timing_loop()
{
  next_report_ns = clock_ns(CLOCK_MONOTONIC) + TIMING_REPORT_NS;
  while (OCUDU_LIKELY(status.load(std::memory_order_relaxed) == worker_status::running)) {
    poll();
  }
  report_if_due(clock_ns(CLOCK_MONOTONIC), true);

  // Acquire/Release semantics - ota_notifiers is cleared and destructed by a different thread.
  status.store(worker_status::stopped, std::memory_order_release);
}

void ru_emulator_timing_notifier::poll()
{
  const std::chrono::duration<double, std::nano> slot_duration(1e6 / pow2(to_numerology_value(scs)));

  const int64_t t_wake = clock_ns(CLOCK_MONOTONIC);
  auto          now    = gps_clock::now();
  auto ns_fraction = calculate_ns_fraction_from(now);

  unsigned current_symbol_index = get_symbol_index(ns_fraction, symbol_duration);
  unsigned delta                = circular_distance(current_symbol_index, previous_symb_index, nof_symbols_per_sec);
  previous_symb_index           = current_symbol_index;

  // Are we still in the same symbol as before?
  if (delta == 0) {
    if (abs_sleep) {
      // Woke a hair before the boundary: sleep to it again.
      sleep_until_next_symbol();
      return;
    }
    const int64_t t0 = clock_ns(CLOCK_MONOTONIC);
    std::this_thread::sleep_for(sleep_time);
    record(overshoot_hist, clock_ns(CLOCK_MONOTONIC) - t0 - sleep_time.count(), "sleep overshoot");
    return;
  }

  // Check if we have missed more than one symbol.
  if (OCUDU_UNLIKELY(delta > 1)) {
    logger.info("RU emulator timing worker woke up late, skipped '{}' symbols", delta);
  }
  if (OCUDU_UNLIKELY(delta >= nof_symbols_per_slot)) {
    logger.warning("RU emulator timing worker woke up late, sleep time has been '{}us', or equivalently, '{}' symbols",
                   std::chrono::duration_cast<std::chrono::microseconds>(delta * symbol_duration).count(),
                   delta);
  }

  uint64_t slot_count = get_current_system_slot(
      now, std::chrono::duration_cast<std::chrono::nanoseconds>(slot_duration), nof_slots_per_hyper_system_frame);

  slot_point        current_slot(to_numerology_value(scs), slot_count);
  slot_symbol_point symbol_point(current_slot, current_symbol_index % nof_symbols_per_slot, nof_symbols_per_slot);

  notify_slot_symbol_point(symbol_point);

  const int64_t t_done = clock_ns(CLOCK_MONOTONIC);
  record(work_hist, t_done - t_wake, "symbol work");
  report_if_due(t_done, false);
  if (abs_sleep) {
    sleep_until_next_symbol();
  }
}

void ru_emulator_timing_notifier::sleep_until_next_symbol()
{
  const int64_t rt_ns   = clock_ns(CLOCK_REALTIME);
  const int64_t gps_ns  = rt_ns - std::chrono::duration_cast<std::chrono::nanoseconds>(gps_offset).count();
  const int64_t frac_ns = gps_ns % 1000000000LL;
  const double  sym_ns  = symbol_duration.count();
  // Next boundary inside (or at the end of) the current second, rounded up to whole nanoseconds.
  const auto    next_ns = static_cast<int64_t>(std::ceil((std::floor(frac_ns / sym_ns) + 1.0) * sym_ns));
  const int64_t target  = rt_ns - frac_ns + next_ns;

  ::timespec ts;
  ts.tv_sec  = target / 1000000000LL;
  ts.tv_nsec = target % 1000000000LL;
  while (::clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
  record(overshoot_hist, clock_ns(CLOCK_REALTIME) - target, "sleep overshoot");
}

void ru_emulator_timing_notifier::histogram::add(int64_t ns)
{
  const int64_t us = (ns < 0) ? 0 : ns / 1000;
  ++buckets[(us < static_cast<int64_t>(nof_hist_buckets) - 1) ? us : nof_hist_buckets - 1];
  ++count;
  max_ns = std::max(max_ns, ns);
  if (ns > TIMING_OUTLIER_NS) {
    ++nof_outliers;
  }
}

unsigned ru_emulator_timing_notifier::histogram::percentile_us(double fraction) const
{
  const auto target = static_cast<uint64_t>(std::ceil(fraction * static_cast<double>(count)));
  uint64_t   acc    = 0;
  for (unsigned i = 0; i != nof_hist_buckets; ++i) {
    acc += buckets[i];
    if (acc >= target) {
      return i + 1;
    }
  }
  return nof_hist_buckets;
}

void ru_emulator_timing_notifier::record(histogram& hist, int64_t ns, const char* what)
{
  hist.add(ns);
  if ((ns > TIMING_OUTLIER_NS) && (nof_outliers_logged < TIMING_MAX_OUTLIER_LOGS)) {
    ++nof_outliers_logged;
    logger.info("Timing worker {} outlier: {} us", what, ns / 1000);
  }
}

void ru_emulator_timing_notifier::report_if_due(int64_t now_mono_ns, bool force)
{
  if (!force && (now_mono_ns < next_report_ns)) {
    return;
  }
  next_report_ns = now_mono_ns + TIMING_REPORT_NS;
  auto line      = [](const histogram& h) {
    return fmt::format("n={} p50<{}us p99<{}us p99.9<{}us max={}us >20us={}",
                       h.count,
                       h.percentile_us(0.5),
                       h.percentile_us(0.99),
                       h.percentile_us(0.999),
                       h.max_ns / 1000,
                       h.nof_outliers);
  };
  logger.info("Timing worker stats ({}): symbol work {}; sleep overshoot {}",
              abs_sleep ? "abs-sleep" : "poll-sleep",
              line(work_hist),
              line(overshoot_hist));
  work_hist           = {};
  overshoot_hist      = {};
  nof_outliers_logged = 0;
}

void ru_emulator_timing_notifier::notify_slot_symbol_point(slot_symbol_point slot)
{
  for (auto* notifier : ota_notifiers) {
    notifier->on_new_symbol({slot, {}});
  }
}

void ru_emulator_timing_notifier::subscribe(span<ota_symbol_boundary_notifier*> notifiers)
{
  // The defer() call in start() synchronizes the contents of ota_notifiers with the worker thread.
  ota_notifiers.assign(notifiers.begin(), notifiers.end());
}
