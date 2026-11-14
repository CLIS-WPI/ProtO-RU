// SPDX-FileCopyrightText: Copyright (C) 2026 OpenRU / WPI
// SPDX-License-Identifier: AGPL-3.0-or-later
/// \file openru_trace.h
/// \brief Lightweight OPENRU_TRACE CSV probes (off by default).
///
/// Set env \c OPENRU_TRACE to a writable CSV path to enable. Emits one row per
/// probe: slot, stage, unix_ns. Stages: uplane_rx, decompress_done, ifft_done,
/// radio_handoff. Compute-time only — not a fronthaul delay profile.
#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>

namespace ocudu {
namespace openru_trace {

inline std::mutex& mutex()
{
  static std::mutex m;
  return m;
}

inline FILE* file()
{
  static FILE*     fp   = nullptr;
  static bool      init = false;
  if (init) {
    return fp;
  }
  init             = true;
  const char* path = std::getenv("OPENRU_TRACE");
  if ((path == nullptr) || (path[0] == '\0')) {
    return nullptr;
  }
  fp = std::fopen(path, "w");
  if (fp != nullptr) {
    std::fprintf(fp, "unix_ns,sfn,slot,stage,label\n");
    std::fflush(fp);
  }
  return fp;
}

/// \brief Record a compute-path probe. No-op unless OPENRU_TRACE is set.
inline void emit(unsigned sfn, unsigned slot, const char* stage, const char* label = "")
{
  FILE* fp = file();
  if (fp == nullptr) {
    return;
  }
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  std::lock_guard<std::mutex> lock(mutex());
  std::fprintf(fp, "%lld,%u,%u,%s,%s\n", static_cast<long long>(ns), sfn, slot, stage, label);
  std::fflush(fp);
}

} // namespace openru_trace
} // namespace ocudu
