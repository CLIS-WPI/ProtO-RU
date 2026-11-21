// SPDX-FileCopyrightText: Copyright (C) 2026 CLIS-WPI / OpenRU
// SPDX-License-Identifier: AGPL-3.0-or-later

#pragma once

#include <cstdint>

namespace ocudu {

/// Slot-anchored RX feed gate for the null radio.
///
/// Playback is held until the DU/radio slot residual has been constant for
/// `stable_slots` consecutive slots and the replay marker exists. It then starts
/// only at symbol 0 of the target wire slot; a target that is already past fails
/// closed (`missed()`) instead of silently shifting the feed.
class null_rx_gate
{
public:
  enum class action { hold, play };

  struct configuration {
    /// Target wire DU slot modulo the OFH period. Negative disables the gate.
    int      start_du_slot    = -1;
    /// Consecutive stable slots required before arming.
    unsigned stable_slots     = 100;
    /// OFH wire-slot period (256 frames x slots per frame).
    unsigned period_slots     = 5120;
  };

  null_rx_gate() = default;
  explicit null_rx_gate(const configuration& cfg_) : cfg(cfg_) {}

  /// Called once per slot boundary (or with new_slot=false for a mid-slot call).
  action update(unsigned du_slot, unsigned radio_slot, bool marker_exists, bool new_slot)
  {
    if (cfg.start_du_slot < 0) {
      state          = action::play;
      started_slot_  = du_slot;
      return action::play;
    }
    if (!new_slot) {
      return state == action::play ? action::play : action::hold;
    }

    if (!have_delta) {
      stable_delta_ = wrap_delta(du_slot, radio_slot);
      have_delta    = true;
      stable_count_ = 1;
      first_du_slot = du_slot;
    } else {
      int delta = wrap_delta(du_slot, radio_slot);
      if (delta == stable_delta_) {
        if (stable_count_ < cfg.stable_slots) {
          ++stable_count_;
        }
      } else {
        stable_delta_ = delta;
        stable_count_ = 1;
        armed_        = false;
      }
    }

    if (!armed_ && stable_count_ >= cfg.stable_slots) {
      armed_ = true;
    }

    if (missed_) {
      return action::hold;
    }

    unsigned target = static_cast<unsigned>(cfg.start_du_slot) % cfg.period_slots;
    if (state == action::play) {
      return action::play;
    }

    if (armed_ && marker_exists) {
      if (du_slot == target) {
        state = action::play;
        started_slot_ = du_slot;
        return action::play;
      }
      // Fail closed if the target has already passed in this period. The window
      // is limited to half a period so a target just before the first observed
      // slot is not treated as missed.
      unsigned since_first = (du_slot - first_du_slot + cfg.period_slots) % cfg.period_slots;
      unsigned target_off  = (target - first_du_slot + cfg.period_slots) % cfg.period_slots;
      if (target_off != 0 && since_first > target_off && since_first - target_off < cfg.period_slots / 2) {
        missed_ = true;
      }
    }
    return action::hold;
  }

  bool started() const { return state == action::play; }
  bool missed() const { return missed_; }
  bool armed() const { return armed_; }
  unsigned stable_count() const { return stable_count_; }
  int stable_delta() const { return stable_delta_; }
  unsigned started_slot() const { return started_slot_; }

private:
  int wrap_delta(unsigned du, unsigned radio) const
  {
    int period = static_cast<int>(cfg.period_slots);
    int delta  = static_cast<int>(du) - static_cast<int>(radio);
    return ((delta + period / 2) % period + period) % period - period / 2;
  }

  configuration cfg;
  action        state{action::hold};
  bool          have_delta{false};
  bool          armed_{false};
  bool          missed_{false};
  int           stable_delta_{0};
  unsigned      stable_count_{0};
  unsigned      first_du_slot{0};
  unsigned      started_slot_{0};
};

} // namespace ocudu
