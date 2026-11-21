// SPDX-FileCopyrightText: Copyright (C) 2026 CLIS-WPI / OpenRU
// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from ProtO-RU/OCUDU radio interfaces (AGPLv3).

#pragma once

#include "null_rx_gate.h"
#include "ocudu/radio/radio_session.h"
#include "ocudu/support/executors/task_executor.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ocudu {

/// Null radio: dumps TX IQ to .cf32; RX from file or zeros. No RF hardware.
/// Clock advances only in receive(). Default pace=realtime (system_clock).
class radio_session_null_impl : public radio_session, public radio_management_plane
{
public:
  enum class pace_mode { realtime, free };

  radio_session_null_impl(const radio_configuration::radio& config,
                          task_executor&                    async_task_executor,
                          radio_event_notifier&             notification_handler);
  ~radio_session_null_impl() override;

  bool is_successful() const { return successful; }

  radio_management_plane&        get_management_plane() override { return *this; }
  baseband_gateway&              get_baseband_gateway(unsigned stream_id) override;
  baseband_gateway_timestamp     read_current_time() override;
  void                           start(baseband_gateway_timestamp init_time) override;
  void                           stop() override;

  bool set_tx_gain(unsigned /*port_id*/, double /*gain_dB*/) override { return true; }
  bool set_rx_gain(unsigned /*port_id*/, double /*gain_dB*/) override { return true; }
  bool set_tx_freq(unsigned /*stream_id*/, double /*center_freq_Hz*/) override { return true; }
  bool set_rx_freq(unsigned /*stream_id*/, double /*center_freq_Hz*/) override { return true; }

private:
  class null_gateway;
  struct tx_block {
    baseband_gateway_timestamp ts{0};
    unsigned                   nof_samples{0};
    unsigned                   nof_channels{0};
    bool                       is_empty{false};
    std::vector<float>         interleaved; // ch-major: ch0 all samples, then ch1...
  };

  ocudulog::basic_logger& logger;
  bool                    successful = false;
  std::string             tx_dump_dir;
  std::string             rx_feed_dir;
  std::string             rx_start_marker;
  int                     rx_start_du_slot = -1;
  unsigned                rx_start_stable_slots = 100;
  null_rx_gate            rx_gate;
  unsigned                rx_last_du_abs = ~0U;
  std::atomic<uint64_t>   rx_samples_read{0};
  bool                    rx_missed_logged = false;
  pace_mode               pace = pace_mode::realtime;
  double                  sampling_rate_Hz = 0.0;
  unsigned                nof_tx_ports     = 0;
  unsigned                nof_rx_ports     = 0;
  unsigned                scs_khz          = 30;
  unsigned                samples_per_slot = 0;
  /// 0 = unlimited (still fails closed if the dump filesystem fills).
  uint64_t                max_dump_bytes = 0;
  /// 0 = unlimited. Caps how many slots are written to .cf32 / tx_index.
  uint64_t                max_dump_slots = 0;
  /// Start dumping only after the DU/OFH-vs-radio slot residual has been constant
  /// for this many consecutive slots. 0 disables the gate (legacy goldens).
  unsigned                dump_start_stable_slots = 100;
  /// Wire DU slot (mod OFH period) the dump window must start at. -1 keeps the
  /// legacy "next SFN 0.0" gate.
  int                     dump_start_du_slot = -1;
  /// Optional marker file: the gate is disarmed until the replayer creates it,
  /// so the window is anchored to the pcap content rather than free-running time.
  std::string             dump_start_marker;
  /// Replayer C-plane lead in microseconds (echoed into iq_meta.json).
  unsigned                align_cplane_lead_us = 0;

  std::atomic<baseband_gateway_timestamp> current_time{0};
  baseband_gateway_timestamp              start_ts{0};
  std::chrono::system_clock::time_point   wall_start{};
  bool                                    started = false;

  std::vector<std::unique_ptr<null_gateway>> gateways;

  // Async TX dump writer (no blocking I/O on transmit path).
  static constexpr size_t TX_RING_CAP = 64;
  std::vector<tx_block>   tx_ring;
  std::mutex              tx_mu;
  std::condition_variable tx_cv;
  size_t                  tx_head{0};
  size_t                  tx_tail{0};
  size_t                  tx_count{0};
  bool                    tx_stop{false};
  std::thread             tx_writer;
  std::ofstream           tx_index; // per buffer; slots in DU/OTA (GPS/OFH) numbering
  baseband_gateway_timestamp tx_file_cursor{0};
  baseband_gateway_timestamp dump_first_ts{0}; // first written block timestamp

  // Dump accounting (writer thread).
  std::atomic<bool>     dump_cap_reached{false};
  std::atomic<uint64_t> dump_bytes_written{0};
  std::atomic<uint64_t> dump_slots_written{0};
  std::atomic<uint64_t> dump_dropped_buffers{0};
  int                   last_du_sfn{-1};
  int                   last_du_slot{-1};
  int                   first_du_sfn{-1};
  int                   first_du_slot{-1};
  uint64_t              first_dump_sample_offset{0}; // file-relative (always 0)
  uint64_t              first_dump_abs_sample_offset{0}; // ts - start_ts at first write
  bool                  dump_started{false};
  int                   stable_delta{0};
  bool                  have_stable_delta{false};
  unsigned              delta_stable_count{0};
  bool                  marker_seen{false};
  std::atomic<uint64_t> dump_prestart_dropped{0};
  std::string           dump_stop_reason;

  void enqueue_tx(tx_block&& blk);
  void writer_loop();
  void write_sidecar_json() const;
  /// Radio-local slot (free-running from start_ts); debug only.
  void slot_of_radio(baseband_gateway_timestamp ts, unsigned& sfn, unsigned& slot) const;
  /// DU/OTA (= GPS/OFH wire, SFN mod 256) slot for air time of sample ts.
  void slot_of_du(baseband_gateway_timestamp ts, unsigned& sfn, unsigned& slot) const;

  static bool parse_args(const std::string& args,
                         std::string&       tx_dump,
                         std::string&       rx_feed,
                         std::string&       rx_start_marker,
                         int&               rx_start_du_slot,
                         unsigned&          rx_start_stable_slots,
                         pace_mode&         pace,
                         unsigned&          scs_khz,
                         uint64_t&          max_dump_bytes,
                         uint64_t&          max_dump_slots,
                         unsigned&          dump_start_stable_slots,
                         int&               dump_start_du_slot,
                         std::string&       dump_start_marker,
                         unsigned&          align_cplane_lead_us);
};

} // namespace ocudu
