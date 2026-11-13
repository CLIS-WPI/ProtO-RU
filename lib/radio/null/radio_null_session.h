// SPDX-FileCopyrightText: Copyright (C) 2026 CLIS-WPI / OpenRU
// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from ProtO-RU/OCUDU radio interfaces (AGPLv3).

#pragma once

#include "ocudu/radio/radio_session.h"
#include "ocudu/support/executors/task_executor.h"
#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ocudu {

/// Null radio: dumps TX IQ to .cf32 files; RX from file or zeros. No RF hardware.
class radio_session_null_impl : public radio_session, public radio_management_plane
{
public:
  radio_session_null_impl(const radio_configuration::radio& config,
                          task_executor&                    async_task_executor,
                          radio_event_notifier&             notification_handler);
  ~radio_session_null_impl() override;

  bool is_successful() const { return successful; }

  radio_management_plane& get_management_plane() override { return *this; }
  baseband_gateway&       get_baseband_gateway(unsigned stream_id) override;
  baseband_gateway_timestamp read_current_time() override;
  void                       start(baseband_gateway_timestamp init_time) override;
  void                       stop() override;

  bool set_tx_gain(unsigned /*port_id*/, double /*gain_dB*/) override { return true; }
  bool set_rx_gain(unsigned /*port_id*/, double /*gain_dB*/) override { return true; }
  bool set_tx_freq(unsigned /*stream_id*/, double /*center_freq_Hz*/) override { return true; }
  bool set_rx_freq(unsigned /*stream_id*/, double /*center_freq_Hz*/) override { return true; }

private:
  class null_gateway;

  ocudulog::basic_logger& logger;
  bool                    successful = false;
  std::string             tx_dump_dir;
  std::string             rx_feed_dir;
  double                  sampling_rate_Hz = 0.0;
  unsigned                nof_tx_ports     = 0;
  unsigned                nof_rx_ports     = 0;
  std::atomic<baseband_gateway_timestamp> current_time{0};
  std::vector<std::unique_ptr<null_gateway>> gateways;
  std::mutex                                 dump_mutex;
  bool                                       started = false;
  baseband_gateway_timestamp                 start_ts{0};

  static bool parse_args(const std::string& args, std::string& tx_dump, std::string& rx_feed);
  void         write_sidecar_json() const;
};

} // namespace ocudu
