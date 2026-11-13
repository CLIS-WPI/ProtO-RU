// SPDX-FileCopyrightText: Copyright (C) 2026 CLIS-WPI / OpenRU
// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from ProtO-RU/OCUDU radio interfaces (AGPLv3).

#include "radio_null_session.h"
#include "ocudu/ocudulog/ocudulog.h"
#include "ocudu/adt/complex.h"
#include "ocudu/gateways/baseband/baseband_gateway.h"
#include "ocudu/gateways/baseband/baseband_gateway_receiver.h"
#include "ocudu/gateways/baseband/baseband_gateway_transmitter.h"
#include "ocudu/gateways/baseband/baseband_gateway_transmitter_metadata.h"
#include "ocudu/support/error_handling.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_reader.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_writer.h"
#include "ocudu/radio/radio_event_notifier.h"
#include <cmath>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <sys/stat.h>

using namespace ocudu;

namespace {

std::string extract_kv(const std::string& args, const std::string& key)
{
  // Comma-separated key=value pairs.
  std::string needle = key + "=";
  size_t      pos    = args.find(needle);
  if (pos == std::string::npos) {
    return {};
  }
  pos += needle.size();
  size_t end = args.find(',', pos);
  return args.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
}

} // namespace

class radio_session_null_impl::null_gateway : public baseband_gateway,
                                              public baseband_gateway_transmitter,
                                              public baseband_gateway_receiver
{
public:
  null_gateway(radio_session_null_impl& parent_, unsigned /*stream_id_*/, unsigned nof_tx_ch, unsigned nof_rx_ch) :
    parent(parent_), nof_tx(nof_tx_ch), nof_rx(nof_rx_ch)
  {
    tx_files.resize(nof_tx);
    rx_files.resize(nof_rx);
    rx_eof.resize(nof_rx, false);
  }

  baseband_gateway_transmitter& get_transmitter() override { return *this; }
  baseband_gateway_receiver&    get_receiver() override { return *this; }

  void open_files(const std::string& tx_dir, const std::string& rx_dir)
  {
    namespace fs = std::filesystem;
    if (!tx_dir.empty()) {
      fs::create_directories(tx_dir);
      for (unsigned p = 0; p < nof_tx; ++p) {
        const auto path = fs::path(tx_dir) / ("port" + std::to_string(p) + ".cf32");
        tx_files[p]     = std::ofstream(path, std::ios::binary | std::ios::trunc);
      }
    }
    if (!rx_dir.empty()) {
      for (unsigned p = 0; p < nof_rx; ++p) {
        const auto path = fs::path(rx_dir) / ("port" + std::to_string(p) + ".cf32");
        if (fs::exists(path)) {
          rx_files[p] = std::ifstream(path, std::ios::binary);
        }
      }
    }
  }

  void transmit(const baseband_gateway_buffer_reader& data, const baseband_gateway_transmitter_metadata& /*md*/) override
  {
    std::lock_guard<std::mutex> lock(parent.dump_mutex);
    const unsigned              ns = data.get_nof_samples();
    const unsigned              nc = std::min(data.get_nof_channels(), nof_tx);
    std::vector<float>          interleaved;
    interleaved.resize(static_cast<size_t>(ns) * 2);
    for (unsigned ch = 0; ch < nc; ++ch) {
      if (!tx_files[ch].is_open()) {
        continue;
      }
      span<const ci16_t> buf = data.get_channel_buffer(ch);
      for (unsigned i = 0; i < ns; ++i) {
        const cf_t s      = to_cf(buf[i]);
        interleaved[2 * i]     = s.real();
        interleaved[2 * i + 1] = s.imag();
      }
      tx_files[ch].write(reinterpret_cast<const char*>(interleaved.data()),
                         static_cast<std::streamsize>(interleaved.size() * sizeof(float)));
    }
    parent.current_time.fetch_add(ns, std::memory_order_relaxed);
  }

  metadata receive(baseband_gateway_buffer_writer& data) override
  {
    const unsigned ns = data.get_nof_samples();
    const unsigned nc = std::min(data.get_nof_channels(), nof_rx);
    std::vector<float> interleaved(static_cast<size_t>(ns) * 2);
    for (unsigned ch = 0; ch < nc; ++ch) {
      span<ci16_t> out = data.get_channel_buffer(ch);
      bool         filled = false;
      if (rx_files[ch].is_open() && !rx_eof[ch]) {
        rx_files[ch].read(reinterpret_cast<char*>(interleaved.data()),
                          static_cast<std::streamsize>(interleaved.size() * sizeof(float)));
        const auto got = static_cast<size_t>(rx_files[ch].gcount() / sizeof(float));
        if (got < interleaved.size()) {
          std::fill(interleaved.begin() + static_cast<std::ptrdiff_t>(got), interleaved.end(), 0.0f);
          rx_eof[ch] = true;
        }
        for (unsigned i = 0; i < ns; ++i) {
          out[i] = to_ci16(cf_t{interleaved[2 * i], interleaved[2 * i + 1]});
        }
        filled = true;
      }
      if (!filled) {
        std::fill(out.begin(), out.end(), ci16_t{0, 0});
      }
    }
    metadata md;
    md.ts = parent.current_time.load(std::memory_order_relaxed);
    parent.current_time.fetch_add(ns, std::memory_order_relaxed);
    return md;
  }

private:
  radio_session_null_impl&      parent;
  unsigned                      nof_tx;
  unsigned                      nof_rx;
  std::vector<std::ofstream>    tx_files;
  std::vector<std::ifstream>    rx_files;
  std::vector<bool>             rx_eof;
};

bool radio_session_null_impl::parse_args(const std::string& args, std::string& tx_dump, std::string& rx_feed)
{
  tx_dump = extract_kv(args, "tx_dump");
  rx_feed = extract_kv(args, "rx_feed");
  // Also accept brief-style keys.
  if (tx_dump.empty()) {
    tx_dump = extract_kv(args, "tx_dump_dir");
  }
  if (rx_feed.empty()) {
    rx_feed = extract_kv(args, "rx_feed_dir");
  }
  return !tx_dump.empty() || !rx_feed.empty() || args.empty();
}

radio_session_null_impl::radio_session_null_impl(const radio_configuration::radio& config,
                                                 task_executor& /*async_task_executor*/,
                                                 radio_event_notifier& /*notification_handler*/) :
  logger(ocudulog::fetch_basic_logger("RADIO", false))
{
  sampling_rate_Hz = config.sampling_rate_Hz;
  if (!parse_args(config.args, tx_dump_dir, rx_feed_dir)) {
    logger.error("null radio: failed to parse args '{}'", config.args);
    return;
  }
  if (config.tx_streams.empty() || config.rx_streams.empty()) {
    logger.error("null radio: need at least one TX and one RX stream");
    return;
  }
  nof_tx_ports = config.tx_streams.front().channels.size();
  nof_rx_ports = config.rx_streams.front().channels.size();
  gateways.push_back(std::make_unique<null_gateway>(*this, 0, nof_tx_ports, nof_rx_ports));
  gateways.front()->open_files(tx_dump_dir, rx_feed_dir);
  successful = true;
  logger.info("null radio ready: tx_dump='{}' rx_feed='{}' tx_ports={} rx_ports={} srate={:.2f}Hz",
              tx_dump_dir,
              rx_feed_dir,
              nof_tx_ports,
              nof_rx_ports,
              sampling_rate_Hz);
}

radio_session_null_impl::~radio_session_null_impl()
{
  stop();
}

baseband_gateway& radio_session_null_impl::get_baseband_gateway(unsigned stream_id)
{
  ocudu_assert(stream_id < gateways.size(), "bad stream id");
  return *gateways[stream_id];
}

baseband_gateway_timestamp radio_session_null_impl::read_current_time()
{
  return current_time.load(std::memory_order_relaxed);
}

void radio_session_null_impl::start(baseband_gateway_timestamp init_time)
{
  start_ts = init_time;
  current_time.store(init_time, std::memory_order_relaxed);
  started = true;
  write_sidecar_json();
}

void radio_session_null_impl::stop()
{
  started = false;
  write_sidecar_json();
}

void radio_session_null_impl::write_sidecar_json() const
{
  if (tx_dump_dir.empty()) {
    return;
  }
  namespace fs = std::filesystem;
  fs::create_directories(tx_dump_dir);
  const auto path = fs::path(tx_dump_dir) / "iq_meta.json";
  std::ofstream out(path);
  out << "{\n"
      << "  \"sample_rate_Hz\": " << sampling_rate_Hz << ",\n"
      << "  \"start_timestamp\": " << start_ts << ",\n"
      << "  \"nof_tx_ports\": " << nof_tx_ports << ",\n"
      << "  \"nof_rx_ports\": " << nof_rx_ports << ",\n"
      << "  \"format\": \"cf32\",\n"
      << "  \"backend\": \"null\"\n"
      << "}\n";
}
