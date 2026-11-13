// SPDX-FileCopyrightText: Copyright (C) 2026 CLIS-WPI / OpenRU
// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from ProtO-RU/OCUDU radio interfaces (AGPLv3).

#include "radio_null_session.h"
#include "ocudu/adt/complex.h"
#include "ocudu/gateways/baseband/baseband_gateway.h"
#include "ocudu/gateways/baseband/baseband_gateway_receiver.h"
#include "ocudu/gateways/baseband/baseband_gateway_transmitter.h"
#include "ocudu/gateways/baseband/baseband_gateway_transmitter_metadata.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_reader.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_writer.h"
#include "ocudu/ocudulog/ocudulog.h"
#include "ocudu/radio/radio_event_notifier.h"
#include "ocudu/support/error_handling.h"
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <pthread.h>
#include <thread>

using namespace ocudu;

namespace {

std::string extract_kv(const std::string& args, const std::string& key)
{
  const std::string needle = key + "=";
  size_t            pos    = args.find(needle);
  if (pos == std::string::npos) {
    return {};
  }
  pos += needle.size();
  const size_t end = args.find(',', pos);
  return args.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
}

} // namespace

class radio_session_null_impl::null_gateway : public baseband_gateway,
                                              public baseband_gateway_transmitter,
                                              public baseband_gateway_receiver
{
public:
  null_gateway(radio_session_null_impl& parent_, unsigned nof_tx_ch, unsigned nof_rx_ch) :
    parent(parent_), nof_tx(nof_tx_ch), nof_rx(nof_rx_ch)
  {
    rx_files.resize(nof_rx);
    rx_eof.resize(nof_rx, false);
  }

  baseband_gateway_transmitter& get_transmitter() override { return *this; }
  baseband_gateway_receiver&    get_receiver() override { return *this; }

  void open_rx(const std::string& rx_dir)
  {
    namespace fs = std::filesystem;
    if (rx_dir.empty()) {
      return;
    }
    for (unsigned p = 0; p < nof_rx; ++p) {
      const auto path = fs::path(rx_dir) / ("port" + std::to_string(p) + ".cf32");
      if (fs::exists(path)) {
        rx_files[p] = std::ifstream(path, std::ios::binary);
      }
    }
  }

  void transmit(const baseband_gateway_buffer_reader& data, const baseband_gateway_transmitter_metadata& md) override
  {
    // Do NOT advance radio clock here — only receive() owns current_time.
    const unsigned ns = data.get_nof_samples();
    const unsigned nc = std::min(data.get_nof_channels(), nof_tx);

    tx_block blk;
    blk.ts          = md.ts;
    blk.nof_samples = ns;
    blk.nof_channels = nc;
    blk.is_empty    = md.is_empty;
    blk.interleaved.resize(static_cast<size_t>(nc) * ns * 2);

    if (!md.is_empty) {
      for (unsigned ch = 0; ch < nc; ++ch) {
        span<const ci16_t> buf = data.get_channel_buffer(ch);
        float*             dst = blk.interleaved.data() + static_cast<size_t>(ch) * ns * 2;
        const unsigned     start = md.tx_start.value_or(0);
        const unsigned     end   = md.tx_end.value_or(ns);
        for (unsigned i = 0; i < ns; ++i) {
          if (i < start || i >= end) {
            dst[2 * i] = dst[2 * i + 1] = 0.0f;
          } else {
            const cf_t s        = to_cf(buf[i]);
            dst[2 * i]     = s.real();
            dst[2 * i + 1] = s.imag();
          }
        }
      }
    } else {
      std::fill(blk.interleaved.begin(), blk.interleaved.end(), 0.0f);
    }

    parent.enqueue_tx(std::move(blk));
  }

  metadata receive(baseband_gateway_buffer_writer& data) override
  {
    const unsigned                   ns = data.get_nof_samples();
    const baseband_gateway_timestamp ts = parent.current_time.load(std::memory_order_relaxed);

    // Realtime pacing: block until wall clock reaches end of this buffer.
    // Uses system_clock — same clock ProtO-RU uses for SDR start_time alignment.
    if (parent.pace == pace_mode::realtime && parent.sampling_rate_Hz > 0.0) {
      const double end_secs =
          static_cast<double>((ts - parent.start_ts) + ns) / parent.sampling_rate_Hz;
      const auto target = parent.wall_start + std::chrono::duration_cast<std::chrono::system_clock::duration>(
                                                  std::chrono::duration<double>(end_secs));
      const auto now = std::chrono::system_clock::now();
      if (target > now) {
        std::this_thread::sleep_until(target);
      }
    }

    const unsigned     nc = std::min(data.get_nof_channels(), nof_rx);
    std::vector<float> interleaved(static_cast<size_t>(ns) * 2);
    for (unsigned ch = 0; ch < nc; ++ch) {
      span<ci16_t> out    = data.get_channel_buffer(ch);
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
    md.ts = ts;
    parent.current_time.fetch_add(ns, std::memory_order_relaxed);

    // Optional pace verification log: wall_unix_s,radio_ts (one line per receive).
    static thread_local unsigned pace_decim = 0;
    if (parent.pace == pace_mode::realtime && (++pace_decim % 2000) == 0) {
      const auto now = std::chrono::system_clock::now().time_since_epoch();
      const double wall_s =
          std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
      parent.logger.info("null_pace wall_s={:.6f} radio_ts={}", wall_s, md.ts + ns);
    }
    return md;
  }

private:
  radio_session_null_impl&   parent;
  unsigned                   nof_tx;
  unsigned                   nof_rx;
  std::vector<std::ifstream> rx_files;
  std::vector<bool>          rx_eof;
};

bool radio_session_null_impl::parse_args(const std::string& args,
                                         std::string&       tx_dump,
                                         std::string&       rx_feed,
                                         pace_mode&         pace,
                                         unsigned&          scs_khz)
{
  tx_dump = extract_kv(args, "tx_dump");
  if (tx_dump.empty()) {
    tx_dump = extract_kv(args, "tx_dump_dir");
  }
  rx_feed = extract_kv(args, "rx_feed");
  if (rx_feed.empty()) {
    rx_feed = extract_kv(args, "rx_feed_dir");
  }
  const std::string pace_s = extract_kv(args, "pace");
  if (pace_s == "free") {
    pace = pace_mode::free;
  } else {
    pace = pace_mode::realtime; // default
  }
  const std::string scs_s = extract_kv(args, "scs_khz");
  if (!scs_s.empty()) {
    scs_khz = static_cast<unsigned>(std::stoul(scs_s));
  }
  return !tx_dump.empty() || !rx_feed.empty() || args.empty() || !pace_s.empty();
}

void radio_session_null_impl::slot_of(baseband_gateway_timestamp ts, unsigned& sfn, unsigned& slot) const
{
  const unsigned slots_per_frame = 10u * (scs_khz / 15u); // 20 for 30 kHz
  const uint64_t abs_slot =
      samples_per_slot > 0 ? static_cast<uint64_t>((ts - start_ts) / samples_per_slot) : 0;
  sfn  = static_cast<unsigned>((abs_slot / slots_per_frame) % 1024u);
  slot = static_cast<unsigned>(abs_slot % slots_per_frame);
}

void radio_session_null_impl::enqueue_tx(tx_block&& blk)
{
  std::unique_lock<std::mutex> lock(tx_mu);
  tx_cv.wait(lock, [this] { return tx_count < TX_RING_CAP || tx_stop; });
  if (tx_stop) {
    return;
  }
  tx_ring[tx_tail] = std::move(blk);
  tx_tail          = (tx_tail + 1) % TX_RING_CAP;
  ++tx_count;
  lock.unlock();
  tx_cv.notify_one();
}

void radio_session_null_impl::writer_loop()
{
  namespace fs = std::filesystem;
  std::vector<std::ofstream> ports;
  if (!tx_dump_dir.empty()) {
    fs::create_directories(tx_dump_dir);
    ports.resize(nof_tx_ports);
    for (unsigned p = 0; p < nof_tx_ports; ++p) {
      ports[p].open(fs::path(tx_dump_dir) / ("port" + std::to_string(p) + ".cf32"),
                    std::ios::binary | std::ios::trunc);
    }
    tx_index.open(fs::path(tx_dump_dir) / "tx_index.jsonl", std::ios::trunc);
  }

  // Lower writer priority (best-effort).
  sched_param sp{};
  sp.sched_priority = 0;
  pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);

  while (true) {
    tx_block blk;
    {
      std::unique_lock<std::mutex> lock(tx_mu);
      tx_cv.wait(lock, [this] { return tx_count > 0 || tx_stop; });
      if (tx_count == 0 && tx_stop) {
        break;
      }
      blk     = std::move(tx_ring[tx_head]);
      tx_head = (tx_head + 1) % TX_RING_CAP;
      --tx_count;
    }
    tx_cv.notify_one();

    if (ports.empty()) {
      continue;
    }

    // Gap-fill so file sample offset == (ts - start_ts).
    if (blk.ts > tx_file_cursor) {
      const uint64_t gap = static_cast<uint64_t>(blk.ts - tx_file_cursor);
      std::vector<float> zeros(static_cast<size_t>(gap) * 2, 0.0f);
      for (unsigned p = 0; p < nof_tx_ports; ++p) {
        if (ports[p].is_open()) {
          ports[p].write(reinterpret_cast<const char*>(zeros.data()),
                         static_cast<std::streamsize>(zeros.size() * sizeof(float)));
        }
      }
      tx_file_cursor = blk.ts;
    }

    unsigned sfn = 0, slot = 0;
    slot_of(blk.ts, sfn, slot);
    if (tx_index.is_open()) {
      tx_index << "{\"ts\":" << blk.ts << ",\"nof_samples\":" << blk.nof_samples
               << ",\"is_empty\":" << (blk.is_empty ? "true" : "false") << ",\"sfn\":" << sfn
               << ",\"slot\":" << slot << ",\"symbol0_sample_offset\":" << (blk.ts - start_ts) << "}\n";
    }

    if (!blk.is_empty && !blk.interleaved.empty()) {
      for (unsigned ch = 0; ch < blk.nof_channels && ch < nof_tx_ports; ++ch) {
        if (!ports[ch].is_open()) {
          continue;
        }
        const float* src = blk.interleaved.data() + static_cast<size_t>(ch) * blk.nof_samples * 2;
        ports[ch].write(reinterpret_cast<const char*>(src),
                        static_cast<std::streamsize>(blk.nof_samples * 2 * sizeof(float)));
      }
    } else if (blk.is_empty) {
      std::vector<float> zeros(static_cast<size_t>(blk.nof_samples) * 2, 0.0f);
      for (unsigned p = 0; p < nof_tx_ports; ++p) {
        if (ports[p].is_open()) {
          ports[p].write(reinterpret_cast<const char*>(zeros.data()),
                         static_cast<std::streamsize>(zeros.size() * sizeof(float)));
        }
      }
    }
    tx_file_cursor = blk.ts + blk.nof_samples;
  }
}

radio_session_null_impl::radio_session_null_impl(const radio_configuration::radio& config,
                                                 task_executor& /*async_task_executor*/,
                                                 radio_event_notifier& /*notification_handler*/) :
  logger(ocudulog::fetch_basic_logger("RADIO", false)), tx_ring(TX_RING_CAP)
{
  sampling_rate_Hz = config.sampling_rate_Hz;
  if (!parse_args(config.args, tx_dump_dir, rx_feed_dir, pace, scs_khz)) {
    logger.error("null radio: failed to parse args '{}'", config.args);
    return;
  }
  if (config.tx_streams.empty() || config.rx_streams.empty()) {
    logger.error("null radio: need at least one TX and one RX stream");
    return;
  }
  nof_tx_ports     = config.tx_streams.front().channels.size();
  nof_rx_ports     = config.rx_streams.front().channels.size();
  samples_per_slot = sampling_rate_Hz > 0.0
                         ? static_cast<unsigned>(std::llround(sampling_rate_Hz / (1000.0 * (scs_khz / 15.0))))
                         : 0;

  // Prefer tmpfs for dump during runs if caller passed a /tmp path; document /dev/shm in profiles.
  gateways.push_back(std::make_unique<null_gateway>(*this, nof_tx_ports, nof_rx_ports));
  gateways.front()->open_rx(rx_feed_dir);

  if (!tx_dump_dir.empty()) {
    tx_writer = std::thread([this] { writer_loop(); });
  }

  successful = true;
  logger.info(
      "null radio ready: tx_dump='{}' rx_feed='{}' pace={} tx_ports={} rx_ports={} srate={:.2f}Hz "
      "samples_per_slot={}{}",
      tx_dump_dir,
      rx_feed_dir,
      pace == pace_mode::realtime ? "realtime" : "free",
      nof_tx_ports,
      nof_rx_ports,
      sampling_rate_Hz,
      samples_per_slot,
      pace == pace_mode::free ? " [OFFLINE THROUGHPUT — not a timing claim]" : "");
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
  wall_start     = std::chrono::system_clock::now();
  tx_file_cursor = init_time;
  started        = true;
  write_sidecar_json();
}

void radio_session_null_impl::stop()
{
  started = false;
  {
    std::lock_guard<std::mutex> lock(tx_mu);
    tx_stop = true;
  }
  tx_cv.notify_all();
  if (tx_writer.joinable()) {
    tx_writer.join();
  }
  write_sidecar_json();
}

void radio_session_null_impl::write_sidecar_json() const
{
  if (tx_dump_dir.empty()) {
    return;
  }
  namespace fs = std::filesystem;
  fs::create_directories(tx_dump_dir);
  std::ofstream out(fs::path(tx_dump_dir) / "iq_meta.json");
  out << "{\n"
      << "  \"sample_rate_Hz\": " << sampling_rate_Hz << ",\n"
      << "  \"start_timestamp\": " << start_ts << ",\n"
      << "  \"nof_tx_ports\": " << nof_tx_ports << ",\n"
      << "  \"nof_rx_ports\": " << nof_rx_ports << ",\n"
      << "  \"format\": \"cf32\",\n"
      << "  \"backend\": \"null\",\n"
      << "  \"pace\": \"" << (pace == pace_mode::realtime ? "realtime" : "free") << "\",\n"
      << "  \"slot_grid\": {\n"
      << "    \"scs_kHz\": " << scs_khz << ",\n"
      << "    \"samples_per_slot\": " << samples_per_slot << ",\n"
      << "    \"slots_per_frame\": " << (10u * (scs_khz / 15u)) << ",\n"
      << "    \"symbol0_sample_offset_basis\": \"ts - start_timestamp\"\n"
      << "  },\n"
      << "  \"tx_index\": \"tx_index.jsonl\",\n"
      << "  \"align\": \"slot\"\n"
      << "}\n";
}
