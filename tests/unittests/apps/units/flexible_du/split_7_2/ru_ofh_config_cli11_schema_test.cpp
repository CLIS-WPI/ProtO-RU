// SPDX-FileCopyrightText: Copyright (C) 2026 National University of Singapore
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "apps/units/flexible_o_du/split_7_2/helpers/ru_ofh_config.h"
#include "apps/units/flexible_o_du/split_7_2/helpers/ru_ofh_config_cli11_schema.h"
#include "apps/units/flexible_o_du/split_7_2/helpers/ru_ofh_config_translator.h"
#include "apps/units/flexible_o_du/split_helpers/flexible_o_du_configs.h"
#include "ocudu/support/config_parsers.h"
#include <array>
#include <gtest/gtest.h>
#include <sstream>
#include <string_view>

using namespace ocudu;

namespace {

void parse_timing_parameter(std::string_view parameter, unsigned value)
{
  CLI::App app("OFH timing range test");
  app.config_formatter(create_yaml_config_parser());
  app.allow_config_extras(CLI::config_extras_mode::error);

  ru_ofh_unit_parsed_config config;
  configure_cli11_with_ru_ofh_config_schema(app, config);

  std::istringstream input("ru_ofh:\n  " + std::string(parameter) + ": " + std::to_string(value) + "\n");
  app.parse_from_stream(input);
}

constexpr std::array<std::string_view, 8> timing_parameters = {"t1a_max_cp_dl",
                                                               "t1a_min_cp_dl",
                                                               "t1a_max_cp_ul",
                                                               "t1a_min_cp_ul",
                                                               "t1a_max_up",
                                                               "t1a_min_up",
                                                               "ta4_max",
                                                               "ta4_min"};

ru_ofh_unit_parsed_config parse_prach_rx_extra_slots(unsigned value)
{
  CLI::App app("OFH PRACH extra slots test");
  app.config_formatter(create_yaml_config_parser());
  app.allow_config_extras(CLI::config_extras_mode::error);

  ru_ofh_unit_parsed_config config;
  configure_cli11_with_ru_ofh_config_schema(app, config);

  std::istringstream input("ru_ofh:\n  prach_rx_extra_slots: " + std::to_string(value) + "\n");
  app.parse_from_stream(input);

  return config;
}

ru_ofh_unit_config make_ru_config(unsigned prach_rx_extra_slots)
{
  ru_ofh_unit_config config;
  config.cells[0].network_interface         = "ofh0";
  config.cells[0].ru_mac_address            = "02:00:00:00:00:20";
  config.cells[0].du_mac_address            = "02:00:00:00:00:10";
  config.cells[0].cell.prach_rx_extra_slots = prach_rx_extra_slots;
  config.cells[0].cell.iq_scaling_config    = ru_ofh_legacy_scaling_config{1.0F};
  return config;
}

flexible_o_du_ru_config make_cell_config()
{
  flexible_o_du_ru_config config;
  config.max_processing_delay = 10;
  config.cells.push_back({.scs              = subcarrier_spacing::kHz30,
                          .cp               = cyclic_prefix::NORMAL,
                          .bw               = bs_channel_bandwidth::MHz20,
                          .nof_rx_antennas  = 2,
                          .nof_tx_antennas  = 2,
                          .tdd_config       = std::nullopt,
                          .dl_arfcn         = 628032,
                          .ul_arfcn         = 628032,
                          .freq_range       = frequency_range::FR1});
  return config;
}

} // namespace

TEST(ru_ofh_config_cli11_schema_test, accepts_extended_timing_upper_bound)
{
  for (std::string_view parameter : timing_parameters) {
    SCOPED_TRACE(parameter);
    EXPECT_NO_THROW(parse_timing_parameter(parameter, 5000));
  }
}

TEST(ru_ofh_config_cli11_schema_test, rejects_timing_value_above_upper_bound)
{
  for (std::string_view parameter : timing_parameters) {
    SCOPED_TRACE(parameter);
    EXPECT_THROW(parse_timing_parameter(parameter, 5001), CLI::ParseError);
  }
}

TEST(ru_ofh_config_cli11_schema_test, parses_prach_rx_extra_slots)
{
  EXPECT_EQ(parse_prach_rx_extra_slots(0).config.cells[0].cell.prach_rx_extra_slots, 0);
  EXPECT_EQ(parse_prach_rx_extra_slots(7).config.cells[0].cell.prach_rx_extra_slots, 7);
}

TEST(ru_ofh_config_cli11_schema_test, accepts_extra_slots_at_prach_ring_limit)
{
  ru_ofh_unit_config      ru_cfg   = make_ru_config(37);
  flexible_o_du_ru_config cell_cfg = make_cell_config();

  EXPECT_NO_THROW(generate_ru_ofh_config(ru_cfg, cell_cfg.cells, cell_cfg.max_processing_delay));
}

TEST(ru_ofh_config_cli11_schema_test, rejects_extra_slots_above_prach_ring_limit)
{
  ru_ofh_unit_config      ru_cfg   = make_ru_config(38);
  flexible_o_du_ru_config cell_cfg = make_cell_config();

  EXPECT_EXIT(generate_ru_ofh_config(ru_cfg, cell_cfg.cells, cell_cfg.max_processing_delay),
              ::testing::ExitedWithCode(1),
              "prach_rx_extra_slots=38");
}
