// SPDX-FileCopyrightText: Copyright (C) 2026 CLIS-WPI / OpenRU
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "radio_factory_null_impl.h"
#include "radio_null_session.h"

using namespace ocudu;

namespace {

class radio_config_null_validator : public radio_configuration::validator
{
public:
  bool is_configuration_valid(const radio_configuration::radio& config) const override
  {
    return config.sampling_rate_Hz > 0.0 && !config.tx_streams.empty() && !config.rx_streams.empty();
  }
};

} // namespace

const radio_configuration::validator& radio_factory_null_impl::get_configuration_validator() const
{
  static radio_config_null_validator v;
  return v;
}

std::unique_ptr<radio_session> radio_factory_null_impl::create(const radio_configuration::radio& config,
                                                               task_executor&                    async_task_executor,
                                                               radio_event_notifier&             notifier)
{
  auto session = std::make_unique<radio_session_null_impl>(config, async_task_executor, notifier);
  if (!session->is_successful()) {
    return nullptr;
  }
  return session;
}
