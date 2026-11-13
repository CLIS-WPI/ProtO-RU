# SPDX-FileCopyrightText: Copyright (C) 2026 CLIS-WPI / OpenRU
# SPDX-License-Identifier: AGPL-3.0-or-later

set(SOURCES_NULL
        radio_factory_null_impl.cpp
        radio_null_session.cpp)
add_library(ocudu_radio_null STATIC ${SOURCES_NULL})
target_link_libraries(ocudu_radio_null fmt ocudulog ocudu_support)
add_to_exported_libs(ocudu_radio_null)
