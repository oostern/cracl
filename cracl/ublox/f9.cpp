// Copyright (C) 2019 Colton Riedel
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see https://www.gnu.org/licenses/
//
// If you are interested in obtaining a copy of this program under a
// different license, or have other questions or comments, contact me at
//
//   coltonriedel at protonmail dot ch

#include "f9.hpp"

#include "../base/device.hpp"

namespace cracl
{

f9::f9(const std::string& location, size_t baud_rate, size_t timeout,
    size_t char_size, std::string delim, size_t max_handlers,
    port_base::parity::type parity,
    port_base::flow_control::type flow_control,
    port_base::stop_bits::type stop_bits)
  : ublox_base(location, baud_rate, timeout, char_size, std::string(delim),
    max_handlers, parity, flow_control, stop_bits)
{ }

std::vector<uint8_t> f9::fetch_ubx(std::string&& msg_class,
    std::string&& msg_id, bool first_try)
{
  size_t i;
  std::vector<uint8_t> temp;

  if (m_ubx_buffer.empty())
    buffer_messages();

  // Look for message with header matching request
  for (i = 0; i < m_ubx_buffer.size(); ++i)
    if (m_ubx_buffer[i][2] == ubx::msg_map.at(msg_class).first
        && m_ubx_buffer[i][3] == ubx::msg_map.at(msg_class).second.at(msg_id))
      break;

  if (i != m_ubx_buffer.size()) // If found, fetch and erase from buffer
  {
    temp = m_ubx_buffer[i];

    m_ubx_buffer.erase(m_ubx_buffer.begin() + i);
  }
  else if (first_try) // If not found and first try, double check for new msgs
  {
    buffer_messages();

    return fetch_ubx(std::forward<std::string>(msg_class),
        std::forward<std::string>(msg_id), false);
  }

  return temp;
}

} // namespace cracl
