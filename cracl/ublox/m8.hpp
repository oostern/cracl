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

#ifndef CRACL_UBLOX_M8_HPP
#define CRACL_UBLOX_M8_HPP

#include "base.hpp"
#include "msg/m8.hpp"
#include "../base/device.hpp"

#include <deque>
#include <iomanip>
#include <string>
#include <vector>

namespace cracl
{

class m8 : public ublox_base
{
public:
  m8(const std::string& location, size_t baud_rate=9600, size_t timeout=500,
      size_t char_size=8, std::string delim="\r\n", size_t max_handlers=100000,
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one);

  using ublox_base::fetch_ubx;

  std::vector<uint8_t> fetch_ubx(std::string&& msg_class,
      std::string&& msg_id, bool first_try=true);

  template <typename... Args>
  void ubx_send(std::string&& msg_class, std::string&& msg_id, Args... args)
  {
    std::vector<uint8_t> message = { 0xb5 /*μ*/, 'b',
      ubx::m8_map.at(msg_class).first,
      ubx::m8_map.at(msg_class).second.at(msg_id) };

    uint16_t length = payload_size(args...);
    message.reserve(2 + length + 2);

    add_ubx_payload(message, length, args...);

    uint8_t check_a = 0;
    uint8_t check_b = 0;

    // Compute 8-bit Fletcher checksum
    for (size_t i = 2; i < message.size(); ++i)
      check_b += (check_a += message[i]);

    message.push_back(check_a);
    message.push_back(check_b);

    write(message);
  }

};

} // namespace cracl

#endif // CRACL_UBLOX_M8_HPP
