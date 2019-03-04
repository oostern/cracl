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

#include "sa45s.hpp"

#include "../base/device.hpp"

#include <string>

namespace cracl
{

sa45s::sa45s(const std::string& location, size_t baud_rate, size_t timeout,
    size_t char_size, std::string delim, size_t max_handlers,
    port_base::parity::type parity,
    port_base::flow_control::type flow_control,
    port_base::stop_bits::type stop_bits)
  : device (location, baud_rate, timeout, char_size, delim, max_handlers,
    parity, flow_control, stop_bits)
{ }

void sa45s::telemetry_header()
{
  write("!6");
}

void sa45s::telemetry_data()
{
  write("!^");
}

void sa45s::steer_freq_abs(int value)
{
  if (value >= -20000000 && value <= 20000000)
    write("!FD" + std::to_string(value));
}

void sa45s::steer_freq_rel(int value)
{
  if (value >= -20000000 && value <= 20000000)
    write("!FD" + std::to_string(value));
}

void sa45s::STEER_FREQ_LOCK()
{
  write("!FL");
}

} // namespace cracl
