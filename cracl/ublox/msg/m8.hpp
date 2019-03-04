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

#ifndef CRACL_UBLOX_MSG_M8_HPP
#define CRACL_UBLOX_MSG_M8_HPP

#include "base.hpp"

#include "class/mon.hpp"
#include "class/nav.hpp"
#include "class/rxm.hpp"

namespace cracl
{

namespace ubx
{

extern const std::map<std::string,
       std::pair<uint8_t, std::map<std::string, uint8_t>>>
  m8_map;

}

}

#endif // CRACL_UBLOX_MSG_M8_HPP
