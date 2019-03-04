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

#include "mon.hpp"

#include "../base.hpp"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace cracl
{

namespace ubx
{

namespace mon
{

hw::hw(std::vector<uint8_t>& message)
{
  update(message);
}

void hw::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    m_pinSel = (*(reinterpret_cast<uint32_t*> (&message[6])));
    m_pinBank = (*(reinterpret_cast<uint32_t*> (&message[10])));
    m_pinDir = (*(reinterpret_cast<uint32_t*> (&message[14])));
    m_pinVal = (*(reinterpret_cast<uint32_t*> (&message[18])));

    m_noisePerMS = (*(reinterpret_cast<uint16_t*> (&message[22])));
    m_agcCnt = (*(reinterpret_cast<uint16_t*> (&message[24])));

    m_aStatus = message[26];
    m_aPower  = message[27];

    uint8_t bitfield = message[28];

    m_rtcCalib = bitfield & 0x01;
    m_safeBoot = bitfield & 0x02;
    m_jammingState = bitfield & 0x0C;
    m_xtalAbsent = bitfield & 0x10;

    m_usedMask = (*(reinterpret_cast<uint32_t*> (&message[30])));

    std::memcpy(m_vp.data(), &message[34], 17);

    m_jamInd = message[51];
    m_pinIrq = (*(reinterpret_cast<uint32_t*> (&message[54])));
    m_pullH = (*(reinterpret_cast<uint32_t*> (&message[58])));
    m_pullL = (*(reinterpret_cast<uint32_t*> (&message[62])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t hw::pinSel()
{
  return m_pinSel;
}

uint32_t hw::pinBank()
{
  return m_pinBank;
}

uint32_t hw::pinDir()
{
  return m_pinDir;
}

uint32_t hw::pinVal()
{
  return m_pinVal;
}

uint16_t hw::noisePerMS()
{
  return m_noisePerMS;
}

uint16_t hw::agcCnt()
{
  return m_agcCnt;
}

uint8_t hw::aStatus()
{
  return m_aStatus;
}

uint8_t hw::aPower()
{
  return m_aPower;
}

uint8_t hw::rtcCalib()
{
  return m_rtcCalib;
}

uint8_t hw::safeBoot()
{
  return m_safeBoot;
}

uint8_t hw::jammingState()
{
  return m_jammingState;
}

uint8_t hw::xtalAbsent()
{
  return m_xtalAbsent;
}

uint32_t hw::usedMask()
{
  return m_usedMask;
}

std::array<uint8_t, 17> hw::vp()
{
  return m_vp;
}

uint8_t hw::jamInd()
{
  return m_jamInd;
}

uint32_t hw::pinIrq()
{
  return m_pinIrq;
}

uint32_t hw::pullH()
{
  return m_pullH;
}

uint32_t hw::pullL()
{
  return m_pullL;
}

bool hw::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("MON").first
      && message[3] == ubx::msg_map.at("MON").second.at("HW"));
}

} // namespace mon

} // namespace ubx

} // namespace cracl
