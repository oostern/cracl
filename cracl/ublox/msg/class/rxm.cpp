// Copyright (C) 2020 Colton Riedel
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "rxm.hpp"

#include "../base.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace cracl
{

namespace ubx
{

namespace rxm
{

measx::measx(std::vector<uint8_t>& message)
{
  update(message);
}

void measx::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    version = message[6];

    gpsTOW = (*(reinterpret_cast<uint32_t*> (&message[10])));
    gloTOW = (*(reinterpret_cast<uint32_t*> (&message[14])));
    bdsTOW = (*(reinterpret_cast<uint32_t*> (&message[18])));
    qzssTOW = (*(reinterpret_cast<uint32_t*> (&message[36])));

    gpsTOWacc = (*(reinterpret_cast<uint16_t*> (&message[30])));
    gloTOWacc = (*(reinterpret_cast<uint16_t*> (&message[32])));
    bdsTOWacc = (*(reinterpret_cast<uint16_t*> (&message[34])));
    qzssTOWacc = (*(reinterpret_cast<uint16_t*> (&message[38])));

    numSV = message[40];
    towSet = message[41] & 0x03;

    gnssId.clear();
    svId.clear();
    cNo.clear();
    mpathIndic.clear();
    dopplerMS.clear();
    dopplerHz.clear();
    wholeChips.clear();
    fracChips.clear();
    codePhase.clear();
    intCodePhase.clear();
    pseuRangeRMSErr.clear();

    for (size_t i = 0; i < numSV; ++i)
    {
      gnssId.push_back(message[50 + (i * 24)]);
      svId.push_back(message[51 + (i * 24)]);
      cNo.push_back(message[52 + (i * 24)]);
      mpathIndic.push_back(message[53 + (i * 24)]);

      dopplerMS.push_back(
          *(reinterpret_cast<int32_t*> (&message[54 + (i * 24)])));
      dopplerHz.push_back(
          *(reinterpret_cast<int32_t*> (&message[58 + (i * 24)])));

      wholeChips.push_back(
          *(reinterpret_cast<uint16_t*> (&message[62 + (i * 24)])));
      fracChips.push_back(
          *(reinterpret_cast<uint16_t*> (&message[64 + (i * 24)])));

      codePhase.push_back(
          *(reinterpret_cast<uint32_t*> (&message[66 + (i * 24)])));

      intCodePhase.push_back(message[70 + (i * 24)]);
      pseuRangeRMSErr.push_back(message[71 + (i * 24)]);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool measx::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("RXM").first
      && message[3] == ubx::msg_map.at("RXM").second.at("MEASX"));
}

rawx::rawx(std::vector<uint8_t>& message)
{
  update(message);
}

void rawx::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    rcvTow = (*(reinterpret_cast<double*> (&message[6])));

    week = (*(reinterpret_cast<uint16_t*> (&message[14])));

    leapS = (*(reinterpret_cast<int8_t*> (&message[16])));

    numMeas = message[17];
    recStat = message[12];

    leapSec = recStat & 0x01;
    clkReset = recStat & 0x02;

    prMes.clear();
    cpMes.clear();
    doMes.clear();
    gnssId.clear();
    svId.clear();
    sigId.clear();
    freqId.clear();
    locktime.clear();
    cno.clear();
    prStdev.clear();
    cpStdev.clear();
    doStdev.clear();
    trkStat.clear();

    for (size_t i = 0; i < numMeas; ++i)
    {
      prMes.push_back(*(reinterpret_cast<double*> (&message[22 + (i * 32)])));
      cpMes.push_back(*(reinterpret_cast<double*> (&message[30 + (i * 32)])));

      doMes.push_back(*(reinterpret_cast<float*> (&message[38 + (i * 32)])));

      gnssId.push_back(message[42 + (i * 32)]);
      svId.push_back(message[43 + (i * 32)]);
      sigId.push_back(message[44 + (i * 32)]);
      freqId.push_back(message[45 + (i * 32)]);

      locktime.push_back(
          *(reinterpret_cast<uint16_t*> (&message[46 + (i * 32)])));

      cno.push_back(message[48 + (i * 32)]);
      prStdev.push_back(message[49 + (i * 32)]);
      cpStdev.push_back(message[50 + (i * 32)]);
      doStdev.push_back(message[51 + (i * 32)]);
      trkStat.push_back(message[52 + (i * 32)]);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool rawx::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("RXM").first
      && message[3] == ubx::msg_map.at("RXM").second.at("RAWX"));
}

} // namespace rxm

} // namespace ubx

} // namespace cracl
