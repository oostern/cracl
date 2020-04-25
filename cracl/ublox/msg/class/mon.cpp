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
    pinSel = (*(reinterpret_cast<uint32_t*> (&message[6])));
    pinBank = (*(reinterpret_cast<uint32_t*> (&message[10])));
    pinDir = (*(reinterpret_cast<uint32_t*> (&message[14])));
    pinVal = (*(reinterpret_cast<uint32_t*> (&message[18])));

    noisePerMS = (*(reinterpret_cast<uint16_t*> (&message[22])));
    agcCnt = (*(reinterpret_cast<uint16_t*> (&message[24])));

    aStatus = message[26];
    aPower  = message[27];

    uint8_t bitfield = message[28];

    rtcCalib = bitfield & 0x01;
    safeBoot = bitfield >> 1 & 0x01;
    jammingState = bitfield >> 2 & 0x03;
    xtalAbsent = bitfield >> 4 & 0x01;

    usedMask = (*(reinterpret_cast<uint32_t*> (&message[30])));

    std::memcpy(vp.data(), &message[34], 17);

    jamInd = message[51];

    pinIrq = (*(reinterpret_cast<uint32_t*> (&message[54])));
    pullH = (*(reinterpret_cast<uint32_t*> (&message[58])));
    pullL = (*(reinterpret_cast<uint32_t*> (&message[62])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool hw::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("MON").first
      && message[3] == ubx::msg_map.at("MON").second.at("HW"));
}

rf::rf(std::vector<uint8_t>& message)
{
  update(message);
}

void rf::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    version = message[6];
    nBlocks = message[7];

    blockId.clear();
    jammingState.clear();
    antStatus.clear();
    antPower.clear();
    postStatus.clear();
    noisePerMS.clear();
    agcCnt.clear();
    jamInd.clear();
    ofsI.clear();
    magI.clear();
    ofsQ.clear();
    magQ.clear();

    for (size_t i = 0; i < nBlocks; ++i)
    {
      blockId.push_back(message[10 + (i * 24)]);

      jammingState.push_back(message[11 + (i * 24)] & 0x03);

      antStatus.push_back(message[12 + (i * 24)]);
      antPower.push_back(message[13 + (i * 24)]);

      postStatus.push_back(
        *(reinterpret_cast<uint32_t*> (&message[14 + (i * 24)])));

      noisePerMS.push_back(
        *(reinterpret_cast<uint16_t*> (&message[22 + (i * 24)])));
      agcCnt.push_back(
        *(reinterpret_cast<uint16_t*> (&message[24 + (i * 24)])));

      jamInd.push_back(message[26 + (i * 24)]);

      ofsI.push_back(*(reinterpret_cast<int8_t*> (&message[27 + (i * 24)])));

      magI.push_back(message[28 + (i * 24)]);

      ofsQ.push_back(*(reinterpret_cast<int8_t*> (&message[29 + (i * 24)])));

      magQ.push_back(message[20 + (i * 24)]);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool rf::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("MON").first
      && message[3] == ubx::msg_map.at("MON").second.at("RF"));
}

ver::ver(std::vector<uint8_t>& message)
{
  update(message);
}

void ver::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    std::array<uint8_t, 30> temp;

    std::memcpy(swVersion.data(), &message[6], 30);
    std::memcpy(hwVersion.data(), &message[36], 10);

    extension.clear();

    for (size_t i = 46; i < message.size(); i = i + 30)
    {
      std::memcpy(temp.data(), &message[i], 30);

      extension.push_back(temp);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool ver::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("MON").first
      && message[3] == ubx::msg_map.at("MON").second.at("VER"));
}

} // namespace mon

} // namespace ubx

} // namespace cracl
