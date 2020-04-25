// Copyright (C) 2020 Colton Riedel
//               2019 Will Bogardus
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

#include "nav.hpp"

#include "../base.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace cracl
{

namespace ubx
{

namespace nav
{

clock::clock(std::vector<uint8_t>& message)
{
  update(message);
}

void clock::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    clkB = (*(reinterpret_cast<int32_t*> (&message[10])));
    clkD = (*(reinterpret_cast<int32_t*> (&message[14])));

    tAcc = (*(reinterpret_cast<uint32_t*> (&message[18])));
    fAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool clock::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("CLOCK"));
}

dop::dop(std::vector<uint8_t>& message)
{
  update(message);
}

void dop::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    gDOP = (*(reinterpret_cast<uint16_t*> (&message[10])));
    pDOP = (*(reinterpret_cast<uint16_t*> (&message[12])));
    tDOP = (*(reinterpret_cast<uint16_t*> (&message[14])));
    vDOP = (*(reinterpret_cast<uint16_t*> (&message[16])));
    hDOP = (*(reinterpret_cast<uint16_t*> (&message[18])));
    nDOP = (*(reinterpret_cast<uint16_t*> (&message[20])));
    eDOP = (*(reinterpret_cast<uint16_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool dop::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("DOP"));
}

posecef::posecef(std::vector<uint8_t>& message)
{
  update(message);
}

void posecef::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    ecefX = (*(reinterpret_cast<int32_t*> (&message[10])));
    ecefY = (*(reinterpret_cast<int32_t*> (&message[14])));
    ecefZ = (*(reinterpret_cast<int32_t*> (&message[18])));

    pAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool posecef::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("POSECEF"));
}

posllh::posllh(std::vector<uint8_t>& message)
{
  update(message);
}

void posllh::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    lon = (*(reinterpret_cast<int32_t*> (&message[10])));
    lat = (*(reinterpret_cast<int32_t*> (&message[14])));
    height = (*(reinterpret_cast<int32_t*> (&message[18])));
    hMSL = (*(reinterpret_cast<int32_t*> (&message[22])));

    hAcc = (*(reinterpret_cast<uint32_t*> (&message[26])));
    vAcc = (*(reinterpret_cast<uint32_t*> (&message[30])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool posllh::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("POSLLH"));
}

sat::sat(std::vector<uint8_t>& message)
{
  update(message);
}

void sat::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    version = message[10];
    numSvs = message[11];

    gnssId.clear();
    svId.clear();
    cno.clear();
    elev.clear();
    azim.clear();
    prRes.clear();
    qualityInd.clear();
    svUsed.clear();
    health.clear();
    diffCorr.clear();
    smoothed.clear();
    orbitSource.clear();
    ephAvail.clear();
    almAvail.clear();
    anoAvail.clear();
    aopAvail.clear();
    sbasCorrUsed.clear();
    rtcmCorrUsed.clear();
    prCorrUsed.clear();
    crCorrUsed.clear();
    doCorrUsed.clear();

    for (size_t i = 0; i < numSvs; ++i)
    {
      gnssId.push_back(message[14 + (i * 12)]);
      svId.push_back(message[15 + (i * 12)]);

      cno.push_back(message[16 + (i * 12)]);

      elev.push_back(message[17 + (i * 12)]);
      azim.push_back(
          *(reinterpret_cast<uint16_t*> (&message[18 + (i * 12)])));
      prRes.push_back(
          *(reinterpret_cast<uint16_t*> (&message[20 + (i * 12)])));

      uint32_t bitfield
        = (*(reinterpret_cast<uint32_t*> (&message[22 + (i * 12)])));

      qualityInd.push_back(bitfield & 0x07);
      svUsed.push_back(bitfield >> 3 & 0x01);
      health.push_back(bitfield >> 4 & 0x03);
      diffCorr.push_back(bitfield >> 6 & 0x01);
      smoothed.push_back(bitfield >> 7 & 0x01);
      orbitSource.push_back(bitfield >> 8 & 0x07);
      ephAvail.push_back(bitfield >> 11 & 0x01);
      almAvail.push_back(bitfield >> 12 & 0x01);
      anoAvail.push_back(bitfield >> 13 & 0x01);
      aopAvail.push_back(bitfield >> 14 & 0x01);
      sbasCorrUsed.push_back(bitfield >> 16 & 0x01);
      rtcmCorrUsed.push_back(bitfield >> 17 & 0x01);
      prCorrUsed.push_back(bitfield >> 20 & 0x01);
      crCorrUsed.push_back(bitfield >> 21 & 0x01);
      doCorrUsed.push_back(bitfield >> 22 & 0x01);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool sat::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("SAT"));
}

sig::sig(std::vector<uint8_t>& message)
{
  update(message);
}

void sig::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    version = message[10];
    numSigs = message[11];

    gnssId.clear();
    svId.clear();
    sigId.clear();
    freqId.clear();
    prRes.clear();
    cno.clear();
    qualityInd.clear();
    corrSource.clear();
    ionoModel.clear();
    health.clear();
    prSmoothed.clear();
    prUsed.clear();
    crUsed.clear();
    doUsed.clear();
    prCorrUsed.clear();
    crCorrUsed.clear();
    doCorrUsed.clear();

    for (size_t i = 0; i < numSigs; ++i)
    {
      gnssId.push_back(message[14 + (i * 16)]);
      svId.push_back(message[15 + (i * 16)]);
      sigId.push_back(message[16 + (i * 16)]);
      freqId.push_back(message[17 + (i * 16)]);

      prRes.push_back(
          *(reinterpret_cast<uint16_t*> (&message[18 + (i * 16)])));

      cno.push_back(message[20 + (i * 16)]);
      qualityInd.push_back(message[21 + (i * 16)]);
      corrSource.push_back(message[22 + (i * 16)]);
      ionoModel.push_back(message[23 + (i * 16)]);

      uint16_t bitfield
        = (*(reinterpret_cast<uint16_t*> (&message[18 + (i * 16)])));

      health.push_back(bitfield & 0x03);
      prSmoothed.push_back(bitfield >> 2 & 0x01);
      prUsed.push_back(bitfield >> 3 & 0x01);
      crUsed.push_back(bitfield >> 4 & 0x01);
      doUsed.push_back(bitfield >> 5 & 0x01);
      prCorrUsed.push_back(bitfield >> 6 & 0x01);
      crCorrUsed.push_back(bitfield >> 7 & 0x01);
      doCorrUsed.push_back(bitfield >> 8 & 0x01);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool sig::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("SIG"));
}

status::status(std::vector<uint8_t>& message)
{
  update(message);
}

void status::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    gpsFix = message[10];

    gpsFixOk = message[11] & 0x01;
    diffSoln = message[11] >> 1 & 0x01;
    wknSet = message[11] >> 2 & 0x01;
    towSet = message[11] >> 3 & 0x01;

    diffCorr = message[12] & 0x01;
    mapMatching = message[12] >> 6 & 0x07;

    psmState = message[13] & 0x03;
    spoofDetState = message[13] >> 3 & 0x03;

    ttff = (*(reinterpret_cast<uint32_t*> (&message[14])));

    msss = (*(reinterpret_cast<uint32_t*> (&message[18])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool status::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("STATUS"));
}

timebds::timebds(std::vector<uint8_t>& message)
{
  update(message);
}

void timebds::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));
    SOW = (*(reinterpret_cast<uint32_t*> (&message[10])));

    fSOW = (*(reinterpret_cast<int32_t*> (&message[14])));

    week = (*(reinterpret_cast<int16_t*> (&message[18])));

    leapS = (*(reinterpret_cast<int8_t*> (&message[20])));

    sowValid = message[21] & 0x01;
    weekValid = message[21] >> 1 & 0x01;
    leapSValid = message[21] >> 2 & 0x01;

    tAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool timebds::type(std::vector<uint8_t>& message)
{
   return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("TIMEBDS"));
}

timegal::timegal(std::vector<uint8_t>& message)
{
  update(message);
}

void timegal::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));
    galTow = (*(reinterpret_cast<uint32_t*> (&message[10])));

    fGalTow = (*(reinterpret_cast<int32_t*> (&message[14])));

    galWno = (*(reinterpret_cast<int16_t*> (&message[18])));

    leapS = (*(reinterpret_cast<int8_t*> (&message[20])));

    galTowValid = message[21] & 0x01;
    galWnoValid = message[21] >> 1 & 0x01;
    leapSValid = message[21] >> 2 & 0x01;

    tAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool timegal::type(std::vector<uint8_t>& message)
{
   return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("TIMEGAL"));
}

timeglo::timeglo(std::vector<uint8_t>& message)
{
  update(message);
}

void timeglo::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));
    TOD = (*(reinterpret_cast<uint32_t*> (&message[10])));

    fTOD = (*(reinterpret_cast<int32_t*> (&message[14])));

    Nt = (*(reinterpret_cast<uint16_t*> (&message[18])));

    N4 = message[20];

    todValid = message[21] & 0x01;
    dateValid = message[21] >> 1 & 0x01;

    tAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool timeglo::type(std::vector<uint8_t>& message)
{
   return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("TIMEGLO"));
}

timegps::timegps(std::vector<uint8_t>& message)
{
  update(message);
}

void timegps::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    fTOD = (*(reinterpret_cast<int32_t*> (&message[10])));

    week = (*(reinterpret_cast<int16_t*> (&message[14])));

    leapS = (*(reinterpret_cast<int8_t*> (&message[16])));

    towValid = message[17] & 0x01;
    weekValid = message[17] >> 1 & 0x01;
    leapSValid = message[17] >> 2 & 0x01;

    tAcc = (*(reinterpret_cast<uint32_t*> (&message[18])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool timegps::type(std::vector<uint8_t>& message)
{
   return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("TIMEGPS"));
}

timeutc::timeutc(std::vector<uint8_t>& message)
{
  update(message);
}

void timeutc::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));
    tAcc = (*(reinterpret_cast<uint32_t*> (&message[10])));

    nano = (*(reinterpret_cast<int32_t*> (&message[14])));

    year = (*(reinterpret_cast<uint16_t*> (&message[18])));

    month = message[20];
    day = message[21];
    hour = message[22];
    min = message[23];
    sec = message[24];

    validTOW = message[25] & 0x01;
    validWKN = message[25] >> 1 & 0x01;
    validUTC = message[25] >> 2 & 0x01;
    utcStandard = message[25] >> 4 & 0x0f;
  }
  else
    throw std::runtime_error("Message type mismatch");
}

bool timeutc::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == ubx::msg_map.at("NAV").first
      && message[3] == ubx::msg_map.at("NAV").second.at("TIMEUTC"));
}

} // namespace nav

} // namespace ubx

} // namespace cracl
