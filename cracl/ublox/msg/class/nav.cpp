// Copyright (C) 2019 Colton Riedel
//               2019 Will Bogardus
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    m_clkB = (*(reinterpret_cast<int32_t*> (&message[10])));
    m_clkD = (*(reinterpret_cast<int32_t*> (&message[14])));

    m_tAcc = (*(reinterpret_cast<uint32_t*> (&message[18])));
    m_fAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t clock::iTOW()
{
  return m_iTOW;
}

int32_t clock::clkB()
{
  return m_clkB;
}

int32_t clock::clkD()
{
  return m_clkD;
}

uint32_t clock::tAcc()
{
  return m_tAcc;
}

uint32_t clock::fAcc()
{
  return m_fAcc;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    m_gDOP = (*(reinterpret_cast<uint16_t*> (&message[10])));
    m_pDOP = (*(reinterpret_cast<uint16_t*> (&message[12])));
    m_tDOP = (*(reinterpret_cast<uint16_t*> (&message[14])));
    m_vDOP = (*(reinterpret_cast<uint16_t*> (&message[16])));
    m_hDOP = (*(reinterpret_cast<uint16_t*> (&message[18])));
    m_nDOP = (*(reinterpret_cast<uint16_t*> (&message[20])));
    m_eDOP = (*(reinterpret_cast<uint16_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t dop::iTOW()
{
  return m_iTOW;
}

uint16_t dop::gDOP()
{
  return m_gDOP;
}

uint16_t dop::pDOP()
{
  return m_pDOP;
}

uint16_t dop::tDOP()
{
  return m_tDOP;
}

uint16_t dop::vDOP()
{
  return m_vDOP;
}

uint16_t dop::hDOP()
{
  return m_hDOP;
}

uint16_t dop::nDOP()
{
  return m_nDOP;
}

uint16_t dop::eDOP()
{
  return m_eDOP;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    m_ecefX = (*(reinterpret_cast<int32_t*> (&message[10])));
    m_ecefY = (*(reinterpret_cast<int32_t*> (&message[14])));
    m_ecefZ = (*(reinterpret_cast<int32_t*> (&message[18])));

    m_pAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t posecef::iTOW()
{
  return m_iTOW;
}

int32_t posecef::ecefX()
{
  return m_ecefX;
}

int32_t posecef::ecefY()
{
  return m_ecefY;
}

int32_t posecef::ecefZ()
{
  return m_ecefZ;
}

uint32_t posecef::pAcc()
{
  return m_pAcc;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    m_lon = (*(reinterpret_cast<int32_t*> (&message[10])));
    m_lat = (*(reinterpret_cast<int32_t*> (&message[14])));
    m_height = (*(reinterpret_cast<int32_t*> (&message[18])));
    m_hMSL = (*(reinterpret_cast<int32_t*> (&message[22])));

    m_hAcc = (*(reinterpret_cast<uint32_t*> (&message[26])));
    m_vAcc = (*(reinterpret_cast<uint32_t*> (&message[30])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t posllh::iTOW()
{
  return m_iTOW;
}

int32_t posllh::lon()
{
  return m_lon;
}

int32_t posllh::lat()
{
  return m_lat;
}

int32_t posllh::height()
{
  return m_height;
}

int32_t posllh::hMSL()
{
  return m_hMSL;
}

uint32_t posllh::hAcc()
{
  return m_hAcc;
}

uint32_t posllh::vAcc()
{
  return m_vAcc;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    m_version = message[10];
    m_numSvs = message[11];

    m_gnssId.clear();
    m_svId.clear();
    m_cno.clear();
    m_elev.clear();
    m_azim.clear();
    m_prRes.clear();
    m_qualityInd.clear();
    m_svUsed.clear();
    m_health.clear();
    m_diffCorr.clear();
    m_smoothed.clear();
    m_orbitSource.clear();
    m_ephAvail.clear();
    m_almAvail.clear();
    m_anoAvail.clear();
    m_aopAvail.clear();
    m_sbasCorrUsed.clear();
    m_rtcmCorrUsed.clear();
    m_prCorrUsed.clear();
    m_crCorrUsed.clear();
    m_doCorrUsed.clear();

    for (size_t i = 0; i < m_numSvs; ++i)
    {
      m_gnssId.push_back(message[14 + (i * 12)]);
      m_svId.push_back(message[15 + (i * 12)]);

      m_cno.push_back(message[16 + (i * 12)]);

      m_elev.push_back(message[17 + (i * 12)]);
      m_azim.push_back(
          *(reinterpret_cast<uint16_t*> (&message[18 + (i * 12)])));
      m_prRes.push_back(
          *(reinterpret_cast<uint16_t*> (&message[20 + (i * 12)])));

      uint32_t bitfield
        = (*(reinterpret_cast<uint32_t*> (&message[22 + (i * 12)])));

      m_qualityInd.push_back(bitfield & 0x07);
      m_svUsed.push_back(bitfield >> 3 & 0x01);
      m_health.push_back(bitfield >> 4 & 0x03);
      m_diffCorr.push_back(bitfield >> 6 & 0x01);
      m_smoothed.push_back(bitfield >> 7 & 0x01);
      m_orbitSource.push_back(bitfield >> 8 & 0x07);
      m_ephAvail.push_back(bitfield >> 11 & 0x01);
      m_almAvail.push_back(bitfield >> 12 & 0x01);
      m_anoAvail.push_back(bitfield >> 13 & 0x01);
      m_aopAvail.push_back(bitfield >> 14 & 0x01);
      m_sbasCorrUsed.push_back(bitfield >> 16 & 0x01);
      m_rtcmCorrUsed.push_back(bitfield >> 17 & 0x01);
      m_prCorrUsed.push_back(bitfield >> 20 & 0x01);
      m_crCorrUsed.push_back(bitfield >> 21 & 0x01);
      m_doCorrUsed.push_back(bitfield >> 22 & 0x01);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t sat::iTOW()
{
  return m_iTOW;
}

uint8_t sat::version()
{
  return m_version;
}

uint8_t sat::numSvs()
{
  return m_numSvs;
}

std::vector<uint8_t> sat::gnssId()
{
  return m_gnssId;
}

std::vector<uint8_t> sat::svId()
{
  return m_svId;
}

std::vector<uint8_t> sat::cno()
{
  return m_cno;
}

std::vector<int8_t> sat::elev()
{
  return m_elev;
}

std::vector<int16_t> sat::azim()
{
  return m_azim;
}

std::vector<int16_t> sat::prRes()
{
  return m_prRes;
}

std::vector<uint8_t> sat::qualityInd()
{
  return m_qualityInd;
}

std::vector<uint8_t> sat::svUsed()
{
  return m_svUsed;
}

std::vector<uint8_t> sat::health()
{
  return m_health;
}

std::vector<uint8_t> sat::diffCorr()
{
  return m_diffCorr;
}

std::vector<uint8_t> sat::smoothed()
{
  return m_smoothed;
}

std::vector<uint8_t> sat::orbitSource()
{
  return m_orbitSource;
}

std::vector<uint8_t> sat::ephAvail()
{
  return m_ephAvail;
}

std::vector<uint8_t> sat::almAvail()
{
  return m_almAvail;
}

std::vector<uint8_t> sat::anoAvail()
{
  return m_anoAvail;
}

std::vector<uint8_t> sat::aopAvail()
{
  return m_aopAvail;
}

std::vector<uint8_t> sat::sbasCorrUsed()
{
  return m_sbasCorrUsed;
}

std::vector<uint8_t> sat::rtcmCorrUsed()
{
  return m_rtcmCorrUsed;
}

std::vector<uint8_t> sat::prCorrUsed()
{
  return m_prCorrUsed;
}

std::vector<uint8_t> sat::crCorrUsed()
{
  return m_crCorrUsed;
}

std::vector<uint8_t> sat::doCorrUsed()
{
  return m_doCorrUsed;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    m_version = message[10];
    m_numSigs = message[11];

    m_gnssId.clear();
    m_svId.clear();
    m_sigId.clear();
    m_freqId.clear();
    m_prRes.clear();
    m_cno.clear();
    m_qualityInd.clear();
    m_corrSource.clear();
    m_ionoModel.clear();
    m_health.clear();
    m_prSmoothed.clear();
    m_prUsed.clear();
    m_crUsed.clear();
    m_doUsed.clear();
    m_prCorrUsed.clear();
    m_crCorrUsed.clear();
    m_doCorrUsed.clear();

    for (size_t i = 0; i < m_numSigs; ++i)
    {
      m_gnssId.push_back(message[14 + (i * 16)]);
      m_svId.push_back(message[15 + (i * 16)]);
      m_sigId.push_back(message[16 + (i * 16)]);
      m_freqId.push_back(message[17 + (i * 16)]);

      m_prRes.push_back(
          *(reinterpret_cast<uint16_t*> (&message[18 + (i * 16)])));

      m_cno.push_back(message[20 + (i * 16)]);
      m_qualityInd.push_back(message[21 + (i * 16)]);
      m_corrSource.push_back(message[22 + (i * 16)]);
      m_ionoModel.push_back(message[23 + (i * 16)]);

      uint16_t bitfield
        = (*(reinterpret_cast<uint16_t*> (&message[18 + (i * 16)])));

      m_health.push_back(bitfield & 0x03);
      m_prSmoothed.push_back(bitfield >> 2 & 0x01);
      m_prUsed.push_back(bitfield >> 3 & 0x01);
      m_crUsed.push_back(bitfield >> 4 & 0x01);
      m_doUsed.push_back(bitfield >> 5 & 0x01);
      m_prCorrUsed.push_back(bitfield >> 6 & 0x01);
      m_crCorrUsed.push_back(bitfield >> 7 & 0x01);
      m_doCorrUsed.push_back(bitfield >> 8 & 0x01);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t sig::iTOW()
{
  return m_iTOW;
}

uint8_t sig::version()
{
  return m_version;
}

uint8_t sig::numSigs()
{
  return m_numSigs;
}

std::vector<uint8_t> sig::gnssId()
{
  return m_gnssId;
}

std::vector<uint8_t> sig::svId()
{
  return m_svId;
}

std::vector<uint8_t> sig::sigId()
{
  return m_sigId;
}

std::vector<uint8_t> sig::freqId()
{
  return m_freqId;
}

std::vector<int16_t> sig::prRes()
{
  return m_prRes;
}

std::vector<uint8_t> sig::cno()
{
  return m_cno;
}

std::vector<uint8_t> sig::qualityInd()
{
  return m_qualityInd;
}

std::vector<uint8_t> sig::corrSource()
{
  return m_corrSource;
}

std::vector<uint8_t> sig::ionoModel()
{
  return m_ionoModel;
}

std::vector<uint8_t> sig::health()
{
  return m_health;
}

std::vector<uint8_t> sig::prSmoothed()
{
  return m_prSmoothed;
}

std::vector<uint8_t> sig::prUsed()
{
  return m_prUsed;
}

std::vector<uint8_t> sig::crUsed()
{
  return m_crUsed;
}

std::vector<uint8_t> sig::doUsed()
{
  return m_doUsed;
}

std::vector<uint8_t> sig::prCorrUsed()
{
  return m_prCorrUsed;
}

std::vector<uint8_t> sig::crCorrUsed()
{
  return m_crCorrUsed;
}

std::vector<uint8_t> sig::doCorrUsed()
{
  return m_doCorrUsed;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    m_gpsFix = message[10];

    m_gpsFixOk = message[11] & 0x01;
    m_diffSoln = message[11] >> 1 & 0x01;
    m_wknSet = message[11] >> 2 & 0x01;
    m_towSet = message[11] >> 3 & 0x01;

    m_diffCorr = message[12] & 0x01;
    m_mapMatching = message[12] >> 6 & 0x07;

    m_psmState = message[13] & 0x03;
    m_spoofDetState = message[13] >> 3 & 0x03;

    m_ttff = (*(reinterpret_cast<uint32_t*> (&message[14])));

    m_msss = (*(reinterpret_cast<uint32_t*> (&message[18])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t status::iTOW()
{
  return m_iTOW;
}

uint8_t status::gpsFix()
{
  return m_gpsFix;
}

uint8_t status::gpsFixOk()
{
  return m_gpsFixOk;
}

uint8_t status::diffSoln()
{
  return m_diffSoln;
}

uint8_t status::wknSet()
{
  return m_wknSet;
}

uint8_t status::towSet()
{
  return m_towSet;
}

uint8_t status::diffCorr()
{
  return m_diffCorr;
}

uint8_t status::mapMatching()
{
  return m_mapMatching;
}

uint8_t status::psmState()
{
  return m_psmState;
}

uint8_t status::spoofDetState()
{
  return m_spoofDetState;
}

uint32_t status::ttff()
{
  return m_ttff;
}

uint32_t status::msss()
{
  return m_msss;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));
    m_SOW = (*(reinterpret_cast<uint32_t*> (&message[10])));

    m_fSOW = (*(reinterpret_cast<int32_t*> (&message[14])));

    m_week = (*(reinterpret_cast<int16_t*> (&message[18])));

    m_leapS = (*(reinterpret_cast<int8_t*> (&message[20])));

    m_sowValid = message[21] & 0x01;
    m_weekValid = message[21] >> 1 & 0x01;
    m_leapSValid = message[21] >> 2 & 0x01;

    m_tAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t timebds::iTOW()
{
  return m_iTOW;
}

uint32_t timebds::sow()
{
  return m_SOW;
}

int32_t timebds::fSOW()
{
  return m_fSOW;
}

int16_t timebds::week()
{
  return m_week;
}

int8_t timebds::leapS()
{
  return m_leapS;
}

uint8_t timebds::sowValid()
{
  return m_sowValid;
}

uint8_t timebds::weekValid()
{
  return m_weekValid;
}

uint8_t timebds::leapSValid()
{
  return m_leapSValid;
}

uint32_t timebds::tAcc()
{
  return m_tAcc;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));
    m_galTow = (*(reinterpret_cast<uint32_t*> (&message[10])));

    m_fGalTow = (*(reinterpret_cast<int32_t*> (&message[14])));

    m_galWno = (*(reinterpret_cast<int16_t*> (&message[18])));

    m_leapS = (*(reinterpret_cast<int8_t*> (&message[20])));

    m_galTowValid = message[21] & 0x01;
    m_galWnoValid = message[21] >> 1 & 0x01;
    m_leapSValid = message[21] >> 2 & 0x01;

    m_tAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t timegal::iTOW()
{
  return m_iTOW;
}

uint32_t timegal::galTow()
{
  return m_galTow;
}

int32_t timegal::fGalTow()
{
  return m_fGalTow;
}

int16_t timegal::galWno()
{
  return m_galWno;
}

int8_t timegal::leapS()
{
  return m_leapS;
}

uint8_t timegal::galTowValid()
{
  return m_galTowValid;
}

uint8_t timegal::galWnoValid()
{
  return m_galWnoValid;
}

uint8_t timegal::leapSValid()
{
  return m_leapSValid;
}

uint32_t timegal::tAcc()
{
  return m_tAcc;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));
    m_TOD = (*(reinterpret_cast<uint32_t*> (&message[10])));

    m_fTOD = (*(reinterpret_cast<int32_t*> (&message[14])));

    m_Nt = (*(reinterpret_cast<uint16_t*> (&message[18])));

    m_N4 = message[20];

    m_todValid = message[21] & 0x01;
    m_dateValid = message[21] >> 1 & 0x01;

    m_tAcc = (*(reinterpret_cast<uint32_t*> (&message[22])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t timeglo::iTOW()
{
  return m_iTOW;
}

uint32_t timeglo::tod()
{
  return m_TOD;
}

int32_t timeglo::fTOD()
{
  return m_fTOD;
}

uint16_t timeglo::nt()
{
  return m_Nt;
}

uint8_t timeglo::n4()
{
  return m_N4;
}

uint8_t timeglo::todValid()
{
  return m_todValid;
}

uint8_t timeglo::dateValid()
{
  return m_dateValid;
}

uint32_t timeglo::tAcc()
{
  return m_tAcc;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

    m_fTOD = (*(reinterpret_cast<int32_t*> (&message[10])));

    m_week = (*(reinterpret_cast<int16_t*> (&message[14])));

    m_leapS = (*(reinterpret_cast<int8_t*> (&message[16])));

    m_towValid = message[17] & 0x01;
    m_weekValid = message[17] >> 1 & 0x01;
    m_leapSValid = message[17] >> 2 & 0x01;

    m_tAcc = (*(reinterpret_cast<uint32_t*> (&message[18])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t timegps::iTOW()
{
  return m_iTOW;
}

int32_t timegps::fTOD()
{
  return m_fTOD;
}

int16_t timegps::week()
{
  return m_week;
}

int8_t timegps::leapS()
{
  return m_leapS;
}

uint8_t timegps::towValid()
{
  return m_towValid;
}

uint8_t timegps::weekValid()
{
  return m_weekValid;
}

uint8_t timegps::leapSValid()
{
  return m_leapSValid;
}

uint32_t timegps::tAcc()
{
  return m_tAcc;
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
    m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));
    m_tAcc = (*(reinterpret_cast<uint32_t*> (&message[10])));

    m_nano = (*(reinterpret_cast<int32_t*> (&message[14])));

    m_year = (*(reinterpret_cast<uint16_t*> (&message[18])));

    m_month = message[20];
    m_day = message[21];
    m_hour = message[22];
    m_min = message[23];
    m_sec = message[24];

    m_validTOW = message[25] & 0x01;
    m_validWKN = message[25] >> 1 & 0x01;
    m_validUTC = message[25] >> 2 & 0x01;
    m_utcStandard = message[25] >> 4 & 0x0f;
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t timeutc::iTOW()
{
  return m_iTOW;
}

uint32_t timeutc::tAcc()
{
  return m_tAcc;
}

int32_t timeutc::nano()
{
  return m_nano;
}

uint16_t timeutc::year()
{
  return m_year;
}

uint8_t timeutc::month()
{
  return m_month;
}

uint8_t timeutc::day()
{
  return m_day;
}

uint8_t timeutc::hour()
{
  return m_hour;
}

uint8_t timeutc::min()
{
  return m_min;
}

uint8_t timeutc::sec()
{
  return m_sec;
}

uint8_t timeutc::validTOW()
{
  return m_validTOW;
}

uint8_t timeutc::validWKN()
{
  return m_validWKN;
}

uint8_t timeutc::validUTC()
{
  return m_validUTC;
}

uint8_t timeutc::utcStandard()
{
  return m_utcStandard;
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
