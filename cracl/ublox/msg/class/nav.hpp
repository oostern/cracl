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

#ifndef CRACL_UBLOX_MSG_CLASS_NAV_HPP
#define CRACL_UBLOX_MSG_CLASS_NAV_HPP

#include <cstdint>
#include <vector>

namespace cracl
{

namespace ubx
{

namespace nav
{

class clock
{
public:
  uint32_t iTOW;

  int32_t clkB;
  int32_t clkD;

  uint32_t tAcc;
  uint32_t fAcc;

  clock(){ }

  clock(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::clock

class dop
{
public:
  uint32_t iTOW;

  uint16_t gDOP;
  uint16_t pDOP;
  uint16_t tDOP;
  uint16_t vDOP;
  uint16_t hDOP;
  uint16_t nDOP;
  uint16_t eDOP;

  dop(){ }

  dop(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::dop

class posecef
{
public:
  uint32_t iTOW;

  int32_t ecefX;
  int32_t ecefY;
  int32_t ecefZ;

  uint32_t pAcc;

  posecef(){ }

  posecef(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::posecef

class posllh
{
public:
  uint32_t iTOW;

  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;

  uint32_t hAcc;
  uint32_t vAcc;

  posllh(){ }

  posllh(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::posllh

class sat
{
public:
  uint32_t iTOW;

  uint8_t version;
  uint8_t numSvs;

  std::vector<uint8_t> gnssId;
  std::vector<uint8_t> svId;
  std::vector<uint8_t> cno;

  std::vector<int8_t> elev;

  std::vector<int16_t> azim;
  std::vector<int16_t> prRes;

  std::vector<uint8_t> qualityInd;
  std::vector<uint8_t> svUsed;
  std::vector<uint8_t> health;
  std::vector<uint8_t> diffCorr;
  std::vector<uint8_t> smoothed;
  std::vector<uint8_t> orbitSource;
  std::vector<uint8_t> ephAvail;
  std::vector<uint8_t> almAvail;
  std::vector<uint8_t> anoAvail;
  std::vector<uint8_t> aopAvail;
  std::vector<uint8_t> sbasCorrUsed;
  std::vector<uint8_t> rtcmCorrUsed;
  std::vector<uint8_t> prCorrUsed;
  std::vector<uint8_t> crCorrUsed;
  std::vector<uint8_t> doCorrUsed;

  sat(){ }

  sat(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::sat

class sig
{
public:
  uint32_t iTOW;

  uint8_t version;
  uint8_t numSigs;

  std::vector<uint8_t> gnssId;
  std::vector<uint8_t> svId;
  std::vector<uint8_t> sigId;
  std::vector<uint8_t> freqId;

  std::vector<int16_t> prRes;

  std::vector<uint8_t> cno;
  std::vector<uint8_t> qualityInd;
  std::vector<uint8_t> corrSource;
  std::vector<uint8_t> ionoModel;

  std::vector<uint8_t> health;
  std::vector<uint8_t> prSmoothed;
  std::vector<uint8_t> prUsed;
  std::vector<uint8_t> crUsed;
  std::vector<uint8_t> doUsed;
  std::vector<uint8_t> prCorrUsed;
  std::vector<uint8_t> crCorrUsed;
  std::vector<uint8_t> doCorrUsed;

  sig(){ }

  sig(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::sig

class status
{
public:
  uint32_t iTOW;

  uint8_t gpsFix;
  uint8_t gpsFixOk;
  uint8_t diffSoln;
  uint8_t wknSet;
  uint8_t towSet;
  uint8_t diffCorr;
  uint8_t mapMatching;
  uint8_t psmState;
  uint8_t spoofDetState;

  uint32_t ttff;
  uint32_t msss;

  status(){ }

  status(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::status

class timebds
{
public:
  uint32_t iTOW;
  uint32_t SOW;

  int32_t fSOW;

  int16_t week;

  int8_t leapS;

  uint8_t sowValid;
  uint8_t weekValid;
  uint8_t leapSValid;

  uint32_t tAcc;

  timebds(){ }

  timebds(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timebds

class timegal
{
public:
  uint32_t iTOW;
  uint32_t galTow;

  int32_t fGalTow;

  int16_t galWno;

  int8_t leapS;

  uint8_t galTowValid;
  uint8_t galWnoValid;
  uint8_t leapSValid;

  uint32_t tAcc;

  timegal(){ }

  timegal(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timegal

class timeglo
{
public:
  uint32_t iTOW;
  uint32_t TOD;

  int32_t fTOD;

  uint16_t Nt;

  uint8_t N4;
  uint8_t todValid;
  uint8_t dateValid;

  uint32_t tAcc;

  timeglo(){ }

  timeglo(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timeglo

class timegps
{
public:
  uint32_t iTOW;

  int32_t fTOD;

  int16_t week;

  int8_t leapS;

  uint8_t towValid;
  uint8_t weekValid;
  uint8_t leapSValid;

  uint32_t tAcc;

  timegps(){ }

  timegps(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timegps

class timeutc
{
public:
  uint32_t iTOW;
  uint32_t tAcc;

  int32_t nano;

  uint16_t year;

  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;

  uint8_t validTOW;
  uint8_t validWKN;
  uint8_t validUTC;
  uint8_t utcStandard;

  timeutc(){ }

  timeutc(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timeutc

} // namespace nav

} // namespace ubx

} // namespace cracl

#endif // CRACL_UBLOX_MSG_CLASS_NAV_HPP
