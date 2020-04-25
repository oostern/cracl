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

#ifndef CRACL_UBLOX_MSG_CLASS_RXM_HPP
#define CRACL_UBLOX_MSG_CLASS_RXM_HPP

#include <cstdint>
#include <vector>

namespace cracl
{

namespace ubx
{

namespace rxm
{

class measx
{
public:
  uint8_t version;

  uint32_t gpsTOW;
  uint32_t gloTOW;
  uint32_t bdsTOW;
  uint32_t qzssTOW;

  uint16_t gpsTOWacc;
  uint16_t gloTOWacc;
  uint16_t bdsTOWacc;
  uint16_t qzssTOWacc;

  uint8_t numSV;
  uint8_t towSet;

  std::vector<uint8_t> gnssId;
  std::vector<uint8_t> svId;
  std::vector<uint8_t> cNo;
  std::vector<uint8_t> mpathIndic;

  std::vector<int32_t> dopplerMS;
  std::vector<int32_t> dopplerHz;

  std::vector<uint16_t> wholeChips;
  std::vector<uint16_t> fracChips;

  std::vector<uint32_t> codePhase;

  std::vector<uint8_t> intCodePhase;
  std::vector<uint8_t> pseuRangeRMSErr;

  measx(){ }

  measx(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::rxm::measx

class rawx
{
public:
  double rcvTow;

  uint16_t week;

  int8_t leapS;

  uint8_t numMeas;
  uint8_t recStat;
  uint8_t leapSec;
  uint8_t clkReset;

  std::vector<double> prMes;
  std::vector<double> cpMes;

  std::vector<float> doMes;

  std::vector<uint8_t> gnssId;
  std::vector<uint8_t> svId;
  std::vector<uint8_t> sigId;
  std::vector<uint8_t> freqId;

  std::vector<uint16_t> locktime;

  std::vector<uint8_t> cno;
  std::vector<uint8_t> prStdev;
  std::vector<uint8_t> cpStdev;
  std::vector<uint8_t> doStdev;
  std::vector<uint8_t> trkStat;

  rawx(){ }

  rawx(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  static bool type(std::vector<uint8_t>& message);

}; // ubx::rxm::rawx

} // namespace rxm

} // namespace ubx

} // namespace cracl

#endif // CRACL_UBLOX_MSG_CLASS_RXM_HPP
