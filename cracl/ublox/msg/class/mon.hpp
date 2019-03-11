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

#ifndef CRACL_UBLOX_MSG_CLASS_MON_HPP
#define CRACL_UBLOX_MSG_CLASS_MON_HPP

#include <array>
#include <cstdint>
#include <vector>

namespace cracl
{

namespace ubx
{

namespace mon
{

class hw
{
  uint32_t m_pinSel;
  uint32_t m_pinBank;
  uint32_t m_pinDir;
  uint32_t m_pinVal;

  uint16_t m_noisePerMS;
  uint16_t m_agcCnt;

  uint8_t m_aStatus;
  uint8_t m_aPower;

  uint8_t m_rtcCalib;
  uint8_t m_safeBoot;
  uint8_t m_jammingState;
  uint8_t m_xtalAbsent;

  uint32_t m_usedMask;

  std::array<uint8_t, 17> m_vp;

  uint8_t m_jamInd;

  uint32_t m_pinIrq;
  uint32_t m_pullH;
  uint32_t m_pullL;

public:
  hw(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t pinSel();

  uint32_t pinBank();

  uint32_t pinDir();

  uint32_t pinVal();

  uint16_t noisePerMS();

  uint16_t agcCnt();

  uint8_t aStatus();

  uint8_t aPower();

  uint8_t rtcCalib();

  uint8_t safeBoot();

  uint8_t jammingState();

  uint8_t xtalAbsent();

  uint32_t usedMask();

  std::array<uint8_t, 17> vp();

  uint8_t jamInd();

  uint32_t pinIrq();

  uint32_t pullH();

  uint32_t pullL();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::mon::hw

class rf
{
  uint8_t m_version;
  uint8_t m_nBlocks;

  std::vector<uint8_t> m_blockId;
  std::vector<uint8_t> m_jammingState;
  std::vector<uint8_t> m_antStatus;
  std::vector<uint8_t> m_antPower;

  std::vector<uint32_t> m_postStatus;

  std::vector<uint16_t> m_noisePerMS;
  std::vector<uint16_t> m_agcCnt;

  std::vector<uint8_t> m_jamInd;

  std::vector<int8_t> m_ofsI;

  std::vector<uint8_t> m_magI;

  std::vector<int8_t> m_ofsQ;

  std::vector<uint8_t> m_magQ;

public:
  rf(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint8_t version();

  uint8_t nBlocks();

  std::vector<uint8_t> blockId();

  std::vector<uint8_t> jammingState();

  std::vector<uint8_t> antStatus();

  std::vector<uint32_t> postStatus();

  std::vector<uint16_t> noisePerMS();

  std::vector<uint16_t> agcCnt();

  std::vector<uint8_t> jamInd();

  std::vector<int8_t> ofsI();

  std::vector<uint8_t> magI();

  std::vector<int8_t> ofsQ();

  std::vector<uint8_t> magQ();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::mon::rf

} // namespace mon

} // namespace ubx

} // namespace cracl

#endif // CRACL_UBLOX_MSG_CLASS_MON_HPP
