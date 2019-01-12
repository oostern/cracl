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

} // namespace mon

} // namespace ubx

} // namespace cracl

#endif // CRACL_UBLOX_MSG_CLASS_MON_HPP
