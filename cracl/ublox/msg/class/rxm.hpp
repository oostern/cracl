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
  uint8_t m_version;

  uint32_t m_gpsTOW;
  uint32_t m_gloTOW;
  uint32_t m_bdsTOW;
  uint32_t m_qzssTOW;

  uint16_t m_gpsTOWacc;
  uint16_t m_gloTOWacc;
  uint16_t m_bdsTOWacc;
  uint16_t m_qzssTOWacc;

  uint8_t m_numSV;
  uint8_t m_towSet;

  std::vector<uint8_t> m_gnssId;
  std::vector<uint8_t> m_svId;
  std::vector<uint8_t> m_cNo;
  std::vector<uint8_t> m_mpathIndic;

  std::vector<int32_t> m_dopplerMS;
  std::vector<int32_t> m_dopplerHz;

  std::vector<uint16_t> m_wholeChips;
  std::vector<uint16_t> m_fracChips;

  std::vector<uint32_t> m_codePhase;

  std::vector<uint8_t> m_intCodePhase;
  std::vector<uint8_t> m_pseuRangeRMSErr;

public:
  measx(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint8_t version();

  uint32_t gpsTOW();

  uint32_t gloTOW();

  uint32_t bdsTOW();

  uint32_t qzssTOW();

  uint16_t gpsTOWacc();

  uint16_t gloTOWacc();

  uint16_t bdsTOWacc();

  uint16_t qzssTOWacc();

  uint8_t numSV();

  uint8_t towSet();

  std::vector<uint8_t> gnssId();

  std::vector<uint8_t> svId();

  std::vector<uint8_t> cNo();

  std::vector<uint8_t> mpathIndic();

  std::vector<int32_t> dopplerMS();

  std::vector<int32_t> dopplerHz();

  std::vector<uint16_t> wholeChips();

  std::vector<uint16_t> fracChips();

  std::vector<uint32_t> codePhase();

  std::vector<uint8_t> intCodePhase();

  std::vector<uint8_t> pseuRangeRMSErr();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::rxm::measx

class rawx
{
  double m_rcvTow;

  uint16_t m_week;

  int8_t m_leapS;

  uint8_t m_numMeas;
  uint8_t m_recStat;
  uint8_t m_leapSec;
  uint8_t m_clkReset;

  std::vector<double> m_prMes;
  std::vector<double> m_cpMes;

  std::vector<float> m_doMes;

  std::vector<uint8_t> m_gnssId;
  std::vector<uint8_t> m_svId;
  std::vector<uint8_t> m_freqId;

  std::vector<uint16_t> m_locktime;

  std::vector<uint8_t> m_cno;
  std::vector<uint8_t> m_prStdev;
  std::vector<uint8_t> m_cpStdev;
  std::vector<uint8_t> m_doStdev;
  std::vector<uint8_t> m_trkStat;

public:
  rawx(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  double rcvTow();

  uint16_t week();

  int8_t leapS();

  uint8_t numMeas();

  uint8_t recStat();

  uint8_t leapSec();

  uint8_t clkReset();

  std::vector<double> prMes();

  std::vector<double> cpMes();

  std::vector<float> doMes();

  std::vector<uint8_t> gnssId();

  std::vector<uint8_t> svId();

  std::vector<uint8_t> freqId();

  std::vector<uint16_t> locktime();

  std::vector<uint8_t> cno();

  std::vector<uint8_t> prStdev();

  std::vector<uint8_t> cpStdev();

  std::vector<uint8_t> doStdev();

  std::vector<uint8_t> trkStat();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::rxm::rawx

} // namespace rxm

} // namespace ubx

} // namespace cracl

#endif // CRACL_UBLOX_MSG_CLASS_RXM_HPP
