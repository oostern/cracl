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
  uint32_t m_iTOW;

  int32_t m_clkB;
  int32_t m_clkD;

  uint32_t m_tAcc;
  uint32_t m_fAcc;

public:
  clock(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  int32_t clkB();

  int32_t clkD();

  uint32_t tAcc();

  uint32_t fAcc();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::clock

class dop
{
  uint32_t m_iTOW;

  uint16_t m_gDOP;
  uint16_t m_pDOP;
  uint16_t m_tDOP;
  uint16_t m_vDOP;
  uint16_t m_hDOP;
  uint16_t m_nDOP;
  uint16_t m_eDOP;

public:
  dop(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  uint16_t gDOP();

  uint16_t pDOP();

  uint16_t tDOP();

  uint16_t vDOP();

  uint16_t hDOP();

  uint16_t nDOP();

  uint16_t eDOP();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::dop

class posecef
{
  uint32_t m_iTOW;

  int32_t m_ecefX;
  int32_t m_ecefY;
  int32_t m_ecefZ;

  uint32_t m_pAcc;

public:
  posecef(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  int32_t ecefX();

  int32_t ecefY();

  int32_t ecefZ();

  uint32_t pAcc();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::posecef

class sat
{
  uint32_t m_iTOW;

  uint8_t m_version;
  uint8_t m_numSvs;

  std::vector<uint8_t> m_gnssId;
  std::vector<uint8_t> m_svId;
  std::vector<uint8_t> m_cno;

  std::vector<int8_t> m_elev;

  std::vector<int16_t> m_azim;
  std::vector<int16_t> m_prRes;

  std::vector<uint8_t> m_qualityInd;
  std::vector<uint8_t> m_svUsed;
  std::vector<uint8_t> m_health;
  std::vector<uint8_t> m_diffCorr;
  std::vector<uint8_t> m_smoothed;
  std::vector<uint8_t> m_orbitSource;
  std::vector<uint8_t> m_ephAvail;
  std::vector<uint8_t> m_almAvail;
  std::vector<uint8_t> m_anoAvail;
  std::vector<uint8_t> m_aopAvail;
  std::vector<uint8_t> m_sbasCorrUsed;
  std::vector<uint8_t> m_rtcmCorrUsed;
  std::vector<uint8_t> m_prCorrUsed;
  std::vector<uint8_t> m_crCorrUsed;
  std::vector<uint8_t> m_doCorrUsed;

public:
  sat(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  uint8_t version();

  uint8_t numSvs();

  std::vector<uint8_t> gnssId();

  std::vector<uint8_t> svId();

  std::vector<uint8_t> cno();

  std::vector<int8_t> elev();

  std::vector<int16_t> azim();

  std::vector<int16_t> prRes();

  std::vector<uint8_t> qualityInd();

  std::vector<uint8_t> svUsed();

  std::vector<uint8_t> health();

  std::vector<uint8_t> diffCorr();

  std::vector<uint8_t> smoothed();

  std::vector<uint8_t> orbitSource();

  std::vector<uint8_t> ephAvail();

  std::vector<uint8_t> almAvail();

  std::vector<uint8_t> anoAvail();

  std::vector<uint8_t> aopAvail();

  std::vector<uint8_t> sbasCorrUsed();

  std::vector<uint8_t> rtcmCorrUsed();

  std::vector<uint8_t> prCorrUsed();

  std::vector<uint8_t> crCorrUsed();

  std::vector<uint8_t> doCorrUsed();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::sat

class sig
{
  uint32_t m_iTOW;

  uint8_t m_version;
  uint8_t m_numSigs;

  std::vector<uint8_t> m_gnssId;
  std::vector<uint8_t> m_svId;
  std::vector<uint8_t> m_sigId;
  std::vector<uint8_t> m_freqId;

  std::vector<int16_t> m_prRes;

  std::vector<uint8_t> m_cno;
  std::vector<uint8_t> m_qualityInd;
  std::vector<uint8_t> m_corrSource;
  std::vector<uint8_t> m_ionoModel;

public:
  sig(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  uint8_t version();

  uint8_t numSigs();

  std::vector<uint8_t> gnssId();

  std::vector<uint8_t> svId();

  std::vector<uint8_t> sigId();

  std::vector<uint8_t> freqId();

  std::vector<uint8_t> cno();

  std::vector<int16_t> prRes();

  std::vector<uint8_t> qualityInd();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::sig

class status
{
  uint32_t m_iTOW;

  uint8_t m_gpsFix;
  uint8_t m_gpsFixOk;
  uint8_t m_diffSoln;
  uint8_t m_wknSet;
  uint8_t m_towSet;
  uint8_t m_diffCorr;
  uint8_t m_mapMatching;
  uint8_t m_psmState;
  uint8_t m_spoofDetState;

  uint32_t m_ttff;
  uint32_t m_msss;

public:
  status(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  uint8_t gpsFix();

  uint8_t gpsFixOk();

  uint8_t diffSoln();

  uint8_t wknSet();

  uint8_t towSet();

  uint8_t diffCorr();

  uint8_t mapMatching();

  uint8_t psmState();

  uint8_t spoofDetState();

  uint32_t ttff();

  uint32_t msss();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::status

class timebds
{
  uint32_t m_iTOW;
  uint32_t m_SOW;

  int32_t m_fSOW;

  int16_t m_week;

  int8_t m_leapS;

  uint8_t m_sowValid;
  uint8_t m_weekValid;
  uint8_t m_leapSValid;

  uint32_t m_tAcc;

public:
  timebds(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  uint32_t sow();

  int32_t fSOW();

  int16_t week();

  int8_t leapS();

  uint8_t sowValid();

  uint8_t weekValid();

  uint8_t leapSValid();

  uint32_t tAcc();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timebds

class timegal
{
  uint32_t m_iTOW;
  uint32_t m_galTow;

  int32_t m_fGalTow;

  int16_t m_galWno;

  int8_t m_leapS;

  uint8_t m_galTowValid;
  uint8_t m_galWnoValid;
  uint8_t m_leapSValid;

  uint32_t m_tAcc;

public:
  timegal(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  uint32_t galTow();

  int32_t fGalTow();

  int16_t galWno();

  int8_t leapS();

  uint8_t galTowValid();

  uint8_t galWnoValid();

  uint8_t leapSValid();

  uint32_t tAcc();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timegal

class timeglo
{
  uint32_t m_iTOW;
  uint32_t m_TOD;

  int32_t m_fTOD;

  uint16_t m_Nt;

  uint8_t m_N4;
  uint8_t m_todValid;
  uint8_t m_dateValid;

  uint32_t m_tAcc;

public:
  timeglo(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  uint32_t tod();

  int32_t fTOD();

  uint16_t nt();

  uint8_t n4();

  uint8_t todValid();

  uint8_t dateValid();

  uint32_t tAcc();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timeglo

class timegps
{
  uint32_t m_iTOW;

  int32_t m_fTOD;

  int16_t m_week;

  int8_t m_leapS;

  uint8_t m_towValid;
  uint8_t m_weekValid;
  uint8_t m_leapSValid;

  uint32_t m_tAcc;

public:
  timegps(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  int32_t fTOD();

  int16_t week();

  int8_t leapS();

  uint8_t towValid();

  uint8_t weekValid();

  uint8_t leapSValid();

  uint32_t tAcc();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timegps

class timeutc
{
  uint32_t m_iTOW;
  uint32_t m_tAcc;

  int32_t m_nano;

  uint16_t m_year;

  uint8_t m_month;
  uint8_t m_day;
  uint8_t m_hour;
  uint8_t m_min;
  uint8_t m_sec;

  uint8_t m_validTOW;
  uint8_t m_validWKN;
  uint8_t m_validUTC;
  uint8_t m_utcStandard;

public:
  timeutc(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint32_t iTOW();

  uint32_t tAcc();

  int32_t nano();

  uint16_t year();

  uint8_t month();

  uint8_t day();

  uint8_t hour();

  uint8_t min();

  uint8_t sec();

  uint8_t validTOW();

  uint8_t validWKN();

  uint8_t validUTC();

  uint8_t utcStandard();

  static bool type(std::vector<uint8_t>& message);

}; // ubx::nav::timeutc

} // namespace nav

} // namespace ubx

} // namespace cracl

#endif // CRACL_UBLOX_MSG_CLASS_NAV_HPP
