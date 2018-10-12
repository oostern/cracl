#ifndef CRACL_RECEIVER_UBLOX_HPP
#define CRACL_RECEIVER_UBLOX_HPP

#include "../device.hpp"

#include <deque>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

namespace cracl
{

const std::map<std::string, std::pair<uint8_t, std::map<std::string, uint8_t>>>
  msg_map = {
    { "ACK",
      { 0x05,
        {
          { "ACK", 0x01 },
          { "NAK", 0x00 }
        }
      }
    },
    { "AID",
      { 0x0b,
        {
          { "ALM", 0x30 },
          { "AOP", 0x30 },
          { "EPH", 0x31 },
          { "HUI", 0x02 },
          { "INI", 0x01 }
        }
      }
    },
    { "CFG",
      { 0x06,
        {
          { "ANT", 0x13 },
          { "CFG", 0x09 },
          { "DAT", 0x06 },
          { "DGNSS", 0x70 },
          { "DOSC", 0x61 },
          { "DYNSEED", 0x85 },
          { "ESRC", 0x60 },
          { "FIXSEED", 0x84 },
          { "GEOFENCE", 0x69 },
          { "GNSS", 0x3E },
          { "HNR", 0x5c },
          { "INF", 0x02 },
          { "IFTM", 0x39 },
          { "LOGFILTER", 0x47 },
          { "MSG", 0x01 },
          { "NAV5", 0x24 },
          { "NAVX5", 0x23 },
          { "NMEA", 0x17 },
          { "ODO", 0x1e },
          { "PM2", 0x3b },
          { "PRT", 0x00 },
          { "PWR", 0x57 },
          { "RATE", 0x08 },
          { "RINV", 0x34 },
          { "RST", 0x04 },
          { "RXM", 0x11 },
          { "SBAS", 0x16 },
          { "SMGR", 0x62 },
          { "TMODE2", 0x3d },
          { "TMODE3", 0x71 },
          { "TP5", 0x31 },
          { "TXSLOT", 0x53 },
          { "USB", 0x1b }
        }
      }
    },
    { "ESF",
      { 0x10,
        {
          { "INS", 0x15 },
          { "MEAS", 0x02 },
          { "RAW", 0x03 },
          { "STATUS", 0x10 }
        }
      }
    },
    { "HNR",
      { 0x28,
        {
          { "PVT", 0x00 }
        }
      }
    },
    { "INF",
      { 0x04,
        {
          { "DEBUG", 0x04 },
          { "ERROR", 0x00 },
          { "NOTICE", 0x02 },
          { "TEST", 0x03 },
          { "WARNING", 0x01 }
        }
      }
    },
    { "LOG",
      { 0x21,
        {
          { "CREATE", 0x07 },
          { "ERASE", 0x03 },
          { "FINDTIME", 0x0e },
          { "INFO", 0x08 },
          { "RETRIEVEPOSEXTRA", 0x0f },
          { "RETRIEVEPOS", 0x0b },
          { "RETRIEVESTRING", 0x0d },
          { "RETRIEVE", 0x09 },
          { "STRING", 0x04 }
        }
      }
    },
    { "MGA",
      { 0x13,
        {
          { "ACK", 0x60 },
          { "ANO", 0x20 },
          { "BDS", 0x03 },
          { "DBD", 0x80 },
          { "FLASH", 0x21 },
          { "GAL", 0x02 },
          { "GLO", 0x06 },
          { "GPS", 0x00 },
          { "INI", 0x40 },
          { "QZSS", 0x05 }
        }
      }
    },
    { "MON",
      { 0x0a,
        {
          { "GNSS", 0x28 },
          { "HW2", 0x0b },
          { "HW", 0x09 },
          { "IO", 0x02 },
          { "MSGPP", 0x06 },
          { "PATCH", 0x27 },
          { "RXBUF", 0x07 },
          { "RXR", 0x21 },
          { "SMGR", 0x2e },
          { "TXBUF", 0x08 },
          { "VER", 0x04 }
        }
      }
    },
    { "NAV",
      { 0x01,
        {
          { "AOPSTATUS", 0x60 },
          { "ATT", 0x05 },
          { "CLOCK", 0x22 },
          { "DGPS", 0x31 },
          { "DOP", 0x04 },
          { "EOE", 0x61 },
          { "GEOFENCE", 0x39 },
          { "HPPOSECEF", 0x13 },
          { "HPPOSLLH", 0x14 },
          { "ODO", 0x09 },
          { "ORB", 0x34 },
          { "POSECEF", 0x01 },
          { "POSLLH", 0x02 },
          { "PVT", 0x07 },
          { "RELPOSNED", 0x3c },
          { "RESETODO", 0x10 },
          { "SAT", 0x35 },
          { "SBAS", 0x32 },
          { "SOL", 0x06 },
          { "STATUS", 0x03 },
          { "SVINFO", 0x30 },
          { "SVIN", 0x3b },
          { "TIMEBDS", 0x24 },
          { "TIMEGAL", 0x25 },
          { "TIMEGLO", 0x23 },
          { "TIMEGPS", 0x20 },
          { "TIMELS", 0x26 },
          { "TIMEUTC", 0x21 },
          { "VELECEF", 0x11 },
          { "VELNED", 0x12 }
        }
      }
    },
    { "RXM",
      { 0x02,
        {
          { "IMES", 0x61 },
          { "MEASX", 0x14 },
          { "PMREQ", 0x41 },
          { "RAWX", 0x15 },
          { "RLM", 0x59 },
          { "RTCM", 0x32 },
          { "SFRBX", 0x13 },
          { "SVSI", 0x20 }
        }
      }
    },
    { "SEC",
      { 0x27,
        {
          { "SIGN", 0x01 },
          { "UNIQID", 0x03 }
        }
      }
    },
    { "TIM",
      { 0x0d,
        {
          { "DOSC", 0x11 },
          { "FCHG", 0x16 },
          { "HOC", 0x17 },
          { "SMEAS", 0x13 },
          { "SVIN", 0x04 },
          { "TM2", 0x03 },
          { "TOS", 0x12 },
          { "TP", 0x01 },
          { "VCOCAL", 0x15 },
          { "VRFY", 0x06 }
        }
      }
    },
    { "UPD",
      { 0x09,
        {
          { "SOS", 0x14 }
        }
      }
    },
    { "PUBX",
      { 0xF1,
        {
          { "CONFIG", 0x41 },
          { "POSITION", 0x00 },
          { "RATE", 0x40 },
          { "SVSTATUS", 0x03 },
          { "TIME", 0x04 }
        }
      }
    }
  };

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

} // namespace nav

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

class ublox_8 : public device
{
  std::vector<uint8_t> m_local_buf;

  std::deque<std::vector<uint8_t>> m_ubx_buffer;
  std::deque<std::vector<uint8_t>> m_nmea_buffer;

  size_t payload_size()
  {
    return 0;
  }

  template <typename T>
  size_t payload_size(T t)
  {
    return sizeof (T);
  }

  template <typename T, typename... Args>
  size_t payload_size(T t, Args... args)
  {
    return sizeof (T) + payload_size(args...);
  }

  void add_pubx_payload(std::vector<char> &message) { }

  template <typename... Args>
  void add_pubx_payload(std::vector<char> &message, const char* t, Args... args)
  {
    message.push_back(',');

    for (const char* ch = t; *ch != 0x00; ++ch)
      message.push_back(*ch);

    add_pubx_payload(message, args...);
  }

  template <typename T, typename... Args>
  void add_pubx_payload(std::vector<char> &message, T t, Args... args)
  {
    auto temp = std::to_string(t);

    message.push_back(',');

    for (auto const& ch : temp)
      message.push_back(ch);

    add_pubx_payload(message, args...);
  }

  void add_ubx_payload(std::vector<uint8_t> &message) { }

  template <typename T, typename... Args>
  void add_ubx_payload(std::vector<uint8_t> &message, T t, Args... args)
  {
    uint8_t *x = reinterpret_cast<uint8_t *>(&t);

    // For little endian systems, reverse loop for big endian
    for (size_t i = 0; i < sizeof (T); ++i)
      message.push_back(x[i]);

    add_ubx_payload(message, args...);
  }

public:
  ublox_8(const std::string& location, size_t baud_rate=9600,
      size_t timeout=500, size_t char_size=8, std::string delim="\r\n",
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one);

  void buffer_messages();

  size_t nmea_queued();

  size_t ubx_queued();

  std::vector<uint8_t> fetch_nmea();

  std::vector<uint8_t> fetch_ubx();

  std::vector<uint8_t> fetch_ubx(std::string&& msg_class, std::string&& msg_id,
      bool first_try=true);

  void flush_nmea();

  void flush_ubx();

  template <typename... Args>
  void pubx_send(std::string&& msg_id, Args... args)
  {
    uint8_t checksum = 0x00;
    std::vector<char> message = { '$', 'P', 'U', 'B', 'X', ',' };

    uint8_t a = (msg_map.at("PUBX").second.at(msg_id) & 0xf0) >> 4;
    uint8_t b = msg_map.at("PUBX").second.at(msg_id) & 0x0f;

    a += a < 0xa ? '0' : ('A' - 0xa);
    b += b < 0xa ? '0' : ('A' - 0xa);

    message.push_back(a);
    message.push_back(b);

    add_pubx_payload(message, args...);

    for (size_t i = 1; i < message.size(); ++i)
      checksum ^= static_cast<uint8_t>(message[i]);

    a = (checksum & 0xf0) >> 4;
    b = checksum & 0x0f;

    a += a < 0xa ? '0' : ('A' - 0xa);
    b += b < 0xa ? '0' : ('A' - 0xa);

    message.push_back('*');

    message.push_back(a);
    message.push_back(b);

    message.push_back('\r');
    message.push_back('\n');

    write(message);
  }

  template <typename... Args>
  void ubx_send(std::string&& msg_class, std::string&& msg_id,
    Args... args)
  {
    std::vector<uint8_t> message = { 0xb5 /*μ*/, 'b',
      msg_map.at(msg_class).first, msg_map.at(msg_class).second.at(msg_id) };

    uint16_t length = payload_size(args...);
    message.reserve(2 + length + 2);

    add_ubx_payload(message, length, args...);

    uint8_t check_a = 0;
    uint8_t check_b = 0;

    for (size_t i = 2; i < message.size(); ++i)
      check_b += (check_a += message[i]);

    message.push_back(check_a);
    message.push_back(check_b);

    write(message);
  }

  void disable_nmea();

};

} // namespace cracl

#endif // CRACL_RECEIVER_UBLOX_HPP
