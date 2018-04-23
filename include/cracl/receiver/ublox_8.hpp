#ifndef CRACL_RECEIVER_UBLOX_HPP
#define CRACL_RECEIVER_UBLOX_HPP

#include <map>
#include <vector>
#include <deque>
#include <string>
#include <iomanip>

#include "../device.hpp"

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

bool valid_checksum(std::vector<uint8_t>& message)
{
  size_t i;
  uint8_t check_a = 0;
  uint8_t check_b = 0;

  for (i = 2; i < message.size() - 2; ++i)
    check_b += (check_a += message[i]);

  return (check_a == message[i] && check_b == message[i + 1]);
}

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
  clock(std::vector<uint8_t>& message)
  {
    update(message);
  }

  void update(std::vector<uint8_t>& message)
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

  uint32_t iTOW()
  {
    return m_iTOW;
  }

  int32_t clkB()
  {
    return m_clkB;
  }

  int32_t clkD()
  {
    return m_clkD;
  }

  uint32_t tAcc()
  {
    return m_tAcc;
  }

  uint32_t fAcc()
  {
    return m_fAcc;
  }

  static bool type(std::vector<uint8_t>& message)
  {
    return (!message.empty()
        && valid_checksum(message)
        && message[2] == msg_map.at("NAV").first
        && message[3] == msg_map.at("NAV").second.at("CLOCK"));
  }
}; // ubx::nav::clock

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
  sat(std::vector<uint8_t>& message)
  {
    update(message);
  }

  void update(std::vector<uint8_t>& message)
  {
    if (type(message))
    {
      m_iTOW = (*(reinterpret_cast<uint32_t*> (&message[6])));

      m_version = message[10];
      m_numSvs = message[11];

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

  uint32_t iTOW()
  {
    return m_iTOW;
  }

  uint8_t version()
  {
    return m_version;
  }

  uint8_t numSvs()
  {
    return m_numSvs;
  }

  std::vector<uint8_t> gnssId()
  {
    return m_gnssId;
  }

  std::vector<uint8_t> svId()
  {
    return m_svId;
  }

  std::vector<uint8_t> cno()
  {
    return m_cno;
  }

  std::vector<int8_t> elev()
  {
    return m_elev;
  }

  std::vector<int16_t> azim()
  {
    return m_azim;
  }

  std::vector<int16_t> prRes()
  {
    return m_prRes;
  }

  std::vector<uint8_t> qualityInd()
  {
    return m_qualityInd;
  }

  std::vector<uint8_t> svUsed()
  {
    return m_svUsed;
  }

  std::vector<uint8_t> health()
  {
    return m_health;
  }

  std::vector<uint8_t> diffCorr()
  {
    return m_diffCorr;
  }

  std::vector<uint8_t> smoothed()
  {
    return m_smoothed;
  }

  std::vector<uint8_t> orbitSource()
  {
    return m_orbitSource;
  }

  std::vector<uint8_t> ephAvail()
  {
    return m_ephAvail;
  }

  std::vector<uint8_t> almAvail()
  {
    return m_almAvail;
  }

  std::vector<uint8_t> anoAvail()
  {
    return m_anoAvail;
  }

  std::vector<uint8_t> aopAvail()
  {
    return m_aopAvail;
  }

  std::vector<uint8_t> sbasCorrUsed()
  {
    return m_sbasCorrUsed;
  }

  std::vector<uint8_t> rtcmCorrUsed()
  {
    return m_rtcmCorrUsed;
  }

  std::vector<uint8_t> prCorrUsed()
  {
    return m_prCorrUsed;
  }

  std::vector<uint8_t> crCorrUsed()
  {
    return m_crCorrUsed;
  }

  std::vector<uint8_t> doCorrUsed()
  {
    return m_doCorrUsed;
  }

  static bool type(std::vector<uint8_t>& message)
  {
    return (!message.empty()
        && valid_checksum(message)
        && message[2] == msg_map.at("NAV").first
        && message[3] == msg_map.at("NAV").second.at("SAT"));
  }
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
  status(std::vector<uint8_t>& message)
  {
    update(message);
  }

  void update(std::vector<uint8_t>& message)
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

  uint32_t iTOW()
  {
    return m_iTOW;
  }

  uint8_t gpsFix()
  {
    return m_gpsFix;
  }

  uint8_t gpsFixOk()
  {
    return m_gpsFixOk;
  }

  uint8_t diffSoln()
  {
    return m_diffSoln;
  }

  uint8_t wknSet()
  {
    return m_wknSet;
  }

  uint8_t towSet()
  {
    return m_towSet;
  }

  uint8_t diffCorr()
  {
    return m_diffCorr;
  }

  uint8_t mapMatching()
  {
    return m_mapMatching;
  }

  uint8_t psmState()
  {
    return m_psmState;
  }

  uint8_t spoofDetState()
  {
    return m_spoofDetState;
  }

  uint32_t ttff()
  {
    return m_ttff;
  }

  uint32_t msss()
  {
    return m_msss;
  }

  static bool type(std::vector<uint8_t>& message)
  {
    return (!message.empty()
        && valid_checksum(message)
        && message[2] == msg_map.at("NAV").first
        && message[3] == msg_map.at("NAV").second.at("STATUS"));
  }
}; // ubx::nav::status

} // namespace nav

} // namespace ubx

class ublox_8 : public device
{
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
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one)
    : device (location, baud_rate, timeout, char_size, std::string(delim),
      parity, flow_control, stop_bits)
  { }

  void buffer_messages()
  {
    std::vector<uint8_t> message;

    uint8_t current = read_byte();

    while (true)
    {
      if (current == 0x00)
        break;
      else if (current == 0x24  // $ - Start of NMEA/PUBX message
          || current == 0x21)   // ! - Start of encapsulated NMEA message
      {
        message.push_back(current);

        while (current != 0x2a) // * - Start of NMEA/PUBX checksum
          message.push_back(current = read_byte());

        message.push_back(read_byte());
        message.push_back(read_byte());

        m_nmea_buffer.push_back(message);
      }
      else if (current == 0xb5) // μ - Start of UBX message
      {
        message.push_back(current);
        std::vector<uint8_t> local_buf = read(5);

        uint16_t length = *(reinterpret_cast<uint16_t *> (&local_buf[3])) + 2;

        message.reserve(6 + length);

        message.insert(message.end(), local_buf.begin(), local_buf.end());

        local_buf = read(length);

        message.insert(message.end(), local_buf.begin(), local_buf.end());

        m_ubx_buffer.push_back(message);
      }

      message.clear();

      current = read_byte();
    }
  }

  size_t nmea_queued()
  {
    return m_nmea_buffer.size();
  }

  size_t ubx_queued()
  {
    return m_ubx_buffer.size();
  }

  std::vector<uint8_t> fetch_nmea()
  {
    if (m_nmea_buffer.empty())
      buffer_messages();

    auto temp = std::move(m_nmea_buffer.front());

    m_nmea_buffer.pop_front();

    return temp;
  }

  std::vector<uint8_t> fetch_ubx()
  {
    if (m_ubx_buffer.empty())
      buffer_messages();

    auto temp = std::move(m_ubx_buffer.front());

    m_ubx_buffer.pop_front();

    return temp;
  }

  std::vector<uint8_t> fetch_ubx(std::string&& msg_class, std::string&& msg_id,
      bool recurse=true)
  {
    size_t i;
    std::vector<uint8_t> temp;

    if (m_ubx_buffer.empty())
      buffer_messages();

    for (i = 0; i < m_ubx_buffer.size(); ++i)
      if (m_ubx_buffer[i][2] == msg_map.at(msg_class).first
          && m_ubx_buffer[i][3] == msg_map.at(msg_class).second.at(msg_id))
        break;

    if (i != m_ubx_buffer.size())
    {
      temp = m_ubx_buffer[i];

      m_ubx_buffer.erase(m_ubx_buffer.begin() + i);
    }

    return temp;
  }

  void flush_nmea()
  {
    m_nmea_buffer.clear();
  }

  void flush_ubx()
  {
    m_ubx_buffer.clear();
  }

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

  void disable_nmea()
  {
    pubx_send("RATE", "DTM", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "GLL", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "GNS", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "GSA", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "GST", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "GSG", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "GSV", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "GGA", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "RMC", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "VTG", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "VLW", 0, 0, 0, 0, 0, 0);
    pubx_send("RATE", "ZDA", 0, 0, 0, 0, 0, 0);
  }
};

} // namespace cracl

#endif // CRACL_RECEIVER_UBLOX_HPP
