#include "ublox_8.hpp"

#include "../device.hpp"

#include <deque>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

namespace cracl
{

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
      && message[2] == msg_map.at("NAV").first
      && message[3] == msg_map.at("NAV").second.at("CLOCK"));
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
      && message[2] == msg_map.at("NAV").first
      && message[3] == msg_map.at("NAV").second.at("SAT"));
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
      && message[2] == msg_map.at("NAV").first
      && message[3] == msg_map.at("NAV").second.at("STATUS"));
}

} // namespace nav

namespace rxm
{

measx::measx(std::vector<uint8_t>& message)
{
  update(message);
}

void measx::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    m_version = message[6];

    m_gpsTOW = (*(reinterpret_cast<uint32_t*> (&message[10])));
    m_gloTOW = (*(reinterpret_cast<uint32_t*> (&message[14])));
    m_bdsTOW = (*(reinterpret_cast<uint32_t*> (&message[18])));
    m_qzssTOW = (*(reinterpret_cast<uint32_t*> (&message[36])));

    m_gpsTOWacc = (*(reinterpret_cast<uint16_t*> (&message[30])));
    m_gloTOWacc = (*(reinterpret_cast<uint16_t*> (&message[32])));
    m_bdsTOWacc = (*(reinterpret_cast<uint16_t*> (&message[34])));
    m_qzssTOWacc = (*(reinterpret_cast<uint16_t*> (&message[38])));

    m_numSV = message[40];
    m_towSet = message[41] & 0x03;

    for (size_t i = 0; i < m_numSV; ++i)
    {
      m_gnssId.push_back(message[50 + (i * 24)]);
      m_svId.push_back(message[51 + (i * 24)]);
      m_cNo.push_back(message[52 + (i * 24)]);
      m_mpathIndic.push_back(message[53 + (i * 24)]);

      m_dopplerMS.push_back(
          *(reinterpret_cast<int32_t*> (&message[54 + (i * 24)])));
      m_dopplerHz.push_back(
          *(reinterpret_cast<int32_t*> (&message[58 + (i * 24)])));

      m_wholeChips.push_back(
          *(reinterpret_cast<uint16_t*> (&message[62 + (i * 24)])));
      m_fracChips.push_back(
          *(reinterpret_cast<uint16_t*> (&message[64 + (i * 24)])));

      m_codePhase.push_back(
          *(reinterpret_cast<uint32_t*> (&message[66 + (i * 24)])));

      m_intCodePhase.push_back(message[70 + (i * 24)]);
      m_pseuRangeRMSErr.push_back(message[71 + (i * 24)]);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint8_t measx::version()
{
  return m_version;
}

uint32_t measx::gpsTOW()
{
  return m_gpsTOW;
}

uint32_t measx::gloTOW()
{
  return m_gloTOW;
}

uint32_t measx::bdsTOW()
{
  return m_bdsTOW;
}

uint32_t measx::qzssTOW()
{
  return m_qzssTOW;
}

uint16_t measx::gpsTOWacc()
{
  return m_gpsTOWacc;
}

uint16_t measx::gloTOWacc()
{
  return m_gloTOWacc;
}

uint16_t measx::bdsTOWacc()
{
  return m_bdsTOWacc;
}

uint16_t measx::qzssTOWacc()
{
  return m_qzssTOWacc;
}

uint8_t measx::numSV()
{
  return m_numSV;
}

uint8_t measx::towSet()
{
  return m_towSet;
}

std::vector<uint8_t> measx::gnssId()
{
  return m_gnssId;
}

std::vector<uint8_t> measx::svId()
{
  return m_svId;
}

std::vector<uint8_t> measx::cNo()
{
  return m_cNo;
}

std::vector<uint8_t> measx::mpathIndic()
{
  return m_mpathIndic;
}

std::vector<int32_t> measx::dopplerMS()
{
  return m_dopplerMS;
}

std::vector<int32_t> measx::dopplerHz()
{
  return m_dopplerHz;
}

std::vector<uint16_t> measx::wholeChips()
{
  return m_wholeChips;
}

std::vector<uint16_t> measx::fracChips()
{
  return m_fracChips;
}

std::vector<uint32_t> measx::codePhase()
{
  return m_codePhase;
}

std::vector<uint8_t> measx::intCodePhase()
{
  return m_intCodePhase;
}

std::vector<uint8_t> measx::pseuRangeRMSErr()
{
  return m_pseuRangeRMSErr;
}

bool measx::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == msg_map.at("RXM").first
      && message[3] == msg_map.at("RXM").second.at("MEASX"));
}

} // namespace rxm

} // namespace ubx

ublox_8::ublox_8(const std::string& location, size_t baud_rate,
    size_t timeout, size_t char_size, std::string delim,
    port_base::parity::type parity,
    port_base::flow_control::type flow_control,
    port_base::stop_bits::type stop_bits)
  : device (location, baud_rate, timeout, char_size, std::string(delim),
    parity, flow_control, stop_bits)
{ }

void ublox_8::buffer_messages()
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

size_t ublox_8::nmea_queued()
{
  return m_nmea_buffer.size();
}

size_t ublox_8::ubx_queued()
{
  return m_ubx_buffer.size();
}

std::vector<uint8_t> ublox_8::fetch_nmea()
{
  if (m_nmea_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_nmea_buffer.front());

  m_nmea_buffer.pop_front();

  return temp;
}

std::vector<uint8_t> ublox_8::fetch_ubx()
{
  if (m_ubx_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_ubx_buffer.front());

  m_ubx_buffer.pop_front();

  return temp;
}

std::vector<uint8_t> ublox_8::fetch_ubx(std::string&& msg_class, std::string&& msg_id)
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

void ublox_8::flush_nmea()
{
  m_nmea_buffer.clear();
}

void ublox_8::flush_ubx()
{
  m_ubx_buffer.clear();
}

void ublox_8::disable_nmea()
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

} // namespace cracl
