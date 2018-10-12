#include "ublox_8.hpp"

#include "../device.hpp"

#include <cstring>
#include <deque>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

 #include <thread>
 #include <iostream>

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

namespace mon
{

hw::hw(std::vector<uint8_t>& message)
{
  update(message);
}

void hw::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    m_pinSel = (*(reinterpret_cast<uint32_t*> (&message[6])));
    m_pinBank = (*(reinterpret_cast<uint32_t*> (&message[10])));
    m_pinDir = (*(reinterpret_cast<uint32_t*> (&message[14])));
    m_pinVal = (*(reinterpret_cast<uint32_t*> (&message[18])));

    m_noisePerMS = (*(reinterpret_cast<uint16_t*> (&message[22])));
    m_agcCnt = (*(reinterpret_cast<uint16_t*> (&message[24])));

    m_aStatus = message[26];
    m_aPower  = message[27];

    uint8_t bitfield = message[28];

    m_rtcCalib = bitfield & 0x01;
    m_safeBoot = bitfield & 0x02;
    m_jammingState = bitfield & 0x0C;
    m_xtalAbsent = bitfield & 0x10;

    m_usedMask = (*(reinterpret_cast<uint32_t*> (&message[30])));

    std::memcpy(m_vp.data(), &message[34], 17);

    m_jamInd = message[51];
    m_pinIrq = (*(reinterpret_cast<uint32_t*> (&message[54])));
    m_pullH = (*(reinterpret_cast<uint32_t*> (&message[58])));
    m_pullL = (*(reinterpret_cast<uint32_t*> (&message[62])));
  }
  else
    throw std::runtime_error("Message type mismatch");
}

uint32_t hw::pinSel()
{
  return m_pinSel;
}

uint32_t hw::pinBank()
{
  return m_pinBank;
}

uint32_t hw::pinDir()
{
  return m_pinDir;
}

uint32_t hw::pinVal()
{
  return m_pinVal;
}

uint16_t hw::noisePerMS()
{
  return m_noisePerMS;
}

uint16_t hw::agcCnt()
{
  return m_agcCnt;
}

uint8_t hw::aStatus()
{
  return m_aStatus;
}

uint8_t hw::aPower()
{
  return m_aPower;
}

uint8_t hw::rtcCalib()
{
  return m_rtcCalib;
}

uint8_t hw::safeBoot()
{
  return m_safeBoot;
}

uint8_t hw::jammingState()
{
  return m_jammingState;
}

uint8_t hw::xtalAbsent()
{
  return m_xtalAbsent;
}

uint32_t hw::usedMask()
{
  return m_usedMask;
}

std::array<uint8_t, 17> hw::vp()
{
  return m_vp;
}

uint8_t hw::jamInd()
{
  return m_jamInd;
}

uint32_t hw::pinIrq()
{
  return m_pinIrq;
}

uint32_t hw::pullH()
{
  return m_pullH;
}

uint32_t hw::pullL()
{
  return m_pullL;
}

bool hw::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == msg_map.at("MON").first
      && message[3] == msg_map.at("MON").second.at("HW"));
}

} // namespace mon

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
      && message[2] == msg_map.at("NAV").first
      && message[3] == msg_map.at("NAV").second.at("DOP"));
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

rawx::rawx(std::vector<uint8_t>& message)
{
  update(message);
}

void rawx::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    m_rcvTow = (*(reinterpret_cast<double*> (&message[6])));

    m_week = (*(reinterpret_cast<uint16_t*> (&message[14])));

    m_leapS = (*(reinterpret_cast<int8_t*> (&message[16])));

    m_numMeas = message[17];
    m_recStat = message[12];

    m_leapSec = m_recStat & 0x01;
    m_clkReset = m_recStat & 0x02;

    for (size_t i = 0; i < m_numMeas; ++i)
    {
      m_prMes.push_back(*(reinterpret_cast<double*> (&message[22 + (i * 32)])));
      m_cpMes.push_back(*(reinterpret_cast<double*> (&message[30 + (i * 32)])));

      m_doMes.push_back(*(reinterpret_cast<float*> (&message[38 + (i * 32)])));

      m_gnssId.push_back(message[42 + (i * 32)]);
      m_svId.push_back(message[43 + (i * 32)]);
      m_freqId.push_back(message[45 + (i * 32)]);

      m_locktime.push_back(
          *(reinterpret_cast<uint16_t*> (&message[46 + (i * 32)])));

      m_cno.push_back(message[48 + (i * 32)]);
      m_prStdev.push_back(message[49 + (i * 32)]);
      m_cpStdev.push_back(message[50 + (i * 32)]);
      m_doStdev.push_back(message[51 + (i * 32)]);
      m_trkStat.push_back(message[52 + (i * 32)]);
    }
  }
  else
    throw std::runtime_error("Message type mismatch");
}

double rawx::rcvTow()
{
  return m_rcvTow;
}

uint16_t rawx::week()
{
  return m_week;
}

int8_t rawx::leapS()
{
  return m_leapS;
}

uint8_t rawx::numMeas()
{
  return m_numMeas;
}

uint8_t rawx::recStat()
{
  return m_recStat;
}

uint8_t rawx::leapSec()
{
  return m_leapSec;
}

uint8_t rawx::clkReset()
{
  return m_clkReset;
}

std::vector<double> rawx::prMes()
{
  return m_prMes;
}

std::vector<double> rawx::cpMes()
{
  return m_cpMes;
}

std::vector<float> rawx::doMes()
{
  return m_doMes;
}

std::vector<uint8_t> rawx::gnssId()
{
  return m_gnssId;
}

std::vector<uint8_t> rawx::svId()
{
  return m_svId;
}

std::vector<uint8_t> rawx::freqId()
{
  return m_freqId;
}

std::vector<uint16_t> rawx::locktime()
{
  return m_locktime;
}

std::vector<uint8_t> rawx::cno()
{
  return m_cno;
}

std::vector<uint8_t> rawx::prStdev()
{
  return m_prStdev;
}

std::vector<uint8_t> rawx::cpStdev()
{
  return m_cpStdev;
}

std::vector<uint8_t> rawx::doStdev()
{
  return m_doStdev;
}

std::vector<uint8_t> rawx::trkStat()
{
  return m_trkStat;
}

bool rawx::type(std::vector<uint8_t>& message)
{
  return (!message.empty()
      && valid_checksum(message)
      && message[2] == msg_map.at("RXM").first
      && message[3] == msg_map.at("RXM").second.at("RAWX"));
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

  std::cout << "\033[1;33m" << std::this_thread::get_id() << " read_byte first time \033[0m" << std::endl;
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
      {
        current = read_byte();

        if (current < 0x20)
          break;
        else
          message.push_back(current);
      }

      message.push_back(read_byte());
      message.push_back(read_byte());

      m_nmea_buffer.push_back(message);
    }
    else if (current == 0xb5) // μ - Start of UBX message
    {
      message.push_back(current);
      m_local_buf = read(5);

      uint16_t length = *(reinterpret_cast<uint16_t *> (&m_local_buf[3])) + 2;

      message.reserve(6 + length);

      message.insert(message.end(), m_local_buf.begin(), m_local_buf.end());

      m_local_buf = read(length);

      message.insert(message.end(), m_local_buf.begin(), m_local_buf.end());

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

std::vector<uint8_t> ublox_8::fetch_ubx(std::string&& msg_class,
    std::string&& msg_id, bool first_try)
{
  size_t i;
  std::vector<uint8_t> temp;

  if (m_ubx_buffer.empty())
  {
    buffer_messages();
  }

  for (i = 0; i < m_ubx_buffer.size(); ++i)
  {
    if (m_ubx_buffer[i][2] == msg_map.at(msg_class).first
        && m_ubx_buffer[i][3] == msg_map.at(msg_class).second.at(msg_id))
      break;
  }

  if (i != m_ubx_buffer.size())
  {
    temp = m_ubx_buffer[i];

    m_ubx_buffer.erase(m_ubx_buffer.begin() + i);
  }
  else if (first_try)
  {
    buffer_messages();

    return fetch_ubx(std::forward<std::string>(msg_class),
        std::forward<std::string>(msg_id), false);
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
