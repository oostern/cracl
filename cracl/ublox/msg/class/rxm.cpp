#include "rxm.hpp"

#include "../base.hpp"

#include <cstdint>
#include <vector>

namespace cracl
{

namespace ubx
{

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
      && message[2] == ubx::msg_map.at("RXM").first
      && message[3] == ubx::msg_map.at("RXM").second.at("MEASX"));
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
      && message[2] == ubx::msg_map.at("RXM").first
      && message[3] == ubx::msg_map.at("RXM").second.at("RAWX"));
}

} // namespace rxm

} // namespace ubx

} // namespace cracl
