#include "gpstcxo.hpp"

#include "../device.hpp"

#include <array>
#include <deque>
#include <string>

namespace cracl
{

std::array<size_t, 5> gpsdo_baud{ 9600, 19200, 38400, 57600, 115200 };

gpstcxo::gpstcxo(const std::string& location, size_t baud_rate,
    size_t timeout, size_t char_size, std::string delim,
    port_base::parity::type parity,
    port_base::flow_control::type flow_control,
    port_base::stop_bits::type stop_bits)
  : device (location, baud_rate, timeout, char_size, std::move(delim), parity,
    flow_control, stop_bits)
{ }

//void gpstcxo::add_pubx_payload(std::vector<char> &message) { }

//template <typename... Args>
//void gpstcxo::add_pubx_payload(std::vector<char> &message, const char* t, Args... args)
//{
//  message.push_back(',');

//  for (const char* ch = t; *ch != 0x00; ++ch)
//    message.push_back(*ch);

//  add_pubx_payload(message, args...);
//}

//template <typename T, typename... Args>
//void gpstcxo::add_pubx_payload(std::vector<char> &message, T t, Args... args)
//{
//  auto temp = std::to_string(t);

//  message.push_back(',');

//  for (auto const& ch : temp)
//    message.push_back(ch);

//  add_pubx_payload(message, args...);
//}

void gpstcxo::buffer_messages()
{
  std::vector<uint8_t> message;

  uint8_t current = read_byte();

  while (true)
  {
    if (current == 0x00)
      break;
    else if (current == 0x24) // $ - Start of NMEA message
    {
      message.push_back(current);

      while (current != 0x2a) // * - Start of NMEA checksum
        message.push_back(current = read_byte());

      message.push_back(read_byte());
      message.push_back(read_byte());

      m_nmea_buffer.push_back(message);

      // Consume '\r\n'
      read(2);
    }
    else
    {
      message.push_back(current);

      auto temp = read();

      message.insert(message.begin() + 1, temp.begin(), temp.end());

      m_scpi_buffer.push_back(message);

      // Consume 'scpi > '
      read(7);
    }

    message.clear();

    current = read_byte();
  }
}

size_t gpstcxo::nmea_queued()
{
  buffer_messages();

  return m_nmea_buffer.size();
}

size_t gpstcxo::scpi_queued()
{
  buffer_messages();

  return m_scpi_buffer.size();
}

std::vector<uint8_t> gpstcxo::fetch_nmea()
{
  if (m_nmea_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_nmea_buffer.front());

  m_nmea_buffer.pop_front();

  return temp;
}

std::vector<uint8_t> gpstcxo::fetch_scpi()
{
  if (m_scpi_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_scpi_buffer.front());

  m_scpi_buffer.pop_front();

  return temp;
}

void gpstcxo::flush_nmea()
{
  m_nmea_buffer.clear();
}

void gpstcxo::flush_scpi()
{
  m_scpi_buffer.clear();
}

//template <typename... Args>
//void gpstcxo::scpi_send(std::string&& msg_id, Args... args)
//{
//  uint8_t checksum = 0x00;
//  std::vector<char> message = { '$', 'P', 'U', 'B', 'X', ',' };

//  uint8_t a = (msg_map.at("PUBX").second.at(msg_id) & 0xf0) >> 4;
//  uint8_t b = msg_map.at("PUBX").second.at(msg_id) & 0x0f;

//  a += a < 0xa ? '0' : ('A' - 0xa);
//  b += b < 0xa ? '0' : ('A' - 0xa);

//  message.push_back(a);
//  message.push_back(b);

//  add_pubx_payload(message, args...);

//  for (size_t i = 1; i < message.size(); ++i)
//    checksum ^= (uint8_t)message[i];

//  a = (checksum & 0xf0) >> 4;
//  b = checksum & 0x0f;

//  a += a < 0xa ? '0' : ('A' - 0xa);
//  b += b < 0xa ? '0' : ('A' - 0xa);

//  message.push_back('*');

//  message.push_back(a);
//  message.push_back(b);

//  message.push_back('\r');
//  message.push_back('\n');

//  write(message);
//}

void gpstcxo::gps()
{
  write("GPS?");
}

void gpstcxo::gps_sat_tra_coun()
{
  write("GPS:SAT:TRA:COUN?");
}

void gpstcxo::gps_sat_vis_coun()
{
  write("GPS:SAT:VIS:COUN?");
}

void gpstcxo::gps_gpgga(size_t freq)
{
  if (freq <= 255)
    write("GPS:GPGGA " + std::to_string(freq));
}

void gpstcxo::gps_ggast(size_t freq)
{
  if (freq <= 255)
    write("GPS:GGAST " + std::to_string(freq));
}

void gpstcxo::gps_gprmc(size_t freq)
{
  if (freq <= 255)
    write("GPS:GPRMC " + std::to_string(freq));
}

void gpstcxo::gps_xyzsp(size_t freq)
{
  if (freq <= 255)
    write("GPS:XYZSP " + std::to_string(freq));
}

void gpstcxo::ptime()
{
  write("PTIME?");
}

void gpstcxo::ptim_date()
{
  write("PTIM:DATE?");
}

void gpstcxo::ptim_time()
{
  write("PTIM:TIME?");
}

void gpstcxo::ptim_time_str()
{
  write("PTIM:TIME:STR?");
}

void gpstcxo::ptim_tint()
{
  write("PTIM:TINT?");
}

void gpstcxo::sync()
{
  write("SYNC?");
}

void gpstcxo::sync_sour_mode(sync_source source)
{
  switch (source)
  {
    case GPS:  write("SYNC:SOUR:MODE GPS" ); break;
    case EXT:  write("SYNC:SOUR:MODE EXT" ); break;
    case AUTO: write("SYNC:SOUR:MODE AUTO"); break;
  }
}

void gpstcxo::sync_sour_mode(std::string&& source)
{
  write("SYNC:SOUR:MODE " + source);
}

void gpstcxo::sync_sour_state()
{
  write("SYNC:SOUR:STATE?");
}

void gpstcxo::sync_hold_dur()
{
  write("SYNC:HOLD:DUR?");
}

void gpstcxo::sync_hold_init()
{
  write("SYNC:HOLD:INIT");
}

void gpstcxo::sync_hold_rec_init()
{
  write("SYNC:HOLD:REC:INIT");
}

void gpstcxo::sync_tint()
{
  write("SYNC:TINT?");
}

void gpstcxo::sync_imme()
{
  write("SYNC:IMME");
}

void gpstcxo::sync_fee()
{
  write("SYNC:FEE?");
}

void gpstcxo::sync_lock()
{
  write("SYNC:LOCK?");
}

void gpstcxo::sync_health()
{
  write("SYNC:HEALTH?");
}

void gpstcxo::diag_rosc_efc_rel()
{
  write("DIAG:ROSC:EFC:REL?");
}

void gpstcxo::diag_rosc_efc_abs()
{
  write("DIAG:ROSC:EFC:ABS?");
}

void gpstcxo::syst_stat()
{
  write("SYST:STAT?");
}

void gpstcxo::syst_comm_ser_echo()
{
  write("SYST:COMM:SER:ECHO?");
}

void gpstcxo::syst_comm_ser_echo(bool state)
{
  std::string command = "SYST:COMM:SER:ECHO "
    + (state ? std::string("ON") : std::string("OFF"));

  write(command.c_str());
}

void gpstcxo::syst_comm_ser_pro()
{
  write("SYST:COMM:SER:PRO?");
}

void gpstcxo::syst_comm_ser_pro(bool state)
{
  std::string command = "SYST:COMM:SER:PRO "
    + (state ? std::string("ON") : std::string("OFF"));

  write(command.c_str());
}

void gpstcxo::syst_comm_ser_baud()
{
  write("SYST:COMM:SER:BAUD?");
}

void gpstcxo::syst_comm_ser_baud(size_t proposed)
{
  for (uint32_t i : gpsdo_baud)
    if (proposed == gpsdo_baud.at(i))
    {
      write("SYST:COMM:SER:BAUD");
      break;
    }
}

void gpstcxo::serv()
{
  write("SERV?");
}

void gpstcxo::serv_coarsd(size_t val)
{
  if (val <= 255)
    write("SERV:COARSD " + std::to_string(val));
}

void gpstcxo::serv_efcs(double value)
{
  if (value >= 0.0 && value <= 500.0)
    write("SERV:EFCS " + std::to_string(value));
}

void gpstcxo::serv_efcd(double value)
{
  if (value >= 0.0 && value <= 4000.0)
    write("SERV:EFCD " + std::to_string(value));
}

void gpstcxo::serv_tempco(double value)
{
  if (value >= -4000.0 && value <= 4000.0)
    write("SERV:TEMPCO " + std::to_string(value));
}

void gpstcxo::serv_aging(double value)
{
  if (value >= -10.0 && value <= 10.0)
    write("SERV:AGING " + std::to_string(value));
}

void gpstcxo::serv_phaseco(double value)
{
  if (value >= -100.0 && value <= 100.0)
  {
    write("SERV:PHASECO " + std::to_string(value));
  }
}

void gpstcxo::serv_1pps()
{
  write("SERV:1PPS?");
}

void gpstcxo::serv_1pps(int offset)
{
  write("SERV:1PPS " + std::to_string(offset));
}

void gpstcxo::serv_trac(size_t freq)
{
  write("SERV:TRAC " + std::to_string(freq));
}

} // namespace cracl