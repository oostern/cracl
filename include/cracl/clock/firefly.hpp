#ifndef CRACL_CLOCK_FIREFLY_HPP
#define CRACL_CLOCK_FIREFLY_HPP

#include <array>
#include <deque>
#include <string>

#include "../device.hpp"

namespace cracl
{

enum sync_source { GPS, EXT, AUTO };

std::array<size_t, 5> gpsdo_baud{ 9600, 19200, 38400, 57600, 115200 };

class firefly : public device
{
public:
  std::deque<std::vector<uint8_t>> m_nmea_buffer;
  std::deque<std::vector<uint8_t>> m_scpi_buffer;

  //void add_pubx_payload(std::vector<char> &message) { }

  //template <typename... Args>
  //void add_pubx_payload(std::vector<char> &message, const char* t, Args... args)
  //{
  //  message.push_back(',');

  //  for (const char* ch = t; *ch != 0x00; ++ch)
  //    message.push_back(*ch);

  //  add_pubx_payload(message, args...);
  //}

  //template <typename T, typename... Args>
  //void add_pubx_payload(std::vector<char> &message, T t, Args... args)
  //{
  //  auto temp = std::to_string(t);

  //  message.push_back(',');

  //  for (auto const& ch : temp)
  //    message.push_back(ch);

  //  add_pubx_payload(message, args...);
  //}

  void buffer_messages()
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

public:
  firefly(const std::string& location, size_t baud_rate=115200,
      size_t timeout=100, size_t char_size=8, std::string delim="\r\n\r\n",
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one)
    : device (location, baud_rate, timeout, char_size, std::move(delim), parity,
      flow_control, stop_bits)
  { }

  size_t nmea_queued()
  {
    buffer_messages();

    return m_nmea_buffer.size();
  }

  size_t scpi_queued()
  {
    buffer_messages();

    return m_scpi_buffer.size();
  }

  std::vector<uint8_t> fetch_nmea()
  {
    if (m_nmea_buffer.empty())
      buffer_messages();

    auto temp = std::move(m_nmea_buffer.front());

    m_nmea_buffer.pop_front();

    return temp;
  }

  std::vector<uint8_t> fetch_scpi()
  {
    if (m_scpi_buffer.empty())
      buffer_messages();

    auto temp = std::move(m_scpi_buffer.front());

    m_scpi_buffer.pop_front();

    return temp;
  }

  void flush_nmea()
  {
    m_nmea_buffer.clear();
  }

  void flush_scpi()
  {
    m_scpi_buffer.clear();
  }

  //template <typename... Args>
  //void scpi_send(std::string&& msg_id, Args... args)
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

  /* @brief Function to query the configuration, position, speed, height, and
   *        other relevant data of the integrated GPS receiver
   */
  void gps()
  {
    write("GPS?");
  }

  /* @brief Function to query the number of tracked satellites
   */
  void gps_sat_tra_coun()
  {
    write("GPS:SAT:TRA:COUN?");
  }

  /* @brief Function to query the number of SV's which should be visible per the
   *        almanac
   */
  void gps_sat_vis_coun()
  {
    write("GPS:SAT:VIS:COUN?");
  }

  /* @brief Function to instruct the GPSDO to transmit GPGGA NMEA messages at a
   *        specified frequency (0:off)
   *
   * Note that this command is disabled during the first 4 minutes of GPSDO
   * operation
   *
   * @param freq The frequency in seconds (0,255) at which GPGGA NMEA messages
   *        should be output
   */
  void gps_gpgga(size_t freq)
  {
    if (freq <= 255)
      write("GPS:GPGGA " + std::to_string(freq));
  }

  /* @brief Function to instruct the GPSDO to transmit modified GPGGA NMEA
   *        messages at a specified frequency (0:off)
   *
   * Messages differ from standard GPGGA messages in that they include the lock
   * state and health of the unit's **XO
   *
   * Note that this command is disabled during the first 7 minutes of GPSDO
   * operation
   *
   * @param freq The frequency in seconds (0,255) at which GPGGA NMEA messages
   *        should be output
   */
  void gps_ggast(size_t freq)
  {
    if (freq <= 255)
      write("GPS:GGAST " + std::to_string(freq));
  }

  /* @brief Function to instruct the GPSDO to transmit GPRMC NMEA messages at a
   *        specified frequency (0:off)
   *
   * Note that this command is disabled during the first 4 minutes of GPSDO
   * operation
   *
   * @param freq The frequency in seconds (0,255) at which GPRMC NMEA messages
   *        should be output
   */
  void gps_gprmc(size_t freq)
  {
    if (freq <= 255)
      write("GPS:GPRMC " + std::to_string(freq));
  }

  /* @brief Function to instruct teh GPSDO to transmit X, Y, and Z speed
   *        including centimeter-level accuracy estimates at a specified
   *        frequency
   *
   * Note that firmware version 0.909 or above is required to support this
   * command
   *
   * @param freq The frequency in seconds (0,255) at which GPRMC NMEA messages
   *        should be output
   */
  void gps_xyzsp(size_t freq)
  {
    if (freq <= 255)
      write("GPS:XYZSP " + std::to_string(freq));
  }

  /* @brief Function to return information about time, including date, time in
   *        UTC, timezone, and time shift between the GPSDO and GPS time
   */
  void ptime()
  {
    write("PTIME?");
  }

  /* @brief Function to query the calendar date (UTC)
   */
  void ptim_date()
  {
    write("PTIM:DATE?");
  }

  /* @brief Function to query the current time (UTC)
   */
  void ptim_time()
  {
    write("PTIM:TIME?");
  }

  /* @brief Function to query the current time (UTC) in a format suitable for
   *        display (colon delimeters)
   */
  void ptim_time_str()
  {
    write("PTIM:TIME:STR?");
  }

  /* @brief Function to query the shift in GPSDO time from GPS time (1E-10
   *        seconds precision)
   *
   * Note that this function is equivalent to the @sync_tint function
   */
  void ptim_tint()
  {
    write("PTIM:TINT?");
  }

  /* @brief Function to query the status of the synchronization system,
   *        including sync source, state, lock status, health, holdover
   *        duration, frequency error estimate, and the shift in GPSDO time
   *        from GPS time
   */
  void sync()
  {
    write("SYNC?");
  }

  /* @brief Function to set the 1 PPS source to be used for synchronization
   *        GPS  : internal GPS receiver
   *        EXT  : external 1 PPS source
   *        AUTO : use the internal receiver when available, fallback to EXT
   *
   * @param source The source to be used for synchronization
   */
  void sync_sour_mode(sync_source source)
  {
    switch (source)
    {
      case GPS:  write("SYNC:SOUR:MODE GPS" ); break;
      case EXT:  write("SYNC:SOUR:MODE EXT" ); break;
      case AUTO: write("SYNC:SOUR:MODE AUTO"); break;
    }
  }

  /* @brief Function to set the 1 PPS source to be used for synchronization
   *        GPS  : internal GPS receiver
   *        EXT  : external 1 PPS source
   *        AUTO : use the internal receiver when available, fallback to EXT
   *
   * @param source The source to be used for synchronization
   */
  void sync_sour_mode(std::string&& source)
  {
    write("SYNC:SOUR:MODE " + source);
  }

  /* @brief Function to query the synchronization source being used
   */
  void sync_sour_state()
  {
    write("SYNC:SOUR:STATE?");
  }

  /* @brief Function to query the length of the most recent holdover duration
   */
  void sync_hold_dur()
  {
    write("SYNC:HOLD:DUR?");
  }

  /* @brief Function to command the GPSDO to immediately enter holdover mode
   *
   */
  void sync_hold_init()
  {
    write("SYNC:HOLD:INIT");
  }

  /* @brief Function to command terminate a manual holdover condition which
   *        was initiated through @sync_hold_init
   */
  void sync_hold_rec_init()
  {
    write("SYNC:HOLD:REC:INIT");
  }

  /* @brief Function to query the shift in GPSDO time from GPS time (1E-10
   *        seconds precision)
   */
  void sync_tint()
  {
    write("SYNC:TINT?");
  }

  /* @brief Function to command the GPSDO to synchronize with the reference
   *        1 PPS signal
   *
   * Note that this command is ignored when the oscillator is in holdover
   */
  void sync_imme()
  {
    write("SYNC:IMME");
  }

  /* @brief Function to query the frequency error estimate
   *
   * Similar to the Allan variance, a 1000s interval is measured. Values below
   * 1E-12 are considered noise
   */
  void sync_fee()
  {
    write("SYNC:FEE?");
  }

  /* @brief Function to query the lock status of the PLL which controls the
   *        oscillator
   */
  void sync_lock()
  {
    write("SYNC:LOCK?");
  }

  /* @brief Function to query the health status of the GPSDO
   *
   * 0x000 : healty and locked
   * 0x001 : OCXO course DAC maxed out at 255
   * 0x002 : OCXO course DAC min-ed out at 0
   * 0x004 : phase offset to UTC > 250ns
   * 0x008 : runtime < 300s
   * 0x010 : holdover > 60s
   * 0x020 : frequency error estimate out of bounds
   * 0x040 : OCXO voltage too high
   * 0x080 : OCXO voltage too low
   * 0x100 : short term (100s) drift > 100ns
   * 0x200 : runtime < 7min after phase-reset
   */
  void sync_health()
  {
    write("SYNC:HEALTH?");
  }

  /* @brief Function to query the electronic frequency control value in percent
   */
  void diag_rosc_efc_rel()
  {
    write("DIAG:ROSC:EFC:REL?");
  }

  /* @brief Function to query the electronic frequency control value in volts
   *        (0 < v < 5)
   */
  void diag_rosc_efc_abs()
  {
    write("DIAG:ROSC:EFC:ABS?");
  }

  /* @brief Function to query the system status
   */
  void syst_stat()
  {
    write("SYST:STAT?");
  }

  /* @brief Function to check if command echo is enabled on RS-232
   */
  void syst_comm_ser_echo()
  {
    write("SYST:COMM:SER:ECHO?");
  }

  /* @brief Function to enable or disable command echo on RS-232
   *
   * Note: This should not be disabled as it is used when parsing messages from
   * the Firefly
   */
  void syst_comm_ser_echo(bool state)
  {
    std::string command = "SYST:COMM:SER:ECHO "
      + (state ? std::string("ON") : std::string("OFF"));

    write(command.c_str());
  }

  /* @brief Function to check of command prompt ("scpi>") is enabled
   */
  void syst_comm_ser_pro()
  {
    write("SYST:COMM:SER:PRO?");
  }

  /* @brief Function to enable or disable command prompt on RS-232
   *
   * Note: This should not be disabled as it is used when parsing messages from
   * the Firefly
   */
  void syst_comm_ser_pro(bool state)
  {
    std::string command = "SYST:COMM:SER:PRO "
      + (state ? std::string("ON") : std::string("OFF"));

    write(command.c_str());
  }

  /* @brief Function to query current baud rate setting for device
   */
  void syst_comm_ser_baud()
  {
    write("SYST:COMM:SER:BAUD?");
  }

  /* @brief Function to change the baud rate for the device
   *
   * Proposed value must be in predefined list of rates. Default baud rate is
   * 115200. Note that the baud rate on the program side should also be adjusted
   * or communication will be lost
   *
   * @param proposed the new value for the baud rate
   */
  void syst_comm_ser_baud(size_t proposed)
  {
    for (uint32_t i : gpsdo_baud)
      if (proposed == gpsdo_baud.at(i))
      {
        write("SYST:COMM:SER:BAUD");
        break;
      }
  }

  /* @brief Function to query the current settings of the servo loop
   */
  void serv()
  {
    write("SERV?");
  }

  /* @brief Function to set the course Dac which controls the EFC. Values should
   *        be in the range [0, 255]
   *
   * Note: You should not need to use this function
   *
   * @param val the new value for the coeffieicent
   */
  void serv_coarsd(size_t val)
  {
    if (val <= 255)
      write("SERV:COARSD " + std::to_string(val));
  }

  /* @brief Function to set the proportional coefficient of the PID loop. Values
   *        are in range [0.0, 500.0]
   *
   * Larger values increase loop control at the expense of noise while locked.
   * Settings which are too high will cause instabilities.
   *
   * Typical values:
   *  0.7 : double oven OCXO
   *  6.0 : single oven OCXO
   *
   * @param New value for coefficient
   */
  void serv_efcs(double value)
  {
    if (value >= 0.0 && value <= 500.0)
      write("SERV:EFCS " + std::to_string(value));
  }

  /* @brief Function to set the low pass filter effectiveness of the DAC. Values
   *        should be in range [0.0, 4000.0], and are typically in [2.0, 50.0]
   *
   * @param New value for coefficient
   */
  void serv_efcd(double value)
  {
    if (value >= 0.0 && value <= 4000.0)
      write("SERV:EFCD " + std::to_string(value));
  }

  /* @brief Function to set the coefficient corresponding to the temperature
   *        compensation. Values should be in the range [-4000.0, 4000.0]
   *
   * @param New value for coefficient
   */
  void serv_tempco(double value)
  {
    if (value >= -4000.0 && value <= 4000.0)
      write("SERV:TEMPCO " + std::to_string(value));
  }

  /* @brief Function to set the aging coefficient for the OCXO. Values should
   *        be in the range [-10.0, 10.0]
   *
   * @param New value for coefficient
   */
  void serv_aging(double value)
  {
    if (value >= -10.0 && value <= 10.0)
      write("SERV:AGING " + std::to_string(value));
  }

  /* @brief Function to set the integral component of the PID loop. Values
   *        should be in the range [-100.0, 100.0]. Typical values are in the
   *        range [10.0, 30.0]
   *
   * A value which is too high will result in instability
   *
   * @param New value for coefficient
   */
  void serv_phaseco(double value)
  {
    if (value >= -100.0 && value <= 100.0)
    {
      write("SERV:PHASECO " + std::to_string(value));
    }
  }

  /* @brief Function to query the GPSDO's offset to UTC
   */
  void serv_1pps()
  {
    write("SERV:1PPS?");
  }

  /* @brief Function to set the GPSDO's offset to UTC in 16.7ns incremnnts
   *
   * @param The new offset
   */
  void serv_1pps(int offset)
  {
    write("SERV:1PPS " + std::to_string(offset));
  }

  /* @brief Function to set the frequency at which a debug trace is produced
   *
   * Format:
   *   <date> <1PPS count> <fine DAC> <UTC offset (ns)> <freq error estimate>
   *   <visible SV's> <tracked SV's> <lock state> <health status>
   *
   * Note: Jackson Labs FW version 0.913+ is required to use this command
   *
   * @param The frequency at which a debug trace should be produced
   */
  void serv_trac(size_t freq)
  {
    write("SERV:TRAC " + std::to_string(freq));
  }
};

} // namespace cracl

#endif // CRACL_CLOCK_FIREFLY_HPP
