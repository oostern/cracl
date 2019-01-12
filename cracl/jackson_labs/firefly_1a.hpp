#ifndef CRACL_JACKSON_LABS_FIREFLY_1A_HPP
#define CRACL_JACKSON_LABS_FIREFLY_1A_HPP

#include "../base/device.hpp"

#include <array>
#include <deque>
#include <string>

namespace cracl
{

enum sync_source { GPS, EXT, AUTO };

class firefly_1a : public device
{
  std::deque<std::vector<uint8_t>> m_nmea_buffer;
  std::deque<std::vector<uint8_t>> m_scpi_buffer;

  //void add_pubx_payload(std::vector<char> &message) { }

  //template <typename... Args>
  //void add_pubx_payload(std::vector<char> &message, const char* t, Args... args)

  //template <typename T, typename... Args>
  //void add_pubx_payload(std::vector<char> &message, T t, Args... args)

  void buffer_messages();

public:
  firefly_1a(const std::string& location, size_t baud_rate=115200,
      size_t timeout=100, size_t char_size=8, std::string delim="\r\n\r\n",
      size_t max_handlers=100000,
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one);

  size_t nmea_queued();

  size_t scpi_queued();

  std::vector<uint8_t> fetch_nmea();

  std::vector<uint8_t> fetch_scpi();

  void flush_nmea();

  void flush_scpi();

  //template <typename... Args>
  //void scpi_send(std::string&& msg_id, Args... args);

  /* @brief Function to query the configuration, position, speed, height, and
   *        other relevant data of the integrated GPS receiver
   */
  void gps();

  /* @brief Function to query the number of tracked satellites
   */
  void gps_sat_tra_coun();

  /* @brief Function to query the number of SV's which should be visible per the
   *        almanac
   */
  void gps_sat_vis_coun();

  /* @brief Function to instruct the GPSDO to transmit GPGGA NMEA messages at a
   *        specified frequency (0:off)
   *
   * Note that this command is disabled during the first 4 minutes of GPSDO
   * operation
   *
   * @param freq The frequency in seconds (0,255) at which GPGGA NMEA messages
   *        should be output
   */
  void gps_gpgga(size_t freq);

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
  void gps_ggast(size_t freq);

  /* @brief Function to instruct the GPSDO to transmit GPRMC NMEA messages at a
   *        specified frequency (0:off)
   *
   * Note that this command is disabled during the first 4 minutes of GPSDO
   * operation
   *
   * @param freq The frequency in seconds (0,255) at which GPRMC NMEA messages
   *        should be output
   */
  void gps_gprmc(size_t freq);

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
  void gps_xyzsp(size_t freq);

  /* @brief Function to return information about time, including date, time in
   *        UTC, timezone, and time shift between the GPSDO and GPS time
   */
  void ptime();

  /* @brief Function to query the calendar date (UTC)
   */
  void ptim_date();

  /* @brief Function to query the current time (UTC)
   */
  void ptim_time();

  /* @brief Function to query the current time (UTC) in a format suitable for
   *        display (colon delimeters)
   */
  void ptim_time_str();

  /* @brief Function to query the shift in GPSDO time from GPS time (1E-10
   *        seconds precision)
   *
   * Note that this function is equivalent to the @sync_tint function
   */
  void ptim_tint();

  /* @brief Function to query the status of the synchronization system,
   *        including sync source, state, lock status, health, holdover
   *        duration, frequency error estimate, and the shift in GPSDO time
   *        from GPS time
   */
  void sync();

  /* @brief Function to set the 1 PPS source to be used for synchronization
   *        GPS  : internal GPS receiver
   *        EXT  : external 1 PPS source
   *        AUTO : use the internal receiver when available, fallback to EXT
   *
   * @param source The source to be used for synchronization
   */
  void sync_sour_mode(sync_source source);

  /* @brief Function to set the 1 PPS source to be used for synchronization
   *        GPS  : internal GPS receiver
   *        EXT  : external 1 PPS source
   *        AUTO : use the internal receiver when available, fallback to EXT
   *
   * @param source The source to be used for synchronization
   */
  void sync_sour_mode(std::string&& source);

  /* @brief Function to query the synchronization source being used
   */
  void sync_sour_state();

  /* @brief Function to query the length of the most recent holdover duration
   */
  void sync_hold_dur();

  /* @brief Function to command the GPSDO to immediately enter holdover mode
   *
   */
  void sync_hold_init();

  /* @brief Function to command terminate a manual holdover condition which
   *        was initiated through @sync_hold_init
   */
  void sync_hold_rec_init();

  /* @brief Function to query the shift in GPSDO time from GPS time (1E-10
   *        seconds precision)
   */
  void sync_tint();

  /* @brief Function to command the GPSDO to synchronize with the reference
   *        1 PPS signal
   *
   * Note that this command is ignored when the oscillator is in holdover
   */
  void sync_imme();

  /* @brief Function to query the frequency error estimate
   *
   * Similar to the Allan variance, a 1000s interval is measured. Values below
   * 1E-12 are considered noise
   */
  void sync_fee();

  /* @brief Function to query the lock status of the PLL which controls the
   *        oscillator
   */
  void sync_lock();

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
  void sync_health();

  /* @brief Function to query the electronic frequency control value in percent
   */
  void diag_rosc_efc_rel();

  /* @brief Function to query the electronic frequency control value in volts
   *        (0 < v < 5)
   */
  void diag_rosc_efc_abs();

  /* @brief Function to query the system status
   */
  void syst_stat();

  /* @brief Function to check if command echo is enabled on RS-232
   */
  void syst_comm_ser_echo();

  /* @brief Function to enable or disable command echo on RS-232
   *
   * Note: This should not be disabled as it is used when parsing messages from
   * the Firefly
   */
  void syst_comm_ser_echo(bool state);

  /* @brief Function to check of command prompt ("scpi>") is enabled
   */
  void syst_comm_ser_pro();

  /* @brief Function to enable or disable command prompt on RS-232
   *
   * Note: This should not be disabled as it is used when parsing messages from
   * the Firefly
   */
  void syst_comm_ser_pro(bool state);

  /* @brief Function to query current baud rate setting for device
   */
  void syst_comm_ser_baud();

  /* @brief Function to change the baud rate for the device
   *
   * Proposed value must be in predefined list of rates. Default baud rate is
   * 115200. Note that the baud rate on the program side should also be adjusted
   * or communication will be lost
   *
   * @param proposed the new value for the baud rate
   */
  void syst_comm_ser_baud(size_t proposed);

  /* @brief Function to query the current settings of the servo loop
   */
  void serv();

  /* @brief Function to set the course Dac which controls the EFC. Values should
   *        be in the range [0, 255]
   *
   * Note: You should not need to use this function
   *
   * @param val the new value for the coeffieicent
   */
  void serv_coarsd(size_t val);

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
  void serv_efcs(double value);

  /* @brief Function to set the low pass filter effectiveness of the DAC. Values
   *        should be in range [0.0, 4000.0], and are typically in [2.0, 50.0]
   *
   * @param New value for coefficient
   */
  void serv_efcd(double value);

  /* @brief Function to set the coefficient corresponding to the temperature
   *        compensation. Values should be in the range [-4000.0, 4000.0]
   *
   * @param New value for coefficient
   */
  void serv_tempco(double value);

  /* @brief Function to set the aging coefficient for the OCXO. Values should
   *        be in the range [-10.0, 10.0]
   *
   * @param New value for coefficient
   */
  void serv_aging(double value);

  /* @brief Function to set the integral component of the PID loop. Values
   *        should be in the range [-100.0, 100.0]. Typical values are in the
   *        range [10.0, 30.0]
   *
   * A value which is too high will result in instability
   *
   * @param New value for coefficient
   */
  void serv_phaseco(double value);

  /* @brief Function to query the GPSDO's offset to UTC
   */
  void serv_1pps();

  /* @brief Function to set the GPSDO's offset to UTC in 16.7ns incremnnts
   *
   * @param The new offset
   */
  void serv_1pps(int offset);

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
  void serv_trac(size_t freq);
};

} // namespace cracl

#endif // CRACL_JACKSON_LABS_FIREFLY_1A_HPP
