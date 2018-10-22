#ifndef CRACL_CLOCK_CSAC_HPP
#define CRACL_CLOCK_CSAC_HPP

#include "../device.hpp"

#include <string>

namespace cracl
{

/* @class csac_device
 *
 * This implementation targets the Microsemi/Symmetricom SA.45 CSAC
 *
 * @brief Class to represent a CSAC
 */
class csac : public device
{
public:
  /* @brief Constructor for csac device
   *
   * @param A shared_ptr to a communication interface
   */
  csac(const std::string& location, size_t baud_rate=57600, size_t timeout=100,
      size_t char_size=8, std::string delim="\r\n", size_t max_handlers=100000,
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one);

  /* @brief Function to get telemetry headers
   *
   * @ void Telemetry headers
   */
  void telemetry_header();

  /* @brief Function to get telemetry data
   *
   * @ void Telemetry data in CSV format
   */
  void telemetry_data();

  /* @brief Function to adjust the absolute operating frequency
   *
   * @param Frequency adjustment value in pp10^15
   * @ Unit response steer
   */
  void steer_freq_abs(int value);

  /* @brief Function to adjust the relative operating frequency
   *
   * @param Frequency adjustment value in pp10^15
   * @ Unit response steer
   */
  void steer_freq_rel(int value);

  /* @brief Function to lock the frequency steering value
   *
   * WARNING: hardware lifecycles provide for a finite number of steering lock
   * writes, so this command should be used sparingly
   */
  void STEER_FREQ_LOCK();
};

} // namespace cracl

#endif // CRACL_CLOCK_CSAC_HPP
