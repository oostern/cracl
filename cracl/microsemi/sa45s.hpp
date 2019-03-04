// Copyright (C) 2019 Colton Riedel
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see https://www.gnu.org/licenses/
//
// If you are interested in obtaining a copy of this program under a
// different license, or have other questions or comments, contact me at
//
//   coltonriedel at protonmail dot ch

#ifndef CRACL_MICROSEMI_SA45S_HPP
#define CRACL_MICROSEMI_SA45S_HPP

#include "../base/device.hpp"

#include <string>

namespace cracl
{

/* @class sa45s_device
 *
 * This implementation targets the Microsemi/Symmetricom SA.45 CSAC
 *
 * @brief Class to represent a CSAC
 */
class sa45s : public device
{
public:
  /* @brief Constructor for sa45s device
   *
   * @param A shared_ptr to a communication interface
   */
  sa45s(const std::string& location, size_t baud_rate=57600, size_t timeout=100,
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

#endif // CRACL_MICROSEMI_SA45S_HPP
