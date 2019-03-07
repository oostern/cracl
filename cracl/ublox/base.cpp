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

#include "base.hpp"

#include "../base/device.hpp"

#include <cstring>
#include <deque>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

namespace cracl
{

ublox_base::ublox_base(const std::string& location, size_t baud_rate,
    size_t timeout, size_t char_size, std::string delim, size_t max_handlers,
    port_base::parity::type parity,
    port_base::flow_control::type flow_control,
    port_base::stop_bits::type stop_bits)
  : device (location, baud_rate, timeout, char_size, std::string(delim),
    max_handlers, parity, flow_control, stop_bits)
{ }

void ublox_base::buffer_messages()
{
  std::vector<uint8_t> message;

  uint8_t m_current = read_byte();

  // Read byte by byte until port returns empty/timeout occurs
  while (m_current != 0x00)
  {
    if (m_current == 0x24       // $ - Start of NMEA/PUBX message
        || m_current == 0x21)   // ! - Start of encapsulated NMEA message
    {
      message.push_back(m_current);

      while (message.size() < 80 // 82  - Max NMEA length (4: $<content>*##<LF>)
          && m_current >= 0x20   // ' ' - Min valid char
          && m_current <= 0x7e   // ~   - Max valid char
          && m_current != 0x2a)  // *   - Start of NMEA/PUBX checksum
        message.push_back(m_current = read_byte());

      if (message.size() == 80)  // Read too many chars without finding checksum
        continue;                //   implying incorrect alignment, jump out
      else if (m_current == 0x00)// Invalid char, also port is empty, jump out
        continue;                //   will also fail while condition and exit
      else if (m_current < 0x20  // Invalid char, jump out and read again to
          || m_current > 0x7e)   //   see if some start byte is recognized
        continue;
      // else: Parsed reasonable number of potentially valid characters, and
      //   found what looks like the start of a checksum field. Read two bytes
      //   for the checksum and one byte for the line ending

      message.push_back(read_byte()); // First checksum byte
      message.push_back(read_byte()); // Second checksum byte
      read_byte();                    // <LF> - just throw it away

      m_nmea_buffer.push_back(message);
    }
    else if (m_current == 0xb5)  // μ - Start of UBX message
    {
      message.push_back(m_current);

      // Read one byte, expected to be second byte of UBX header
      m_current = read_byte();

      if (m_current != 0x62)     // Not second byte of UBX header (b), alignment
        continue;                //   must not be correct, jump out
      else                       // Else push into message
        message.push_back(m_current);

      // Read two bytes, expected to be a UBX message class and message ID
      message.push_back(read_byte());
      message.push_back(read_byte());

      // Read two bytes, expected to be message length
      message.push_back(read_byte());
      message.push_back(read_byte());

      // Interpret length from bytes fetched so far
      uint16_t length = *(reinterpret_cast<uint16_t *> (&message[4])) + 2;

      if (length > 4096)        // If length is absurdly large assume it's
        continue;               //   incorrect, jump out

      message.reserve(6 + length);

      // Read length number of bytes into message
      for (size_t i = 0; i < length; ++i)
        message.push_back(read_byte());

      m_ubx_buffer.push_back(message);
    }

    message.clear();

    m_current = read_byte();
  }
}

size_t ublox_base::nmea_queued()
{
  return m_nmea_buffer.size();
}

size_t ublox_base::ubx_queued()
{
  return m_ubx_buffer.size();
}

std::vector<uint8_t> ublox_base::fetch_nmea()
{
  if (m_nmea_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_nmea_buffer.front());

  m_nmea_buffer.pop_front();

  return temp;
}

std::vector<uint8_t> ublox_base::fetch_ubx()
{
  if (m_ubx_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_ubx_buffer.front());

  m_ubx_buffer.pop_front();

  return temp;
}

void ublox_base::flush_nmea()
{
  m_nmea_buffer.clear();
}

void ublox_base::flush_ubx()
{
  m_ubx_buffer.clear();
}

void ublox_base::disable_nmea()
{
  // Disable all NMEA message types (0) for all ports
  // RATE - NMEA TYPE - DDC - USART1 - USART2 - USB - SPI - reserved
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
