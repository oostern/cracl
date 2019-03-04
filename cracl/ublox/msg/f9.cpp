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

#include "f9.hpp"

#include <map>
#include <string>

namespace cracl
{

namespace ubx
{

extern const std::map<std::string,
       std::pair<uint8_t, std::map<std::string, uint8_t>>>
  f9_map = {
    { "ACK",
      { 0x05,
        {
          { "ACK", 0x01 },
          { "NAK", 0x00 }
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
          { "GEOFENCE", 0x69 },
          { "GNSS", 0x3E },
          { "INF", 0x02 },
          { "IFTM", 0x39 },
          { "LOGFILTER", 0x47 },
          { "MSG", 0x01 },
          { "NAV5", 0x24 },
          { "NAVX5", 0x23 },
          { "NMEA", 0x17 },
          { "ODO", 0x1e },
          { "PRT", 0x00 },
          { "PWR", 0x57 },
          { "RATE", 0x08 },
          { "RINV", 0x34 },
          { "RST", 0x04 },
          { "TMODE3", 0x71 },
          { "TP5", 0x31 },
          { "USB", 0x1b },
          { "VALDEL", 0x8c },
          { "VALGET", 0x8b },
          { "VALSET", 0x8a }
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
          { "COMMS", 0x36 },
          { "GNSS", 0x28 },
          { "HW2", 0x0b },
          { "HW3", 0x37 },
          { "HW", 0x09 },
          { "IO", 0x02 },
          { "MSGPP", 0x06 },
          { "PATCH", 0x27 },
          { "RF", 0x38 },
          { "RXBUF", 0x07 },
          { "RXR", 0x21 },
          { "TXBUF", 0x08 },
          { "VER", 0x04 }
        }
      }
    },
    { "NAV",
      { 0x01,
        {
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
          { "SIG", 0x43 },
          { "STATUS", 0x03 },
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
          { "MEASX", 0x14 },
          { "PMREQ", 0x41 },
          { "RAWX", 0x15 },
          { "RLM", 0x59 },
          { "RTCM", 0x32 },
          { "SFRBX", 0x13 },
        }
      }
    },
    { "SEC",
      { 0x27,
        {
          { "UNIQID", 0x03 }
        }
      }
    },
    { "TIM",
      { 0x0d,
        {
          { "TM2", 0x03 },
          { "TP", 0x01 },
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

} // namespace ubx

} // namespace cracl
