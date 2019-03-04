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

#include "m8.hpp"

#include <map>
#include <string>

namespace cracl
{

namespace ubx
{

extern const std::map<std::string,
       std::pair<uint8_t, std::map<std::string, uint8_t>>>
  m8_map = {
    { "ACK",
      { 0x05,
        {
          { "ACK", 0x01 },
          { "NAK", 0x00 }
        }
      }
    },
    { "AID",
      { 0x0b,
        {
          { "ALM", 0x30 },
          { "AOP", 0x30 },
          { "EPH", 0x31 },
          { "HUI", 0x02 },
          { "INI", 0x01 }
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
          { "DOSC", 0x61 },
          { "DYNSEED", 0x85 },
          { "ESRC", 0x60 },
          { "FIXSEED", 0x84 },
          { "GEOFENCE", 0x69 },
          { "GNSS", 0x3e },
          { "HNR", 0x5c },
          { "INF", 0x02 },
          { "IFTM", 0x39 },
          { "LOGFILTER", 0x47 },
          { "MSG", 0x01 },
          { "NAV5", 0x24 },
          { "NAVX5", 0x23 },
          { "NMEA", 0x17 },
          { "ODO", 0x1e },
          { "PM2", 0x3b },
          { "PRT", 0x00 },
          { "PWR", 0x57 },
          { "RATE", 0x08 },
          { "RINV", 0x34 },
          { "RST", 0x04 },
          { "RXM", 0x11 },
          { "SBAS", 0x16 },
          { "SMGR", 0x62 },
          { "TMODE2", 0x3d },
          { "TMODE3", 0x71 },
          { "TP5", 0x31 },
          { "TXSLOT", 0x53 },
          { "USB", 0x1b }
        }
      }
    },
    { "ESF",
      { 0x10,
        {
          { "INS", 0x15 },
          { "MEAS", 0x02 },
          { "RAW", 0x03 },
          { "STATUS", 0x10 }
        }
      }
    },
    { "HNR",
      { 0x28,
        {
          { "PVT", 0x00 }
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
          { "FLASH", 0x21 },
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
          { "GNSS", 0x28 },
          { "HW2", 0x0b },
          { "HW", 0x09 },
          { "IO", 0x02 },
          { "MSGPP", 0x06 },
          { "PATCH", 0x27 },
          { "RXBUF", 0x07 },
          { "RXR", 0x21 },
          { "SMGR", 0x2e },
          { "TXBUF", 0x08 },
          { "VER", 0x04 }
        }
      }
    },
    { "NAV",
      { 0x01,
        {
          { "AOPSTATUS", 0x60 },
          { "ATT", 0x05 },
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
          { "SBAS", 0x32 },
          { "SOL", 0x06 },
          { "STATUS", 0x03 },
          { "SVINFO", 0x30 },
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
          { "IMES", 0x61 },
          { "MEASX", 0x14 },
          { "PMREQ", 0x41 },
          { "RAWX", 0x15 },
          { "RLM", 0x59 },
          { "RTCM", 0x32 },
          { "SFRBX", 0x13 },
          { "SVSI", 0x20 }
        }
      }
    },
    { "SEC",
      { 0x27,
        {
          { "SIGN", 0x01 },
          { "UNIQID", 0x03 }
        }
      }
    },
    { "TIM",
      { 0x0d,
        {
          { "DOSC", 0x11 },
          { "FCHG", 0x16 },
          { "HOC", 0x17 },
          { "SMEAS", 0x13 },
          { "SVIN", 0x04 },
          { "TM2", 0x03 },
          { "TOS", 0x12 },
          { "TP", 0x01 },
          { "VCOCAL", 0x15 },
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
