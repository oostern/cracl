#ifndef CRACL_UBLOX_MSG_F9_HPP
#define CRACL_UBLOX_MSG_F9_HPP

#include "base.hpp"

#include "class/mon.hpp"
#include "class/nav.hpp"
#include "class/rxm.hpp"

namespace cracl
{

namespace ubx
{

extern const std::map<std::string,
       std::pair<uint8_t, std::map<std::string, uint8_t>>>
  f9_map;

}

}

#endif // CRACL_UBLOX_MSG_F9_HPP
