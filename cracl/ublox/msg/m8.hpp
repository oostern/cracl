#ifndef CRACL_UBLOX_MSG_M8_HPP
#define CRACL_UBLOX_MSG_M8_HPP

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
  m8_map;

}

}

#endif // CRACL_UBLOX_MSG_M8_HPP
