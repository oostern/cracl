#ifndef CRACL_UBLOX_MSG_BASE_HPP
#define CRACL_UBLOX_MSG_BASE_HPP

#include <map>
#include <string>
#include <vector>

namespace cracl
{

namespace ubx
{

extern const std::map<std::string,
       std::pair<uint8_t, std::map<std::string, uint8_t>>>
  msg_map;

extern bool valid_checksum(std::vector<uint8_t>& message);

} // namespace ubx

} // namespace cracl

#endif // CRACL_UBLOX_MSG_BASE_HPP
