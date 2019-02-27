#ifndef CRACL_UBLOX_F9_HPP
#define CRACL_UBLOX_F9_HPP

#include "base.hpp"
#include "msg/f9.hpp"
#include "../base/device.hpp"

#include <deque>
#include <iomanip>
#include <string>
#include <vector>

namespace cracl
{

class f9 : public ublox_base
{
public:
  f9(const std::string& location, size_t baud_rate=9600, size_t timeout=500,
      size_t char_size=8, std::string delim="\r\n", size_t max_handlers=100000,
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one);

  using ublox_base::fetch_ubx;

  std::vector<uint8_t> fetch_ubx(std::string&& msg_class,
      std::string&& msg_id, bool first_try=true);

  template <typename... Args>
  void ubx_send(std::string&& msg_class, std::string&& msg_id, Args... args)
  {
    std::vector<uint8_t> message = { 0xb5 /*μ*/, 'b',
      ubx::f9_map.at(msg_class).first,
      ubx::f9_map.at(msg_class).second.at(msg_id) };

    uint16_t length = payload_size(args...);
    message.reserve(2 + length + 2);

    add_ubx_payload(message, length, args...);

    uint8_t check_a = 0;
    uint8_t check_b = 0;

    // Compute 8-bit Fletcher checksum
    for (size_t i = 2; i < message.size(); ++i)
      check_b += (check_a += message[i]);

    message.push_back(check_a);
    message.push_back(check_b);

    write(message);
  }

};

} // namespace cracl

#endif // CRACL_UBLOX_F9_HPP
