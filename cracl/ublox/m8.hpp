#ifndef CRACL_UBLOX_M8_HPP
#define CRACL_UBLOX_M8_HPP

#include "msg/m8.hpp"
#include "../base/device.hpp"

#include <deque>
#include <iomanip>
#include <string>
#include <vector>

namespace cracl
{

class m8 : public device
{
  uint8_t m_current;
  std::vector<uint8_t> m_local_buf;

  std::deque<std::vector<uint8_t>> m_ubx_buffer;
  std::deque<std::vector<uint8_t>> m_nmea_buffer;

  size_t payload_size()
  {
    return 0;
  }

  template <typename T>
  size_t payload_size(T t)
  {
    return sizeof (T);
  }

  template <typename T, typename... Args>
  size_t payload_size(T t, Args... args)
  {
    return sizeof (T) + payload_size(args...);
  }

  void add_pubx_payload(std::vector<char> &message) { }

  template <typename... Args>
  void add_pubx_payload(std::vector<char> &message, const char* t, Args... args)
  {
    message.push_back(',');

    for (const char* ch = t; *ch != 0x00; ++ch)
      message.push_back(*ch);

    add_pubx_payload(message, args...);
  }

  template <typename T, typename... Args>
  void add_pubx_payload(std::vector<char> &message, T t, Args... args)
  {
    auto temp = std::to_string(t);

    message.push_back(',');

    for (auto const& ch : temp)
      message.push_back(ch);

    add_pubx_payload(message, args...);
  }

  void add_ubx_payload(std::vector<uint8_t> &message) { }

  template <typename T, typename... Args>
  void add_ubx_payload(std::vector<uint8_t> &message, T t, Args... args)
  {
    uint8_t *x = reinterpret_cast<uint8_t *>(&t);

    // For little endian systems, reverse loop for big endian
    for (size_t i = 0; i < sizeof (T); ++i)
      message.push_back(x[i]);

    add_ubx_payload(message, args...);
  }

public:
  m8(const std::string& location, size_t baud_rate=9600,
      size_t timeout=500, size_t char_size=8, std::string delim="\r\n",
      size_t max_handlers=100000,
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one);

  void buffer_messages();

  size_t nmea_queued();

  size_t ubx_queued();

  std::vector<uint8_t> fetch_nmea();

  std::vector<uint8_t> fetch_ubx();

  std::vector<uint8_t> fetch_ubx(std::string&& msg_class, std::string&& msg_id,
      bool first_try=true);

  void flush_nmea();

  void flush_ubx();

  template <typename... Args>
  void pubx_send(std::string&& msg_id, Args... args)
  {
    uint8_t checksum = 0x00;
    std::vector<char> message = { '$', 'P', 'U', 'B', 'X', ',' };

    uint8_t a = (ubx::msg_map.at("PUBX").second.at(msg_id) & 0xf0) >> 4;
    uint8_t b = ubx::msg_map.at("PUBX").second.at(msg_id) & 0x0f;

    a += a < 0xa ? '0' : ('A' - 0xa);
    b += b < 0xa ? '0' : ('A' - 0xa);

    message.push_back(a);
    message.push_back(b);

    add_pubx_payload(message, args...);

    // Compute XOR checksum
    for (size_t i = 1; i < message.size(); ++i)
      checksum ^= static_cast<uint8_t>(message[i]);

    a = (checksum & 0xf0) >> 4;
    b = checksum & 0x0f;

    // Convert values to plaintext hexadecimal
    a += a < 0xa ? '0' : ('A' - 0xa);
    b += b < 0xa ? '0' : ('A' - 0xa);

    message.push_back('*');

    message.push_back(a);
    message.push_back(b);

    message.push_back('\r');
    message.push_back('\n');

    write(message);
  }

  template <typename... Args>
  void ubx_send(std::string&& msg_class, std::string&& msg_id,
    Args... args)
  {
    std::vector<uint8_t> message = { 0xb5 /*μ*/, 'b',
      ubx::msg_map.at(msg_class).first,
      ubx::msg_map.at(msg_class).second.at(msg_id) };

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

  void disable_nmea();

};

} // namespace cracl

#endif // CRACL_UBLOX_M8_HPP
