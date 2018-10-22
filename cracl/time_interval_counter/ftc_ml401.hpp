#ifndef CRACL_TIME_INTERVAL_COUNTER_FTC_ML401_HPP
#define CRACL_TIME_INTERVAL_COUNTER_FTC_ML401_HPP

#include "../device.hpp"

#include <deque>
#include <vector>
#include <string>

namespace cracl
{

namespace ftc
{

class b_msg
{
  uint8_t m_timestamp;
  uint8_t m_channelId;

  uint32_t m_count;

public:
  b_msg(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint8_t timestamp();

  uint8_t channelId();

  uint32_t count();

  static bool type(std::vector<uint8_t>& message);
};

class m_msg
{
  uint8_t m_timestamp;

  uint32_t m_count;

public:
  m_msg(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint8_t timestamp();

  uint32_t count();

  static bool type(std::vector<uint8_t>& message);
};

class p_msg
{
  uint8_t m_timestamp;
  uint8_t m_channelId;

  uint32_t m_count;

public:
  p_msg(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint8_t timestamp();

  uint8_t channelId();

  uint32_t count();

  static bool type(std::vector<uint8_t>& message);
};

class r_msg
{
  uint8_t m_timestamp;
  uint8_t m_channelId;

  uint8_t m_second;
  uint8_t m_minute;
  uint8_t m_hour;
  uint8_t m_year;

  uint16_t m_day;

public:
  r_msg(std::vector<uint8_t>& message);

  void update(std::vector<uint8_t>& message);

  uint8_t timestamp();

  uint8_t channelId();

  uint8_t second();

  uint8_t minute();

  uint8_t hour();

  uint16_t day();

  uint8_t year();

  static bool type(std::vector<uint8_t>& message);
};

} // namespace ftc

class ftc_ml401 : public device
{
  bool m_overload = false;

  std::deque<std::vector<uint8_t>> m_b_buffer;
  std::deque<std::vector<uint8_t>> m_m_buffer;
  std::deque<std::vector<uint8_t>> m_p_buffer;
  std::deque<std::vector<uint8_t>> m_r_buffer;

  void buffer_messages(size_t limit=64);

public:
  ftc_ml401(const std::string& location, size_t baud_rate=115200,
      size_t timeout=100, size_t char_size=8, std::string delim="",
      size_t max_handlers=100000,
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one);

  bool overload();

  size_t b_queued();

  size_t m_queued();

  size_t p_queued();

  size_t r_queued();

  std::vector<uint8_t> fetch_b();

  std::vector<uint8_t> fetch_m();

  std::vector<uint8_t> fetch_p();

  std::vector<uint8_t> fetch_r();

  void flush_b();

  void flush_m();

  void flush_p();

  void flush_r();
};

} // namespace cracl

#endif // CRACL_TIME_INTERVAL_COUNTER_FTC_ML401_HPP
