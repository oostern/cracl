#include "ftc_ml401.hpp"

#include "../device.hpp"

#include <deque>
#include <vector>
#include <string>

namespace cracl
{

namespace ftc
{

b_msg::b_msg(std::vector<uint8_t>& message)
{
  update(message);
}

void b_msg::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    m_timestamp = message[1];
    m_channelId = message[3];

    uint8_t temp[4] = {message[7], message[6], message[5], message[4]};

    std::memcpy(&m_count, temp, sizeof(m_count));
  }
}

uint8_t b_msg::timestamp()
{
  return m_timestamp;
}

uint8_t b_msg::channelId()
{
  return m_channelId;
}

uint32_t b_msg::count()
{
  return m_count;
}

bool b_msg::type(std::vector<uint8_t>& message)
{
  return (message.size() != 9 && message[2] == 0x42);
}

m_msg::m_msg(std::vector<uint8_t>& message)
{
  update(message);
}

void m_msg::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    m_timestamp = message[1];

    uint8_t temp[4] = {message[6], message[5], message[4], message[3]};

    std::memcpy(&m_count, temp, sizeof(m_count));
  }
}

uint8_t m_msg::timestamp()
{
  return m_timestamp;
}

uint32_t m_msg::count()
{
  return m_count;
}

bool m_msg::type(std::vector<uint8_t>& message)
{
  return (message.size() == 8 && message[2] == 0x4d);
}

p_msg::p_msg(std::vector<uint8_t>& message)
{
  update(message);
}

void p_msg::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    m_timestamp = message[1];
    m_channelId = message[3];

    uint8_t temp[4] = {message[7], message[6], message[5], message[4]};

    std::memcpy(&m_count, temp, sizeof(m_count));
  }
}

uint8_t p_msg::timestamp()
{
  return m_timestamp;
}

uint8_t p_msg::channelId()
{
  return m_channelId;
}

uint32_t p_msg::count()
{
  return m_count;
}

bool p_msg::type(std::vector<uint8_t>& message)
{
  return (message.size() == 9 && message[2] == 0x50);
}

r_msg::r_msg(std::vector<uint8_t>& message)
{
  update(message);
}

void r_msg::update(std::vector<uint8_t>& message)
{
  if (type(message))
  {
    m_timestamp = message[1];
    m_channelId = message[3];

    m_second = message[4];
    m_minute = message[5];
    m_hour = message[6];

    uint8_t temp[2] = {message[7], message[6]};

    std::memcpy(&m_day, temp, sizeof(m_day));

    m_year = message[9];
  }
}

uint8_t r_msg::timestamp()
{
  return m_timestamp;
}

uint8_t r_msg::channelId()
{
  return m_channelId;
}

uint8_t r_msg::second()
{
  return m_second;
}

uint8_t r_msg::minute()
{
  return m_minute;
}

uint8_t r_msg::hour()
{
  return m_hour;
}

uint16_t r_msg::day()
{
  return m_day;
}

uint8_t r_msg::year()
{
  return m_year;
}

bool r_msg::type(std::vector<uint8_t>& message)
{
  return (message.size() == 11 && message[2] == 0x52);
}

} // namespace ftc

ftc_ml401::ftc_ml401(const std::string& location, size_t baud_rate,
    size_t timeout, size_t char_size, std::string delim, size_t max_handlers,
    port_base::parity::type parity,
    port_base::flow_control::type flow_control,
    port_base::stop_bits::type stop_bits)
  : device (location, baud_rate, timeout, char_size, delim, max_handlers,
    parity, flow_control, stop_bits)
{ }

void ftc_ml401::buffer_messages(size_t limit)
{
  std::vector<uint8_t> message_0;
  std::vector<uint8_t> message_1;

  size_t buffered = 0;
  uint8_t current = read_byte();

  while (true)
  {
    while (true)
    {
      if (current == 0x02)
      {
        std::vector<uint8_t> local_buf;

        message_0.push_back(current);
        message_0.push_back(read_byte());

        current = read_byte();

        if (current == 0x4d) // Type M
          local_buf = read(5);
        else if (current == 0x42) // Type B
          local_buf = read(6);
        else if (current == 0x50) // Type P
          local_buf = read(6);
        else if (current == 0x52) // Type R
          local_buf = read(8);
        else
        {
          message_0.clear();

          if (current != 0x02)
            current = read_byte();

          continue;
        }

        if (local_buf.back() != 0x03)
        {
          message_0.clear();

          current = read_byte();

          continue;
        }

        message_0.push_back(current);
        message_0.insert(message_0.end(), local_buf.begin(), local_buf.end());

        break;
      }
      else if (current == 0x00)
        return;
      else
        current = read_byte();
    }

    while (true)
    {
      if (current == 0x02)
      {
        std::vector<uint8_t> local_buf;

        message_1.push_back(current);
        message_1.push_back(read_byte());

        current = read_byte();

        if (current == 0xb2) // Type M
          local_buf = read(5);
        else if (current == 0xbd) // Type B
          local_buf = read(6);
        else if (current == 0xaf) // Type P
          local_buf = read(6);
        else if (current == 0xbd) // Type R
          local_buf = read(8);
        else
        {
          message_1.clear();

          if (current != 0x02)
            current = read_byte();

          continue;
        }

        if (local_buf.back() != 0x03)
        {
          message_1.clear();

          current = read_byte();

          continue;
        }

        message_1.push_back(current);
        message_1.insert(message_1.end(), local_buf.begin(), local_buf.end());

        if (message_0.size() != message_1.size()
            || (uint8_t)message_0[2] != (uint8_t)(~message_1[2])
            || (uint8_t)message_0[1] != (uint8_t)(~message_1[1]))
        {
          message_0.clear();

          message_0.swap(message_1);

          current = read_byte();

          continue;
        }

        bool invalid = false;
        for (size_t i = 3; !invalid && i < message_0.size() - 1; ++i)
          if ((uint8_t)message_0[i] != (uint8_t)(~message_1[i]))
            invalid = true;

        if (invalid)
        {
          message_0.clear();

          message_0.swap(message_1);

          current = read_byte();

          continue;
        }

        break;
      }
      else if (current == 0x00)
        return;
      else
        current = read_byte();
    }

    switch (message_0[2])
    {
      case 0x42: m_b_buffer.push_back(message_0); break;
      case 0x4d: m_m_buffer.push_back(message_0); break;
      case 0x50: m_p_buffer.push_back(message_0); break;
      case 0x52: m_r_buffer.push_back(message_0); break;
    }

    if (++buffered > limit)
    {
      m_overload = true;
      return;
    }

    message_0.clear();
    message_1.clear();
  }
}

bool ftc_ml401::overload()
{
  return m_overload;
}

size_t ftc_ml401::b_queued()
{
  buffer_messages();

  return m_b_buffer.size();
}

size_t ftc_ml401::m_queued()
{
  buffer_messages();

  return m_m_buffer.size();
}

size_t ftc_ml401::p_queued()
{
  buffer_messages();

  return m_p_buffer.size();
}

size_t ftc_ml401::r_queued()
{
  buffer_messages();

  return m_r_buffer.size();
}

std::vector<uint8_t> ftc_ml401::fetch_b()
{
  if (m_b_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_b_buffer.front());

  m_b_buffer.pop_front();

  return temp;
}

std::vector<uint8_t> ftc_ml401::fetch_m()
{
  if (m_m_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_m_buffer.front());

  m_m_buffer.pop_front();

  return temp;
}

std::vector<uint8_t> ftc_ml401::fetch_p()
{
  if (m_p_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_p_buffer.front());

  m_p_buffer.pop_front();

  return temp;
}

std::vector<uint8_t> ftc_ml401::fetch_r()
{
  if (m_r_buffer.empty())
    buffer_messages();

  auto temp = std::move(m_r_buffer.front());

  m_r_buffer.pop_front();

  return temp;
}

void ftc_ml401::flush_b()
{
  m_b_buffer.clear();
}

void ftc_ml401::flush_m()
{
  m_m_buffer.clear();
}

void ftc_ml401::flush_p()
{
  m_p_buffer.clear();
}

void ftc_ml401::flush_r()
{
  m_r_buffer.clear();
}

} // namespace cracl
