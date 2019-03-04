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

#include "device.hpp"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>

#include <array>
#include <memory>
#include <mutex>
#include <string>

namespace cracl
{

device::device(const std::string& location, size_t baud_rate, size_t timeout,
    size_t char_size, std::string delim,size_t max_handlers,
    port_base::parity::type parity, port_base::flow_control::type flow_control,
    port_base::stop_bits::type stop_bits)
  : m_timeout(timeout), m_max_handlers(max_handlers), m_location(location),
    m_delim(std::move(delim)), m_io(), m_port(m_io), m_timer(m_io)
{
  // Create a shared pointer to this instance so that we can track the number of
  //   handlers stored by io_service, and destruct/reconstruct m_io as needed
  //   to manage memory usage. The use count should always be >1 as the instance
  //   will own a pointer to itself, but pass a null destructor to emphasize
  //   that the shared_ptr is used for counting and not management.
  m_this = std::shared_ptr<device>(this, [=](device* d) { });

  m_port.open(m_location);

  m_port.set_option(port_base::baud_rate(baud_rate));
  m_port.set_option(port_base::character_size(char_size));
  m_port.set_option(port_base::parity(parity));
  m_port.set_option(port_base::flow_control(flow_control));
  m_port.set_option(port_base::stop_bits(stop_bits));

  if (!m_port.is_open())
    throw std::runtime_error(std::string("Could not open port at: "
          + location));
}

void device::read_callback(const boost::system::error_code& error,
    const size_t size_transferred)
{
  if (!error)
  {
    m_timer.cancel();

    m_read_status = read_status::finalized;
    m_read_size = size_transferred;
  }
  else
  {
#ifdef __APPLE__
    if (error.value() == 45)
#elif _WIN32
    if (error.value() == 995)
#else
    if (error.value() == 125)
#endif
      m_read_status = read_status::timeout;
    else
      m_read_status = read_status::error;
  }
}

void device::timeout_callback(const boost::system::error_code& error)
{
  if (!error && m_read_status == ongoing)
    m_read_status = read_status::finalized;
#ifdef __APPLE__
  else if (error.value() == 45)
#elif _WIN32
  else if (error.value() == 995)
#else
  else if (error.value() == 125)
#endif
    m_read_status = read_status::timeout;
  else if (error)
    m_read_status = read_status::error;
}

void device::flush_handlers()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  m_io.stop();

  port_base::baud_rate baud_rate;
  port_base::character_size char_size;
  port_base::parity parity;
  port_base::flow_control flow_control;
  port_base::stop_bits stop_bits;

  m_port.get_option(baud_rate);
  m_port.get_option(char_size);
  m_port.get_option(parity);
  m_port.get_option(flow_control);
  m_port.get_option(stop_bits);

  // Seperated destruction and construction of port and timer:
  //   Destructing either also calls basic_io_context destructor, preventing the
  //   process from being interleaved

  // Manually destruct port
  m_port.~basic_serial_port();

  // Manually destruct timer
  m_timer.~basic_deadline_timer();

  // Manually call destructor and perform in-place new construction to manage
  //   lifecycle. Destructing io_serice results in the destruction of stored
  //   handlers from previous read and write operations, which would otherwise
  //   grow without bound
  m_io.~io_service();
  new (&m_io) boost::asio::io_service();

  // Manually re-construct port with new io_service
  new (&m_port) boost::asio::serial_port(m_io);

  m_port.open(m_location);

  m_port.set_option(baud_rate);
  m_port.set_option(char_size);
  m_port.set_option(parity);
  m_port.set_option(flow_control);
  m_port.set_option(stop_bits);

  if (!m_port.is_open())
    throw std::runtime_error(std::string("Could not re-open port at: "
          + m_location));

  // Manually re-construct timer with new io_service
  new (&m_timer) boost::asio::deadline_timer(m_io);
}

void device::baud_rate(size_t baud_rate)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  m_port.set_option(port_base::baud_rate(baud_rate));
}

size_t device::timeout()
{
  return m_timeout;
}

void device::timeout(size_t timeout)
{
  m_timeout = timeout;
}

void device::write(const char* data, size_t size)
{
  if (handler_count() > m_max_handlers)
    flush_handlers();

  std::lock_guard<std::mutex> lock(m_mutex);


  boost::asio::write(m_port, boost::asio::buffer(data, size));
}

void device::write(const std::vector<uint8_t>& data)
{
  if (handler_count() > m_max_handlers)
    flush_handlers();

  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::write(m_port, boost::asio::buffer(data.data(), data.size()));
}

void device::write(const std::vector<char>& data)
{
  if (handler_count() > m_max_handlers)
    flush_handlers();

  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::write(m_port, boost::asio::buffer(data.data(), data.size()));
}

void device::write(const std::string& data)
{
  if (handler_count() > m_max_handlers)
    flush_handlers();

  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::write(m_port, boost::asio::buffer(data.c_str(), data.size()));
}

std::vector<uint8_t> device::read()
{
  if (handler_count() > m_max_handlers)
    flush_handlers();

  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::async_read_until(m_port, m_buf, m_delim,
      boost::bind(&device::read_callback, m_this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred)
      );

  m_timer.expires_from_now(boost::posix_time::millisec(m_timeout));

  m_timer.async_wait(boost::bind(&device::timeout_callback, m_this,
        boost::asio::placeholders::error));

  m_read_status = read_status::ongoing;
  m_read_size = 0;

  while (true)
  {
    m_io.run_one();

    if (m_read_status == finalized)
    {
      m_timer.cancel();

      std::istream is(&m_buf);

      m_result_vector.resize(m_read_size);

      char* data = reinterpret_cast<char*> (m_result_vector.data());

      is.read(data, m_read_size);

      break;
    }
    else if (m_read_status == read_status::timeout)
    {
      m_port.cancel();

      break;
    }
    else if (m_read_status == error)
    {
      m_timer.cancel();
      m_port.cancel();

      break;
    }
  }

  m_io.reset();

  return m_result_vector;
}

std::vector<uint8_t> device::read(size_t size)
{
  if (handler_count() > m_max_handlers)
    flush_handlers();

  std::lock_guard<std::mutex> lock(m_mutex);

  m_result_vector.resize(size);

  char* data = reinterpret_cast<char*> (m_result_vector.data());

  if (m_buf.size() > 0)
  {
    std::istream is(&m_buf);

    size_t toRead = std::min(m_buf.size(), size);

    is.read(data, toRead);

    data += toRead;
    size -= toRead;
  }

  if (size != 0)
  {
    boost::asio::async_read(m_port, boost::asio::buffer(data, size),
        boost::bind(&device::read_callback, m_this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred)
        );

    m_timer.expires_from_now(boost::posix_time::millisec(m_timeout));

    m_timer.async_wait(boost::bind(&device::timeout_callback, m_this,
          boost::asio::placeholders::error));

    m_read_status = read_status::ongoing;
    m_read_size = 0;

    while (true)
    {
      m_io.run_one();

      if (m_read_status == finalized)
      {
        m_timer.cancel();

        break;
      }
      else if (m_read_status == read_status::timeout)
      {
        m_port.cancel();

        break;
      }
      else if (m_read_status == error)
      {
        m_timer.cancel();

        m_port.cancel();

        break;
      }
    }

    m_io.reset();
  }

  return m_result_vector;
}

uint8_t device::read_byte()
{
  if (handler_count() > m_max_handlers)
    flush_handlers();

  std::lock_guard<std::mutex> lock(m_mutex);

  m_result_byte = 0x00;

  if (m_buf.size() > 0)
  {
    std::istream is(&m_buf);

    is.read(reinterpret_cast<char*> (&m_result_byte), 1);
  }
  else
  {
    boost::asio::async_read(m_port, boost::asio::buffer(&m_result_byte, 1),
        boost::bind(&device::read_callback, m_this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred)
        );

    m_timer.expires_from_now(boost::posix_time::millisec(m_timeout));

    m_timer.async_wait(boost::bind(&device::timeout_callback, m_this,
          boost::asio::placeholders::error));

    m_read_status = read_status::ongoing;
    m_read_size = 0;

    while (true)
    {
      m_io.run_one();

      if (m_read_status == finalized)
      {
        m_timer.cancel();

        break;
      }
      else if (m_read_status == read_status::timeout)
      {
        m_port.cancel();

        break;
      }
      else if (m_read_status == error)
      {
        m_timer.cancel();

        m_port.cancel();

        break;
      }
    }

    m_io.reset();
  }

  return m_result_byte;
}

} // namespace cracl
