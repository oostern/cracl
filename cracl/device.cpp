#include "device.hpp"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>

#include <array>
#include <mutex>
#include <string>

namespace cracl
{

device::device(const std::string& location, size_t baud_rate=115200,
    size_t timeout=100, size_t char_size=8, std::string delim="\r\n",
    port_base::parity::type parity=port_base::parity::none,
    port_base::flow_control::type flow_control=port_base::flow_control::none,
    port_base::stop_bits::type stop_bits=port_base::stop_bits::one)
  : m_timeout(timeout), m_location(location), m_delim(std::move(delim)),
    m_io(), m_port(m_io), m_timer(m_io)
{
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

void device::reset()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  m_port.cancel();

  m_io.reset();
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
  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::write(m_port, boost::asio::buffer(data, size));
}

void device::write(const std::vector<uint8_t>& data)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::write(m_port, boost::asio::buffer(data.data(), data.size()));
}

void device::write(const std::vector<char>& data)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::write(m_port, boost::asio::buffer(data.data(), data.size()));
}

void device::write(const std::string& data)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::write(m_port, boost::asio::buffer(data.c_str(), data.size()));
}

std::vector<uint8_t> device::read()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  boost::asio::async_read_until(m_port, m_buf, m_delim,
      boost::bind(&device::read_callback, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred)
      );

  m_timer.expires_from_now(boost::posix_time::millisec(m_timeout));

  m_timer.async_wait(boost::bind(&device::timeout_callback, this,
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
        boost::bind(&device::read_callback, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred)
        );

    m_timer.expires_from_now(boost::posix_time::millisec(m_timeout));

    m_timer.async_wait(boost::bind(&device::timeout_callback, this,
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
        boost::bind(&device::read_callback, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred)
        );

    m_timer.expires_from_now(boost::posix_time::millisec(m_timeout));

    m_timer.async_wait(boost::bind(&device::timeout_callback, this,
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
