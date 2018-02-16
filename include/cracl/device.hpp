#ifndef CRACL_DEVICE_HPP
#define CRACL_DEVICE_HPP

#include <array>
#include <string>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

using port_base = boost::asio::serial_port_base;

namespace cracl
{

enum read_status { ongoing, finalized, error, timeout };

class device
{
  char* buffer[1024];

  size_t m_timeout;
  size_t m_read_size;
  read_status m_read_status;

  std::string m_location;
  std::string m_delim;

  boost::asio::io_service m_io;
  boost::asio::serial_port m_port;
  boost::asio::streambuf m_buf;
  boost::asio::deadline_timer m_timer;

  void read_size_callback(const boost::system::error_code& error,
      const size_t size_transferred)
  {
    if (!error)
    {
      m_read_status = read_status::finalized;
      m_read_size = size_transferred;
    }
    else
    {
#ifdef __APPLE__
      if (error.value() != 45)
#elif _WIN32
      if (error.value() != 995)
#else
      if (error.value() != 125)
#endif
        m_read_status = read_status::error;
#ifdef __APPLE__
      else
        boost::asio::async_read(m_port, boost::asio::buffer(data, size),
            boost::bind(&device::read_size_callback, this,
              boost::asio::placeholders::error,
              boost::asio::placeholders::bytes_transferred)
            );
#endif
    }
  }

  void read_delim_callback(const boost::system::error_code& error,
      const size_t size_transferred)
  {
    if (!error)
    {
      m_read_status = read_status::finalized;
      m_read_size = size_transferred;
    }
    else
    {
#ifdef __APPLE__
      if (error.value() != 45)
#elif _WIN32
      if (error.value() != 995)
#else
      if (error.value() != 125)
#endif
        m_read_status = read_status::error;
#ifdef __APPLE__
      else
        boost::asio::async_read_until(m_port, m_buf, m_delim,
            boost::bind(&device::read_delim_callback, this,
              boost::asio::placeholders::error,
              boost::asio::placeholders::bytes_transferred)
            );
#endif
    }
  }

  void timeout_callback(const boost::system::error_code& error)
  {
    if (!error && m_read_status == ongoing)
      m_read_status = read_status::timeout;
  }

public:
  device(const std::string& location, size_t baud_rate=115200,
      size_t timeout=100, size_t char_size=8, std::string delim="\r\n",
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one)
    : m_timeout(timeout), m_location(location), m_delim(delim),
      m_io(), m_port(m_io), m_timer(m_io)
  {
    m_port.open(m_location);

    m_port.set_option(port_base::baud_rate(baud_rate));
    m_port.set_option(port_base::character_size(char_size));
    m_port.set_option(port_base::parity(parity));
    m_port.set_option(port_base::flow_control(flow_control));
    m_port.set_option(port_base::stop_bits(stop_bits));

    if (!m_port.is_open())
      throw new std::runtime_error(std::string("Could not open port at: "
            + location));
  }

  void reset()
  {
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

    m_io.reset();

    m_port.cancel();

    new (&m_io) boost::asio::io_service();
    new (&m_port) boost::asio::serial_port(m_io);
    new (&m_buf) boost::asio::streambuf();
    new (&m_timer) boost::asio::deadline_timer(m_io);

    m_port.open(m_location);

    m_port.set_option(baud_rate);
    m_port.set_option(char_size);
    m_port.set_option(parity);
    m_port.set_option(flow_control);
    m_port.set_option(stop_bits);

    if (!m_port.is_open())
      throw new std::runtime_error(std::string("Could not re-open port at: "
            + m_location));
  }

  void baud_rate(size_t baud_rate)
  {
    m_port.set_option(port_base::baud_rate(baud_rate));
  }

  size_t timeout()
  {
    return m_timeout;
  }

  void timeout(size_t timeout)
  {
    m_timeout = timeout;
  }

  void write(const char* data, size_t size)
  {
    boost::asio::write(m_port, boost::asio::buffer(data, size));
  }

  void write(const std::vector<uint8_t>& data)
  {
    boost::asio::write(m_port, boost::asio::buffer(&data[0], data.size()));
  }

  void write(const std::vector<char>& data)
  {
    boost::asio::write(m_port, boost::asio::buffer(&data[0], data.size()));
  }

  void write(const std::string& data)
  {
    boost::asio::write(m_port, boost::asio::buffer(data.c_str(), data.size()));
  }

  std::vector<uint8_t> read()
  {
    std::vector<uint8_t> result;

    boost::asio::async_read_until(m_port, m_buf, m_delim,
        boost::bind(&device::read_delim_callback, this,
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

        result.resize(m_read_size);

        char* data = reinterpret_cast<char*> (result.data());

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

    return result;
  }

  std::vector<uint8_t> read(size_t size)
  {
    std::vector<uint8_t> result(size, 0x00);
    char* data = reinterpret_cast<char*> (result.data());

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
          boost::bind(&device::read_size_callback, this,
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
    }

    return result;
  }

  uint8_t read_byte()
  {
    uint8_t result = 0x00;

    if (m_buf.size() > 0)
    {
      std::istream is(&m_buf);

      is.read(reinterpret_cast<char*> (&result), 1);
    }
    else
    {
      boost::asio::async_read(m_port, boost::asio::buffer(&result, 1),
          boost::bind(&device::read_size_callback, this,
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
    }

    return result;
  }
};

} // namespace cracl

#endif // CRACL_DEVICE_HPP
