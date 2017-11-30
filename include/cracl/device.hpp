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

class port
{
  char* buffer[1024];

  size_t m_timeout;
  size_t m_read_size;
  read_status m_read_status;

  std::string m_location;
  std::string m_delim = "\r\n";

  boost::asio::io_service m_io;
  boost::asio::serial_port m_port;
  boost::asio::streambuf m_buf;
  boost::asio::deadline_timer m_timer;

public:
  port(const std::string& location, size_t baud_rate=115200,
      size_t timeout=500, size_t char_size=8,
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one)
    : m_io(), m_port(m_io), m_timer(m_io), m_location(location),
      m_timeout(timeout)
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

  //template <typename T>
  //void set_option(const T &t)
  //{
  //  m_port.set_option(t);
  //}

  void write(const char* data, size_t size)
  {
    boost::asio::write(m_port, boost::asio::buffer(data, size));
  }

  void write(const std::vector<char>& data)
  {
    boost::asio::write(m_port, boost::asio::buffer(&data[0], data.size()));
  }

  void write(const std::string& data)
  {
    boost::asio::write(m_port, boost::asio::buffer(data.c_str(), data.size()));
  }

  /*
  std::vector<char> read()
  {
    // Note: if readData contains some previously read data, the call to
    // async_read_until (which is done in performReadSetup) correctly handles
    // it. If the data is enough it will also immediately call readCompleted()
    setupParameters=ReadSetupParameters(delim);
    performReadSetup(setupParameters);

    //For this code to work, there should always be a timeout, so the
    //request for no timeout is translated into a very long timeout
    if(timeout!=posix_time::seconds(0)) timer.expires_from_now(timeout);
    else timer.expires_from_now(posix_time::hours(100000));

    timer.async_wait(boost::bind(&TimeoutSerial::timeoutExpired,this,
          asio::placeholders::error));

    result=resultInProgress;
    bytesTransferred=0;

    for(;;)
    {
      io.run_one();
      switch(result)
      {
        case resultSuccess:
          {
            timer.cancel();
            bytesTransferred-=delim.size();//Don't count delim
            istream is(&m_buf);
            string result(bytesTransferred,'\0');//Alloc string
            is.read(&result[0],bytesTransferred);//Fill values
            is.ignore(delim.size());//Remove delimiter from stream
            return result;
          }
        case resultTimeoutExpired:
          port.cancel();
          throw(timeout_exception("Timeout expired"));
        case resultError:
          timer.cancel();
          port.cancel();
          throw(boost::system::system_error(boost::system::error_code(),
                "Error while reading"));
          //if resultInProgress remain in the loop
      }
    }

  }
  */

  void read_callback(const boost::system::error_code& error,
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
            boost::bind(&port::read_callback, this,
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

  std::string read()
  {
      boost::asio::async_read_until(m_port, m_buf, m_delim,
          boost::bind(&port::read_callback, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred)
          );

      std::string result;

      m_timer.expires_from_now(boost::posix_time::millisec(m_timeout));

      m_timer.async_wait(boost::bind(&port::timeout_callback, this,
            boost::asio::placeholders::error));

      m_read_status = read_status::ongoing;
      m_read_size = 0;

      while (true)
      {
        m_io.run_one();

        if (m_read_status == finalized)
        {
          m_timer.cancel();

          m_read_size -= m_delim.size();

          std::istream is(&m_buf);

          result = std::string(m_read_size, '\0');

          is.read(&result[0], m_read_size);
          is.ignore(m_delim.size());

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

  std::vector<char> read(size_t size)
  {
    std::vector<char> result(size, 'B');
    char* data = &(result[0]);

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
      boost::asio::async_read_until(m_port, m_buf, m_delim,
          boost::bind(&port::read_callback, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred)
          );

      m_timer.expires_from_now(boost::posix_time::millisec(m_timeout));

      m_timer.async_wait(boost::bind(&port::timeout_callback, this,
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

  template <typename... Args>
  std::string query(Args... args)
  {
    write(args...);

    return read();
  }

  /*
  template <typename... Args>
  std::vector<char> query(Args... args)
  {
    write(args...);

    return read();
  }
  */
};

} // namespace cracl

#endif // CRACL_DEVICE_HPP
