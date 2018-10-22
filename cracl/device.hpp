#ifndef CRACL_DEVICE_HPP
#define CRACL_DEVICE_HPP

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>

#include <array>
#include <memory>
#include <mutex>
#include <string>

using port_base = boost::asio::serial_port_base;

namespace cracl
{

enum read_status { ongoing, finalized, error, timeout };

class device
{
  size_t m_timeout;
  size_t m_read_size;
  size_t m_max_handlers;
  read_status m_read_status;

  uint8_t m_result_byte;
  std::vector<uint8_t> m_result_vector;

  std::string m_location;
  std::string m_delim;

  std::mutex m_mutex;

  std::shared_ptr<device> m_this;

  boost::asio::io_service m_io;
  boost::asio::serial_port m_port;
  boost::asio::streambuf m_buf;
  boost::asio::deadline_timer m_timer;

  void read_callback(const boost::system::error_code& error,
      const size_t size_transferred);

  void timeout_callback(const boost::system::error_code& error);

  /* @brief Private function to deconstruct and reconstruct io_service,
   *        serial_port, and deadline_timer so that the lifecycle of Boost
   *        handlers can be managed to control heap memory usage.
   */
  void flush_handlers();

public:
  device(const std::string& location, size_t baud_rate=115200,
      size_t timeout=100, size_t char_size=8, std::string delim="\r\n",
      size_t max_handlers=1000000,
      port_base::parity::type parity=port_base::parity::none,
      port_base::flow_control::type flow_control=port_base::flow_control::none,
      port_base::stop_bits::type stop_bits=port_base::stop_bits::one);

  inline size_t handler_count() { return m_this.use_count() - 1; };

  void baud_rate(size_t baud_rate);

  size_t timeout();

  void timeout(size_t timeout);

  void write(const char* data, size_t size);

  void write(const std::vector<uint8_t>& data);

  void write(const std::vector<char>& data);

  void write(const std::string& data);

  std::vector<uint8_t> read();

  std::vector<uint8_t> read(size_t size);

  uint8_t read_byte();
};

} // namespace cracl

#endif // CRACL_DEVICE_HPP
