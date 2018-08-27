#include "csac.hpp"

#include "../device.hpp"

#include <string>

namespace cracl
{

csac::csac(const std::string& location, size_t baud_rate,
    size_t timeout, size_t char_size, std::string delim,
    port_base::parity::type parity,
    port_base::flow_control::type flow_control,
    port_base::stop_bits::type stop_bits)
  : device (location, baud_rate, timeout, char_size, delim, parity,
    flow_control, stop_bits)
{ }

void csac::telemetry_header()
{
  write("!6");
}

void csac::telemetry_data()
{
  write("!^");
}

void csac::steer_freq_abs(int value)
{
  if (value >= -20000000 && value <= 20000000)
    write("!FD" + std::to_string(value));
}

void csac::steer_freq_rel(int value)
{
  if (value >= -20000000 && value <= 20000000)
    write("!FD" + std::to_string(value));
}

void csac::STEER_FREQ_LOCK()
{
  write("!FL");
}

} // namespace cracl
