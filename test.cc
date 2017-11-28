#include <iostream>

#include "include/cracl/device.hpp"

int main(int argc, char* argv[])
{
  //std::string name = "/dev/ttyUSB0";
  //std::string name = "/dev/ttyACM0";
  std::string name = "/dev/ttyACM0";
  cracl::port ublox(name, 9600, 100000);

  while (true)
  {
    std::string x = ublox.read();

    if (!x.empty())
      std::cout << x <<std::endl;
  }
}
