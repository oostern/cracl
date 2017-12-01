#include <iostream>

#include <cracl/device.hpp>
#include <cracl/receiver/ublox_8.hpp>

int main(int argc, char* argv[])
{

  auto x = cracl::ublox_8();


  std::cout << "UBX-MON-HW\n"
    << x.ubx_msg(std::string("MON"), std::string("HW")) << std::endl;
  std::cout << "UBX-MON-HW2\n"
    << x.ubx_msg(std::string("MON"), std::string("HW2")) << std::endl;
  std::cout << "UBX-MON-VER\n"
    << x.ubx_msg(std::string("MON"), std::string("VER")) << std::endl;
  std::cout << "UBX-NAV-STATUS\n"
    << x.ubx_msg(std::string("NAV"), std::string("STATUS")) << std::endl;
  std::cout << "UBX-CFG-NMEA\n"
    << x.ubx_msg(std::string("CFG"), std::string("NMEA")) << std::endl;
  std::cout << "UBX-CFG-RATE\n"
    << x.ubx_msg(std::string("CFG"), std::string("RATE")) << std::endl;
  uint16_t rate=2000;
  std::cout << "UBX-CFG-RATE\n"
    << x.ubx_msg(std::string("CFG"), std::string("RATE"), rate, uint16_t(1),uint16_t(0)) << std::endl;




  //std::string name = "/dev/ttyUSB0";
  //std::string name = "/dev/ttyACM0";
  //std::string name = "/dev/ttyACM0";
  //cracl::port ublox(name, 9600, 100000);

  //while (true)
  //{
  //  std::string x = ublox.read();

  //  if (!x.empty())
  //    std::cout << x <<std::endl;
  //}
}
