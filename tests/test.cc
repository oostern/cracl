#include <iostream>

#include <cracl/device.hpp>
#include <cracl/clock/csac.hpp>
#include <cracl/clock/firefly.hpp>
#include <cracl/receiver/ublox_8.hpp>

int main(int argc, char* argv[])
{
  using namespace cracl;

  ublox_8 x("/dev/ttyACM1");
  firefly g("/dev/ttyUSB0");
  csac c("/dev/ttyUSB1");

  // Test connectioon with CSAC
  c.write("!^\r\n");
  sleep(2);

  auto res = c.read();

  for ( auto i : res)
    std::cout << i ;
  std::cout << std::endl;

  // Test connection with Firefly GPSDO
  g.write("SYNC?\r\n");
  sleep(2);

  res = g.read();

  for ( auto i : res)
    std::cout << i ;
  std::cout << std::endl;

  // Disable/Enable NMEA messages on uBlox
  //x.pubx_send("RATE","GLL",0,0,0,0,0,0);
  //x.pubx_send("RATE","RMC",0,0,0,0,0,0);
  //x.pubx_send("RATE","VTG",0,0,0,0,0,0);
  //x.pubx_send("RATE","GSG",0,0,0,0,0,0);
  //x.pubx_send("RATE","GSA",0,0,0,0,0,0);
  //x.pubx_send("RATE","GSV",0,0,0,0,0,0);
  //x.pubx_send("RATE","GGA",0,0,0,0,0,0);
  //x.pubx_send("RATE","ZDA",0,0,0,0,0,0);
  //x.pubx_send("CONFIG",1,"0007","0003",19200,0);

  // Test connection with uBlox
  x.ubx_send("NAV", "STATUS");
  sleep(2);

  while (x.ubx_queued())
  {
    auto msg = x.fetch_ubx();

    if (ubx::nav::status::type(msg))
    {
      ubx::nav::status parsed = ubx::nav::status(msg);

      std::cout << "Spoof Status: ";

      switch (parsed.spoofDetState())
      {
        case 0: std::cout << "UNKNOWN" << std::endl; break;
        case 1: std::cout << "OKAY" << std::endl; break;
        case 2: std::cout << "SPOOF INDICATORS" << std::endl; break;
        case 3: std::cout << "MULTIPLE SPOOF INDICATORS" << std::endl;
      }
    }
  }
}
