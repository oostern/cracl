#include <iostream>

#include <cracl/device.hpp>
#include <cracl/receiver/ublox_8.hpp>

int main(int argc, char* argv[])
{
  cracl::ublox_8 x("/dev/ttyACM5");

  using namespace cracl;

  //x.pubx_send("RATE","GLL",0,0,0,0,0,0);
  //x.pubx_send("RATE","RMC",0,0,0,0,0,0);
  //x.pubx_send("RATE","VTG",0,0,0,0,0,0);
  //x.pubx_send("RATE","GSG",0,0,0,0,0,0);
  //x.pubx_send("RATE","GSA",0,0,0,0,0,0);
  //x.pubx_send("RATE","GSV",0,0,0,0,0,0);
  //x.pubx_send("RATE","GGA",0,0,0,0,0,0);
  //x.pubx_send("RATE","ZDA",0,0,0,0,0,0);
  //x.pubx_send("CONFIG",1,"0007","0003",19200,0);

  x.ubx_send("NAV", "STATUS");
  sleep(2);

  while (x.ubx_queued())
  {
    auto msg = x.fetch_ubx();

    if (ubx::nav::status_type(msg))
    {
      ubx::nav::status parsed = ubx::nav::status(msg);

      std::cout << "Spoof Status: ";

      switch (parsed.spoofDetState)
      {
        case 0: std::cout << "UNKNOWN" << std::endl; break;
        case 1: std::cout << "OKAY" << std::endl; break;
        case 2: std::cout << "SPOOF INDICATORS" << std::endl; break;
        case 3: std::cout << "MULTIPLE SPOOF INDICATORS" << std::endl;
      }
    }
  }
}
