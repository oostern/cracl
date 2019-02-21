#include <iostream>

#include <cracl/microsemi/sa45s.hpp>
#include <cracl/jackson_labs/firefly_1a.hpp>
#include <cracl/ublox/m8.hpp>

int main(int argc, char* argv[])
{
  using namespace cracl;

  m8 x("/dev/ttyACM1");
  firefly_1a g("/dev/ttyUSB0");
  sa45s c("/dev/ttyUSB1");

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

  // Disable NMEA messages on uBlox
  x.disable_nmea();

  // Test connection with uBlox
  x.ubx_send("NAV", "STATUS");
  sleep(2);

  auto m = x.fetch_ubx("NAV", "STATUS");
  if (ubx::nav::status::type(m))
  {
    ubx::nav::status parsed = ubx::nav::status(m);

    std::cout << "Spoof Status: ";

    switch (parsed.spoofDetState())
    {
      case 0: std::cout << "UNKNOWN" << std::endl; break;
      case 1: std::cout << "OKAY" << std::endl; break;
      case 2: std::cout << "SPOOF INDICATORS" << std::endl; break;
      case 3: std::cout << "MULTIPLE SPOOF INDICATORS" << std::endl;
    }
  }

  std::cout << "Second fetch empty?: "
    << (x.fetch_ubx("NAV", "STATUS").empty() ? "YES\n" : "NO\n") << std::endl;

  x.ubx_send("NAV", "SAT");
  sleep(2);

  auto nav_sat = x.fetch_ubx("NAV", "SAT");
  if (ubx::nav::sat::type(nav_sat))
  {
    ubx::nav::sat parsed = ubx::nav::sat(nav_sat);

    std::cout << "Num SVs: " << (int)parsed.numSvs() << std::endl;

    for (size_t i = 0; i < parsed.numSvs(); ++i)
    {
      switch (parsed.gnssId()[i])
      {
        case 0: std::cout << "\t    GPS "; break;
        case 1: std::cout << "\t   SBAS "; break;
        case 2: std::cout << "\tGalileo "; break;
        case 3: std::cout << "\t BeiDou "; break;
        case 4: std::cout << "\t   IMES "; break;
        case 5: std::cout << "\t   QZSS "; break;
        case 6: std::cout << "\tGLONASS "; break;
      }

      std::cout << (int)parsed.svId()[i]
        << "\t\tUsed: " << (int)parsed.svUsed()[i]
        << "\t\tCNO: " << (int)parsed.cno()[i]
        << "\t\tResidual: " << ((int)parsed.prRes()[i] / 10.0)
        << "\t\tQuality: ";

      switch (parsed.qualityInd()[i])
      {
        case 0: std::cout << "No Signal "; break;
        case 1: std::cout << "Searching"; break;
        case 2: std::cout << "Acquired"; break;
        case 3: std::cout << "Unusable"; break;
        case 4: std::cout << "Locked and Time Sync"; break;
        case 5:
        case 6:
        case 7: std::cout << "Carrier Locked"; break;
      }

      std::cout << std::endl;
    }
  }
}
