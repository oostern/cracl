#include <iostream>

#include <cracl/ublox/m8.hpp>

int main(int argc, char* argv[])
{
  using namespace cracl;

  m8 x("/dev/ttyACM0");

  // Disable NMEA messages on uBlox
  x.disable_nmea();

  //GLO Demo
  std::cout << "TIMEGLO Demo" << std::endl;
  x.ubx_send("NAV", "TIMEGLO");
  sleep(2);

  auto nav_timeglo = x.fetch_ubx("NAV", "TIMEGLO");

  if (ubx::nav::timeglo::type(nav_timeglo))
  {
    ubx::nav::timeglo parsed = ubx::nav::timeglo(nav_timeglo);

    std::cout << "iTOW: " << parsed.iTOW() << "ms" << std::endl;
    std::cout << "TOD: " << parsed.TOD() << "s" << std::endl;
    std::cout << "fTOD: " << parsed.fTOD() << "ns" << std::endl;
    std::cout << "Nt: " << parsed.Nt() << "days" << std::endl;
    std::cout << "N4: " << (int)parsed.N4() << std::endl;

    std::cout << "todValid: " << (int)parsed.todValid() << std::endl;
    std::cout << "dateValid: " << (int)parsed.dateValid() << std::endl;

    std::cout << "tAcc: " << (int)parsed.tAcc() << "ns" << std::endl;
    std::cout << std::endl;
  }
  else std::cout << "Type error" << std::endl;
  //GLO Demo End

  //GPS Demo
  std::cout << "TIMEGPS Demo" << std::endl;
  x.ubx_send("NAV", "TIMEGPS");
  sleep(2);

  auto nav_timegps = x.fetch_ubx("NAV", "TIMEGPS");

  if (ubx::nav::timegps::type(nav_timegps))
  {
    ubx::nav::timegps parsed = ubx::nav::timegps(nav_timegps);

    std::cout << "iTOW: " << parsed.iTOW() << "ms" << std::endl;
    std::cout << "fTOD: " << parsed.fTOD() << "ns" << std::endl;
    std::cout << "week: " << parsed.week() << std::endl;
    std::cout << "leapS: " << (int)parsed.leapS() << "s" << std::endl;

    std::cout << "twoValid: " << (int)parsed.towValid() << std::endl;
    std::cout << "weekValid: " << (int)parsed.weekValid() << std::endl;
    std::cout << "leapSValid: " << (int)parsed.leapSValid() << std::endl;

    std::cout << "tAcc: " << (int)parsed.tAcc() << "ns" << std::endl;
    std::cout << std::endl;
  }
  else std::cout << "Type error" << std::endl;
  //GPS Demo End

  //GAL Demo
  std::cout << "TIMEGAL Demo" << std::endl;
  x.ubx_send("NAV", "TIMEGAL");
  sleep(2);

  auto nav_timegal = x.fetch_ubx("NAV", "TIMEGAL");

  if (ubx::nav::timegal::type(nav_timegal))
  {
    ubx::nav::timegal parsed = ubx::nav::timegal(nav_timegal);

    std::cout << "iTOW: " << parsed.iTOW() << "ms" << std::endl;
    std::cout << "galTow: " << parsed.galTow() << "s" << std::endl;
    std::cout << "fGalTow: " << parsed.fGalTow() << "ns" << std::endl;
    std::cout << "galWno: " << parsed.galWno() << std::endl;
    std::cout << "leapS: " << (int)parsed.leapS() << "s" << std::endl;

    std::cout << "galTowValid: " << (int)parsed.galTowValid() << std::endl;
    std::cout << "galWnoValid: " << (int)parsed.galWnoValid() << std::endl;
    std::cout << "leapSValid: " << (int)parsed.leapSValid() << std::endl;

    std::cout << "tAcc: " << (int)parsed.tAcc() << "ns" << std::endl;
    std::cout << std::endl;
  }
  else std::cout << "Type error" << std::endl;
  //GAL Demo End

//BDS Demo
  std::cout << "TIMEBDS Demo" << std::endl;
  x.ubx_send("NAV", "TIMEBDS");
  sleep(2);

  auto nav_timebds = x.fetch_ubx("NAV", "TIMEBDS");

  if (ubx::nav::timebds::type(nav_timebds))
  {
    ubx::nav::timebds parsed = ubx::nav::timebds(nav_timebds);

    std::cout << "iTOW: " << parsed.iTOW() << "ms" << std::endl;
    std::cout << "SOW: " << parsed.SOW() << "s" << std::endl;
    std::cout << "fSOW: " << parsed.fSOW() << "ns" << std::endl;
    std::cout << "week: " << parsed.week() << std::endl;
    std::cout << "leapS: " << (int)parsed.leapS() << "s" << std::endl;

    std::cout << "sowValid: " << (int)parsed.sowValid() << std::endl;
    std::cout << "weekValid: " << (int)parsed.weekValid() << std::endl;
    std::cout << "leapSValid: " << (int)parsed.leapSValid() << std::endl;

    std::cout << "tAcc: " << (int)parsed.tAcc() << "ns" << std::endl;
    std::cout << std::endl;
  }
  else std::cout << "Type error" << std::endl;
  //BDS Demo End

  //UTC Demo
  std::cout << "TIMEUTC Demo" << std::endl;
  x.ubx_send("NAV", "TIMEUTC");
  sleep(2);

  auto nav_timeutc = x.fetch_ubx("NAV", "TIMEUTC");

  if (ubx::nav::timeutc::type(nav_timeutc))
  {
    ubx::nav::timeutc parsed = ubx::nav::timeutc(nav_timeutc);

    std::cout << "iTOW: " << parsed.iTOW() << "ms" << std::endl;
    std::cout << "tAcc: " << parsed.tAcc() << "ns" << std::endl;
    std::cout << "nano: " << parsed.nano() << std::endl;
    std::cout << "year: " << parsed.year() << std::endl;
    std::cout << "month: " << (int)parsed.month() << std::endl;
    std::cout << "day: " << (int)parsed.day() << std::endl;
    std::cout << "hour: " << (int)parsed.hour() << std::endl;
    std::cout << "min: " << (int)parsed.min() << std::endl;
    std::cout << "sec: " << (int)parsed.sec() << std::endl;

    std::cout << "validTOW: " << (int)parsed.validTOW() << std::endl;
    std::cout << "validWKN: " << (int)parsed.validWKN() << std::endl;
    std::cout << "validUTC: " << (int)parsed.validUTC() << std::endl;

    switch (parsed.utcStandard())
    {
      case 0: std::cout << "Information no available" << std::endl; break;
      case 1: std::cout << "Communications Research Labratory (CRL)" << std::endl; break;
      case 2: std::cout << "National Institute of Standards and Technology (NIST)" << std::endl; break;
      case 3: std::cout << "U.S. Naval Observatory (USNO)" << std::endl; break;
      case 4: std::cout << "International Bureau of Weights and Measures (BIPM)" << std::endl; break;
      case 5: std::cout << "European Laboratory (tbd)" << std::endl; break;
      case 6: std::cout << "Former Soviet Union (SU)" << std::endl; break;
      case 7: std::cout << "National Time Service Center, China (NTSC)" << std::endl; break;
      case 15: std::cout << "Unknown" << std::endl;
    }

    std::cout << std::endl;
  }
  else std::cout << "Type error" << std::endl;
  //UTC Demo End
}
