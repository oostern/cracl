# cracl

## Common Receiver And Clock Library

A library to interface with various devices, including
* u-blox receivers (e.g. M8T and M6T)
* Jackson Labs oscillators (e.g. Firefly-1A, the OCXO used in USRP N2XX)
* Microsemi clocks (e.g. sa45s CSAC and GPS300)

Feature development has been laregly as needed, though the non-UBX stuff is pretty simple and therefore mostly complete.

Sending of all UBX messages should be supported, though not fully tested. When sending a message that has an actual payload (i.e. not a basic poll message), simply pass the arguments in the call to `ubx_send`. They *must* be the correct data type per the UBX spec (e.g. 16 bit unsigned int), since under the hood `ubx_send` blindly bundles up anything you give it.

PUBX messages with the u-blox are similar, just pass whatever you want into the `pubx_send` function. NMEA messages are returned as strings for you to parse yourself.

We use Ubuntu 18.04 and 16.04, but it should theoretically be portable, you'll just need Boost ASIO.

A Makefile is included which generates a shared library file to link against. By default it places it two directories up in a folder called lib (that must exist). You might want to change this. After including the headers in your code, just link against the library.

The constructors for each device require the port at a minimum, but you can also set the baud rate, timeout, etc if needed.
### Examples:

#### Communicating with u-blox LEA-M8T/NEO-M8T


Include relevant headers
```
#include <cracl/ublox/m8.hpp>
```

Create the connection
```
using namespace cracl;
m8 x("/dev/ttyACM1");
```

Send a UBX message
```
x.ubx_send("NAV", "STATUS");
```

Receive a response
```
auto m = x.fetch_ubx("NAV", "STATUS");
```

Parse the message and use contents
```
// Verify (confirms a non-empty message was received, and checksum is good)
if (ubx::nav::status::type(m))
{
  ubx::nav::status parsed = ubx::nav::status(m);

  std::cout << "Spoof Status: ";

  // Read spoofDetState attribute (names and values correspond to field names in
  //                               UBX protocol, except when field name is
  //                               capilatized)
  switch (parsed.spoofDetState())
  {
    case 0: std::cout << "UNKNOWN" << std::endl; break;
    case 1: std::cout << "OKAY" << std::endl; break;
    case 2: std::cout << "SPOOF INDICATORS" << std::endl; break;
    case 3: std::cout << "MULTIPLE SPOOF INDICATORS" << std::endl;
  }
}
```

#### Communicating with FireFly-1A GPSDO

Include relevant headers
```
#include <cracl/jackson_labs/firefly_1a.hpp>
```

Create the connection
```
using namespace cracl;
firefly_1a g("/dev/ttyUSB0");
```

Disable GPS disciplining
```
g.sync_hold_init();
```

Enable GPS disciplining
```
g.sync_hold_rec_init();
```
