CXX=g++-7
CXXFLAGS=-std=c++14 -O3 -Wall -Werror
CXXINCLUDE=-I /usr/include -I.
LDFLAGS=-L/usr/lib/arm-linux-gnueabihf
LDLIBS=-lboost_system -lpthread -lrt

DEPS=cracl/base/device.hpp \
		 $(wildcard cracl/microsemi/*.hpp) \
		 $(wildcard cracl/jackson_labs/*.hpp) \
		 $(wildcard cracl/ublox/msg/class/*.hpp) \
		 $(wildcard cracl/ublox/msg/*.hpp) \
		 $(wildcard cracl/ublox/*.hpp) \

OBJS=$(DEPS:%.hpp=%.o)

TEST_SRCS=$(wildcard tests/*.cc)

TESTS=$(TEST_SRCS:%.cc=%.elf)

%.o: %.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) $(CXXINCLUDE) -c -o $@ $< $(LDFLAGS) $(LDLIBS)

lib: $(OBJS)
	ar rs libCracl.a $(OBJS)

%.elf: %.cc lib
	$(CXX) $(CXXFLAGS) $(CXXINCLUDE) -o $@ $< -L. -lCracl $(LDFLAGS) $(LDLIBS)

cracl: lib
	mv libCracl.a ../../lib/

test: $(TESTS)

clean:
	rm -f libCracl.a $(OBJS) $(TESTS)
