CXX=g++
CXX_FLAGS=-std=c++14 -O3 -Wall -Werror
CXX_INCLUDE=-I /usr/include -I.
BOOST_FLAGS=-L/usr/lib/x86_64-linux-gnu -lboost_system -lpthread -lrt

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
	$(CXX) $(CXX_FLAGS) -c -o $@ $< $(BOOST_FLAGS)

lib: $(OBJS)
	ar rvs libCracl.a $(OBJS)

%.elf: %.cc lib
	$(CXX) $(CXX_FLAGS) $(CXX_INCLUDE) -o $@ $< -L. -lCracl $(BOOST_FLAGS)

cracl: lib
	mv libCracl.a ../../lib/

test: $(TESTS)

clean:
	rm -f libCracl.a $(OBJS) $(TESTS)
