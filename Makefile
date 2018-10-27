CXX=g++
CXX_FLAGS=-std=c++14 -O3 -Wall -Werror
CXX_INCLUDE=-I /usr/include -I.
BOOST_FLAGS=-L/usr/lib/x86_64-linux-gnu -lboost_system -lpthread -lrt

DEPS=cracl/base/device.hpp cracl/ublox/msg/base.hpp \
		 $(wildcard cracl/microsemi/*.hpp) \
		 $(wildcard cracl/jackson_labs/*.hpp) \
		 $(wildcard cracl/ublox/*.hpp) \
		 $(wildcard cracl/ublox/msg/class/*.hpp)

OBJS=$(DEPS:%.hpp=%.o)

%.o: %.cpp $(DEPS)
	$(CXX) $(CXX_FLAGS) -c -o $@ $< $(BOOST_FLAGS)

cracl: $(OBJS)
	ar rvs libCracl.a $(OBJS)
	mv libCracl.a ../../lib/

clean:
	rm $(OBJS)
