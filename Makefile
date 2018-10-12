CXX=g++
CXX_FLAGS=-std=c++14 -O3 -Wall -Werror
CXX_INCLUDE=-I /usr/include -I.
BOOST_FLAGS=-L/usr/lib/x86_64-linux-gnu -lboost_system -lpthread -lrt
DEPS=cracl/device.hpp cracl/clock/csac.hpp cracl/clock/firefly.hpp \
		 cracl/clock/gpstcxo.hpp cracl/receiver/ublox_8.hpp
OBJS=cracl/device.o cracl/clock/csac.o cracl/clock/firefly.o \
		 cracl/clock/gpstcxo.o cracl/receiver/ublox_8.o

%.o: %.cpp $(DEPS)
	$(CXX) $(CXX_FLAGS) -c -o $@ $< $(BOOST_FLAGS)

cracl: $(OBJS)
	ar rvs libCracl.a $(OBJS)
	mv libCracl.a ../../lib/

clean:
	rm $(OBJS)
