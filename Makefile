CXX=g++
CXX_FLAGS=-std=c++14 -O3 -Wall -Werror
CXX_INCLUDE=-I /usr/include -I.
BOOST_FLAGS=-L/usr/lib/x86_64-linux-gnu -lboost_system -lpthread -lrt
DEPS=cracl/base/device.hpp \
		 cracl/microsemi/gps300.hpp \
		 cracl/microsemi/sa45s.hpp \
		 cracl/jackson_labs/firefly_1a.hpp \
		 cracl/ublox/m8.hpp
OBJS=cracl/base/device.o \
		 cracl/microsemi/gps300.o \
		 cracl/microsemi/sa45s.o \
		 cracl/jackson_labs/firefly_1a.o \
		 cracl/ublox/m8.o

%.o: %.cpp $(DEPS)
	$(CXX) $(CXX_FLAGS) -c -o $@ $< $(BOOST_FLAGS)

cracl: $(OBJS)
	ar rvs libCracl.a $(OBJS)
	mv libCracl.a ../../lib/

clean:
	rm $(OBJS)
