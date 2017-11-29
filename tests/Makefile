CXX_FLAGS=-std=c++14 -O3
CXX_INCLUDE=-I ../include

all:
	g++ ${CXX_FLAGS} ${CXX_INCLUDE} -o run test.cc -L/usr/lib/x86_64-linux-gnu -lboost_system -lpthread

clean:
	rm run *.o
