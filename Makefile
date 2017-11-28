CXX_FLAGS=-std=c++14 -O3

all:
	g++ ${CXX_FLAGS} -o run test.cc -L/usr/lib/x86_64-linux-gnu -lboost_system -lpthread

clean:
	rm run *.o
