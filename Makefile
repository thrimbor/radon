# For debugging purposes, set -O0 and -g.
# For production use, set -O3.
#CXXFLAGS = -O3 -g -Wall -pedantic -fopenmp
CXXFLAGS = -O3 -Wall -pedantic -march=native -mtune=native -msse3 -ftree-vectorize -ffast-math  -std=gnu++11 -g
LDLIBS = -fopenmp -lOpenCL
CXX = g++
COMPILE.cc = ${CXX} ${CXXFLAGS} ${CPPFLAGS} -c

SOURCES := $(wildcard [^_]*.cpp)
OBJECTS := ${SOURCES:.cpp=.o}
BINARY = raytrace

all: $(OBJECTS)
	$(CXX) -o $(BINARY) $(OBJECTS) $(LDLIBS)

%.o: %.cpp
	${COMPILE.cc} -o $@ $<

clean:
	$(RM) $(BINARY) $(OBJECTS)
