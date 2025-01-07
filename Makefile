CXX       := g++
CXXFLAGS  := -std=c++11 -Wall -Wextra
LDLIBS    := -lhiredis

TARGET    := simulation

SRCS      := simulation.cpp send_data.cpp
OBJS      := $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<
clean:
	rm -f $(OBJS) $(TARGET)
