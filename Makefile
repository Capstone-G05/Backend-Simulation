CXX      := g++
CXXFLAGS := -Wall -Wextra -g
LDFLAGS  := -lhiredis

TARGET   := simulation

SRCS := simulation.cpp \
        i2c_interface.cpp \
        redis_interface.cpp

OBJS := $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f $(TARGET) $(OBJS)
