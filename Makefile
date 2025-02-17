CXX = g++

CXXFLAGS = -std=c++11 -Wall -Wextra -O2

LIBS = -lhiredis

TARGET = simulation

SRCS = simulation.cpp

OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


clean:
	rm -f $(OBJS) $(TARGET)

run: $(TARGET)
	./$(TARGET)
