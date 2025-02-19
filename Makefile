# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17
LDFLAGS = -lhiredis

# Source files
SRCS = simulation.cpp
OBJS = $(SRCS:.cpp=.o)

# Output executable
TARGET = simulation

# Default target
all: $(TARGET)

# Compile source files
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

# Compile object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean project
clean:
	rm -f $(OBJS) $(TARGET)

# Rebuild
rebuild: clean all