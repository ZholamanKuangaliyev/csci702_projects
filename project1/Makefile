# Makefile for Raspberry Pi 3 Model B

# Compiler
CXX = g++

# Compiler flags
# -Wall: Enable all warnings
# -std=c++11: Ensure C++11 support
CXXFLAGS = -Wall -std=c++11

# Target executable name
TARGET = main

# Source files
SRCS = main.cpp

# Default target
all: $(TARGET)

# Rule to build the executable
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

# Clean up build files
clean:
	rm -f $(TARGET)

.PHONY: all clean
