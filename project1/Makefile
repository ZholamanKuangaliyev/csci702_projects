# Combined Makefile for Project 1
CXX = g++
CXXFLAGS = -Wall -std=c++11

# Executable names
TARGET_MAIN = main
TARGET_TASK7 = task7

# Source files
SRCS_MAIN = main.cpp
SRCS_TASK7 = task7.cpp

# Default target: builds both programs
all: $(TARGET_MAIN) $(TARGET_TASK7)

# Rule for the original main project
$(TARGET_MAIN): $(SRCS_MAIN)
	$(CXX) $(CXXFLAGS) -o $(TARGET_MAIN) $(SRCS_MAIN)

# Rule for Task 7
$(TARGET_TASK7): $(SRCS_TASK7)
	$(CXX) $(CXXFLAGS) -o $(TARGET_TASK7) $(SRCS_TASK7)

# Clean up all executables
clean:
	rm -f $(TARGET_MAIN) $(TARGET_TASK7)

.PHONY: all clean