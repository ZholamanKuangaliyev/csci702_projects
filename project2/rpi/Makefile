# Makefile for Raspberry Pi (IMU UDP Sender)
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

TARGET = pi_sender
SRCS = pi_sender.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)