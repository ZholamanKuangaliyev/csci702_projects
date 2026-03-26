# RPi Makefile for Project 3 tasks (compile on the Raspberry Pi)
CXX      = g++
CXXFLAGS = -O2 -std=c++17 -Wall

all: task1_webserver/imu_cgi task3_server task4_threaded_server

task1_webserver/imu_cgi: task1_webserver/imu_cgi.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

task3_server: task3_server.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

task4_threaded_server: task4_threaded_server.cpp
	$(CXX) $(CXXFLAGS) -pthread -o $@ $< -lm

clean:
	rm -f task1_webserver/imu_cgi task3_server task4_threaded_server

.PHONY: all clean
