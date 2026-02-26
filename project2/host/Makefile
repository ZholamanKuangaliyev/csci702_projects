# Makefile for Ubuntu Host PC (IMU UDP Receiver/Visualizer)
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

# Linker flags for FreeGLUT and OpenGL
LDFLAGS = -lglut -lGL -lGLU

TARGET = pc_receiver
SRCS = pc_receiver.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)