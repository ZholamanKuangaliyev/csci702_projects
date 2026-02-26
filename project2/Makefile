# Compiler settings
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

# Linker flags for FreeGLUT and OpenGL on Raspberry Pi
# Included -lX11 as recommended by the project manual for linker errors
LDFLAGS = -lglut -lGL -lGLU -lX11

# Target executable name
TARGET = cube_quat

# Source files
SRCS = imu_cube_vis.cpp

# Default rule to build the target
all: $(TARGET)

# Rule to link and compile
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $(TARGET)

# Rule to clean the workspace
clean:
	rm -f $(TARGET)

# Rule to build and run the program immediately
run: $(TARGET)
	./$(TARGET)