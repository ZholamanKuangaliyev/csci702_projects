CXX      = g++
CXXFLAGS = -std=c++17 -O2
LIBS_BASE = -lpigpiod_if2 -lpthread

# Task 2 — Dual motor PWM control
task2: motor_l298n_pigpio.cpp
	$(CXX) $(CXXFLAGS) motor_l298n_pigpio.cpp -o motor_l298n_pigpio $(LIBS_BASE)

# Task 3 — Virtual joystick via WebSocket (Crow)
task3: joystick.cpp
	$(CXX) $(CXXFLAGS) joystick.cpp -o joystick -I. $(LIBS_BASE) -latomic

# Task 4 — PD yaw control via IMU
task4: pd_control.cpp
	$(CXX) $(CXXFLAGS) pd_control.cpp -o pd_control $(LIBS_BASE)

# Task 5 — Encoder reading
task5: encoder.cpp
	$(CXX) $(CXXFLAGS) encoder.cpp -o encoder $(LIBS_BASE)

# Build all
all: task2 task3 task4 task5

clean:
	rm -f motor_l298n_pigpio joystick encoder
