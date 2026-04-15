// Dagu Rover 5 — Task 2: Dual motor PWM control
// Uses pigpiod daemon interface (pigpiod_if2) — pigpiod must be running:
//   sudo systemctl start pigpiod
//
// Compile:
//   g++ motor_l298n_pigpio.cpp -o motor_l298n_pigpio -lpigpiod_if2 -lpthread
//
// Run:
//   ./motor_l298n_pigpio
//
// Test sequence: 30% fwd → 60% fwd → 100% fwd → stop → 30% reverse → stop

#include <pigpiod_if2.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

class MotorL298N {
public:
    MotorL298N(int pi, int enaPin, int in1Pin, int in2Pin)
        : pi(pi), ena(enaPin), in1(in1Pin), in2(in2Pin) {}

    void setup() {
        set_mode(pi, in1, PI_OUTPUT);
        set_mode(pi, in2, PI_OUTPUT);
        gpio_write(pi, in1, 0);
        gpio_write(pi, in2, 0);

        set_PWM_frequency(pi, ena, 1000);
        set_PWM_range(pi, ena, 255);
        set_PWM_dutycycle(pi, ena, 0);
    }

    void forward(int speedPercent) {
        gpio_write(pi, in1, 1);
        gpio_write(pi, in2, 0);
        set_PWM_dutycycle(pi, ena, percentToPwm(speedPercent));
    }

    void reverse(int speedPercent) {
        gpio_write(pi, in1, 0);
        gpio_write(pi, in2, 1);
        set_PWM_dutycycle(pi, ena, percentToPwm(speedPercent));
    }

    void stop() {
        set_PWM_dutycycle(pi, ena, 0);
        gpio_write(pi, in1, 0);
        gpio_write(pi, in2, 0);
    }

private:
    int pi, ena, in1, in2;

    int percentToPwm(int pct) {
        pct = std::clamp(pct, 0, 100);
        return pct * 255 / 100;
    }
};

static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cerr << "pigpiod connection failed — is pigpiod running? Try: sudo systemctl start pigpiod" << std::endl;
        return 1;
    }
    std::cout << "Connected to pigpiod OK" << std::endl;

    // Left:  ENA=GPIO12, IN1=GPIO23, IN2=GPIO24
    // Right: ENB=GPIO13, IN3=GPIO27, IN4=GPIO22
    MotorL298N left (pi, 12, 23, 24);
    MotorL298N right(pi, 13, 27, 22);

    left.setup();
    right.setup();

    std::cout << "\n=== Task 2: Dual motor test ===" << std::endl;

    std::cout << "[1] 30% forward (3 s)" << std::endl;
    left.forward(30);
    right.forward(30);
    sleep_ms(3000);

    std::cout << "[2] 60% forward (3 s)" << std::endl;
    left.forward(60);
    right.forward(60);
    sleep_ms(3000);

    std::cout << "[3] 100% forward (3 s)" << std::endl;
    left.forward(100);
    right.forward(100);
    sleep_ms(3000);

    std::cout << "[4] Stop (2 s)" << std::endl;
    left.stop();
    right.stop();
    sleep_ms(2000);

    std::cout << "[5] 30% reverse (3 s)" << std::endl;
    left.reverse(30);
    right.reverse(30);
    sleep_ms(3000);

    std::cout << "[6] Stop" << std::endl;
    left.stop();
    right.stop();

    pigpio_stop(pi);
    std::cout << "Done." << std::endl;
    return 0;
}
