// Dagu Rover 5 — Task 2: Dual motor PWM control
// Test sequence: 30% fwd → 60% fwd → stop → 30% reverse → stop

#include <pigpio.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

class MotorL298N {
public:
    MotorL298N(int enaPin, int in1Pin, int in2Pin)
        : ena(enaPin), in1(in1Pin), in2(in2Pin) {}

    void setup() {
        gpioSetMode(ena, PI_OUTPUT);
        gpioSetMode(in1, PI_OUTPUT);
        gpioSetMode(in2, PI_OUTPUT);

        gpioWrite(in1, 0);
        gpioWrite(in2, 0);

        gpioSetPWMfrequency(ena, 1000);
        gpioSetPWMrange(ena, 255);
        gpioPWM(ena, 0);
    }

    void forward(int speedPercent) {
        gpioWrite(in1, 1);
        gpioWrite(in2, 0);
        gpioPWM(ena, percentToPwm(speedPercent));
    }

    void reverse(int speedPercent) {
        gpioWrite(in1, 0);
        gpioWrite(in2, 1);
        gpioPWM(ena, percentToPwm(speedPercent));
    }

    void stop() {
        gpioPWM(ena, 0);
        gpioWrite(in1, 0);
        gpioWrite(in2, 0);
    }

private:
    int ena, in1, in2;

    int percentToPwm(int pct) {
        pct = std::clamp(pct, 0, 100);
        return pct * 255 / 100;
    }
};

static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed" << std::endl;
        return 1;
    }

    // Left:  ENA=GPIO12, IN1=GPIO23, IN2=GPIO24
    // Right: ENB=GPIO13, IN3=GPIO27, IN4=GPIO22
    MotorL298N left (12, 23, 24);
    MotorL298N right(13, 27, 22);

    left.setup();
    right.setup();

    std::cout << "=== Task 2: Dual motor test ===" << std::endl;

    std::cout << "[1] 30% forward (3 s)" << std::endl;
    left.forward(30);
    right.forward(30);
    sleep_ms(3000);

    std::cout << "[2] 60% forward (3 s)" << std::endl;
    left.forward(60);
    right.forward(60);
    sleep_ms(3000);

    std::cout << "[3] Stop (2 s)" << std::endl;
    left.stop();
    right.stop();
    sleep_ms(2000);

    std::cout << "[4] 30% reverse (3 s)" << std::endl;
    left.reverse(30);
    right.reverse(30);
    sleep_ms(3000);

    std::cout << "[5] Stop" << std::endl;
    left.stop();
    right.stop();

    gpioTerminate();
    std::cout << "Done." << std::endl;
    return 0;
}
