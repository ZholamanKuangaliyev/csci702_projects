// Dagu Rover 5 — Task 2: Dual motor PWM control
// Test sequence: 30% fwd → 60% fwd → stop → 30% reverse → stop
// HW PWM on GPIO12 (left ENA) and GPIO13 (right ENB)
//
// NOTE: stop pigpiod before running:
//   sudo systemctl stop pigpiod
//   sudo ./motor_l298n_pigpio

#include <pigpio.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdlib>

static constexpr unsigned PWM_FREQ  = 1000;      // Hz
static constexpr unsigned PWM_RANGE = 1000000;   // 0–1,000,000 (pigpio HW PWM)

class MotorL298N {
public:
    MotorL298N(int enaPin, int in1Pin, int in2Pin, const char* name)
        : ena(enaPin), in1(in1Pin), in2(in2Pin), name(name) {}

    void setup() {
        gpioSetMode(in1, PI_OUTPUT);
        gpioSetMode(in2, PI_OUTPUT);
        gpioWrite(in1, 0);
        gpioWrite(in2, 0);
        int r = gpioHardwarePWM(ena, PWM_FREQ, 0);
        if (r != 0) {
            std::cerr << name << ": gpioHardwarePWM init failed, code=" << r << std::endl;
        }
    }

    void forward(int speedPercent) {
        gpioWrite(in1, 1);
        gpioWrite(in2, 0);
        int r = gpioHardwarePWM(ena, PWM_FREQ, percentToDuty(speedPercent));
        std::cout << "  " << name << " forward " << speedPercent << "% (duty="
                  << percentToDuty(speedPercent) << ", r=" << r << ")" << std::endl;
    }

    void reverse(int speedPercent) {
        gpioWrite(in1, 0);
        gpioWrite(in2, 1);
        int r = gpioHardwarePWM(ena, PWM_FREQ, percentToDuty(speedPercent));
        std::cout << "  " << name << " reverse " << speedPercent << "% (duty="
                  << percentToDuty(speedPercent) << ", r=" << r << ")" << std::endl;
    }

    void stop() {
        gpioHardwarePWM(ena, PWM_FREQ, 0);
        gpioWrite(in1, 0);
        gpioWrite(in2, 0);
        std::cout << "  " << name << " stop" << std::endl;
    }

private:
    int ena, in1, in2;
    const char* name;

    unsigned percentToDuty(int pct) {
        pct = std::clamp(pct, 0, 100);
        return static_cast<unsigned>(pct * PWM_RANGE / 100);
    }
};

static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed — is pigpiod still running? Try: sudo systemctl stop pigpiod" << std::endl;
        return 1;
    }
    std::cout << "pigpio OK" << std::endl;

    // Left:  ENA=GPIO12, IN1=GPIO23, IN2=GPIO24
    // Right: ENB=GPIO13, IN3=GPIO27, IN4=GPIO22
    MotorL298N left (12, 23, 24, "LEFT ");
    MotorL298N right(13, 27, 22, "RIGHT");

    left.setup();
    right.setup();

    std::cout << "\n=== Task 2: Dual motor test ===" << std::endl;

    std::cout << "\n[1] 30% forward (3 s)" << std::endl;
    left.forward(30);
    right.forward(30);
    sleep_ms(3000);

    std::cout << "\n[2] 60% forward (3 s)" << std::endl;
    left.forward(60);
    right.forward(60);
    sleep_ms(3000);

    std::cout << "\n[3] Stop (2 s)" << std::endl;
    left.stop();
    right.stop();
    sleep_ms(2000);

    std::cout << "\n[4] 30% reverse (3 s)" << std::endl;
    left.reverse(30);
    right.reverse(30);
    sleep_ms(3000);

    std::cout << "\n[5] Stop" << std::endl;
    left.stop();
    right.stop();

    gpioTerminate();
    std::cout << "\nDone." << std::endl;
    return 0;
}
