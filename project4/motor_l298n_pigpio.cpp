#include <pigpio.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

class MotorL298N {
public:
    MotorL298N(int enaPin, int in1Pin, int in2Pin)
        : ena(enaPin), in1(in1Pin), in2(in2Pin) {}

    bool init() {
        if (gpioInitialise() < 0) {
            std::cerr << "pigpio initialization failed." << std::endl;
            return false;
        }

        gpioSetMode(ena, PI_OUTPUT);
        gpioSetMode(in1, PI_OUTPUT);
        gpioSetMode(in2, PI_OUTPUT);

        gpioWrite(in1, 0);
        gpioWrite(in2, 0);

        // PWM frequency for ENA
        gpioSetPWMfrequency(ena, 1000);   // 1 kHz
        gpioSetPWMrange(ena, 255);        // duty cycle range 0..255
        gpioPWM(ena, 0);

        return true;
    }

    void forward(int speedPercent) {
        int pwmValue = percentToPwm(speedPercent);
        gpioWrite(in1, 1);
        gpioWrite(in2, 0);
        gpioPWM(ena, pwmValue);
    }

    void reverse(int speedPercent) {
        int pwmValue = percentToPwm(speedPercent);
        gpioWrite(in1, 0);
        gpioWrite(in2, 1);
        gpioPWM(ena, pwmValue);
    }

    void stop() {
        gpioPWM(ena, 0);
        gpioWrite(in1, 0);
        gpioWrite(in2, 0);
    }

    void shutdown() {
        stop();
        gpioTerminate();
    }

private:
    int ena;
    int in1;
    int in2;

    int percentToPwm(int speedPercent) {
        speedPercent = std::clamp(speedPercent, 0, 100);
        return speedPercent * 255 / 100;
    }
};

int main() {
    MotorL298N motor(12, 23, 24);

    if (!motor.init()) {
        return 1;
    }

    std::cout << "Forward ramp..." << std::endl;
    for (int speed = 0; speed <= 100; speed += 10) {
        motor.forward(speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    motor.stop();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "Reverse ramp..." << std::endl;
    for (int speed = 0; speed <= 100; speed += 10) {
        motor.reverse(speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    motor.shutdown();
    return 0;
}