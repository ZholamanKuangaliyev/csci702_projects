// Dagu Rover 5 — Task 5: Encoder reading
// Drives left motor at 30% → 60% → 100% → stop
// Prints count, wheel angle (rad), and RPM every 100 ms
//
// Encoder pins: A=GPIO17, B=GPIO25 (via level shifter)
// Motor:        ENA=GPIO12, IN1=GPIO23, IN2=GPIO24
//
// Requires pigpiod running:
//   sudo systemctl start pigpiod
//
// Compile:
//   g++ encoder.cpp -o encoder -lpigpiod_if2 -lpthread
//
// Run:
//   ./encoder

#include <pigpiod_if2.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <algorithm>

// Encoder resolution: 333 counts/rev x4 decoding = 1332 counts/rev
static constexpr double CPR = 1332.0;
static constexpr double TWO_PI = 2.0 * M_PI;

// ── QuadratureEncoder ────────────────────────────────────────────────────────

class QuadratureEncoder {
public:
    QuadratureEncoder(int pi, int gpioA, int gpioB, unsigned glitch_us = 50)
        : pi_(pi), gpioA_(gpioA), gpioB_(gpioB), glitch_us_(glitch_us) {}

    void init() {
        set_mode(pi_, gpioA_, PI_INPUT);
        set_mode(pi_, gpioB_, PI_INPUT);
        set_pull_up_down(pi_, gpioA_, PI_PUD_UP);
        set_pull_up_down(pi_, gpioB_, PI_PUD_UP);
        set_glitch_filter(pi_, gpioA_, glitch_us_);
        set_glitch_filter(pi_, gpioB_, glitch_us_);

        int a = gpio_read(pi_, gpioA_);
        int b = gpio_read(pi_, gpioB_);
        last_state_.store((a << 1) | b);

        cbA_ = callback_ex(pi_, gpioA_, EITHER_EDGE, alertTrampoline, this);
        cbB_ = callback_ex(pi_, gpioB_, EITHER_EDGE, alertTrampoline, this);

        last_vel_tick_  = get_current_tick(pi_);
        last_vel_count_ = count_.load();
    }

    void shutdown() {
        callback_cancel(cbA_);
        callback_cancel(cbB_);
    }

    int32_t getCount() const { return count_.load(); }
    void    resetCount()      { count_.store(0); }

    // Wheel angle in radians
    double getAngleRad() const {
        return (TWO_PI / CPR) * count_.load();
    }

    // Angular velocity in RPM — call at fixed interval (e.g. 100 ms)
    double getRPM() {
        uint32_t now_tick  = get_current_tick(pi_);
        int32_t  now_count = count_.load();

        uint32_t dt_us = now_tick - last_vel_tick_;
        if (dt_us == 0) dt_us = 1;

        int32_t dc = now_count - last_vel_count_;

        last_vel_tick_  = now_tick;
        last_vel_count_ = now_count;

        double dt_s      = static_cast<double>(dt_us) * 1e-6;
        double rev_per_s = (static_cast<double>(dc) / CPR) / dt_s;
        return rev_per_s * 60.0;
    }

private:
    int      pi_, gpioA_, gpioB_;
    unsigned glitch_us_;
    int      cbA_{0}, cbB_{0};

    std::atomic<int32_t>  count_{0};
    std::atomic<int>      last_state_{0};
    uint32_t              last_vel_tick_{0};
    int32_t               last_vel_count_{0};

    static void alertTrampoline(int /*pi*/, unsigned /*gpio*/, unsigned level,
                                uint32_t /*tick*/, void* user) {
        if (level == PI_TIMEOUT) return;
        static_cast<QuadratureEncoder*>(user)->onEdge();
    }

    void onEdge() {
        const int a = gpio_read(pi_, gpioA_);
        const int b = gpio_read(pi_, gpioB_);
        const int new_state = (a << 1) | b;
        const int old_state = last_state_.exchange(new_state);

        const int idx = (old_state << 2) | new_state;
        static const int8_t delta[16] = {
             0, +1, -1,  0,
            -1,  0,  0, +1,
            +1,  0,  0, -1,
             0, -1, +1,  0
        };
        const int8_t d = delta[idx & 0x0F];
        if (d != 0) count_.fetch_add(d);
    }
};

// ── MotorL298N ───────────────────────────────────────────────────────────────

class MotorL298N {
public:
    MotorL298N(int pi, int enaPin, int in1Pin, int in2Pin)
        : pi_(pi), ena_(enaPin), in1_(in1Pin), in2_(in2Pin) {}

    void setup() {
        set_mode(pi_, in1_, PI_OUTPUT);
        set_mode(pi_, in2_, PI_OUTPUT);
        gpio_write(pi_, in1_, 0);
        gpio_write(pi_, in2_, 0);
        set_PWM_frequency(pi_, ena_, 1000);
        set_PWM_range(pi_, ena_, 255);
        set_PWM_dutycycle(pi_, ena_, 0);
    }

    void forward(int pct) {
        gpio_write(pi_, in1_, 1);
        gpio_write(pi_, in2_, 0);
        set_PWM_dutycycle(pi_, ena_, pct * 255 / 100);
    }

    void stop() {
        set_PWM_dutycycle(pi_, ena_, 0);
        gpio_write(pi_, in1_, 0);
        gpio_write(pi_, in2_, 0);
    }

private:
    int pi_, ena_, in1_, in2_;
};

// ── helpers ──────────────────────────────────────────────────────────────────

static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static void printHeader() {
    std::cout << std::left
              << std::setw(8)  << "PWM%"
              << std::setw(10) << "Count"
              << std::setw(14) << "Angle(rad)"
              << std::setw(10) << "RPM"
              << std::endl;
    std::cout << std::string(42, '-') << std::endl;
}

static void printRow(int pct, int32_t count, double angle, double rpm) {
    std::cout << std::left  << std::fixed << std::setprecision(3)
              << std::setw(8)  << pct
              << std::setw(10) << count
              << std::setw(14) << angle
              << std::setw(10) << rpm
              << std::endl;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cerr << "pigpiod connection failed — run: sudo systemctl start pigpiod" << std::endl;
        return 1;
    }
    std::cout << "Connected to pigpiod OK\n" << std::endl;

    // Left motor: ENA=GPIO12, IN1=GPIO23, IN2=GPIO24
    // Encoder:    A=GPIO17,   B=GPIO25
    MotorL298N      motor(pi, 12, 23, 24);
    QuadratureEncoder enc(pi, 25, 17);

    motor.setup();
    enc.init();

    printHeader();

    // Run at 30%, 60%, 100% — sample every 100 ms for 3 s each
    for (int pct : {30, 60, 100}) {
        enc.resetCount();
        motor.forward(pct);
        for (int i = 0; i < 30; ++i) {   // 30 × 100 ms = 3 s
            sleep_ms(100);
            printRow(pct, enc.getCount(), enc.getAngleRad(), enc.getRPM());
        }
    }

    motor.stop();

    std::cout << std::string(42, '-') << std::endl;
    std::cout << "Final count: " << enc.getCount()
              << "  angle: " << std::fixed << std::setprecision(3)
              << enc.getAngleRad() << " rad" << std::endl;

    enc.shutdown();
    pigpio_stop(pi);
    std::cout << "Done." << std::endl;
    return 0;
}
