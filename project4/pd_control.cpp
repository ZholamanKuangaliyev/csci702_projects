// Dagu Rover 5 — Task 4: PD Yaw Control via IMU
// IMU: AltIMU-10 v5, LSM6DS33 @ I2C bus 1, address 0x6B
// Motor driver: L298N via pigpiod_if2
//
// Architecture (two threads):
//   Thread 1 — IMU: reads sensor at 100 Hz, runs Madgwick filter
//   Thread 2 — Control: PD yaw controller at 50 Hz, commands motors
//
// Requires pigpiod running:
//   sudo systemctl start pigpiod
//
// Compile:
//   g++ pd_control.cpp -o pd_control -lpigpiod_if2 -lpthread -std=c++17 -O2
//
// Run:
//   ./pd_control <target_yaw_degrees>
//   e.g.  ./pd_control 90   (turn 90° left)

#include <pigpiod_if2.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <mutex>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ── PD tuning parameters ─────────────────────────────────────────────────────
static constexpr double Kp          = 1.5;    // proportional gain
static constexpr double Kd          = 0.15;   // derivative gain (gyro damping)
static constexpr double OMEGA_MAX   = 3.0;    // max robot yaw rate (rad/s)
static constexpr double OMEGA_MIN   = 0.4;    // dead-zone: min effective yaw rate
static constexpr double EPS_PSI     = 0.052;  // stop if |error| < 3°
static constexpr double EPS_PSIDOT  = 0.08;   // stop if |yaw rate| < 0.08 rad/s

// ── Rover geometry ───────────────────────────────────────────────────────────
static constexpr double L           = 0.17;   // wheelbase (m)
static constexpr double r           = 0.03;   // wheel radius (m)
static constexpr double MAX_WHEEL_RADS = 1.6; // rad/s at 100% PWM (from encoder)

// ── Shared IMU state ─────────────────────────────────────────────────────────
struct Quat { double w{1}, x{0}, y{0}, z{0}; };

static std::mutex  imu_mutex;
static Quat        shared_quat;
static double      shared_gz_rads = 0.0;   // gyro z-rate in rad/s
static std::atomic<bool> running{true};

// ── I2CDevice ────────────────────────────────────────────────────────────────

class I2CDevice {
protected:
    int file;
public:
    I2CDevice(int bus, int address) {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-%d", bus);
        file = open(filename, O_RDWR);
        if (file < 0 || ioctl(file, I2C_SLAVE, address) < 0) {
            std::cerr << "I2C init failed for address 0x" << std::hex << address << std::dec << "\n";
            file = -1;
        }
    }
    virtual ~I2CDevice() { if (file >= 0) close(file); }

    void writeRegister(uint8_t reg, uint8_t value) {
        if (file < 0) return;
        uint8_t buf[2] = {reg, value};
        write(file, buf, 2);
    }

    int16_t readRegister16(uint8_t reg) {
        if (file < 0) return 0;
        if (write(file, &reg, 1) != 1) return 0;
        uint8_t buf[2];
        if (read(file, buf, 2) != 2) return 0;
        return (int16_t)(buf[0] | (buf[1] << 8));
    }
};

// ── MadgwickFilter ───────────────────────────────────────────────────────────

class MadgwickFilter {
    float q0{1}, q1{0}, q2{0}, q3{0};
    float beta{0.1f};
public:
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        gx *= M_PI / 180.0f;
        gy *= M_PI / 180.0f;
        gz *= M_PI / 180.0f;

        float norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm < 1e-6f) return;
        ax /= norm; ay /= norm; az /= norm;

        float _2q0=2*q0, _2q1=2*q1, _2q2=2*q2, _2q3=2*q3;
        float _4q0=4*q0, _4q1=4*q1, _4q2=4*q2;
        float _8q1=8*q1, _8q2=8*q2;
        float q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;

        float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        float s1 = _4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        float s2 = 4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        float s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay;

        norm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (norm > 1e-6f) { s0/=norm; s1/=norm; s2/=norm; s3/=norm; }

        float qDot1 = 0.5f*(-q1*gx - q2*gy - q3*gz) - beta*s0;
        float qDot2 = 0.5f*( q0*gx + q2*gz - q3*gy) - beta*s1;
        float qDot3 = 0.5f*( q0*gy - q1*gz + q3*gx) - beta*s2;
        float qDot4 = 0.5f*( q0*gz + q1*gy - q2*gx) - beta*s3;

        q0 += qDot1*dt; q1 += qDot2*dt;
        q2 += qDot3*dt; q3 += qDot4*dt;

        norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0/=norm; q1/=norm; q2/=norm; q3/=norm;
    }

    Quat getQuaternion() { return {q0, q1, q2, q3}; }
};

// ── IMUSensor ────────────────────────────────────────────────────────────────

class IMUSensor : public I2CDevice {
    float accelScale, gyroScale;
    MadgwickFilter filter;
public:
    IMUSensor(int bus, int address) : I2CDevice(bus, address) {
        accelScale = 0.061f / 1000.0f;
        gyroScale  = 8.75f  / 1000.0f;
    }

    void configure() {
        writeRegister(0x10, 0x40); // CTRL1_XL: 104 Hz, ±2g
        writeRegister(0x11, 0x40); // CTRL2_G:  104 Hz, ±245 dps
        std::cout << "LSM6DS33 configured\n";
    }

    // Returns {ax, ay, az, gx, gy, gz}
    std::vector<float> readRaw() {
        std::vector<float> d(6);
        d[0] = readRegister16(0x28) * accelScale;
        d[1] = readRegister16(0x2A) * accelScale;
        d[2] = readRegister16(0x2C) * accelScale;
        d[3] = readRegister16(0x22) * gyroScale;
        d[4] = readRegister16(0x24) * gyroScale;
        d[5] = readRegister16(0x26) * gyroScale;
        return d;
    }

    // Returns quaternion and stores gz (deg/s) separately
    Quat update(float dt, float& gz_out) {
        auto d = readRaw();
        gz_out = d[5];
        filter.update(d[3], d[4], d[5], d[0], d[1], d[2], dt);
        return filter.getQuaternion();
    }
};

// ── MotorL298N ───────────────────────────────────────────────────────────────

class MotorL298N {
    int pi_, ena_, in1_, in2_;
public:
    MotorL298N(int pi, int ena, int in1, int in2)
        : pi_(pi), ena_(ena), in1_(in1), in2_(in2) {}

    void setup() {
        set_mode(pi_, in1_, PI_OUTPUT);
        set_mode(pi_, in2_, PI_OUTPUT);
        gpio_write(pi_, in1_, 0);
        gpio_write(pi_, in2_, 0);
        set_PWM_frequency(pi_, ena_, 1000);
        set_PWM_range(pi_, ena_, 255);
        set_PWM_dutycycle(pi_, ena_, 0);
    }

    // speed: -1.0 … +1.0
    void setSpeed(double speed) {
        speed = std::clamp(speed, -1.0, 1.0);
        int pwm = static_cast<int>(std::abs(speed) * 255);
        if (speed > 0.01) {
            gpio_write(pi_, in1_, 1); gpio_write(pi_, in2_, 0);
        } else if (speed < -0.01) {
            gpio_write(pi_, in1_, 0); gpio_write(pi_, in2_, 1);
        } else {
            gpio_write(pi_, in1_, 0); gpio_write(pi_, in2_, 0);
            pwm = 0;
        }
        set_PWM_dutycycle(pi_, ena_, pwm);
    }

    void stop() { setSpeed(0.0); }
};

// ── Yaw from quaternion ───────────────────────────────────────────────────────

static double yawFromQuat(const Quat& q) {
    // ψ = atan2(2(wz + xy), 1 − 2(y² + z²))
    return std::atan2(2.0*(q.w*q.z + q.x*q.y),
                      1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

// ── IMU thread (100 Hz) ───────────────────────────────────────────────────────

static void imuThread() {
    IMUSensor imu(1, 0x6B);
    imu.configure();

    const float dt = 0.01f;
    while (running.load()) {
        float gz_degs = 0.0f;
        Quat q = imu.update(dt, gz_degs);

        {
            std::lock_guard<std::mutex> lk(imu_mutex);
            shared_quat    = q;
            shared_gz_rads = gz_degs * M_PI / 180.0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ── Control thread (50 Hz) ───────────────────────────────────────────────────

static void controlThread(MotorL298N& left, MotorL298N& right, double psi_ref) {
    std::cout << "Control thread started. Target yaw: "
              << psi_ref * 180.0 / M_PI << "°\n";

    while (running.load()) {
        // Read IMU state
        Quat   q;
        double psi_dot;
        {
            std::lock_guard<std::mutex> lk(imu_mutex);
            q       = shared_quat;
            psi_dot = shared_gz_rads;
        }

        double psi = yawFromQuat(q);

        // Wrapped yaw error
        double e_psi = std::atan2(std::sin(psi_ref - psi),
                                  std::cos(psi_ref - psi));

        // PD controller
        double omega_cmd = Kp * e_psi - Kd * psi_dot;

        // Stop condition
        if (std::abs(e_psi) < EPS_PSI && std::abs(psi_dot) < EPS_PSIDOT) {
            left.stop();
            right.stop();
            std::cout << "Target reached! yaw=" << psi * 180.0 / M_PI
                      << "°  error=" << e_psi * 180.0 / M_PI << "°\n";
            running.store(false);
            return;
        }

        // Clamp and apply dead zone
        omega_cmd = std::clamp(omega_cmd, -OMEGA_MAX, OMEGA_MAX);
        if (std::abs(e_psi) >= EPS_PSI) {
            omega_cmd = std::copysign(
                std::max(std::abs(omega_cmd), OMEGA_MIN), omega_cmd);
        }

        // Differential drive: in-place rotation
        // vR = (L/2)*omega_cmd,  vL = -(L/2)*omega_cmd
        double phi_R = (L / 2.0) * omega_cmd / r;   // wheel rad/s
        double phi_L = -(L / 2.0) * omega_cmd / r;

        double pwm_R = phi_R / MAX_WHEEL_RADS;
        double pwm_L = phi_L / MAX_WHEEL_RADS;

        right.setSpeed(pwm_R);
        left.setSpeed(pwm_L);

        // Log every ~0.5 s
        static int tick = 0;
        if (tick++ % 25 == 0) {
            std::cout << "yaw=" << psi * 180.0 / M_PI
                      << "°  err=" << e_psi * 180.0 / M_PI
                      << "°  ω=" << omega_cmd
                      << "  L=" << pwm_L << "  R=" << pwm_R << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    double target_deg = 90.0;
    if (argc >= 2) target_deg = std::stod(argv[1]);
    double psi_ref = target_deg * M_PI / 180.0;

    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cerr << "pigpiod connection failed\n"; return 1;
    }
    std::cout << "Connected to pigpiod OK\n";

    // Left:  ENA=GPIO12, IN1=GPIO23, IN2=GPIO24
    // Right: ENB=GPIO13, IN3=GPIO27, IN4=GPIO22
    MotorL298N left (pi, 12, 23, 24);
    MotorL298N right(pi, 13, 27, 22);
    left.setup();
    right.setup();

    // Signal handler for clean exit
    std::signal(SIGINT, [](int) { running.store(false); });

    std::thread t_imu(imuThread);
    // Let IMU stabilise for 1 s before starting control
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::thread t_ctrl(controlThread, std::ref(left), std::ref(right), psi_ref);

    t_ctrl.join();
    running.store(false);
    t_imu.join();

    left.stop();
    right.stop();
    pigpio_stop(pi);
    std::cout << "Done.\n";
    return 0;
}
