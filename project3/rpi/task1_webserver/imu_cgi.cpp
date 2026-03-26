// Task 1 – CGI script: reads LSM6DS33 via I2C, runs Madgwick filter,
// outputs an HTML page showing the live quaternion [w, x, y, z].
// Compile: g++ -O2 -o imu_cgi imu_cgi.cpp -lm
// Deploy:  sudo cp imu_cgi /usr/lib/cgi-bin/imu_cgi

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Quat { double w{1}, x{0}, y{0}, z{0}; };

// ── I2C device ──────────────────────────────────────────────────────────────
class I2CDevice {
protected:
    int file;
public:
    I2CDevice(int bus, int address) {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-%d", bus);
        file = open(filename, O_RDWR);
        if (file < 0 || ioctl(file, I2C_SLAVE, address) < 0) {
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

    bool ok() const { return file >= 0; }
};

// ── Madgwick filter ─────────────────────────────────────────────────────────
class MadgwickFilter {
    float q0{1}, q1{0}, q2{0}, q3{0};
    float beta{0.1f};
public:
    void update(float gx, float gy, float gz,
                float ax, float ay, float az, float dt) {
        gx *= M_PI / 180.0f; gy *= M_PI / 180.0f; gz *= M_PI / 180.0f;

        float norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm < 1e-6f) return;
        ax /= norm; ay /= norm; az /= norm;

        float _2q0=2*q0, _2q1=2*q1, _2q2=2*q2, _2q3=2*q3;
        float _4q0=4*q0, _4q1=4*q1, _4q2=4*q2;
        float _8q1=8*q1, _8q2=8*q2;
        float q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;

        float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        float s1 = _4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay
                   - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        float s2 = 4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay
                   - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        float s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay;

        norm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (norm > 1e-6f) { s0/=norm; s1/=norm; s2/=norm; s3/=norm; }

        float qd0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - beta*s0;
        float qd1 = 0.5f*( q0*gx + q2*gz - q3*gy) - beta*s1;
        float qd2 = 0.5f*( q0*gy - q1*gz + q3*gx) - beta*s2;
        float qd3 = 0.5f*( q0*gz + q1*gy - q2*gx) - beta*s3;

        q0 += qd0*dt; q1 += qd1*dt; q2 += qd2*dt; q3 += qd3*dt;

        norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0/=norm; q1/=norm; q2/=norm; q3/=norm;
    }

    Quat getQuaternion() const { return {q0, q1, q2, q3}; }
};

// ── IMU sensor ──────────────────────────────────────────────────────────────
class IMUSensor : public I2CDevice {
    float accelScale{0.061f / 1000.0f};  // mg → g
    float gyroScale {8.75f  / 1000.0f};  // mdps → dps
    MadgwickFilter filter;
public:
    IMUSensor(int bus, int addr) : I2CDevice(bus, addr) {}

    void configureIMU() {
        writeRegister(0x10, 0x40); // CTRL1_XL: 104 Hz, ±2 g
        writeRegister(0x11, 0x40); // CTRL2_G : 104 Hz, ±245 dps
    }

    std::vector<float> readPhysicalValues() {
        std::vector<float> d(6, 0.0f);
        d[0] = readRegister16(0x28) * accelScale; // ax
        d[1] = readRegister16(0x2A) * accelScale; // ay
        d[2] = readRegister16(0x2C) * accelScale; // az
        d[3] = readRegister16(0x22) * gyroScale;  // gx
        d[4] = readRegister16(0x24) * gyroScale;  // gy
        d[5] = readRegister16(0x26) * gyroScale;  // gz
        return d;
    }

    Quat updateAndGetOrientation(float dt) {
        auto d = readPhysicalValues();
        filter.update(d[3], d[4], d[5], d[0], d[1], d[2], dt);
        return filter.getQuaternion();
    }
};

// ── HTML output helper ───────────────────────────────────────────────────────
static std::string fmt(double v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << v;
    return ss.str();
}

// ── Main (CGI entry point) ───────────────────────────────────────────────────
int main() {
    // CGI mandatory header
    std::cout << "Content-Type: text/html; charset=UTF-8\r\n\r\n";

    IMUSensor imu(1, 0x6B);
    bool sensor_ok = imu.ok();

    Quat q{1, 0, 0, 0};

    if (sensor_ok) {
        imu.configureIMU();
        // Warm up the Madgwick filter with ~50 samples (~500 ms)
        const float dt = 0.01f;
        for (int i = 0; i < 50; ++i) {
            q = imu.updateAndGetOrientation(dt);
            usleep(10000); // 10 ms
        }
        // Normalize output quaternion before display
        double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        if (n > 1e-9) { q.w/=n; q.x/=n; q.y/=n; q.z/=n; }
    }

    std::cout << R"(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta http-equiv="refresh" content="1">
  <title>RPi IMU – Live Quaternion</title>
  <style>
    body { font-family: Arial, sans-serif; background:#1a1a2e; color:#eee;
           display:flex; flex-direction:column; align-items:center;
           justify-content:center; min-height:100vh; margin:0; }
    h1   { color:#00d4ff; }
    .card{ background:#16213e; border-radius:12px; padding:2rem 3rem;
           box-shadow:0 4px 20px rgba(0,0,0,0.5); text-align:center; }
    table{ border-collapse:collapse; margin:1rem auto; font-size:1.2rem; }
    th   { color:#00d4ff; padding:0.4rem 1.5rem; border-bottom:1px solid #333; }
    td   { padding:0.4rem 1.5rem; font-family:monospace; font-size:1.1rem; }
    .ok  { color:#00ff88; } .err { color:#ff4444; }
    .note{ font-size:0.85rem; color:#888; margin-top:1rem; }
  </style>
</head>
<body>
  <div class="card">
    <h1>IMU Orientation – Madgwick Quaternion</h1>
)";

    if (sensor_ok) {
        std::cout << "    <p class=\"ok\">&#10003; LSM6DS33 connected on I2C bus 1 (0x6B)</p>\n";
    } else {
        std::cout << "    <p class=\"err\">&#10007; IMU sensor not accessible (check I2C / permissions)</p>\n";
    }

    std::cout << "    <table>\n"
              << "      <tr><th>Component</th><th>Value</th></tr>\n"
              << "      <tr><td>w</td><td>" << fmt(q.w) << "</td></tr>\n"
              << "      <tr><td>x</td><td>" << fmt(q.x) << "</td></tr>\n"
              << "      <tr><td>y</td><td>" << fmt(q.y) << "</td></tr>\n"
              << "      <tr><td>z</td><td>" << fmt(q.z) << "</td></tr>\n"
              << "    </table>\n"
              << "    <p class=\"note\">Page auto-refreshes every second. "
                 "50 Madgwick iterations at 100 Hz were run per request.</p>\n"
              << "  </div>\n</body>\n</html>\n";

    return 0;
}
