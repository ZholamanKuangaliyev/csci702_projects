// Task 3 – RPi TCP server: reuses I2CDevice + IMUSensor + MadgwickFilter
// from Project 2 and replaces the UDP sender with a TCP server socket.
// The Qt client (qt_imu_client) connects to this server on port 8080.
//
// Compile on RPi: g++ -O2 -o task3_server task3_server.cpp -lm
// Run:            ./task3_server

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <cmath>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <cerrno>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TCP_PORT 8080

struct Quat { double w{1}, x{0}, y{0}, z{0}; };

// ── I2C device ───────────────────────────────────────────────────────────────
class I2CDevice {
protected:
    int file;
public:
    I2CDevice(int bus, int address) {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-%d", bus);
        file = open(filename, O_RDWR);
        if (file < 0 || ioctl(file, I2C_SLAVE, address) < 0) {
            std::cerr << "Warning: Failed to open I2C device.\n";
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

// ── Madgwick filter ──────────────────────────────────────────────────────────
class MadgwickFilter {
    float q0{1}, q1{0}, q2{0}, q3{0};
    float beta{0.1f};
public:
    void update(float gx, float gy, float gz,
                float ax, float ay, float az, float dt) {
        gx *= M_PI/180.0f; gy *= M_PI/180.0f; gz *= M_PI/180.0f;

        float norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm < 1e-6f) return;
        ax/=norm; ay/=norm; az/=norm;

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

        q0+=qd0*dt; q1+=qd1*dt; q2+=qd2*dt; q3+=qd3*dt;

        norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0/=norm; q1/=norm; q2/=norm; q3/=norm;
    }

    Quat getQuaternion() const { return {q0, q1, q2, q3}; }
};

// ── IMU sensor ───────────────────────────────────────────────────────────────
class IMUSensor : public I2CDevice {
    float accelScale{0.061f / 1000.0f};
    float gyroScale {8.75f  / 1000.0f};
    MadgwickFilter filter;
public:
    IMUSensor(int bus, int addr) : I2CDevice(bus, addr) {}

    void configureIMU() {
        writeRegister(0x10, 0x40); // CTRL1_XL: 104 Hz, ±2 g
        writeRegister(0x11, 0x40); // CTRL2_G:  104 Hz, ±245 dps
        std::cout << "LSM6DS33 configured\n";
    }

    std::vector<float> readPhysicalValues() {
        std::vector<float> d(6, 0.0f);
        d[0] = readRegister16(0x28) * accelScale;
        d[1] = readRegister16(0x2A) * accelScale;
        d[2] = readRegister16(0x2C) * accelScale;
        d[3] = readRegister16(0x22) * gyroScale;
        d[4] = readRegister16(0x24) * gyroScale;
        d[5] = readRegister16(0x26) * gyroScale;
        return d;
    }

    Quat updateAndGetOrientation(float dt) {
        auto d = readPhysicalValues();
        filter.update(d[3], d[4], d[5], d[0], d[1], d[2], dt);
        Quat q = filter.getQuaternion();
        // Normalize output quaternion before sending
        double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        if (n > 1e-9) { q.w/=n; q.x/=n; q.y/=n; q.z/=n; }
        return q;
    }
};

// ── TCP server helpers ───────────────────────────────────────────────────────
// Returns server fd bound and listening on TCP_PORT, or -1 on error.
static int createServerSocket() {
    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) { perror("socket"); return -1; }

    int opt = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(TCP_PORT);

    if (bind(sfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(sfd); return -1;
    }
    if (listen(sfd, 1) < 0) {
        perror("listen"); close(sfd); return -1;
    }
    return sfd;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    IMUSensor imu(1, 0x6B);
    imu.configureIMU();

    int serverfd = createServerSocket();
    if (serverfd < 0) return 1;
    std::cout << "Task 3 TCP server listening on port " << TCP_PORT << " ...\n";

    while (true) {
        struct sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        int clientfd = accept(serverfd, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientfd < 0) { perror("accept"); continue; }

        char ipStr[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &clientAddr.sin_addr, ipStr, sizeof(ipStr));
        std::cout << "Client connected: " << ipStr << "\n";

        const float dt = 0.01f;
        int frame = 0;
        while (true) {
            Quat q = imu.updateAndGetOrientation(dt);

            if (frame++ % 100 == 0) {
                std::cout << "q: [" << q.w << ", " << q.x
                          << ", " << q.y << ", " << q.z << "]\n";
            }

            double packet[4] = {q.w, q.x, q.y, q.z};
            ssize_t sent = send(clientfd, packet, sizeof(packet), MSG_NOSIGNAL);
            if (sent <= 0) {
                std::cout << "Client disconnected.\n";
                break;
            }

            usleep(10000); // ~100 Hz
        }
        close(clientfd);
    }

    close(serverfd);
    return 0;
}
