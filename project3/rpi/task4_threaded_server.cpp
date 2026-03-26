// Task 4 – Multithreaded RPi TCP server
// Three std::thread threads communicate via mutex-protected shared structs:
//
//   Thread 1  →  reads raw IMU data, converts to physical values
//   Thread 2  →  runs MadgwickFilter on Thread 1 data, produces quaternion
//   Thread 3  →  sends quaternion over TCP socket to Qt client at ~100 Hz
//
// Compile on RPi: g++ -O2 -std=c++17 -pthread -o task4_threaded_server task4_threaded_server.cpp -lm
// Run:            ./task4_threaded_server

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <vector>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cerrno>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TCP_PORT 8080

// ── Shared data structures ───────────────────────────────────────────────────

struct IMUData {
    float ax{0}, ay{0}, az{1};   // physical accel (g) – default: 1 g on z
    float gx{0}, gy{0}, gz{0};   // physical gyro  (dps)
    std::mutex mtx;
};

struct QuatData {
    double w{1}, x{0}, y{0}, z{0};
    std::mutex mtx;
};

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

    // q0..q3 exposed so Thread 2 can copy them
    float q0f() const { return q0; }
    float q1f() const { return q1; }
    float q2f() const { return q2; }
    float q3f() const { return q3; }
};

// ── IMU sensor (Thread-1 uses this) ─────────────────────────────────────────
class IMUSensor : public I2CDevice {
    float accelScale{0.061f / 1000.0f};
    float gyroScale {8.75f  / 1000.0f};
public:
    IMUSensor(int bus, int addr) : I2CDevice(bus, addr) {}

    void configureIMU() {
        writeRegister(0x10, 0x40);
        writeRegister(0x11, 0x40);
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
};

// ── TCP server helper ─────────────────────────────────────────────────────────
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

// ── Thread functions ──────────────────────────────────────────────────────────

// Thread 1: read IMU → store in imuData
void thread1_readIMU(IMUSensor& sensor, IMUData& imuData,
                     std::atomic<bool>& running) {
    std::cout << "[T1] IMU reader started\n";
    while (running) {
        auto d = sensor.readPhysicalValues();
        {
            std::lock_guard<std::mutex> lock(imuData.mtx);
            imuData.ax = d[0]; imuData.ay = d[1]; imuData.az = d[2];
            imuData.gx = d[3]; imuData.gy = d[4]; imuData.gz = d[5];
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[T1] IMU reader stopped\n";
}

// Thread 2: run Madgwick filter → store normalized quaternion in quatData
void thread2_madgwick(IMUData& imuData, QuatData& quatData,
                      std::atomic<bool>& running) {
    MadgwickFilter filter;
    const float dt = 0.01f;
    std::cout << "[T2] Madgwick filter started\n";
    while (running) {
        float ax, ay, az, gx, gy, gz;
        {
            std::lock_guard<std::mutex> lock(imuData.mtx);
            ax = imuData.ax; ay = imuData.ay; az = imuData.az;
            gx = imuData.gx; gy = imuData.gy; gz = imuData.gz;
        }

        filter.update(gx, gy, gz, ax, ay, az, dt);

        // Normalize quaternion before storing
        double w = filter.q0f(), x = filter.q1f(),
               y = filter.q2f(), z = filter.q3f();
        double n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n > 1e-9) { w/=n; x/=n; y/=n; z/=n; }

        {
            std::lock_guard<std::mutex> lock(quatData.mtx);
            quatData.w = w; quatData.x = x;
            quatData.y = y; quatData.z = z;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[T2] Madgwick filter stopped\n";
}

// Thread 3: read quaternion and send over TCP socket at ~100 Hz
void thread3_tcpSend(int clientfd, QuatData& quatData,
                     std::atomic<bool>& running) {
    std::cout << "[T3] TCP sender started\n";
    int frame = 0;
    while (running) {
        double w, x, y, z;
        {
            std::lock_guard<std::mutex> lock(quatData.mtx);
            w = quatData.w; x = quatData.x;
            y = quatData.y; z = quatData.z;
        }

        if (frame++ % 100 == 0) {
            std::cout << "[T3] q: [" << w << ", " << x
                      << ", " << y << ", " << z << "]\n";
        }

        double packet[4] = {w, x, y, z};
        ssize_t sent = send(clientfd, packet, sizeof(packet), MSG_NOSIGNAL);
        if (sent <= 0) {
            std::cout << "[T3] Client disconnected\n";
            running = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[T3] TCP sender stopped\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    IMUSensor sensor(1, 0x6B);
    sensor.configureIMU();

    int serverfd = createServerSocket();
    if (serverfd < 0) return 1;
    std::cout << "Task 4 threaded TCP server on port " << TCP_PORT << " ...\n";

    while (true) {
        struct sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        int clientfd = accept(serverfd, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientfd < 0) { perror("accept"); continue; }

        char ipStr[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &clientAddr.sin_addr, ipStr, sizeof(ipStr));
        std::cout << "Client connected: " << ipStr << "\n";

        // Shared data between threads
        IMUData  imuData;
        QuatData quatData;
        std::atomic<bool> running{true};

        // Start the three threads
        std::thread t1(thread1_readIMU,  std::ref(sensor),  std::ref(imuData),
                       std::ref(running));
        std::thread t2(thread2_madgwick, std::ref(imuData), std::ref(quatData),
                       std::ref(running));
        std::thread t3(thread3_tcpSend,  clientfd,          std::ref(quatData),
                       std::ref(running));

        // Wait for Thread 3 to signal disconnect (running = false)
        t3.join();
        running = false;
        t1.join();
        t2.join();

        close(clientfd);
        std::cout << "Ready for next client...\n";
    }

    close(serverfd);
    return 0;
}
