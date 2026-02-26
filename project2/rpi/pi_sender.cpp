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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PORT 8080
#define PC_IP "172.20.10.6" 

struct Quat { double w{1}, x{0}, y{0}, z{0}; };

class I2CDevice {
protected:
    int file, bus, deviceAddress;
public:
    I2CDevice(int bus, int address) : bus(bus), deviceAddress(address) {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-%d", bus);
        file = open(filename, O_RDWR);
        if (file < 0 || ioctl(file, I2C_SLAVE, deviceAddress) < 0) {
            std::cerr << "Warning: Failed to initialize I2C bus or device.\n";
            file = -1;
        }
    }
    virtual ~I2CDevice() { if (file >= 0) close(file); }

    void writeRegister(uint8_t reg, uint8_t value) {
        if (file < 0) return;
        uint8_t buffer[2] = {reg, value};
        write(file, buffer, 2);
    }

    int16_t readRegister16(uint8_t reg) {
        if (file < 0) return 0;
        // Write register address
        if (write(file, &reg, 1) != 1) return 0;
        // Read 2 bytes (LSM6DS33 is little-endian: low byte first)
        uint8_t buffer[2];
        if (read(file, buffer, 2) != 2) return 0;
        return (int16_t)(buffer[0] | (buffer[1] << 8));
    }
};

class MadgwickFilter {
private:
    float q0{1.0f}, q1{0.0f}, q2{0.0f}, q3{0.0f};
    float beta{0.1f}; // Filter gain (tune this: 0.033 - 0.5)

public:
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        // Convert gyro from deg/s to rad/s
        gx *= M_PI / 180.0f;
        gy *= M_PI / 180.0f;
        gz *= M_PI / 180.0f;

        // Normalize accelerometer measurement
        float norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm < 1e-6f) return; // Avoid division by zero
        ax /= norm;
        ay /= norm;
        az /= norm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        // Gradient descent algorithm corrective step
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        // Normalize gradient
        norm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (norm > 1e-6f) {
            s0 /= norm;
            s1 /= norm;
            s2 /= norm;
            s3 /= norm;
        }

        // Compute rate of change of quaternion from gyroscope
        float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Apply feedback step (gradient descent correction)
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;

        // Integrate to yield quaternion
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        // Normalize quaternion
        norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }

    Quat getQuaternion() { return {q0, q1, q2, q3}; }
};

class IMUSensor : public I2CDevice {
private:
    float accelScale, gyroScale;
    MadgwickFilter filter;
public:
    // LSM6DS33 sensor address: 0x6B
    IMUSensor(int bus, int address) : I2CDevice(bus, address) {
        // LSM6DS33 scales: ±2g for accel, ±245 dps for gyro
        accelScale = 0.061f / 1000.0f; // mg to g
        gyroScale = 8.75f / 1000.0f;    // mdps to dps
    }

    void configureIMU() {
        // CTRL1_XL (0x10): 0x40 = 104 Hz, ±2g
        writeRegister(0x10, 0x40);
        // CTRL2_G (0x11): 0x40 = 104 Hz, ±245 dps
        writeRegister(0x11, 0x40);
        std::cout << "LSM6DS33 configured\n";
    }

    std::vector<float> readPhysicalValues() {
        std::vector<float> data(6, 0.0f);
        // LSM6DS33 register addresses
        // Accel: OUTX_L_XL=0x28, OUTY_L_XL=0x2A, OUTZ_L_XL=0x2C
        // Gyro:  OUTX_L_G=0x22,  OUTY_L_G=0x24,  OUTZ_L_G=0x26
        data[0] = readRegister16(0x28) * accelScale; // ax
        data[1] = readRegister16(0x2A) * accelScale; // ay
        data[2] = readRegister16(0x2C) * accelScale; // az
        data[3] = readRegister16(0x22) * gyroScale;  // gx
        data[4] = readRegister16(0x24) * gyroScale;  // gy
        data[5] = readRegister16(0x26) * gyroScale;  // gz

        static int debug_count = 0;
        if (debug_count++ % 100 == 0) {
            std::cout << "Sensor: ax=" << data[0] << " ay=" << data[1] << " az=" << data[2]
                      << " gx=" << data[3] << " gy=" << data[4] << " gz=" << data[5] << "\n";
        }

        return data;
    }

    Quat updateAndGetOrientation(float dt) {
        std::vector<float> phys = readPhysicalValues();
        filter.update(phys[3], phys[4], phys[5], phys[0], phys[1], phys[2], dt);
        return filter.getQuaternion();
    }
};

int main() {
    // LSM6DS33 address is 0x6B
    IMUSensor myIMU(1, 0x6B);
    myIMU.configureIMU();

    int sockfd;
    struct sockaddr_in servaddr;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed\n";
        return -1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr(PC_IP);

    std::cout << "Streaming IMU data to " << PC_IP << ":" << PORT << "...\n";

    const float dt = 0.01f; // 10ms = 100Hz
    int frame = 0;
    while (true) {
        Quat q = myIMU.updateAndGetOrientation(dt);

        // Debug: print quaternion every 100 frames (~1 second)
        if (frame++ % 100 == 0) {
            std::cout << "q: [" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";
        }

        double packet[4] = {q.w, q.x, q.y, q.z};
        sendto(sockfd, (const char *)packet, sizeof(packet), MSG_CONFIRM, (const struct sockaddr *)&servaddr, sizeof(servaddr));
        usleep(10000); // ~100Hz
    }
    close(sockfd);
    return 0;
}