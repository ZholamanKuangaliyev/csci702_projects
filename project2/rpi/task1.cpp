#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstdint>

// ---------- 1. DATA STRUCTURE ----------

struct Quat {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};
};

// ---------- 2. HARDWARE ABSTRACTION LAYER (HAL) ----------

class I2CDevice {
protected:
    int file;
    int bus;
    int deviceAddress;

public:
    I2CDevice(int bus, int address) : bus(bus), deviceAddress(address) {
        char filename[20];
        snprintf(filename, 19, "/dev/i2c-%d", bus);
        file = open(filename, O_RDWR);
        if (file < 0 || ioctl(file, I2C_SLAVE, deviceAddress) < 0) {
            std::cerr << "Hardware Error: Failed to initialize I2C bus or find device at address 0x" 
                      << std::hex << address << std::endl;
            file = -1; 
        }
    }

    virtual ~I2CDevice() {
        if (file >= 0) close(file);
    }

    void writeRegister8(uint8_t reg, uint8_t value) {
        if (file < 0) return;
        uint8_t buffer[2] = {reg, value};
        write(file, buffer, 2);
    }

    int16_t readRegister16(uint8_t reg) {
        if (file < 0) return 0; 
        uint8_t buffer[2];
        if (write(file, &reg, 1) != 1) return 0;
        if (read(file, buffer, 2) != 2) return 0;
        return (int16_t)((buffer[0] << 8) | buffer[1]); 
    }
};

// ---------- 3. REAL MADGWICK FILTER ----------

class MadgwickFilter {
private:
    float beta{0.1f}; // 2 * proportional gain
    float q0{1.0f}, q1{0.0f}, q2{0.0f}, q3{0.0f};

public:
    // Implementation of Sebastian Madgwick's 6DOF AHRS algorithm
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Convert gyroscope degrees/sec to radians/sec
        gx *= M_PI / 180.0f; gy *= M_PI / 180.0f; gz *= M_PI / 180.0f;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalize accelerometer measurement
            recipNorm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
            q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            
            recipNorm = 1.0f / std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
            s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;

        // Normalize quaternion
        recipNorm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    }

    Quat getQuaternion() { return {q0, q1, q2, q3}; }
};

// ---------- 4. IMU SENSOR IMPLEMENTATION ----------

class IMUSensor : public I2CDevice {
private:
    float accelScale, gyroScale;
    MadgwickFilter filter;

public:
    IMUSensor(int bus, int address) : I2CDevice(bus, address) {
        accelScale = 2.0f / 32768.0f;  
        gyroScale = 250.0f / 32768.0f; 
    }

    void configureIMU() {
        // WAKE UP THE SENSOR! 
        // 0x6B is the Power Management 1 register for MPU6050. Writing 0x00 wakes it up.
        writeRegister8(0x6B, 0x00); 
    }

    std::vector<float> readPhysicalValues() {
        std::vector<float> data(6, 0.0f);
        
        data[0] = readRegister16(0x3B) * accelScale; // Accel X
        data[1] = readRegister16(0x3D) * accelScale; // Accel Y
        data[2] = readRegister16(0x3F) * accelScale; // Accel Z
        
        data[3] = readRegister16(0x43) * gyroScale;  // Gyro X
        data[4] = readRegister16(0x45) * gyroScale;  // Gyro Y
        data[5] = readRegister16(0x47) * gyroScale;  // Gyro Z
        
        return data;
    }

    Quat updateAndGetOrientation(float dt) {
        std::vector<float> phys = readPhysicalValues();
        filter.update(phys[3], phys[4], phys[5], phys[0], phys[1], phys[2], dt);
        return filter.getQuaternion();
    }
};

// ---------- 5. MAIN EXECUTION ----------

int main() {
    // Assuming standard MPU6050 on Bus 1, Address 0x68
    IMUSensor myIMU(1, 0x68);
    myIMU.configureIMU();

    std::cout << "Starting Hardware IMU Read and Real Madgwick Fusion..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;

    while (true) {
        // We use dt = 0.01f because we sleep for 10ms (100Hz)
        Quat q = myIMU.updateAndGetOrientation(0.01f);

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Quaternion -> w: " << q.w 
                  << " | x: " << q.x 
                  << " | y: " << q.y 
                  << " | z: " << q.z << "\r" << std::flush;

        usleep(10000); 
    }

    return 0;
}