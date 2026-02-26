#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstdint> // Added this line to fix the uint8_t and int16_t errors

// ---------- 1. DATA STRUCTURE ----------

// Structure to hold our quaternion orientation [cite: 38]
struct Quat {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};
};

// ---------- 2. HARDWARE ABSTRACTION LAYER (HAL) ----------

// Base class encapsulating low-level I2C communication [cite: 20, 21]
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
            std::cerr << "Warning: Failed to initialize I2C bus. Running in simulation mode." << std::endl;
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

    // Read a 16-bit data word by combining readings from two 8-bit data registers (high and low) [cite: 35]
    int16_t readRegister16(uint8_t reg) {
        if (file < 0) return 0; 
        
        uint8_t buffer[2];
        // Point to the register
        if (write(file, &reg, 1) != 1) return 0;
        // Read 2 consecutive bytes
        if (read(file, buffer, 2) != 2) return 0;
        
        // Combine bytes (Assuming MSB first)
        return (int16_t)((buffer[0] << 8) | buffer[1]); 
    }
};

// ---------- 3. MADGWICK FILTER WRAPPER ----------

// OOP Wrapper for the downloaded Madgwick open-source C algorithm [cite: 37]
class MadgwickFilter {
private:
    float q0{1.0f}, q1{0.0f}, q2{0.0f}, q3{0.0f};

public:
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        // IMPORTANT: 
        // This is where you call the actual MadgwickAHRSupdateIMU() function 
        // from the C code you downloaded from http://www.x-io.co.uk/[cite: 37].
        // 
        // Example integration:
        // MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        // q0 = q0_madgwick; q1 = q1_madgwick; ...
        
        // Placeholder simulation for demonstration:
        q0 = std::cos(dt); q1 = std::sin(dt); q2 = 0.0f; q3 = 0.0f;
    }

    Quat getQuaternion() {
        return {q0, q1, q2, q3};
    }
};

// ---------- 4. IMU SENSOR IMPLEMENTATION ----------

// Derived class for the specific IMU wrapping the hardware [cite: 34, 50]
class IMUSensor : public I2CDevice {
private:
    float accelScale, gyroScale;
    MadgwickFilter filter;

public:
    IMUSensor(int bus, int address) : I2CDevice(bus, address) {
        // Define scaling factors based on maximum measurement values 
        // Example: MPU6050 set to +-2g and +-250 deg/s
        accelScale = 2.0f / 32768.0f;  
        gyroScale = 250.0f / 32768.0f; 
    }

    void configureIMU() {
        // Set configuration registers to wake up the sensor [cite: 34]
        // writeRegister8(0x6B, 0x00); 
    }

    // Convert the sensor raw measurements to physical values 
    std::vector<float> readPhysicalValues() {
        std::vector<float> data(6, 0.0f);
        
        // Note: Replace these hex addresses with your actual IMU's register addresses!
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
        
        // Interface the measurement data with the algorithm code [cite: 43]
        filter.update(phys[3], phys[4], phys[5], phys[0], phys[1], phys[2], dt);
        
        return filter.getQuaternion();
    }
};

// ---------- 5. MAIN EXECUTION ----------

int main() {
    // Initialize IMU on I2C bus 1
    IMUSensor myIMU(1, 0x68);
    myIMU.configureIMU();

    std::cout << "Starting IMU Read and Madgwick Fusion..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    float simulated_time = 0.0f;

    // Continuous loop to output estimations in the terminal window 
    while (true) {
        Quat q = myIMU.updateAndGetOrientation(simulated_time);
        simulated_time += 0.01f; // Simulated dt for the loop

        // Print the quaternion format to the terminal [cite: 30]
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Quaternion -> w: " << q.w 
                  << " | x: " << q.x 
                  << " | y: " << q.y 
                  << " | z: " << q.z << "\r" << std::flush;

        // Sleep for ~10ms to create a 100Hz read loop
        usleep(10000); 
    }

    return 0;
}