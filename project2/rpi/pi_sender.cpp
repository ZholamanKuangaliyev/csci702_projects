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

    int16_t readRegister16(uint8_t reg) {
        if (file < 0) return 0; 
        uint8_t buffer[2];
        if (write(file, &reg, 1) != 1) return 0;
        if (read(file, buffer, 2) != 2) return 0;
        return (int16_t)((buffer[0] << 8) | buffer[1]); 
    }
};

class MadgwickFilter {
private:
    float q0{1.0f}, q1{0.0f}, q2{0.0f}, q3{0.0f};
public:
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        // Actual Madgwick math goes here. For now, we simulate a spinning rotation.
        q0 = std::cos(dt); q1 = std::sin(dt); q2 = 0; q3 = 0;
    }
    Quat getQuaternion() { return {q0, q1, q2, q3}; }
};

class IMUSensor : public I2CDevice {
private:
    float accelScale, gyroScale;
    MadgwickFilter filter;
public:
    IMUSensor(int bus, int address) : I2CDevice(bus, address) {
        accelScale = 2.0f / 32768.0f; gyroScale = 250.0f / 32768.0f;
    }
    void configureIMU() { /* Write wake-up registers */ }
    
    std::vector<float> readPhysicalValues() {
        std::vector<float> data(6, 0.0f);
        data[0] = readRegister16(0x3B) * accelScale; data[1] = readRegister16(0x3D) * accelScale;
        data[2] = readRegister16(0x3F) * accelScale; data[3] = readRegister16(0x43) * gyroScale;
        data[4] = readRegister16(0x45) * gyroScale;  data[5] = readRegister16(0x47) * gyroScale;
        return data;
    }

    Quat updateAndGetOrientation(float dt) {
        std::vector<float> phys = readPhysicalValues();
        filter.update(phys[3], phys[4], phys[5], phys[0], phys[1], phys[2], dt);
        return filter.getQuaternion();
    }
};

int main() {
    IMUSensor myIMU(1, 0x68);
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

    float simulated_time = 0.0f;
    while (true) {
        Quat q = myIMU.updateAndGetOrientation(simulated_time);
        simulated_time += 0.01f;

        double packet[4] = {q.w, q.x, q.y, q.z};
        sendto(sockfd, (const char *)packet, sizeof(packet), MSG_CONFIRM, (const struct sockaddr *)&servaddr, sizeof(servaddr));
        usleep(10000); // ~100Hz
    }
    close(sockfd);
    return 0;
}