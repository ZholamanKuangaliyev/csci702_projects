#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>


// Device Addresses
#define LSM6_ADDR 0x6B
#define LIS3_ADDR 0x1E

// Registers
#define CTRL1_XL 0x10
#define CTRL_REG1_M 0x20
#define CTRL_REG3_M 0x22
#define ACCEL_X_L 0x28
#define MAG_X_L 0x28

int open_i2c() {
  int file = open("/dev/i2c-1", O_RDWR);
  if (file < 0) {
    std::cerr << "Failed to open the I2C bus. Check if I2C is enabled."
              << std::endl;
    exit(1);
  }
  return file;
}

void write_reg(int file, uint8_t addr, uint8_t reg, uint8_t value) {
  ioctl(file, I2C_SLAVE, addr);
  uint8_t buf[2] = {reg, value};
  if (write(file, buf, 2) != 2) {
    std::cerr << "Failed to write to register " << (int)reg << std::endl;
  }
}

int16_t read_sensor_axis(int file, uint8_t addr, uint8_t reg_low) {
  ioctl(file, I2C_SLAVE, addr);
  uint8_t reg = reg_low;
  write(file, &reg, 1);

  uint8_t data[2];
  if (read(file, data, 2) != 2)
    return 0;

  // Combine: High byte is shifted left by 8, then ORed with Low byte
  return (int16_t)(data[0] | (data[1] << 8));
}

int main() {
  int file = open_i2c();

  // Phase 2 Task 6: Wake up sensors using Control Words
  write_reg(file, LSM6_ADDR, CTRL1_XL, 0x40);    // Accel: 104Hz
  write_reg(file, LIS3_ADDR, CTRL_REG1_M, 0x70); // Mag: Ultra-high performance
  write_reg(file, LIS3_ADDR, CTRL_REG3_M, 0x00); // Mag: Continuous mode

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Starting Task 7: Raw IMU Data Stream\n";
  std::cout << "-------------------------------------------------------\n";
  std::cout << "ACCEL (X, Y, Z) | MAG (X, Y, Z)\n";

  while (true) {
    // Read Accelerometer
    int16_t ax = read_sensor_axis(file, LSM6_ADDR, 0x28);
    int16_t ay = read_sensor_axis(file, LSM6_ADDR, 0x2A);
    int16_t az = read_sensor_axis(file, LSM6_ADDR, 0x2C);

    // Read Magnetometer
    int16_t mx = read_sensor_axis(file, LIS3_ADDR, 0x28);
    int16_t my = read_sensor_axis(file, LIS3_ADDR, 0x2A);
    int16_t mz = read_sensor_axis(file, LIS3_ADDR, 0x2C);

    std::cout << "\rA: " << std::setw(6) << ax << " " << std::setw(6) << ay
              << " " << std::setw(6) << az << " | M: " << std::setw(6) << mx
              << " " << std::setw(6) << my << " " << std::setw(6) << mz
              << std::flush;

    usleep(100000); // 10Hz Refresh
  }

  close(file);
  return 0;
}