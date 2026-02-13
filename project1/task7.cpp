#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define LSM6_ADDR 0x6B
#define LIS3_ADDR 0x1E

#define CTRL1_XL 0x10
#define CTRL_REG1_M 0x20
#define CTRL_REG3_M 0x22

int open_i2c() {
  int file = open("/dev/i2c-1", O_RDWR);
  if (file < 0) {
    std::cerr << "Failed to open the I2C bus." << std::endl;
    exit(1);
  }
  return file;
}

void write_reg(int file, uint8_t addr, uint8_t reg, uint8_t value) {
  ioctl(file, I2C_SLAVE, addr);
  uint8_t buf[2] = {reg, value};
  write(file, buf, 2);
}

int16_t read_sensor_axis(int file, uint8_t addr, uint8_t reg_low) {
  ioctl(file, I2C_SLAVE, addr);
  uint8_t reg = reg_low;
  write(file, &reg, 1);

  uint8_t data[2];
  if (read(file, data, 2) != 2)
    return 0;

  return (int16_t)(data[0] | (data[1] << 8));
}

int main() {
  int file = open_i2c();

  write_reg(file, LSM6_ADDR, CTRL1_XL, 0x40);
  write_reg(file, LIS3_ADDR, CTRL_REG1_M, 0x70);
  write_reg(file, LIS3_ADDR, CTRL_REG3_M, 0x00);

  std::cout << "Starting Task 7: Raw IMU Data Stream" << std::endl;

  while (true) {
    int16_t ax = read_sensor_axis(file, LSM6_ADDR, 0x28);
    int16_t ay = read_sensor_axis(file, LSM6_ADDR, 0x2A);
    int16_t az = read_sensor_axis(file, LSM6_ADDR, 0x2C);

    std::cout << "\rAccel X: " << std::setw(6) << ax << " Y: " << std::setw(6)
              << ay << " Z: " << std::setw(6) << az << std::flush;

    usleep(100000);
  }

  close(file);
  return 0;
}