#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define LSM6DS33_ADDR 0x6B
#define LIS3MDL_ADDR 0x1E

#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL_REG1_M 0x20
#define CTRL_REG3_M 0x22
#define OUTX_L_XL 0x28

int main() {
  int file;
  const char *bus = "/dev/i2c-1";

  if ((file = open(bus, O_RDWR)) < 0) {
    return 1;
  }

  ioctl(file, I2C_SLAVE, LSM6DS33_ADDR);
  unsigned char config_xl[] = {CTRL1_XL, 0x40};
  write(file, config_xl, 2);
  unsigned char config_g[] = {CTRL2_G, 0x40};
  write(file, config_g, 2);

  ioctl(file, I2C_SLAVE, LIS3MDL_ADDR);
  unsigned char config_m1[] = {CTRL_REG1_M, 0x70};
  write(file, config_m1, 2);
  unsigned char config_m3[] = {CTRL_REG3_M, 0x00};
  write(file, config_m3, 2);

  ioctl(file, I2C_SLAVE, LSM6DS33_ADDR);
  unsigned char reg = OUTX_L_XL;
  write(file, &reg, 1);

  unsigned char data[2];
  read(file, data, 2);

  int16_t x_accel = (int16_t)(data[0] | (data[1] << 8));
  std::cout << "Accel X: " << x_accel << std::endl;

  close(file);
  return 0;
}