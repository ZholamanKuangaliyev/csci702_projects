#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>


// Device Addresses
#define LSM6DS33_ADDR 0x6B
#define LIS3MDL_ADDR 0x1E

// Register Addresses [cite: 3075, 4983]
#define CTRL1_XL 0x10    // Accel control
#define CTRL2_G 0x11     // Gyro control
#define CTRL_REG1_M 0x20 // Mag control
#define CTRL_REG3_M 0x22 // Mag mode
#define OUTX_L_XL 0x28   // Accel X low byte

int main() {
  int file;
  const char *bus = "/dev/i2c-1";

  if ((file = open(bus, O_RDWR)) < 0) {
    return 1;
  }

  // --- INITIALIZE LSM6DS33 (Accel & Gyro) ---
  ioctl(file, I2C_SLAVE, LSM6DS33_ADDR);
  // Enable Accel: 104Hz, +/- 2g [cite: 5151, 5153]
  unsigned char config_xl[] = {CTRL1_XL, 0x40};
  write(file, config_xl, 2);
  // Enable Gyro: 104Hz, 245 dps [cite: 5173, 5175]
  unsigned char config_g[] = {CTRL2_G, 0x40};
  write(file, config_g, 2);

  // --- INITIALIZE LIS3MDL (Magnetometer) ---
  ioctl(file, I2C_SLAVE, LIS3MDL_ADDR);
  // Set Mag to Ultra-high-performance, 10Hz [cite: 3085, 3091]
  unsigned char config_m1[] = {CTRL_REG1_M, 0x70};
  write(file, config_m1, 2);
  // Continuous-conversion mode [cite: 3101, 3103]
  unsigned char config_m3[] = {CTRL_REG3_M, 0x00};
  write(file, config_m3, 2);

  // --- READ ACCELEROMETER X-AXIS ---
  ioctl(file, I2C_SLAVE, LSM6DS33_ADDR);
  unsigned char reg = OUTX_L_XL;
  write(file, &reg, 1);

  unsigned char data[2];
  read(file, data, 2); // Read L and H bytes [cite: 4018]

  // Combine bytes (Two's Complement) [cite: 4110]
  int16_t x_accel = (int16_t)(data[0] | (data[1] << 8));
  std::cout << "Accel X: " << x_accel << std::endl;

  close(file);
  return 0;
}