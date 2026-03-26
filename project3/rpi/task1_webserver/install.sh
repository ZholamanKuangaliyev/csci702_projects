#!/bin/bash
# Task 1 – Build and deploy the IMU CGI script on the Raspberry Pi.
# Run this script on the RPi:  chmod +x install.sh && sudo ./install.sh

set -e

echo "=== Task 1: Building imu_cgi ==="
g++ -O2 -o imu_cgi imu_cgi.cpp -lm
echo "Build OK"

echo "=== Installing packages (nginx + fcgiwrap) ==="
apt-get install -y nginx fcgiwrap

echo "=== Deploying CGI binary ==="
cp imu_cgi /usr/lib/cgi-bin/imu_cgi
chmod 755 /usr/lib/cgi-bin/imu_cgi
# Allow www-data to access I2C bus
usermod -aG i2c www-data 2>/dev/null || true

echo "=== Deploying nginx config ==="
cp nginx_imu.conf /etc/nginx/sites-available/imu
ln -sf /etc/nginx/sites-available/imu /etc/nginx/sites-enabled/imu

echo "=== Enabling services ==="
systemctl enable --now fcgiwrap
systemctl reload nginx

RPi_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "Done! Open http://${RPi_IP}/cgi-bin/imu_cgi from the host PC browser."
