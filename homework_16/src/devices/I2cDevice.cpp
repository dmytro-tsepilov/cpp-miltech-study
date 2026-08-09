#include "devices/I2cDevice.h"

#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

I2cDevice::I2cDevice(std::string_view path) : fd_(-1), path_(path) {}

I2cDevice::~I2cDevice() {
    close();
}

bool I2cDevice::open() {
    fd_ = ::open(path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        perror("[I2cDevice] open");
        return false;
    }
    return true;
}

void I2cDevice::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool I2cDevice::setAddress(uint8_t addr) {
    int ret = ioctl(fd_, I2C_SLAVE, addr);
    if (ret < 0) {
        perror("[I2cDevice] ioctl I2C_SLAVE");
        return false;
    }
    return true;
}

bool I2cDevice::writeReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    ssize_t n = ::write(fd_, buf, sizeof(buf));
    if (n != static_cast<ssize_t>(sizeof(buf))) {
        perror("[I2cDevice] writeReg");
        return false;
    }
    return true;
}

bool I2cDevice::readReg(uint8_t reg, uint8_t* buf, size_t len) {
    // Phase 1: write the register pointer
    if (::write(fd_, &reg, 1) != 1) {
        perror("[I2cDevice] readReg phase1");
        return false;
    }
    // Phase 2: read data
    ssize_t n = ::read(fd_, buf, len);
    if (n != static_cast<ssize_t>(len)) {
        perror("[I2cDevice] readReg phase2");
        return false;
    }
    return true;
}

int I2cDevice::getFd() const {
    return fd_;
}
