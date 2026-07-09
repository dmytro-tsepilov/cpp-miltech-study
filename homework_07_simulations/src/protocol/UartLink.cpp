#include "protocol/IUartLink.h"
#include <memory>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cerrno>

class UartLink : public IUartLink {
private:
    int fd_ = -1;
    bool open_ = false;
    dlink::Parser parser_;

public:
    UartLink() = default;
    ~UartLink() override { close(); }

    bool open(const std::string& device) override {
        if (open_) return true;

        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            perror("UartLink: open");
            return false;
        }

        termios tio{};
        tcgetattr(fd_, &tio);
        cfmakeraw(&tio);
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tio.c_cflag |= (CLOCAL | CREAD);
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            perror("UartLink: tcsetattr");
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        open_ = true;
        return true;
    }

    void close() override {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        open_ = false;
    }

    int readBytes(uint8_t* buf, int bufSize) override {
        if (!open_ || fd_ < 0) return -1;
        int n = ::read(fd_, buf, static_cast<size_t>(bufSize));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            return -1;
        }
        return n;
    }

    bool parseNext(dlink::PacketType& outType,
                   uint8_t* outPayload,
                   uint8_t& outLen) override {
        if (!open_) return false;

        uint8_t tmpBuf[256];
        while (true) {
            int n = readBytes(tmpBuf, static_cast<int>(sizeof(tmpBuf)));
            if (n <= 0) break;

            for (int i = 0; i < n; ++i) {
                if (parser_.feed(tmpBuf[i],
                                 reinterpret_cast<uint8_t&>(outType),
                                 outPayload,
                                 outLen)) {
                    return true;
                }
            }
        }
        return false;
    }

    bool send(dlink::PacketType type, const void* payload, uint8_t payloadLen) override {
        if (!open_ || fd_ < 0) return false;

        uint8_t outBuf[262];
        size_t m = dlink::encode(type, payload, payloadLen, outBuf);

        size_t written = 0;
        while (written < m) {
            ssize_t n = ::write(fd_, outBuf + written, m - written);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    usleep(1000);
                    continue;
                }
                return false;
            }
            written += static_cast<size_t>(n);
        }
        return true;
    }

    bool sendControl(float accel, float turnRate) override {
        dlink::Control ctrl{accel, turnRate};
        return send(dlink::PKT_CONTROL, &ctrl, sizeof(ctrl));
    }

    bool isOpen() const override { return open_; }
};

// Factory function declaration (to be called from main.cpp or factory)
std::unique_ptr<IUartLink> createUartLink() {
    return std::make_unique<UartLink>();
}
