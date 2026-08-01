#pragma once
// IUartLink — UART communication interface for HW22 drone control

#ifndef I_UART_LINK_H
#define I_UART_LINK_H

#include <string>
#include "protocol/drone_link.h"

class IUartLink {
public:
    virtual ~IUartLink() = default;

    // Open UART device (e.g. "/tmp/ttyA" for sim, "/dev/ttyAMA1" for real Pi)
    // Returns true on success
    virtual bool open(const std::string& device) = 0;

    // Close UART device
    virtual void close() = 0;

    // Read available bytes from UART (non-blocking).
    // Returns number of bytes read, or -1 if no data available (EAGAIN).
    virtual int readBytes(uint8_t* buf, int bufSize) = 0;

    // Feed all available bytes to the internal parser and return the next valid packet.
    // Returns true if a valid packet is ready.
    virtual bool parseNext(dlink::PacketType& outType,
                           uint8_t* outPayload,
                           uint8_t& outLen) = 0;

    // Encode and send a packet over UART.
    virtual bool send(dlink::PacketType type, const void* payload, uint8_t payloadLen) = 0;

    // Send CONTROL command (convenience wrapper).
    virtual bool sendControl(float accel, float turnRate) = 0;

    // Check if UART is open and ready
    virtual bool isOpen() const = 0;
};

#endif // I_UART_LINK_H
