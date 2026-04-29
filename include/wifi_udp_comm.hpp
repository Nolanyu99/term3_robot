#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

class WifiUdpComms {
public:
    explicit WifiUdpComms(uint16_t udp_port);

    int begin(const char* ssid, const char* password);
    int read(uint8_t* buffer, size_t buffer_size);
    bool send_reply(const uint8_t* data, size_t size);
    bool connected() const;
    IPAddress local_ip() const;

private:
    uint16_t m_udp_port;
    WiFiUDP m_udp;
    IPAddress m_remote_ip;
    uint16_t m_remote_port = 0;
};
