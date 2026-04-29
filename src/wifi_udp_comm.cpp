#include "wifi_udp_comm.hpp"

#include "serial_logger.hpp"

WifiUdpComms::WifiUdpComms(uint16_t udp_port) : m_udp_port(udp_port) {}

int WifiUdpComms::begin(const char* ssid, const char* password) {
    if (WiFi.status() == WL_NO_MODULE) {
        LOG_ERROR("WiFi module not found");
        return 1;
    }

    LOG_INFO("Scanning WiFi networks");
    const int network_count = WiFi.scanNetworks();
    bool found = false;
    for (int i = 0; i < network_count; ++i) {
        if (WiFi.SSID(i) == String(ssid)) {
            found = true;
        }
    }
    if (!found) {
        LOG_WARN("Target SSID was not found during scan");
    }

    LOG_INFO("Connecting WiFi");
    WiFi.begin(ssid, password);

    for (uint8_t attempt = 0; attempt < 40 && WiFi.status() != WL_CONNECTED; ++attempt) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        LOG_ERROR("WiFi connection failed");
        return 2;
    }

    m_udp.begin(m_udp_port);
    LOG_INFO("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("UDP port: ");
    Serial.println(m_udp_port);
    return 0;
}

int WifiUdpComms::read(uint8_t* buffer, size_t buffer_size) {
    const int packet_size = m_udp.parsePacket();
    if (packet_size <= 0) {
        return 0;
    }
    m_remote_ip = m_udp.remoteIP();
    m_remote_port = m_udp.remotePort();
    return m_udp.read(buffer, buffer_size);
}

bool WifiUdpComms::send_reply(const uint8_t* data, size_t size) {
    if (m_remote_port == 0) {
        return false;
    }
    m_udp.beginPacket(m_remote_ip, m_remote_port);
    m_udp.write(data, size);
    return m_udp.endPacket() == 1;
}

bool WifiUdpComms::connected() const {
    return WiFi.status() == WL_CONNECTED;
}

IPAddress WifiUdpComms::local_ip() const {
    return WiFi.localIP();
}
