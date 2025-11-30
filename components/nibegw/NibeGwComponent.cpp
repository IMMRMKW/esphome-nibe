#include "NibeGwComponent.h"

#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"
#include "esphome/core/log.h"

namespace esphome {
namespace nibegw {

    static const char* const TAG = "nibegw";

    // Helpers to convert between ESPHome IP type and sockaddr_in
    static inline void fill_sockaddr(struct sockaddr_in& sa, const esphome::network::IPAddress& ip, uint16_t port)
    {
        sa.sin_family = AF_INET;
        sa.sin_port = htons(port);
        // Extract bytes from ip_addr_ field (assume IPv4)
        const uint8_t* arr = reinterpret_cast<const uint8_t*>(&ip);
        uint32_t addr = (uint32_t(arr[0]) << 24) | (uint32_t(arr[1]) << 16) | (uint32_t(arr[2]) << 8) | (uint32_t(arr[3]) << 0);
        sa.sin_addr.s_addr = htonl(addr);
    }

    static inline esphome::network::IPAddress to_esphome_ip(const struct sockaddr_in& sa)
    {
        uint32_t addr = ntohl(sa.sin_addr.s_addr);
        return esphome::network::IPAddress {
            uint8_t((addr >> 24) & 0xFF),
            uint8_t((addr >> 16) & 0xFF),
            uint8_t((addr >> 8) & 0xFF),
            uint8_t((addr >> 0) & 0xFF)
        };
    }

    NibeGwComponent::NibeGwComponent(esphome::GPIOPin* dir_pin)
        : udp_read_sock_(-1)
        , udp_write_sock_(-1)
    {
        gw_ = new NibeGw(this, dir_pin);
        gw_->setCallback(
            std::bind(&NibeGwComponent::callback_msg_received, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NibeGwComponent::callback_msg_token_received, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
    }

    // Deduplication helper
    static request_data_type dedup(const uint8_t* data, int len, uint8_t val)
    {
        request_data_type message;
        uint8_t value = ~val;
        for (int i = 5; i < len - 1; i++) {
            if (data[i] == val && value == val) {
                value = ~val;
                continue;
            }
            value = data[i];
            message.push_back(value);
        }
        return message;
    }

    // Called by NibeGw when a message is received; forward to UDP targets via sendto()
    void NibeGwComponent::callback_msg_received(const uint8_t* data, int len)
    {
        {
            request_key_type key { uint16_t(data[2] | (data[1] << 8)), static_cast<uint8_t>(data[3]) };
            const auto& it = message_listener_.find(key);
            if (it != message_listener_.end()) {
                it->second(dedup(data, len, STARTBYTE_MASTER));
            }
        }

        if (!is_connected_)
            return;

        for (auto target = udp_targets_.begin(); target != udp_targets_.end(); target++) {
            const auto& ip = std::get<0>(*target);
            uint16_t port = std::get<1>(*target);

            struct sockaddr_in sa {};
            fill_sockaddr(sa, ip, port);

            int sent = sendto(udp_write_sock_, data, len, 0, (struct sockaddr*)&sa, sizeof(sa));
            if (sent < 0 || sent != len) {
                ESP_LOGW(TAG, "UDP Packet send failed to %s:%d (err=%d)", ip.str().c_str(), port, errno);
            }
        }
    }

    // Handle packets received on read/write ports
    void NibeGwComponent::token_request_cache_from_sock(int sock_fd, uint8_t address, uint8_t token)
    {
        if (!is_connected_)
            return;

        uint8_t buf[MAX_DATA_LEN];
        struct sockaddr_in source_addr {};
        socklen_t socklen = sizeof(source_addr);

        int size = recvfrom(sock_fd, buf, sizeof(buf), 0, (struct sockaddr*)&source_addr, &socklen);
        if (size <= 0)
            return; // no data

        ESP_LOGV(TAG, "UDP Packet token data of %d bytes received", size);

        if (size > MAX_DATA_LEN) {
            ESP_LOGE(TAG, "UDP Packet too large: %d", size);
            return;
        }

        auto ip = to_esphome_ip(source_addr);

        if (!udp_source_ip_.empty() && std::count(udp_source_ip_.begin(), udp_source_ip_.end(), ip) == 0) {
            ESP_LOGW(TAG, "UDP Packet wrong ip ignored %s", ip.str().c_str());
            return;
        }

        request_data_type request;
        request.assign(buf, buf + size);
        add_queued_request(address, token, std::move(request));
    }

    static int copy_request(const request_data_type& request, uint8_t* data)
    {
        auto len = std::min(request.size(), (size_t)MAX_DATA_LEN);
        std::copy_n(request.begin(), len, data);
        return (int)len;
    }

    int NibeGwComponent::callback_msg_token_received(uint16_t address, uint8_t command, uint8_t* data)
    {
        request_key_type key { address, command };

        {
            const auto& it = requests_.find(key);
            if (it != requests_.end()) {
                auto& queue = it->second;
                if (!queue.empty()) {
                    auto len = copy_request(queue.front(), data);
                    queue.pop();
                    ESP_LOGD(TAG, "Response to address: 0x%x token: 0x%x bytes: %d", std::get<0>(key), std::get<1>(key), len);
                    return len;
                }
            }
        }

        {
            const auto& it = requests_provider_.find(key);
            if (it != requests_provider_.end()) {
                auto len = copy_request(it->second(), data);
                ESP_LOGD(TAG, "Response to address: 0x%x token: 0x%x bytes: %d", std::get<0>(key), std::get<1>(key), len);
                return len;
            }
        }

        return 0;
    }

    void NibeGwComponent::setup()
    {
        ESP_LOGI(TAG, "Starting up");
        gw_->connect();
    }

    void NibeGwComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "NibeGw");
        for (auto target = udp_targets_.begin(); target != udp_targets_.end(); target++) {
            ESP_LOGCONFIG(TAG, " Target: %s:%d", std::get<0>(*target).str().c_str(), std::get<1>(*target));
        }
        for (auto address = udp_source_ip_.begin(); address != udp_source_ip_.end(); address++) {
            ESP_LOGCONFIG(TAG, " Source: %s", address->str().c_str());
        }
        ESP_LOGCONFIG(TAG, " Read Port: %d", udp_read_port_);
        ESP_LOGCONFIG(TAG, " Write Port: %d", udp_write_port_);
    }

    int NibeGwComponent::open_udp_socket(uint16_t port)
    {
        // Create UDP socket
        int sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
            return -1;
        }

        // Set non-blocking
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        // Allow reuse
        int yes = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        // Bind to given port
        struct sockaddr_in sa {};
        sa.sin_family = AF_INET;
        sa.sin_port = htons(port);
        sa.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sock, (struct sockaddr*)&sa, sizeof(sa)) < 0) {
            ESP_LOGE(TAG, "Failed to bind socket to port %d: errno %d", port, errno);
            ::close(sock);
            return -1;
        }

        ESP_LOGI(TAG, "UDP socket bound on port %d", port);
        return sock;
    }

    void NibeGwComponent::loop()
    {
        // Connect sockets when network comes up
        if (network::is_connected() && !is_connected_) {
            ESP_LOGI(TAG, "Connecting network ports.");
            udp_read_sock_ = open_udp_socket(udp_read_port_);
            udp_write_sock_ = open_udp_socket(udp_write_port_);
            if (udp_read_sock_ < 0 || udp_write_sock_ < 0) {
                ESP_LOGE(TAG, "Failed to open UDP sockets");
                if (udp_read_sock_ >= 0) {
                    ::close(udp_read_sock_);
                    udp_read_sock_ = -1;
                }
                if (udp_write_sock_ >= 0) {
                    ::close(udp_write_sock_);
                    udp_write_sock_ = -1;
                }
            } else {
                is_connected_ = true;
            }
        }

        // Disconnect sockets when network goes down
        if (!network::is_connected() && is_connected_) {
            ESP_LOGI(TAG, "Disconnecting network ports.");
            if (udp_read_sock_ >= 0) {
                ::close(udp_read_sock_);
                udp_read_sock_ = -1;
            }
            if (udp_write_sock_ >= 0) {
                ::close(udp_write_sock_);
                udp_write_sock_ = -1;
            }
            is_connected_ = false;
        }

        // Poll incoming packets if connected
        if (is_connected_) {
            if (udp_read_sock_ >= 0) {
                token_request_cache_from_sock(udp_read_sock_, MODBUS40, READ_TOKEN);
            }
            if (udp_write_sock_ >= 0) {
                token_request_cache_from_sock(udp_write_sock_, MODBUS40, WRITE_TOKEN);
            }
        }

        // High‑frequency handling
        if (gw_->messageStillOnProgress()) {
            high_freq_.start();
        } else {
            high_freq_.stop();
        }

        // Drive gateway logic
        gw_->loop();
    }

} // namespace nibegw
} // namespace esphome
