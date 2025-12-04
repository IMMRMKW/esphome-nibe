#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>  // for uint8_t, uint16_t
#include <map>
#include <queue>
#include <set>
#include <vector>

#include "esphome.h"
#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "NibeGw.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include <errno.h>
#include <fcntl.h>

namespace esphome {
namespace nibegw {

using request_key_type = std::tuple<uint16_t, uint8_t>;
using request_data_type = std::vector<uint8_t>;
using request_provider_type = std::function<request_data_type(void)>;
using target_type = std::tuple<network::IPAddress, int>;
using message_listener_type = std::function<void(const request_data_type &)>;

class NibeGwComponent : public esphome::Component, public esphome::uart::UARTDevice {
  float get_setup_priority() const override {
    return setup_priority::PROCESSOR;
  }

  const char *TAG = "nibegw";
  const int requests_queue_max = 3;

  int udp_read_port_ = 9999;
  int udp_write_port_ = 10000;

  std::vector<network::IPAddress> udp_source_ip_;
  bool is_connected_ = false;

  std::vector<target_type> udp_targets_;
  std::map<request_key_type, std::queue<request_data_type>> requests_;
  std::map<request_key_type, request_provider_type> requests_provider_;
  std::map<request_key_type, message_listener_type> message_listener_;
  HighFrequencyLoopRequester high_freq_;

  NibeGw *gw_ = nullptr;

  // lwIP socket FDs
  int udp_read_sock_ = -1;
  int udp_write_sock_ = -1;

  // Internal helpers
  void callback_msg_received(const uint8_t *data, int len);
  int callback_msg_token_received(uint16_t address, uint8_t command, uint8_t *data);
  void callback_debug(uint8_t verbose, char *data);

  void token_request_cache_from_sock(int sock_fd, uint8_t address, uint8_t token);
  void handle_udp_packet(const uint8_t *data, int len, const network::IPAddress &from_ip, uint8_t address,
                         uint8_t token);
  bool is_source_ip_allowed(const network::IPAddress &ip);
  int open_udp_socket(uint16_t port);

 public:
  void set_read_port(int port) {
    udp_read_port_ = port;
  }
  void set_write_port(int port) {
    udp_write_port_ = port;
  }

  void add_target(const network::IPAddress &ip, int port) {
    udp_targets_.emplace_back(ip, port);
  }

  void add_source_ip(const network::IPAddress &ip) {
    udp_source_ip_.push_back(ip);
  }

  void set_request(int address, int token, request_data_type request) {
    set_request(address, token, [request] { return request; });
  }

  void set_request(int address, int token, request_provider_type provider) {
    requests_provider_[request_key_type(address, token)] = provider;
  }

  void add_listener(int address, int token, message_listener_type listener) {
    message_listener_[request_key_type(address, token)] = listener;
  }

  void add_queued_request(int address, int token, request_data_type request) {
    auto &queue = requests_[request_key_type(address, token)];
    if (queue.size() >= requests_queue_max) {
      queue.pop();
    }
    queue.push(std::move(request));
  }

  NibeGw &gw() {
    return *gw_;
  }

  NibeGwComponent(GPIOPin *dir_pin);

  void setup();
  void dump_config();
  void loop();
};

}  // namespace nibegw
}  // namespace esphome
