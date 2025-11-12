#pragma once

#include <Arduino.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <cstdint>
#include <memory>

#include <api/HardwareCAN.h>

// CANHub + CANClient
// CANHub: single reader from physical arduino::HardwareCAN; demultiplexes
// messages to multiple CANClient objects. Each CANClient behaves like a
// lightweight arduino::HardwareCAN that only exposes messages matching its
// subscription (individual IDs and/or ranges). Writes from clients are
// forwarded to the underlying physical CAN.

class CANHub;  // forward

class CANClient : public arduino::HardwareCAN {
public:
  // `hub` is owner that distributes messages.
  CANClient(CANHub* hub) : hub_(hub) {}

  // Add a single exact ID subscription
  void addId(uint32_t id) {
    ids_.push_back(id);
  }

  // Add a range subscription: [start, start+count)
  void addRange(uint32_t start, uint32_t count) {
    ranges_.emplace_back(start, start + count);
  }

  // Check buffer (this triggers hub poll to collect incoming messages)
  size_t available() override;

  // Read next message
  CanMsg read() override;

  // Write forwards to hub which forwards to physical CAN
  int write(const CanMsg& msg) override;

  // Begin/end forwarded to hub
  bool begin(CanBitRate const can_bitrate) override;
  void end() override;

private:
  friend class CANHub;
  bool matches(uint32_t id) const {
    for (auto& r : ranges_) {
      if (id >= r.first && id < r.second) return true;
    }
    for (auto& v : ids_)
      if (id == v) return true;
    return false;
  }

  CANHub* hub_ = nullptr;
  std::vector<uint32_t> ids_;
  std::vector<std::pair<uint32_t, uint32_t>> ranges_;
  std::deque<CanMsg> rxq_;
  static constexpr size_t kMaxQueue = 128;
};

class CANHub : public arduino::HardwareCAN {
public:
  CANHub(arduino::HardwareCAN* base) : base_(base) {}

  // Create a new client; caller should keep the pointer alive (we own it)
  CANClient* createClient() {
    clients_.emplace_back(std::make_unique<CANClient>(this));
    return clients_.back().get();
  }

  // Convenience: create client and add a list of ids
  CANClient* createClientWithIds(std::initializer_list<uint32_t> ids) {
    CANClient* c = createClient();
    for (auto id : ids) c->addId(id);
    return c;
  }

  // Convenience: create client and add a range [start, start+count)
  CANClient* createClientWithRange(uint32_t start, uint32_t count) {
    CANClient* c = createClient();
    c->addRange(start, count);
    return c;
  }

  // Poll the physical CAN and distribute messages to clients' queues.
  // Safe to call repeatedly; it will drain available messages.
  void poll() {
    if (base_ == nullptr) return;
    while (base_->available()) {
      CanMsg m = base_->read();
      uint32_t id = m.getStandardId();
      for (auto& cptr : clients_) {
        CANClient* c = cptr.get();
        if (c->matches(id)) {
          if (c->rxq_.size() < CANClient::kMaxQueue) c->rxq_.push_back(m);
        }
      }
    }
  }

  // Forward writes to the physical CAN
  int write(const CanMsg& msg) override {
    if (base_ == nullptr) return -1;
    return base_->write(msg);
  }

  // The hub itself can expose the physical available/read if needed
  size_t available() override {
    return base_ ? base_->available() : 0;
  }
  CanMsg read() override {
    return base_ ? base_->read() : CanMsg();
  }

  bool begin(CanBitRate const can_bitrate) override {
    if (base_ == nullptr) return false;
    return base_->begin(can_bitrate);
  }

  void end() override {
    if (base_ == nullptr) return;
    base_->end();
  }

private:
  arduino::HardwareCAN* base_ = nullptr;
  std::vector<std::unique_ptr<CANClient>> clients_;
};

// CANClient method implementations that need CANHub definition
inline size_t CANClient::available() {
  if (hub_) hub_->poll();
  return rxq_.size();
}

inline CanMsg CANClient::read() {
  CanMsg empty{};
  if (rxq_.empty()) return empty;
  CanMsg m = rxq_.front();
  rxq_.pop_front();
  return m;
}

inline int CANClient::write(const CanMsg& msg) {
  if (!hub_) return -1;
  return hub_->write(msg);
}

inline bool CANClient::begin(CanBitRate const can_bitrate) {
  if (!hub_) return false;
  return hub_->begin(can_bitrate);
}

inline void CANClient::end() {
  if (!hub_) return;
  hub_->end();
}
