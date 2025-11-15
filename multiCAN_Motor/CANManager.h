#pragma once

#include <Arduino.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <map>
#include <set>

#include <api/HardwareCAN.h>

class CANHub;   // forward
class CANClient; // forward

/**
 * @brief ユーザーが直接扱うための、軽量なCANClientのハンドルクラス。
 * arduino::HardwareCANを継承しているため、setCAN()に渡すことができる。
 * このクラス自体はコピー可能で、ポインタのように扱う必要がない。
 */
class CANClientHandle : public arduino::HardwareCAN {
public:
  CANClientHandle(CANClient* client = nullptr) : client_(client) {}

  // arduino::HardwareCANの仮想メソッドをオーバーライドし、
  // 実際の処理を内部のCANClientオブジェクトに転送する。
  size_t available() override;
  CanMsg read() override;
  int write(const CanMsg& msg) override;
  bool begin(CanBitRate const can_bitrate) override;
  void end() override;

  // CANClientが持つ購読メソッドも転送する。
  void addId(uint32_t id);
  void addRange(uint32_t start, uint32_t count);

private:
  CANClient* client_;
};


/**
 * @brief CANメッセージのフィルターとバッファを持つ、CANClientの実体。
 * このクラスはCANHubによって内部的に所有・管理される。
 */
class CANClient : public arduino::HardwareCAN {
public:
  CANClient(CANHub* hub) : hub_(hub) {}

  void addId(uint32_t id);
  void addRange(uint32_t start, uint32_t count);

  size_t available() override;
  CanMsg read() override;
  int write(const CanMsg& msg) override;
  bool begin(CanBitRate const can_bitrate) override;
  void end() override;

private:
  friend class CANHub;
  bool matchesRange(uint32_t id) const {
    for (auto& r : ranges_) {
      if (id >= r.first && id < r.second) return true;
    }
    return false;
  }

  CANHub* hub_ = nullptr;
  std::vector<std::pair<uint32_t, uint32_t>> ranges_;
  std::deque<CanMsg> rxq_;
  static constexpr size_t kMaxQueue = 128;
};


/**
 * @brief 物理CANからメッセージを読み取り、各CANClientに分配するハブクラス。
 */
class CANHub : public arduino::HardwareCAN {
public:
  CANHub(arduino::HardwareCAN* base) : base_(base) {}

  // createClient...メソッドがCANClientHandleを返すように変更
  CANClientHandle createClient() {
    clients_.emplace_back(std::make_unique<CANClient>(this));
    return CANClientHandle(clients_.back().get());
  }

  CANClientHandle createClientWithIds(std::initializer_list<uint32_t> ids) {
    CANClientHandle c = createClient();
    for (auto id : ids) c.addId(id);
    return c;
  }

  CANClientHandle createClientWithRange(uint32_t start, uint32_t count) {
    CANClientHandle c = createClient();
    c.addRange(start, count);
    return c;
  }

  void poll() {
    if (base_ == nullptr) return;
    while (base_->available()) {
      CanMsg m = base_->read();
      uint32_t id = m.getStandardId();

      std::set<CANClient*> recipients;

      auto it = id_subscriptions_.find(id);
      if (it != id_subscriptions_.end()) {
        recipients.insert(it->second.begin(), it->second.end());
      }

      for (CANClient* c : range_subscription_clients_) {
        if (c->matchesRange(id)) {
          recipients.insert(c);
        }
      }

      for (CANClient* c : recipients) {
        if (c->rxq_.size() < CANClient::kMaxQueue) {
          c->rxq_.push_back(m);
        }
      }
    }
  }

  int write(const CanMsg& msg) override {
    if (base_ == nullptr) return -1;
    return base_->write(msg);
  }

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
  friend class CANClient;
  void subscribeId(uint32_t id, CANClient* client) {
    id_subscriptions_[id].push_back(client);
  }

  void subscribeRange(CANClient* client) {
    if (std::find(range_subscription_clients_.begin(), range_subscription_clients_.end(), client) == range_subscription_clients_.end()) {
      range_subscription_clients_.push_back(client);
    }
  }

  arduino::HardwareCAN* base_ = nullptr;
  std::vector<std::unique_ptr<CANClient>> clients_;
  std::map<uint32_t, std::vector<CANClient*>> id_subscriptions_;
  std::vector<CANClient*> range_subscription_clients_;
};

// --- CANClientHandleの実装 ---
inline size_t CANClientHandle::available() { return client_ ? client_->available() : 0; }
inline CanMsg CANClientHandle::read() { return client_ ? client_->read() : CanMsg{}; }
inline int CANClientHandle::write(const CanMsg& msg) { return client_ ? client_->write(msg) : -1; }
inline bool CANClientHandle::begin(CanBitRate const can_bitrate) { return client_ ? client_->begin(can_bitrate) : false; }
inline void CANClientHandle::end() { if (client_) client_->end(); }
inline void CANClientHandle::addId(uint32_t id) { if (client_) client_->addId(id); }
inline void CANClientHandle::addRange(uint32_t start, uint32_t count) { if (client_) client_->addRange(start, count); }


// --- CANClientの実装 ---
inline void CANClient::addId(uint32_t id) {
  if (hub_) hub_->subscribeId(id, this);
}

inline void CANClient::addRange(uint32_t start, uint32_t count) {
  ranges_.emplace_back(start, start + count);
  if (hub_) hub_->subscribeRange(this);
}

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
