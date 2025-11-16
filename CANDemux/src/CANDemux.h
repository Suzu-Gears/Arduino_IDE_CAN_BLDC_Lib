#pragma once

#include <Arduino.h>
#include <vector>
#include <deque>
#include <cstdint>
#include <memory>
#include <map>
#include <set>
#include <functional>
#include <utility>          // for std::pair, std::move
#include <initializer_list> // for std::initializer_list

#include <api/HardwareCAN.h>

class CANDemux;        // forward
class VirtualCANImpl;  // forward

/**
 * @brief ユーザーが直接操作するための、軽量な仮想CANバスハンドル。
 * arduino::HardwareCANを継承しており、setCAN()などの関数に渡すことが可能。
 * コピー可能で、ポインタ管理の複雑さを伴わない。
 */
class VirtualCAN : public arduino::HardwareCAN {
public:
  VirtualCAN(VirtualCANImpl* client = nullptr);

  bool begin(CanBitRate const can_bitrate) override;
  void end() noexcept override;
  int write(const CanMsg& msg) override;
  size_t available() noexcept override;
  CanMsg read() noexcept override;

  // VirtualCANImplが持つ購読メソッドも転送する。
  void addId(uint32_t id);
  void addRange(uint32_t start, uint32_t count);

  // キュー管理メソッド
  enum class OverflowPolicy { DropNewest,
                              DropOldest };
  void setOverflowPolicy(OverflowPolicy policy);
  void onQueueOverflow(std::function<void()> callback);

private:
  VirtualCANImpl* client_;
};


/**
 * @class VirtualCANImpl
 * @brief 物理CANバスへの仮想インターフェースを実装するクラス。
 * CANDemuxによって特定のCAN IDまたはID範囲のメッセージのみがこの仮想バスにルーティングされる。
 * arduino::HardwareCANインターフェースを実装する。
 */
class VirtualCANImpl : public arduino::HardwareCAN {
public:
  using OverflowPolicy = VirtualCAN::OverflowPolicy;

  // `hub`はメッセージを分配するオーナー
  explicit VirtualCANImpl(CANDemux* hub, size_t queue_size) noexcept
    : hub_(hub), max_queue_size_(queue_size) {}

  // 単一のIDを購読
  void addId(uint32_t id);

  // [start, start+count) の範囲を購読
  void addRange(uint32_t start, uint32_t count);

  // arduino::HardwareCAN interface
  bool begin(CanBitRate const can_bitrate) override;
  void end() noexcept override;
  int write(const CanMsg& msg) override;
  size_t available() noexcept override;
  CanMsg read() noexcept override;

  // キュー管理
  void setOverflowPolicy(OverflowPolicy policy) {
    overflow_policy_ = policy;
  }
  void onQueueOverflow(std::function<void()> callback) {
    overflow_callback_ = callback;
  }

private:
  friend class CANDemux;
  bool matchesRange(uint32_t id) const noexcept {
    for (const auto& r : ranges_) {
      if (id >= r.first && id < r.second) return true;
    }
    return false;
  }

  void push(const CanMsg& msg) {
    if (rxq_.size() < max_queue_size_) {
      rxq_.push_back(msg);
    } else {
      if (overflow_callback_) {
        overflow_callback_();
      }
      if (overflow_policy_ == OverflowPolicy::DropOldest) {
        rxq_.pop_front();
        rxq_.push_back(msg);
      }
      // DropNewest (default) の場合は何もしない
    }
  }

  CANDemux* hub_ = nullptr;
  std::vector<std::pair<uint32_t, uint32_t>> ranges_;
  std::deque<CanMsg> rxq_;
  size_t max_queue_size_;
  OverflowPolicy overflow_policy_ = OverflowPolicy::DropNewest;
  std::function<void()> overflow_callback_ = nullptr;
};

/**
 * @class CANDemux
 * @brief 物理CANバスを抽象化し、受信メッセージを複数のVirtualCANImplインスタンスに分配（デマルチプレクス）する。
 */
class CANDemux {
public:
  explicit CANDemux(arduino::HardwareCAN* base) noexcept : base_(base) {}

  // CANDemuxはコピー不可/ムーブ不可 (クライアントがハブへのポインタを持つため)
  CANDemux(const CANDemux&) = delete;
  CANDemux& operator=(const CANDemux&) = delete;

  /**
   * @brief 新しい仮想CANバスクライアントを作成する。
   * 作成されたVirtualCANImplインスタンスの所有権はCANDemuxが管理する。
   * @param queue_size 受信メッセージキューの最大サイズ。
   * @return 新しく作成された仮想CANバスのハンドル。
   */
  VirtualCAN createClient(size_t queue_size = 128) {
    clients_.emplace_back(std::make_unique<VirtualCANImpl>(this, queue_size));
    return VirtualCAN(clients_.back().get());
  }

  template<typename T>
  VirtualCAN createClientWithIds(const T& ids, size_t queue_size = 128) {
    VirtualCAN c = createClient(queue_size);
    for (const uint32_t id : ids) {
      c.addId(id);
    }
    return c;
  }

  VirtualCAN createClientWithIds(std::initializer_list<uint32_t> ids, size_t queue_size = 128) {
    VirtualCAN c = createClient(queue_size);
    for (auto id : ids) c.addId(id);
    return c;
  }

  VirtualCAN createClientWithRange(uint32_t start, uint32_t count, size_t queue_size = 128) {
    VirtualCAN c = createClient(queue_size);
    c.addRange(start, count);
    return c;
  }

  /**
     * @brief 物理CANバスをポーリングし、購読しているクライアントにメッセージを分配する。
     * この関数はVirtualCANImpl::available()から自動的に呼び出される。
     */
  void poll() {
    if (base_ == nullptr) return;

    while (base_->available()) {
      CanMsg m = base_->read();
      const uint32_t id = m.id;

      std::set<VirtualCANImpl*> recipients;

      auto it = id_subscriptions_.find(id);
      if (it != id_subscriptions_.end()) {
        recipients.insert(it->second.begin(), it->second.end());
      }

      for (VirtualCANImpl* c : range_subscription_clients_) {
        if (c->matchesRange(id)) {
          recipients.insert(c);
        }
      }

      for (VirtualCANImpl* c : recipients) {
        c->push(m);
      }
    }
  }

  // 物理CANに直接書き込み
  int write(const CanMsg& msg) {
    if (base_ == nullptr) return -1;
    return base_->write(msg);
  }

private:
  friend class VirtualCANImpl;
  void subscribeId(uint32_t id, VirtualCANImpl* client) {
    id_subscriptions_[id].push_back(client);
  }

  void subscribeRange(VirtualCANImpl* client) {
    range_subscription_clients_.insert(client);
  }

  arduino::HardwareCAN* base_ = nullptr;
  std::vector<std::unique_ptr<VirtualCANImpl>> clients_;
  std::map<uint32_t, std::vector<VirtualCANImpl*>> id_subscriptions_;
  std::set<VirtualCANImpl*> range_subscription_clients_;
};

// --- VirtualCAN メソッドの実装 ---
inline VirtualCAN::VirtualCAN(VirtualCANImpl* client) : client_(client) {}
inline bool VirtualCAN::begin(CanBitRate const can_bitrate) {
  return client_ ? client_->begin(can_bitrate) : false;
}
inline void VirtualCAN::end() noexcept {
  if (client_) client_->end();
}
inline int VirtualCAN::write(const CanMsg& msg) {
  return client_ ? client_->write(msg) : -1;
}
inline size_t VirtualCAN::available() noexcept {
  return client_ ? client_->available() : 0;
}
inline CanMsg VirtualCAN::read() noexcept {
  return client_ ? client_->read() : CanMsg{};
}
inline void VirtualCAN::addId(uint32_t id) {
  if (client_) client_->addId(id);
}
inline void VirtualCAN::addRange(uint32_t start, uint32_t count) {
  if (client_) client_->addRange(start, count);
}
inline void VirtualCAN::setOverflowPolicy(OverflowPolicy policy) {
  if (client_) client_->setOverflowPolicy(policy);
}
inline void VirtualCAN::onQueueOverflow(std::function<void()> callback) {
  if (client_) client_->onQueueOverflow(callback);
}


// --- VirtualCANImpl メソッドの実装 ---
inline void VirtualCANImpl::addId(uint32_t id) {
  if (hub_) hub_->subscribeId(id, this);
}

inline void VirtualCANImpl::addRange(uint32_t start, uint32_t count) {
  ranges_.emplace_back(start, start + count);
  if (hub_) hub_->subscribeRange(this);
}

inline bool VirtualCANImpl::begin(CanBitRate const can_bitrate) {
  (void)can_bitrate;  // "unused parameter" 警告を抑制
  return hub_ != nullptr;
}

inline void VirtualCANImpl::end() noexcept {
  // no-op
}

inline int VirtualCANImpl::write(const CanMsg& msg) {
  if (!hub_) return -1;
  return hub_->write(msg);
}

inline size_t VirtualCANImpl::available() noexcept {
  if (hub_) hub_->poll();
  return rxq_.size();
}

inline CanMsg VirtualCANImpl::read() noexcept {
  if (rxq_.empty()) return CanMsg{};
  CanMsg m = std::move(rxq_.front());
  rxq_.pop_front();
  return m;
}
