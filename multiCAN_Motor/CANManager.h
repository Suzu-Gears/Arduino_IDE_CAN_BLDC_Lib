#pragma once

#include <Arduino.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <map>
#include <set>
#include <functional> // for std::function

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
    CANClientHandle(CANClient* client = nullptr);

    // arduino::HardwareCANの仮想メソッドをオーバーライドし、
    // 実際の処理を内部のCANClientオブジェクトに転送する。
    bool begin(CanBitRate const can_bitrate) override;
    void end() noexcept override;
    int write(const CanMsg& msg) override;
    size_t available() noexcept override;
    CanMsg read() noexcept override;

    // CANClientが持つ購読メソッドも転送する。
    void addId(uint32_t id);
    void addRange(uint32_t start, uint32_t count);

    // キュー管理メソッド
    enum class OverflowPolicy { DropNewest, DropOldest };
    void setOverflowPolicy(OverflowPolicy policy);
    void onQueueOverflow(std::function<void()> callback);

private:
    CANClient* client_;
};


/**
 * @class CANClient
 * @brief 物理CANバスへの仮想インターフェース。
 * CANHubによって特定のIDまたはID範囲のメッセージのみがルーティングされる。
 * arduino::HardwareCANインターフェースを実装する。
 */
class CANClient : public arduino::HardwareCAN {
public:
    using OverflowPolicy = CANClientHandle::OverflowPolicy;

    // `hub`はメッセージを分配するオーナー
    explicit CANClient(CANHub* hub, size_t queue_size) noexcept 
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
    friend class CANHub;
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

    CANHub* hub_ = nullptr;
    std::vector<std::pair<uint32_t, uint32_t>> ranges_;
    std::deque<CanMsg> rxq_;
    size_t max_queue_size_;
    OverflowPolicy overflow_policy_ = OverflowPolicy::DropNewest;
    std::function<void()> overflow_callback_ = nullptr;
};

/**
 * @class CANHub
 * @brief 物理CANをラップし、メッセージを複数のCANClientに分配(demultiplex)する。
 */
class CANHub {
public:
    explicit CANHub(arduino::HardwareCAN* base) noexcept : base_(base) {}

    // CANHubはコピー不可/ムーブ不可 (クライアントがハブへのポインタを持つため)
    CANHub(const CANHub&) = delete;
    CANHub& operator=(const CANHub&) = delete;

    // クライアントを作成 (所有権はHubが持つ)
    CANClientHandle createClient(size_t queue_size = 128) {
        clients_.emplace_back(std::make_unique<CANClient>(this, queue_size));
        return CANClientHandle(clients_.back().get());
    }

    template<typename T>
    CANClientHandle createClientWithIds(const T& ids, size_t queue_size = 128) {
        CANClientHandle c = createClient(queue_size);
        for (const uint32_t id : ids) {
            c.addId(id);
        }
        return c;
    }

    CANClientHandle createClientWithIds(std::initializer_list<uint32_t> ids, size_t queue_size = 128) {
        CANClientHandle c = createClient(queue_size);
        for (auto id : ids) c.addId(id);
        return c;
    }

    CANClientHandle createClientWithRange(uint32_t start, uint32_t count, size_t queue_size = 128) {
        CANClientHandle c = createClient(queue_size);
        c.addRange(start, count);
        return c;
    }

    /**
     * @brief 物理CANバスをポーリングし、購読しているクライアントにメッセージを分配する。
     * この関数はCANClient::available()から自動的に呼び出される。
     */
    void poll() {
        if (base_ == nullptr) return;

        while (base_->available()) {
            CanMsg m = base_->read();
            const uint32_t id = m.id;

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
    friend class CANClient;
    void subscribeId(uint32_t id, CANClient* client) {
        id_subscriptions_[id].push_back(client);
    }

    void subscribeRange(CANClient* client) {
        range_subscription_clients_.insert(client);
    }

    arduino::HardwareCAN* base_ = nullptr;
    std::vector<std::unique_ptr<CANClient>> clients_;
    std::map<uint32_t, std::vector<CANClient*>> id_subscriptions_;
    std::set<CANClient*> range_subscription_clients_;
};

// --- CANClientHandle メソッドの実装 ---
inline CANClientHandle::CANClientHandle(CANClient* client) : client_(client) {}
inline bool CANClientHandle::begin(CanBitRate const can_bitrate) { return client_ ? client_->begin(can_bitrate) : false; }
inline void CANClientHandle::end() noexcept { if (client_) client_->end(); }
inline int CANClientHandle::write(const CanMsg& msg) { return client_ ? client_->write(msg) : -1; }
inline size_t CANClientHandle::available() noexcept { return client_ ? client_->available() : 0; }
inline CanMsg CANClientHandle::read() noexcept { return client_ ? client_->read() : CanMsg{}; }
inline void CANClientHandle::addId(uint32_t id) { if (client_) client_->addId(id); }
inline void CANClientHandle::addRange(uint32_t start, uint32_t count) { if (client_) client_->addRange(start, count); }
inline void CANClientHandle::setOverflowPolicy(OverflowPolicy policy) { if (client_) client_->setOverflowPolicy(policy); }
inline void CANClientHandle::onQueueOverflow(std::function<void()> callback) { if (client_) client_->onQueueOverflow(callback); }


// --- CANClient メソッドの実装 ---

inline void CANClient::addId(uint32_t id) {
    if (hub_) hub_->subscribeId(id, this);
}

inline void CANClient::addRange(uint32_t start, uint32_t count) {
    ranges_.emplace_back(start, start + count);
    if (hub_) hub_->subscribeRange(this);
}

inline bool CANClient::begin(CanBitRate const can_bitrate) {
    (void)can_bitrate; // "unused parameter" 警告を抑制
    return hub_ != nullptr;
}

inline void CANClient::end() noexcept {
    // no-op
}

inline int CANClient::write(const CanMsg& msg) {
    if (!hub_) return -1;
    return hub_->write(msg);
}

inline size_t CANClient::available() noexcept {
    if (hub_) hub_->poll();
    return rxq_.size();
}

inline CanMsg CANClient::read() noexcept {
    if (rxq_.empty()) return CanMsg{};
    CanMsg m = std::move(rxq_.front());
    rxq_.pop_front();
    return m;
}