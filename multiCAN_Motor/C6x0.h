#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#ifdef ARDUINO_ARCH_ESP32
#include <ESP32_TWAI.h>
#endif

#include <api/HardwareCAN.h>

enum C6x0Id {
  C610_ID_1,
  C610_ID_2,
  C610_ID_3,
  C610_ID_4,
  C610_ID_5,
  C610_ID_6,
  C610_ID_7,
  C610_ID_8,

  C620_ID_1,
  C620_ID_2,
  C620_ID_3,
  C620_ID_4,
  C620_ID_5,
  C620_ID_6,
  C620_ID_7,
  C620_ID_8,
};

class C6x0 {
public:
  void setCAN(arduino::HardwareCAN *can) {
    can_ = can;
  }

  void update() {
    while (can_->available()) {
      CanMsg msg = can_->read();
      uint32_t id = msg.getStandardId();
      if (0x201 <= id && id < 0x201 + 8) {
        Param &param = params_[id - 0x201];
        int16_t position = static_cast<int16_t>(msg.data[0] << 8 | msg.data[1]);
        if (param.prev_position != INT16_MAX) {
          int16_t delta = position - param.prev_position;
          if (delta > 4096) {
            delta -= 8192;
          } else if (delta < -4096) {
            delta += 8192;
          }
          param.position += delta;
        } else {
          param.position = position;
        }
        param.prev_position = position;
        param.rpm = static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);
        param.current_raw =
          static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
      }
    }
  }

  bool transmit() {
    CanMsg msg;
    msg.id = CanStandardId(0x200);
    msg.data_length = 8;
    for (size_t i = 0; i < 4; ++i) {
      msg.data[i * 2] = params_[i].current_ref_raw >> 8;
      msg.data[i * 2 + 1] = params_[i].current_ref_raw;
    }
    if (can_->write(msg) < 0) {
      return false;
    }
    msg.id = CanStandardId(0x1FF);
    for (size_t i = 0; i < 4; ++i) {
      msg.data[i * 2] = params_[i + 4].current_ref_raw >> 8;
      msg.data[i * 2 + 1] = params_[i + 4].current_ref_raw;
    }
    return can_->write(msg) >= 0;
  }

  float getPosition(C6x0Id id) {
    auto id_ = toUnderlying(id);
    if (id_ >= toUnderlying(C620_ID_1)) {
      id_ -= toUnderlying(C620_ID_1);
    }
    return params_[id_].position / 8192.0f;
  }

  float getRpm(C6x0Id id) {
    auto id_ = toUnderlying(id);
    if (id_ >= toUnderlying(C620_ID_1)) {
      id_ -= toUnderlying(C620_ID_1);
    }
    return params_[id_].rpm;
  }

  float getCurrent(C6x0Id id) {
    auto id_ = toUnderlying(id);
    if (id_ >= toUnderlying(C620_ID_1)) {
      id_ -= toUnderlying(C620_ID_1);
      return params_[id_].current_raw / 16384.0f * 20000.0f;
    }
    return params_[id_].current_raw;
  }

  void setCurrent(C6x0Id id, float current) {
    auto id_ = toUnderlying(id);
    if (id_ >= toUnderlying(C620_ID_1)) {
      id_ -= toUnderlying(C620_ID_1);
      params_[id_].current_ref_raw = constrain(current, -10000.0f, 10000.0f);
    } else {
      params_[id_].current_ref_raw = constrain(current, -10000.0f, 10000.0f);
    }
  }

private:
  struct Param {
    int64_t position = 0;
    int16_t prev_position = INT16_MAX;
    int16_t rpm = 0;
    int16_t current_raw = 0;
    int16_t current_ref_raw = 0;
  };

  arduino::HardwareCAN *can_;
  std::array<Param, 8> params_{};

  template<class T>
  constexpr typename std::underlying_type<T>::type
  toUnderlying(T value) noexcept {
    return static_cast<typename std::underlying_type<T>::type>(value);
  }
};
