#pragma once

#include <Arduino.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <cstring>

#include <api/HardwareCAN.h>


namespace DM {

enum DM_MotorType {
  DM_MT_DM3519 = 0,
  DM_MT_DM2325 = 1,
};

enum DM_ControlMode : uint32_t {
  DM_CM_MIT = 1,
  DM_CM_POSITION = 2,
  DM_CM_VELOCITY = 3
};

enum DM_RID : uint8_t {
  DM_RID_UV_Value = 0,
  DM_RID_KT_Value = 1,
  DM_RID_OT_Value = 2,
  DM_RID_OC_Value = 3,
  DM_RID_ACC = 4,
  DM_RID_DEC = 5,
  DM_RID_MAX_SPD = 6,
  DM_RID_MST_ID = 7,
  DM_RID_ESC_ID = 8,
  DM_RID_TIMEOUT = 9,
  DM_RID_CTRL_MODE = 10,
  DM_RID_Damp = 11,
  DM_RID_Inertia = 12,
  DM_RID_hw_ver = 13,
  DM_RID_sw_ver = 14,
  DM_RID_SN = 15,
  DM_RID_NPP = 16,
  DM_RID_Rs = 17,
  DM_RID_LS = 18,
  DM_RID_Flux = 19,
  DM_RID_Gr = 20,
  DM_RID_PMAX = 21,
  DM_RID_VMAX = 22,
  DM_RID_TMAX = 23
};

enum DM_Status {
  DM_STATUS_DISABLED = 0x00,        // 無効
  DM_STATUS_ENABLED = 0x01,         // 有効
  DM_STATUS_SENSOR_ERROR = 0x05,    // センサー読み取りエラー
  DM_STATUS_PARAM_ERROR = 0x06,     // モーターパラメータ読み取りエラー
  DM_STATUS_OVER_VOLTAGE = 0x08,    // 過電圧
  DM_STATUS_UNDER_VOLTAGE = 0x09,   // 低電圧
  DM_STATUS_OVER_CURRENT = 0x0A,    // 過電流
  DM_STATUS_MOS_OVER_TEMP = 0x0B,   // MOS過熱
  DM_STATUS_COIL_OVER_TEMP = 0x0C,  // モーターコイル過熱
  DM_STATUS_COMM_LOST = 0x0D,       // 通信喪失
  DM_STATUS_OVER_LOAD = 0x0E,       // 過負荷
};

class DM2325 {
public:
  // ctor: set master and slave ids
  DM2325(arduino::HardwareCAN *can, uint32_t masterId, uint32_t slaveId)
    : can_(can), feedback_(), mappingrange_(), masterId_(masterId), slaveId_(slaveId), type_(DM_MT_DM2325), currentMode_(DM_CM_POSITION) {}

  void initialize() {
    getPMAX();
    getVMAX();
    getTMAX();
  }

  void update() {
    while (can_->available()) {
      CanMsg msg = can_->read();
      uint32_t masterid_feedback = msg.getStandardId();
      if (masterid_feedback == masterId_) {
        // uint8_t motor_id_from_feedback = (uint8_t)(msg.data[0] & 0x0F);  // lower 4 bits
        uint8_t motor_id_from_feedback = (uint8_t)(msg.data[0]);  // lower 4 bits
        if (motor_id_from_feedback != slaveId_) continue;
        feedback_.status = (uint8_t)(msg.data[0] << 4);
        uint16_t pos_raw = (uint16_t)(msg.data[1] << 8 | msg.data[2]);
        uint16_t vel_raw = (uint16_t)((msg.data[3] << 4) | (msg.data[4] >> 4));
        feedback_.torque = (uint8_t)(msg.data[5]);
        feedback_.temp_mos = (int8_t)msg.data[6];
        feedback_.temp_rotor = (int8_t)msg.data[7];

        auto uint_to_float = [](uint16_t x, float min_val, float max_val, int bits) -> float {
          float span = max_val - min_val;
          float normalized = (float)x / (float)((1 << bits) - 1);
          return normalized * span + min_val;
        };

        feedback_.position = uint_to_float(pos_raw, -getPMAX(), getPMAX(), 16);
        feedback_.velocity = uint_to_float(vel_raw, -getVMAX(), getVMAX(), 12);
      }
    }
  }

  void enableMotor() {
    // placeholder: implement CAN enable sequence
  }

  void disableMotor() {
    // placeholder: implement CAN disable sequence
  }

  // send commands (placeholders) - check mode where appropriate
  void sendMIT() {
    if (currentMode_ != DM_CM_MIT) return;  // only allowed in MIT
    // placeholder: pack and send CAN MIT packet
  }

  void sendPosition(float position_rad, float vel_limit_rad_s) {
    if (currentMode_ != DM_CM_POSITION) return;  // only allowed in POSITION
    // placeholder: pack and send position CAN packet
  }

  void sendVelocity(float velocity_rad_s) {
    if (currentMode_ != DM_CM_VELOCITY) return;  // only allowed in VELOCITY
    // placeholder: pack and send velocity CAN packet
  }

  float getPositionDeg() const {
    return feedback_.position * 180.0f / PI;
  }
  float getRPM() const {
    return feedback_.velocity * 180.0f / PI;
  }
  float getRPS() const {
    return getRPM() / 60.0f;
  }

  DM_ControlMode getMode() const {
    return currentMode_;
  }
  uint32_t getSlaveId() const {
    return slaveId_;
  }
  uint32_t getMasterId() const {
    return masterId_;
  }
  float getPMAX() {
    if (mappingrange_.pmax == 0) readParamFloat(DM_RID_PMAX, mappingrange_.pmax);
    return abs(mappingrange_.pmax);
  }
  float getVMAX() {
    if (mappingrange_.vmax == 0) readParamFloat(DM_RID_VMAX, mappingrange_.vmax);
    return abs(mappingrange_.vmax);
  }
  float getTMAX() {
    if (mappingrange_.tmax == 0) readParamFloat(DM_RID_TMAX, mappingrange_.tmax);
    return abs(mappingrange_.tmax);
  }

  bool readParamUInt32(DM_RID rid, uint32_t &out, uint32_t timeout_ms = 200) {
    if (can_ == nullptr) return false;

    CanMsg tx{};
    tx.id = CanStandardId(0x7FF);
    tx.data_length = 8;
    tx.data[0] = static_cast<uint8_t>(slaveId_ & 0xFF);
    tx.data[1] = static_cast<uint8_t>((slaveId_ >> 8) & 0xFF);
    tx.data[2] = 0x33;
    tx.data[3] = static_cast<uint8_t>(rid);
    for (int i = 4; i < 8; ++i) tx.data[i] = 0x00;

    if (can_->write(tx) < 0) return false;

    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
      if (can_->available()) {
        CanMsg rx = can_->read();
        uint32_t rid_from = rx.getStandardId();
        if (rid_from != masterId_) continue;  // only consider frames from master
        if (rx.data_length < 8) continue;
        if (rx.data[2] != 0x33) continue;
        if (rx.data[3] != static_cast<uint8_t>(rid)) continue;
        // data bytes 4..7 are little-endian uint32
        uint32_t val = (uint32_t)rx.data[4] | ((uint32_t)rx.data[5] << 8) | ((uint32_t)rx.data[6] << 16) | ((uint32_t)rx.data[7] << 24);
        out = val;
        return true;
      }
    }
    return false;
  }

  bool readParamFloat(DM_RID rid, float &out, uint32_t timeout_ms = 200) {
    uint32_t raw;
    if (!readParamUInt32(rid, raw, timeout_ms)) return false;
    float f;
    ::memcpy(&f, &raw, sizeof(float));
    out = f;
    return true;
  }

private:
  struct Feedback {
    uint8_t slaveid_lower4bit = 1;  //下位4bit (0~255)の範囲でしかESCを見分けられないので、slaveidはこの範囲内に抑えるのが良さそう
    uint8_t status = 0;
    float position = 0.0f;  //rad   電源投入時0.0rad 減速後の出力軸の位置を表す
    float velocity = 0.0f;  //rad/s 減速後の出力軸の速度を表す
    float torque = 0.0f;    //Nm    減速後の出力軸のトルクを表す
    int8_t temp_mos = 0;    //℃    ESCのMOS-FET温度
    int8_t temp_rotor = 0;  //℃    ローターのコイル温度
  };

  struct MappingRange {
    float pmax = 0.0f;  //デフォルト値 12.5f  2325と3519で一緒
    float vmax = 0.0f;  //デフォルト値 200.0f
    float tmax = 0.0f;  //デフォルト値 10.0f
  };

  arduino::HardwareCAN *can_;
  Feedback feedback_;
  MappingRange mappingrange_;
  uint32_t masterId_ = 0;
  uint32_t slaveId_ = 1;
  DM_MotorType type_ = DM_MT_DM2325;
  DM_ControlMode currentMode_ = DM_CM_POSITION;
};
}  // namespace DM
