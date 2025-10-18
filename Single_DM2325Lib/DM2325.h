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
  DM_RID_CTRL_MODE = 10,  //MIT, POS, VEL
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
  DM_RID_PMAX = 21,  //フィードバック読むのに必要
  DM_RID_VMAX = 22,  //フィードバック読むのに必要
  DM_RID_TMAX = 23,  //フィードバック読むのに必要
  DM_RID_T_I_BW = 24,
  DM_RID_T_KP_ASR = 25,
  DM_RID_T_KI_ASR = 26,
  DM_RID_T_KP_APR = 27,
  DM_RID_T_KI_APR = 28,
  DM_RID_T_OV_Value = 29,
  DM_RID_T_GREF = 30,
  DM_RID_T_Deta = 31,
  DM_RID_T_V_BW = 32,
  DM_RID_T_IQ_c1 = 33,
  DM_RID_T_VL_c1 = 34,
  DM_RID_T_can_br = 35,  //CANボーレート
  DM_RID_T_sub_ver = 36,
  DM_RID_T_u_off = 50,
  DM_RID_T_v_off = 51,
  DM_RID_T_k1 = 52,
  DM_RID_T_k2 = 53,
  DM_RID_T_m_off = 54,
  DM_RID_T_dir = 55,
  DM_RID_T_p_m = 80,
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
        uint8_t motor_id_from_feedback = (uint8_t)(msg.data[0] & 0x0F);  // lower 4 bits: slave id
        if (motor_id_from_feedback != slaveId_) continue;
        feedback_.status = static_cast<uint8_t>((msg.data[0] >> 4) & 0x0F);
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
    CanMsg txMsg = {};
    txMsg.id = slaveId_;
    txMsg.data_length = 8;
    for (int i = 0; i < 7; i++) { txMsg.data[i] = 0xFF; }
    txMsg.data[7] = 0xFC;
    can_->write(txMsg);
    delay(100);
  }

  void disableMotor() {
    // placeholder: implement CAN disable sequence
  }

  void setControlMode(DM_ControlMode mode) {
    currentMode_ = mode;
    sendParamUInt32(DM_RID_CTRL_MODE, static_cast<uint32_t>(mode));
    return;
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

  bool sendVelocityRPS(float velocity_rps) {
    return sendVelocityRPM(velocity_rps * 60);
  }

  bool sendVelocityRPM(float velocity_rpm) {
    return sendVelocity(velocity_rpm * PI * 2 / 60.0f);
  }

  bool sendVelocity(float velocity_rad_s) {
    CanMsg tx = {};
    uint32_t raw;
    ::memcpy(&raw, &velocity_rad_s, sizeof(raw));

    tx.id = 0x200 + slaveId_;
    tx.data_length = 4;
    tx.data[0] = static_cast<uint8_t>(raw & 0xFF);
    tx.data[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    tx.data[2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    tx.data[3] = static_cast<uint8_t>((raw >> 24) & 0xFF);

    return can_->write(tx) >= 0;
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
  uint16_t getTorque() const {
    return feedback_.torque;
  }
  uint16_t getMOSTemp() const {
    return feedback_.temp_mos;
  }
  uint16_t getRotorTemp() const {
    return feedback_.temp_rotor;
  }

  DM_ControlMode getMode() {
    uint32_t currentMode = 0;
    readParamUInt32(DM_RID_CTRL_MODE, currentMode);
    return static_cast<DM_ControlMode>(currentMode);
  }
  uint32_t getSlaveId() const {
    return slaveId_;
  }
  uint32_t getMasterId() const {
    return masterId_;
  }
  uint8_t getStatus() const {
    return feedback_.status;
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

  // write a uint32 parameter: send D2=0x55 request and wait for MST_ID response (D2=0x33)
  // This version returns only success/failure (no out parameter).
  bool sendParamUInt32(DM_RID rid, uint32_t value_to_write, uint32_t timeout_ms = 200) {
    if (can_ == nullptr) return false;

    CanMsg tx{};
    tx.id = CanStandardId(0x7FF);
    tx.data_length = 8;
    tx.data[0] = static_cast<uint8_t>(slaveId_ & 0xFF);
    tx.data[1] = static_cast<uint8_t>((slaveId_ >> 8) & 0xFF);
    tx.data[2] = 0x55;  // write command
    tx.data[3] = static_cast<uint8_t>(rid);
    // payload little-endian
    tx.data[4] = static_cast<uint8_t>(value_to_write & 0xFF);
    tx.data[5] = static_cast<uint8_t>((value_to_write >> 8) & 0xFF);
    tx.data[6] = static_cast<uint8_t>((value_to_write >> 16) & 0xFF);
    tx.data[7] = static_cast<uint8_t>((value_to_write >> 24) & 0xFF);

    if (can_->write(tx) < 0) return false;

    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
      if (can_->available()) {
        CanMsg rx = can_->read();
        uint32_t rid_from = rx.getStandardId();
        if (rid_from != masterId_) continue;  // only consider frames from master
        if (rx.data_length < 8) continue;
        if (rx.data[2] != 0x33) continue;  // response marker
        if (rx.data[3] != static_cast<uint8_t>(rid)) continue;
        // data bytes 4..7 are little-endian uint32 (returned value or original reg)
        uint32_t val = (uint32_t)rx.data[4] | ((uint32_t)rx.data[5] << 8) | ((uint32_t)rx.data[6] << 16) | ((uint32_t)rx.data[7] << 24);
        // if the returned value equals requested value, write succeeded
        bool success = (val == value_to_write);
        // update cached mappingrange if writing PMAX/VMAX/TMAX
        if (success) {
          float vf;
          ::memcpy(&vf, &val, sizeof(vf));
          if (rid == DM_RID_PMAX) mappingrange_.pmax = vf;
          if (rid == DM_RID_VMAX) mappingrange_.vmax = vf;
          if (rid == DM_RID_TMAX) mappingrange_.tmax = vf;
        }
        return success;
      }
    }
    return false;
  }

  bool sendParamFloat(DM_RID rid, float value_to_write, uint32_t timeout_ms = 200) {
    uint32_t raw_value;
    ::memcpy(&raw_value, &value_to_write, sizeof(uint32_t));
    return sendParamUInt32(rid, raw_value, timeout_ms);
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
