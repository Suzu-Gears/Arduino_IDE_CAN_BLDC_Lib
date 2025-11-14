#pragma once

#include <Arduino.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <cstring>

#include <api/HardwareCAN.h>


// Default Mapping Ranges
/* DM4310~DMG6220 from https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py
  PMAX    VMAX    TMAX    Model
  12.5f,  30.0f,  10.0f,  DM4310
  12.5f,  50.0f,  10.0f,  DM4310_48V
  12.5f,   8.0f,  28.0f,  DM4340
  12.5f,  10.0f,  28.0f,  DM4340_48V
  12.5f,  45.0f,  20.0f,  DM6006
  12.5f,  45.0f,  40.0f,  DM8006
  12.5f,  45.0f,  54.0f,  DM8009
  12.5f,  25.0f, 200.0f,  DM10010L
  12.5f,  20.0f, 200.0f,  DM10010
  12.5f, 280.0f,   1.0f,  DMH3510
  12.5f,  45.0f,  10.0f,  DMH6215
  12.5f,  45.0f,  10.0f,  DMG6220
  12.5f, 200.0f,  10.0f,  DM3519
  12.5f, 200.0f,  10.0f,  DM2325
*/

enum class DM_ControlMode : uint32_t {
  DM_CM_MIT = 1,
  DM_CM_POS_VEL = 2,
  DM_CM_VEL = 3,
};

enum class DM_RID : uint8_t {
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
  DM_RID_CTRL_MODE = 10,  //MIT, POS_VEL, VELのモード切り替え
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

enum class DM_Status : uint8_t {
  DM_STATUS_DISABLED = 0x00,        // モーター無効
  DM_STATUS_ENABLED = 0x01,         // モーター有効
  DM_STATUS_SENSOR_ERROR = 0x05,    // センサー読み取りエラー
  DM_STATUS_PARAM_ERROR = 0x06,     // モーターパラメータ読み取りエラー
  DM_STATUS_OVER_VOLTAGE = 0x08,    // 過電圧 60V
  DM_STATUS_UNDER_VOLTAGE = 0x09,   // 低電圧 15V
  DM_STATUS_OVER_CURRENT = 0x0A,    // 過電流 13A
  DM_STATUS_MOS_OVER_TEMP = 0x0B,   // MOS過熱
  DM_STATUS_COIL_OVER_TEMP = 0x0C,  // モーターコイル過熱
  DM_STATUS_COMM_LOST = 0x0D,       // 通信喪失
  DM_STATUS_OVER_LOAD = 0x0E,       // 過負荷
};

class DMMotor {
public:
  // Constants for message parsing, made public for use by DMManager
  static constexpr uint8_t FEEDBACK_SLAVE_ID_MASK = 0x0F;
  static constexpr int FEEDBACK_STATUS_SHIFT = 4;
  static constexpr uint8_t FEEDBACK_STATUS_MASK = 0x0F;

  DMMotor(arduino::HardwareCAN *can, uint32_t masterId, uint32_t slaveId, DM_ControlMode mode)
    : can_(can), masterId_(masterId), slaveId_(slaveId), currentMode_(mode) {}

  void initialize() {
    bool pmax_ok = readParamFloat(DM_RID::DM_RID_PMAX, mappingrange_.pmax);
    bool vmax_ok = readParamFloat(DM_RID::DM_RID_VMAX, mappingrange_.vmax);
    bool tmax_ok = readParamFloat(DM_RID::DM_RID_TMAX, mappingrange_.tmax);

    if (pmax_ok && vmax_ok && tmax_ok) {
      setControlMode(currentMode_);
      is_initialized_ = true;
    } else {
      is_initialized_ = false;
      // NOTE: Consider logging an error here, e.g., via Serial.println("DM Motor Init Failed")
    }
  }

  void update() {
    if (!is_initialized_) return;
    while (can_->available()) {
      processMessage(can_->read());
    }
  }

  void processMessage(const CanMsg &msg) {
    if (!is_initialized_) return;

    if (msg.getStandardId() == masterId_) {
      uint8_t motor_id_from_feedback = (uint8_t)(msg.data[0] & FEEDBACK_SLAVE_ID_MASK);
      if (motor_id_from_feedback == slaveId_) {
        feedback_.status = static_cast<DM_Status>((msg.data[0] >> FEEDBACK_STATUS_SHIFT) & FEEDBACK_STATUS_MASK);
        uint16_t pos_raw = (uint16_t)(msg.data[1] << 8 | msg.data[2]);
        uint16_t vel_raw = (uint16_t)((msg.data[3] << VEL_RAW_SHIFT) | (msg.data[4] >> VEL_RAW_SHIFT));
        uint16_t torque_raw = (uint16_t)(((msg.data[4] & VEL_RAW_LOWER_MASK) << TORQUE_RAW_SHIFT) | msg.data[5]);
        feedback_.temp_mos = (int8_t)msg.data[6];
        feedback_.temp_rotor = (int8_t)msg.data[7];

        feedback_.position = uintToFloat(pos_raw, -getPMAX(), getPMAX(), 16);
        feedback_.velocity = uintToFloat(vel_raw, -getVMAX(), getVMAX(), 12);
        feedback_.torque = uintToFloat(torque_raw, -getTMAX(), getTMAX(), 12);
      }
    }
  }

  bool enable() {
    return sendSystemCommand(CMD_BYTE_ENABLE);
  }

  bool disable() {
    return sendSystemCommand(CMD_BYTE_DISABLE);
  }

  bool setZeroPoint() {
    return sendSystemCommand(CMD_BYTE_SET_ZERO);
  }

  void setControlMode(DM_ControlMode mode) {
    currentMode_ = mode;
    sendParamUInt32(DM_RID::DM_RID_CTRL_MODE, static_cast<uint32_t>(mode));
  }

  bool sendMIT(float position_rad, float velocity_rad_s, float kp, float kd, float torque_ff) {
    if (!is_initialized_ || currentMode_ != DM_ControlMode::DM_CM_MIT) return false;

    uint16_t position_raw = floatToUint(position_rad, -getPMAX(), getPMAX(), 16);
    uint16_t velocity_raw = floatToUint(velocity_rad_s, -getVMAX(), getVMAX(), 12);
    uint16_t kp_raw = floatToUint(kp, 0.0f, 500.0f, 12);
    uint16_t kd_raw = floatToUint(kd, 0.0f, 5.0f, 12);
    uint16_t torque_raw = floatToUint(torque_ff, -getTMAX(), getTMAX(), 12);

    CanMsg tx = {};
    tx.id = slaveId_;
    tx.data_length = 8;
    tx.data[0] = static_cast<uint8_t>(position_raw >> 8);
    tx.data[1] = static_cast<uint8_t>(position_raw & 0xFF);
    tx.data[2] = static_cast<uint8_t>(velocity_raw >> MIT_VEL_SHIFT);
    tx.data[3] = static_cast<uint8_t>((velocity_raw & MIT_VEL_LOWER_MASK) << MIT_VEL_SHIFT) | (kp_raw >> MIT_KP_SHIFT);
    tx.data[4] = static_cast<uint8_t>(kp_raw & 0xFF);
    tx.data[5] = static_cast<uint8_t>(kd_raw >> MIT_KD_SHIFT);
    tx.data[6] = static_cast<uint8_t>((kd_raw & MIT_KD_LOWER_MASK) << MIT_KD_SHIFT) | (torque_raw >> MIT_TORQUE_SHIFT);
    tx.data[7] = static_cast<uint8_t>(torque_raw & 0xFF);

    return can_->write(tx) >= 0;
  }

  bool sendPosition(float position_rad, float velocity_limit_rad_s) {
    if (!is_initialized_ || currentMode_ != DM_ControlMode::DM_CM_POS_VEL) return false;

    uint32_t position_raw, velocity_raw;
    ::memcpy(&position_raw, &position_rad, sizeof(position_raw));
    ::memcpy(&velocity_raw, &velocity_limit_rad_s, sizeof(velocity_raw));

    CanMsg tx = {};
    tx.id = POS_VEL_ID_OFFSET + slaveId_;
    tx.data_length = 8;
    tx.data[0] = static_cast<uint8_t>(position_raw & 0xFF);
    tx.data[1] = static_cast<uint8_t>((position_raw >> 8) & 0xFF);
    tx.data[2] = static_cast<uint8_t>((position_raw >> 16) & 0xFF);
    tx.data[3] = static_cast<uint8_t>((position_raw >> 24) & 0xFF);
    tx.data[4] = static_cast<uint8_t>(velocity_raw & 0xFF);
    tx.data[5] = static_cast<uint8_t>((velocity_raw >> 8) & 0xFF);
    tx.data[6] = static_cast<uint8_t>((velocity_raw >> 16) & 0xFF);
    tx.data[7] = static_cast<uint8_t>((velocity_raw >> 24) & 0xFF);

    return can_->write(tx) >= 0;
  }

  bool sendVelocityRPM(float velocity_rpm) {
    return sendVelocity(velocity_rpm * PI / 30.0f);
  }

  bool sendVelocityRPS(float velocity_rps) {
    return sendVelocity(velocity_rps * PI * 2.0f);
  }

  bool sendVelocity(float velocity_rad_s) {
    if (!is_initialized_ || currentMode_ != DM_ControlMode::DM_CM_VEL) return false;

    uint32_t raw;
    ::memcpy(&raw, &velocity_rad_s, sizeof(raw));

    CanMsg tx = {};
    tx.id = VEL_ID_OFFSET + slaveId_;
    tx.data_length = 4;
    tx.data[0] = static_cast<uint8_t>(raw & 0xFF);
    tx.data[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    tx.data[2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    tx.data[3] = static_cast<uint8_t>((raw >> 24) & 0xFF);

    return can_->write(tx) >= 0;
  }

  float getPosition() const {
    return feedback_.position;
  }
  float getPositionDeg() const {
    return feedback_.position * 180.0f / PI;
  }
  float getVelocity() const {
    return feedback_.velocity;
  }
  float getRPM() const {
    return feedback_.velocity * 30.0f / PI;
  }
  float getRPS() const {
    return feedback_.velocity / (2.0f * PI);
  }
  float getTorque() const {
    return feedback_.torque;
  }
  int8_t getMOSTemp() const {
    return feedback_.temp_mos;
  }
  int8_t getRotorTemp() const {
    return feedback_.temp_rotor;
  }

  DM_ControlMode getMode() {
    uint32_t currentModeVal = 0;
    readParamUInt32(DM_RID::DM_RID_CTRL_MODE, currentModeVal);
    return static_cast<DM_ControlMode>(currentModeVal);
  }
  uint32_t getSlaveId() const {
    return slaveId_;
  }
  uint32_t getMasterId() const {
    return masterId_;
  }
  DM_Status getStatus() const {
    return feedback_.status;
  }

  float getPMAX() const {
    return abs(mappingrange_.pmax);
  }
  float getVMAX() const {
    return abs(mappingrange_.vmax);
  }
  float getTMAX() const {
    return abs(mappingrange_.tmax);
  }

  bool readParamUInt32(DM_RID rid, uint32_t &out, uint32_t timeout_ms = 200) {
    if (can_ == nullptr) return false;

    CanMsg tx{};
    tx.id = CanStandardId(PARAM_CAN_ID);
    tx.data_length = 8;
    tx.data[0] = static_cast<uint8_t>(slaveId_ & 0xFF);
    tx.data[1] = static_cast<uint8_t>((slaveId_ >> 8) & 0xFF);
    tx.data[2] = PARAM_CMD_READ;
    tx.data[3] = static_cast<uint8_t>(rid);
    std::fill(tx.data + 4, tx.data + 8, 0x00);

    if (can_->write(tx) < 0) return false;

    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
      if (can_->available()) {
        CanMsg rx = can_->read();
        if (rx.getStandardId() != masterId_) continue;
        if (rx.data_length < 8) continue;
        if (rx.data[2] != PARAM_CMD_READ) continue;
        if (rx.data[3] != static_cast<uint8_t>(rid)) continue;

        out = (uint32_t)rx.data[4] | ((uint32_t)rx.data[5] << 8) | ((uint32_t)rx.data[6] << 16) | ((uint32_t)rx.data[7] << 24);
        return true;
      }
    }
    return false;
  }

  bool readParamFloat(DM_RID rid, float &out, uint32_t timeout_ms = 200) {
    uint32_t raw;
    if (!readParamUInt32(rid, raw, timeout_ms)) return false;
    ::memcpy(&out, &raw, sizeof(float));
    return true;
  }

  bool sendParamUInt32(DM_RID rid, uint32_t value_to_write, uint32_t timeout_ms = 200) {
    if (can_ == nullptr) return false;

    CanMsg tx{};
    tx.id = CanStandardId(PARAM_CAN_ID);
    tx.data_length = 8;
    tx.data[0] = static_cast<uint8_t>(slaveId_ & 0xFF);
    tx.data[1] = static_cast<uint8_t>((slaveId_ >> 8) & 0xFF);
    tx.data[2] = PARAM_CMD_WRITE;
    tx.data[3] = static_cast<uint8_t>(rid);
    tx.data[4] = static_cast<uint8_t>(value_to_write & 0xFF);
    tx.data[5] = static_cast<uint8_t>((value_to_write >> 8) & 0xFF);
    tx.data[6] = static_cast<uint8_t>((value_to_write >> 16) & 0xFF);
    tx.data[7] = static_cast<uint8_t>((value_to_write >> 24) & 0xFF);

    if (can_->write(tx) < 0) return false;

    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
      if (can_->available()) {
        CanMsg rx = can_->read();
        if (rx.getStandardId() != masterId_) continue;
        if (rx.data_length < 8) continue;
        if (rx.data[2] != PARAM_CMD_READ) continue;  // Note: device responds with a READ command
        if (rx.data[3] != static_cast<uint8_t>(rid)) continue;

        uint32_t val = (uint32_t)rx.data[4] | ((uint32_t)rx.data[5] << 8) | ((uint32_t)rx.data[6] << 16) | ((uint32_t)rx.data[7] << 24);
        bool success = (val == value_to_write);

        if (success) {
          float vf;
          ::memcpy(&vf, &val, sizeof(vf));
          if (rid == DM_RID::DM_RID_PMAX) mappingrange_.pmax = vf;
          if (rid == DM_RID::DM_RID_VMAX) mappingrange_.vmax = vf;
          if (rid == DM_RID::DM_RID_TMAX) mappingrange_.tmax = vf;
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
    DM_Status status = DM_Status::DM_STATUS_DISABLED;  // Current status of the motor
    float position = 0.0f;                             //rad   電源投入時0.0rad 減速後の出力軸の位置を表す
    float velocity = 0.0f;                             //rad/s 減速後の出力軸の速度を表す
    float torque = 0.0f;                               //Nm    減速後の出力軸のトルクを表す
    int8_t temp_mos = 0;                               //℃    ESCのMOS-FET温度
    int8_t temp_rotor = 0;                             //℃    ローターのコイル温度
  };

  struct MappingRange {
    float pmax = 0.0f;
    float vmax = 0.0f;
    float tmax = 0.0f;
  };

  // Constants
  static constexpr uint8_t CMD_BYTE_ENABLE = 0xFC;
  static constexpr uint8_t CMD_BYTE_DISABLE = 0xFD;
  static constexpr uint8_t CMD_BYTE_SET_ZERO = 0xFE;
  static constexpr uint8_t CMD_DATA_PADDING = 0xFF;

  static constexpr uint32_t PARAM_CAN_ID = 0x7FF;
  static constexpr uint8_t PARAM_CMD_READ = 0x33;
  static constexpr uint8_t PARAM_CMD_WRITE = 0x55;

  static constexpr uint32_t POS_VEL_ID_OFFSET = 0x100;
  static constexpr uint32_t VEL_ID_OFFSET = 0x200;

  static constexpr int VEL_RAW_SHIFT = 4;
  static constexpr uint16_t VEL_RAW_LOWER_MASK = 0x0F;
  static constexpr int TORQUE_RAW_SHIFT = 8;

  static constexpr int MIT_VEL_SHIFT = 4;
  static constexpr uint16_t MIT_VEL_LOWER_MASK = 0x0F;
  static constexpr int MIT_KP_SHIFT = 8;
  static constexpr int MIT_KD_SHIFT = 4;
  static constexpr uint16_t MIT_KD_LOWER_MASK = 0x0F;
  static constexpr int MIT_TORQUE_SHIFT = 8;

  // Member variables
  arduino::HardwareCAN *can_;
  Feedback feedback_{};
  MappingRange mappingrange_{};
  uint32_t masterId_;
  uint32_t slaveId_;
  DM_ControlMode currentMode_;
  bool is_initialized_ = false;

  bool sendSystemCommand(uint8_t commandByte) {
    CanMsg tx = {};
    tx.id = slaveId_;
    tx.data_length = 8;
    std::fill(tx.data, tx.data + 7, CMD_DATA_PADDING);
    tx.data[7] = commandByte;
    return can_->write(tx) >= 0;
  }

  static float uintToFloat(uint16_t x, float min_val, float max_val, int bits) {
    float span = max_val - min_val;
    float normalized = (float)x / (float)((1 << bits) - 1);
    return normalized * span + min_val;
  }

  static uint16_t floatToUint(float x, float min_val, float max_val, int bits) {
    float span = max_val - min_val;
    float clamped_x = std::clamp(x, min_val, max_val);
    float normalized = (clamped_x - min_val) / span;
    return (uint16_t)(normalized * ((1 << bits) - 1));
  }
};
