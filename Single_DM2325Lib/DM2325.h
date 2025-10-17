#ifndef DM2325_H
#define DM2325_H

#include <Arduino.h>
#include <RP2040PIO_CAN.h>
#include <array>

// Single-header, header-only DM / DM Motor Controller
// This file combines the DMMotorController class implementation
// and provides a DM2325 multi-manager wrapper where useful.

// --- DMMotorController class (from DMMotorController.{h,cpp}) ---
struct MotorFeedback {
  float position_deg;
  float velocity_rpm;
  float torque_nm;
  int8_t temp_mos;
  int8_t temp_rotor;
  uint8_t status;
};

class DMMotorController {
public:
  DMMotorController(byte slaveId, RP2040PIO_CAN &can)
    : _slaveId(slaveId), _can(&can), _currentMode(MIT) {}

  void begin() {
    enableMotor();
  }

  enum ControlMode {
    MIT = 1,
    POSITION = 2,
    VELOCITY = 3
  };

  void setMode(ControlMode mode) {
    _currentMode = mode;
    switchControlMode(mode);
  }
  ControlMode getMode() {
    return _currentMode;
  }

  void goToPosition(float target_rad, float speed_rpm) {
    float velocity_limit_rad_s = speed_rpm * (PI / 30.0f);
    sendPositionCommand(target_rad, velocity_limit_rad_s);
  }

  void setVelocityRPM(float target_rpm) {
    float velocity_rad_s = target_rpm * (PI / 30.0f);
    sendVelocityCommand(velocity_rad_s);
  }

  void setMIT(float p_des, float v_des, float kp, float kd, float t_ff) {
    sendMITCommand(p_des, v_des, kp, kd, t_ff);
  }

  void playStartupBeep(int frequency, int duration_ms) {
    long period_us = 1000000 / frequency;
    long half_period_us = period_us / 2;
    long iterations = (long)duration_ms * 1000 / period_us;

    ControlMode original_mode = _currentMode;
    switchControlMode((uint8_t)3);
    delay(10);

    for (long i = 0; i < iterations; i++) {
      sendVelocityCommand(0.5f);
      delayMicroseconds(half_period_us);
      sendVelocityCommand(-0.5f);
      delayMicroseconds(half_period_us);
    }
    sendVelocityCommand(0.0f);

    delay(10);
    switchControlMode((uint8_t)original_mode);
  }

  bool readFeedback(struct MotorFeedback &feedback) {
    if (_can->available() > 0) {
      CanMsg rxMsg = _can->read();
      if (rxMsg.id == 0x000) {
        feedback.status = rxMsg.data[0];
        uint16_t pos_raw = (uint16_t)(rxMsg.data[1] << 8 | rxMsg.data[2]);
        uint16_t vel_raw = (uint16_t)((rxMsg.data[3] << 4) | (rxMsg.data[4] >> 4));
        uint16_t tor_raw = (uint16_t)(((rxMsg.data[4] & 0x0F) << 8) | rxMsg.data[5]);

        float position_rad = uint_to_float(pos_raw, -P_MAX, P_MAX, 16);

        feedback.position_deg = position_rad * 180.0f / PI;
        feedback.velocity_rpm = uint_to_float(vel_raw, -V_MAX, V_MAX, 12) * 30.0f / PI;
        feedback.torque_nm = uint_to_float(tor_raw, -T_MAX, T_MAX, 12);
        feedback.temp_mos = (int8_t)rxMsg.data[6];
        feedback.temp_rotor = (int8_t)rxMsg.data[7];
        return true;
      }
    }
    return false;
  }

  void enableMotor() {
    CanMsg txMsg = {};
    txMsg.id = _slaveId;
    txMsg.data_length = 8;
    for (int i = 0; i < 7; i++) { txMsg.data[i] = 0xFF; }
    txMsg.data[7] = 0xFC;
    _can->write(txMsg);
    delay(100);
  }

private:
  struct MotorFeedback {
    float position_deg;
    float velocity_rpm;
    float torque_nm;
    int8_t temp_mos;
    int8_t temp_rotor;
    uint8_t status;
  };

  void switchControlMode(uint8_t mode_code) {
    CanMsg txMsg = {};
    txMsg.id = 0x7FF;
    txMsg.data_length = 8;
    txMsg.data[0] = _slaveId;
    txMsg.data[1] = 0x00;
    txMsg.data[2] = 0x55;
    txMsg.data[3] = 0x0A;
    txMsg.data[4] = mode_code;
    for (int i = 5; i < 8; i++) { txMsg.data[i] = 0x00; }
    _can->write(txMsg);
    delay(10);
  }

  void sendVelocityCommand(float velocity_rad_s) {
    CanMsg txMsg = {};
    union {
      float f;
      byte b[4];
    } converter;
    converter.f = velocity_rad_s;

    txMsg.id = 0x200 + _slaveId;
    txMsg.data_length = 8;
    for (int i = 0; i < 4; i++) { txMsg.data[i] = converter.b[i]; }
    for (int i = 4; i < 8; i++) { txMsg.data[i] = 0; }
    _can->write(txMsg);
  }

  void sendPositionCommand(float position_rad, float velocity_limit_rad_s) {
    CanMsg txMsg = {};
    union {
      float f;
      byte b[4];
    } pos_conv, vel_conv;
    pos_conv.f = position_rad;
    vel_conv.f = velocity_limit_rad_s;

    txMsg.id = 0x100 + _slaveId;
    txMsg.data_length = 8;
    for (int i = 0; i < 4; i++) { txMsg.data[i] = pos_conv.b[i]; }
    for (int i = 0; i < 4; i++) { txMsg.data[i + 4] = vel_conv.b[i]; }
    _can->write(txMsg);
  }

  void sendMITCommand(float p_des, float v_des, float kp, float kd, float t_ff) {
    CanMsg txMsg = {};
    txMsg.id = _slaveId;
    txMsg.data_length = 8;

    uint16_t p_uint = float_to_uint(p_des, -P_MAX, P_MAX, 16);
    uint16_t v_uint = float_to_uint(v_des, -V_MAX, V_MAX, 12);
    uint16_t kp_uint = float_to_uint(kp, 0, KP_MAX, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, KD_MAX, 12);
    uint16_t t_uint = float_to_uint(t_ff, -T_MAX, T_MAX, 12);

    txMsg.data[0] = p_uint >> 8;
    txMsg.data[1] = p_uint & 0xFF;
    txMsg.data[2] = v_uint >> 4;
    txMsg.data[3] = ((v_uint & 0x0F) << 4) | (kp_uint >> 8);
    txMsg.data[4] = kp_uint & 0xFF;
    txMsg.data[5] = kd_uint >> 4;
    txMsg.data[6] = ((kd_uint & 0x0F) << 4) | (t_uint >> 8);
    txMsg.data[7] = t_uint & 0xFF;

    _can->write(txMsg);
  }

  static float uint_to_float(uint16_t x, float min_val, float max_val, int bits) {
    float span = max_val - min_val;
    float normalized = (float)x / (float)((1 << bits) - 1);
    return normalized * span + min_val;
  }

  static uint16_t float_to_uint(float x, float min_val, float max_val, int bits) {
    float span = max_val - min_val;
    float clamped_x = (x < min_val) ? min_val : ((x > max_val) ? max_val : x);
    float normalized = (clamped_x - min_val) / span;
    return (uint16_t)(normalized * ((1 << bits) - 1));
  }

  RP2040PIO_CAN *_can;
  byte _slaveId;
  ControlMode _currentMode;

  static const float P_MAX;
  static const float V_MAX;
  static const float T_MAX;
  static const float KP_MAX;
  static const float KD_MAX;
};

// Motor manager DM2325 (single-header simplified rewrite)
enum DM2325Id {
  DM2325_ID_1 = 0,
  DM2325_ID_2,
  DM2325_ID_3,
  DM2325_ID_4,
  DM2325_ID_5,
  DM2325_ID_6,
  DM2325_ID_7,
  DM2325_ID_8,
};

class DM2325 {
public:
  DM2325() = default;
  void setCAN(RP2040PIO_CAN *can) {
    can_ = can;
  }

  void update() {
    if (!can_) return;
    while (can_->available() > 0) {
      CanMsg msg = can_->read();
      uint32_t id = msg.id;
      if (0x200 <= id && id < 0x200 + 8) {
        Param &param = params_[id - 0x200];

        int16_t pos_raw = (int16_t)(msg.data[0] << 8 | msg.data[1]);
        if (param.prev_position != INT16_MAX) {
          int16_t delta = pos_raw - param.prev_position;
          if (delta > 4096) delta -= 8192;
          else if (delta < -4096) delta += 8192;
          param.position_total += delta;
        } else {
          param.position_total = pos_raw;
        }
        param.prev_position = pos_raw;

        uint16_t vel_raw = (uint16_t)((msg.data[3] << 4) | (msg.data[4] >> 4));
        uint16_t tor_raw = (uint16_t)(((msg.data[4] & 0x0F) << 8) | msg.data[5]);

        param.position_rad = uint_to_float((uint16_t)pos_raw, -P_MAX, P_MAX, 16);
        param.velocity_rpm = uint_to_float(vel_raw, -V_MAX, V_MAX, 12) * 30.0f / PI;
        param.torque_nm = uint_to_float(tor_raw, -T_MAX, T_MAX, 12);
        param.temp_mos = (int8_t)msg.data[6];
        param.temp_rotor = (int8_t)msg.data[7];
        param.status = msg.data[0];
      }
    }
  }

  void sendVelocity(uint8_t slaveId, float velocity_rad_s) {
    if (!can_) return;
    CanMsg txMsg = {};
    union {
      float f;
      byte b[4];
    } converter;
    converter.f = velocity_rad_s;
    txMsg.id = 0x200 + slaveId;
    txMsg.data_length = 8;
    for (int i = 0; i < 4; i++) txMsg.data[i] = converter.b[i];
    for (int i = 4; i < 8; i++) txMsg.data[i] = 0;
    can_->write(txMsg);
  }

  void sendPosition(uint8_t slaveId, float position_rad, float velocity_limit_rad_s) {
    if (!can_) return;
    CanMsg txMsg = {};
    union {
      float f;
      byte b[4];
    } pos_conv, vel_conv;
    pos_conv.f = position_rad;
    vel_conv.f = velocity_limit_rad_s;
    txMsg.id = 0x100 + slaveId;
    txMsg.data_length = 8;
    for (int i = 0; i < 4; i++) txMsg.data[i] = pos_conv.b[i];
    for (int i = 0; i < 4; i++) txMsg.data[i + 4] = vel_conv.b[i];
    can_->write(txMsg);
  }

  void sendMIT(uint8_t slaveId, float p_des, float v_des, float kp, float kd, float t_ff) {
    if (!can_) return;
    CanMsg txMsg = {};
    txMsg.id = slaveId;
    txMsg.data_length = 8;

    uint16_t p_uint = float_to_uint(p_des, -P_MAX, P_MAX, 16);
    uint16_t v_uint = float_to_uint(v_des, -V_MAX, V_MAX, 12);
    uint16_t kp_uint = float_to_uint(kp, 0, KP_MAX, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, KD_MAX, 12);
    uint16_t t_uint = float_to_uint(t_ff, -T_MAX, T_MAX, 12);

    txMsg.data[0] = p_uint >> 8;
    txMsg.data[1] = p_uint & 0xFF;
    txMsg.data[2] = v_uint >> 4;
    txMsg.data[3] = ((v_uint & 0x0F) << 4) | (kp_uint >> 8);
    txMsg.data[4] = kp_uint & 0xFF;
    txMsg.data[5] = kd_uint >> 4;
    txMsg.data[6] = ((kd_uint & 0x0F) << 4) | (t_uint >> 8);
    txMsg.data[7] = t_uint & 0xFF;

    can_->write(txMsg);
  }

  void enableMotor(uint8_t slaveId) {
    if (!can_) return;
    CanMsg txMsg = {};
    txMsg.id = slaveId;
    txMsg.data_length = 8;
    for (int i = 0; i < 7; i++) txMsg.data[i] = 0xFF;
    txMsg.data[7] = 0xFC;
    can_->write(txMsg);
    delay(100);
  }

  void switchControlMode(uint8_t slaveId, uint8_t mode_code) {
    if (!can_) return;
    CanMsg txMsg = {};
    txMsg.id = 0x7FF;
    txMsg.data_length = 8;
    txMsg.data[0] = slaveId;
    txMsg.data[1] = 0x00;
    txMsg.data[2] = 0x55;
    txMsg.data[3] = 0x0A;
    txMsg.data[4] = mode_code;
    for (int i = 5; i < 8; i++) txMsg.data[i] = 0x00;
    can_->write(txMsg);
    delay(10);
  }

  // getters
  float getPositionRad(DM2325Id id) {
    return params_[toIndex(id)].position_rad;
  }
  float getPositionTurns(DM2325Id id) {
    return params_[toIndex(id)].position_total / 8192.0f;
  }
  float getRpm(DM2325Id id) {
    return params_[toIndex(id)].velocity_rpm;
  }
  float getTorque(DM2325Id id) {
    return params_[toIndex(id)].torque_nm;
  }
  int8_t getTempMos(DM2325Id id) {
    return params_[toIndex(id)].temp_mos;
  }
  int8_t getTempRotor(DM2325Id id) {
    return params_[toIndex(id)].temp_rotor;
  }
  uint8_t getStatus(DM2325Id id) {
    return params_[toIndex(id)].status;
  }

private:
  struct Param {
    int64_t position_total = 0;
    int16_t prev_position = INT16_MAX;
    float position_rad = 0.0f;
    float velocity_rpm = 0.0f;
    float torque_nm = 0.0f;
    int8_t temp_mos = 0;
    int8_t temp_rotor = 0;
    uint8_t status = 0;
  };

  RP2040PIO_CAN *can_ = nullptr;
  std::array<Param, 8> params_{};

  inline int toIndex(DM2325Id id) {
    int idx = static_cast<int>(id);
    if (idx < 0) idx = 0;
    if (idx >= 8) idx = 7;
    return idx;
  }

  static constexpr float P_MAX = 12.5f;
  static constexpr float V_MAX = 30.0f;
  static constexpr float T_MAX = 10.0f;
  static constexpr float KP_MAX = 500.0f;
  static constexpr float KD_MAX = 5.0f;

  static float uint_to_float(uint16_t x, float min_val, float max_val, int bits) {
    float span = max_val - min_val;
    float normalized = (float)x / (float)((1 << bits) - 1);
    return normalized * span + min_val;
  }

  static uint16_t float_to_uint(float x, float min_val, float max_val, int bits) {
    float span = max_val - min_val;
    float clamped_x = (x < min_val) ? min_val : ((x > max_val) ? max_val : x);
    float normalized = (clamped_x - min_val) / span;
    return (uint16_t)(normalized * ((1 << bits) - 1));
  }
};

// definitions for DMMotorController static consts
const float DMMotorController::P_MAX = 12.5f;
const float DMMotorController::V_MAX = 30.0f;
const float DMMotorController::T_MAX = 10.0f;
const float DMMotorController::KP_MAX = 500.0f;
const float DMMotorController::KD_MAX = 5.0f;

#endif  // DM2325_H
