#include "DM.h"
#include <algorithm>
#include <cstring>

// --- Private DMMotor Structs ---
struct DMMotor::Feedback {
  DM_Status status = DM_Status::DM_STATUS_DISABLED;
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  int8_t temp_mos = 0;
  int8_t temp_rotor = 0;
};

struct DMMotor::MappingRange {
  float pmax = 0.0f;
  float vmax = 0.0f;
  float tmax = 0.0f;
};


// --- DMMotor Implementations ---
DMMotor::DMMotor(DMManager* manager, uint32_t slaveId, DM_ControlMode mode)
  : can_(nullptr), masterId_(0), slaveId_(slaveId), currentMode_(mode), is_initialized_(false) {
  feedback_ = new Feedback();
  mappingrange_ = new MappingRange();
  if (manager) {
    manager->registerMotor(slaveId, this);
  }
}

void DMMotor::setMasterID(uint32_t masterId) {
  masterId_ = masterId;
}

void DMMotor::setCAN(arduino::HardwareCAN* can) {
  can_ = can;
}

uint8_t DMMotor::getSlaveIdFromMessage(const CanMsg& msg) {
  return msg.data[0] & 0x0F;
}

void DMMotor::initialize() {
  if (!can_) return;
  bool pmax_ok = readParamFloat(DM_RID::DM_RID_PMAX, mappingrange_->pmax);
  bool vmax_ok = readParamFloat(DM_RID::DM_RID_VMAX, mappingrange_->vmax);
  bool tmax_ok = readParamFloat(DM_RID::DM_RID_TMAX, mappingrange_->tmax);
  is_initialized_ = (pmax_ok && vmax_ok && tmax_ok);
  if (is_initialized_) { setControlMode(currentMode_); }
}

void DMMotor::processMessage(const CanMsg& msg) {
  if (!is_initialized_) return;

  // このモーター宛のメッセージか確認 (マスターIDが一致し、かつフレーム内のスレーブIDが一致)
  if (msg.getStandardId() == masterId_ && (msg.data[0] & 0x0F) == slaveId_) {
    // CANフレームのデータレイアウトに従って値をデコード
    // data[0]: [ステータス(4bit), スレーブID(4bit)]
    feedback_->status = static_cast<DM_Status>((msg.data[0] >> 4) & 0x0F);
    
    // data[1-2]: 位置データ (16bit)
    uint16_t pos_raw = (uint16_t)(msg.data[1] << 8 | msg.data[2]);
    
    // data[3]とdata[4]の上位4bit: 速度データ (12bit)
    uint16_t vel_raw = (uint16_t)((msg.data[3] << 4) | (msg.data[4] >> 4));
    
    // data[4]の下位4bitとdata[5]: トルクデータ (12bit)
    uint16_t torque_raw = (uint16_t)(((msg.data[4] & 0x0F) << 8) | msg.data[5]);
    
    // data[6]: MOSFET温度
    feedback_->temp_mos = (int8_t)msg.data[6];
    
    // data[7]: ローター温度
    feedback_->temp_rotor = (int8_t)msg.data[7];

    // RAW値を物理量に変換
    feedback_->position = uintToFloat(pos_raw, -getPMAX(), getPMAX(), 16);
    feedback_->velocity = uintToFloat(vel_raw, -getVMAX(), getVMAX(), 12);
    feedback_->torque = uintToFloat(torque_raw, -getTMAX(), getTMAX(), 12);
  }
}

void DMMotor::update() {
  if (!is_initialized_ || !can_) return;
  while (can_->available()) {
    processMessage(can_->read());
  }
}

bool DMMotor::enable() {
  if (!can_) {
    return false;
  }
  bool success = sendSystemCommand(0xFC);
  return success;
}
bool DMMotor::disable() {
  return sendSystemCommand(0xFD);
}
bool DMMotor::setZeroPoint() {
  return sendSystemCommand(0xFE);
}

void DMMotor::setControlMode(DM_ControlMode mode) {
  currentMode_ = mode;
  sendParamUInt32(DM_RID::DM_RID_CTRL_MODE, static_cast<uint32_t>(mode));
}

bool DMMotor::sendMIT(float position_rad, float velocity_rad_s, float kp, float kd, float torque_ff) {
  if (currentMode_ != DM_ControlMode::DM_CM_MIT) return false;  // only allowed in MIT MODE

  // 物理量をRAW値に変換
  uint16_t position_raw = floatToUint(position_rad, -getPMAX(), getPMAX(), 16);
  uint16_t velocity_raw = floatToUint(velocity_rad_s, -getVMAX(), getVMAX(), 12);
  uint16_t kp_raw = floatToUint(kp, 0.0f, 500.0f, 12);
  uint16_t kd_raw = floatToUint(kd, 0.0f, 5.0f, 12);
  uint16_t torque_raw = floatToUint(torque_ff, -getTMAX(), getTMAX(), 12);

  CanMsg tx = {};
  tx.id = slaveId_;
  tx.data_length = 8;

  // CANフレームのデータレイアウトに従って値をエンコード
  // [P_des(16), V_des(12), Kp(12), Kd(12), T_ff(12)]
  // P_des: 16bit (data[0], data[1])
  tx.data[0] = static_cast<uint8_t>(position_raw >> 8);
  tx.data[1] = static_cast<uint8_t>(position_raw & 0xFF);
  // V_des: 12bit (data[2], data[3]の上位4bit)
  tx.data[2] = static_cast<uint8_t>(velocity_raw >> 4);
  // Kp: 12bit (data[3]の下位4bit, data[4])
  tx.data[3] = static_cast<uint8_t>(((velocity_raw & 0x0F) << 4) | (kp_raw >> 8));
  tx.data[4] = static_cast<uint8_t>(kp_raw & 0xFF);
  // Kd: 12bit (data[5], data[6]の上位4bit)
  tx.data[5] = static_cast<uint8_t>(kd_raw >> 4);
  // T_ff: 12bit (data[6]の下位4bit, data[7])
  tx.data[6] = static_cast<uint8_t>(((kd_raw & 0x0F) << 4) | (torque_raw >> 8));
  tx.data[7] = static_cast<uint8_t>(torque_raw & 0xFF);

  return can_->write(tx) >= 0;
}

bool DMMotor::sendPosition(float position_rad, float velocity_limit_rad_s) {
  if (currentMode_ != DM_ControlMode::DM_CM_POS_VEL) return false;  // only allowed in POSITION_VELOCITY MODE

  uint32_t position_raw, velocity_raw;
  ::memcpy(&position_raw, &position_rad, sizeof(position_raw));
  ::memcpy(&velocity_raw, &velocity_limit_rad_s, sizeof(velocity_raw));

  CanMsg tx = {};
  tx.id = 0x100 + slaveId_;  // 位置速度制御用idオフセット0x100
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

bool DMMotor::sendVelocity(float velocity_rad_s) {
  if (currentMode_ != DM_ControlMode::DM_CM_VEL) return false;  // only allowed in VELOCITY MODE

  uint32_t raw;
  ::memcpy(&raw, &velocity_rad_s, sizeof(raw));

  CanMsg tx = {};
  tx.id = 0x200 + slaveId_;  // 速度制御用idオフセット0x200
  tx.data_length = 4;
  tx.data[0] = static_cast<uint8_t>(raw & 0xFF);
  tx.data[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  tx.data[2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  tx.data[3] = static_cast<uint8_t>((raw >> 24) & 0xFF);

  return can_->write(tx) >= 0;
}

bool DMMotor::sendVelocityRPM(float v) {
  return sendVelocity(v * PI / 30.0f);
}
bool DMMotor::sendVelocityRPS(float v) {
  return sendVelocity(v * 2.0f * PI);
}
float DMMotor::getPosition() const {
  return feedback_->position;
}
float DMMotor::getPositionDeg() const {
  return feedback_->position * 180.0f / PI;
}
float DMMotor::getVelocity() const {
  return feedback_->velocity;
}
float DMMotor::getRPM() const {
  return feedback_->velocity * 30.0f / PI;
}
float DMMotor::getRPS() const {
  return feedback_->velocity / (2.0f * PI);
}
float DMMotor::getTorque() const {
  return feedback_->torque;
}
int8_t DMMotor::getMOSTemp() const {
  return feedback_->temp_mos;
}
int8_t DMMotor::getRotorTemp() const {
  return feedback_->temp_rotor;
}
DM_ControlMode DMMotor::getMode() {
  DM_ControlMode m;
  uint32_t raw;
  readParamUInt32(DM_RID::DM_RID_CTRL_MODE, raw);
  m = (DM_ControlMode)raw;
  return m;
}
uint32_t DMMotor::getSlaveId() const {
  return slaveId_;
}
DM_Status DMMotor::getStatus() const {
  return feedback_->status;
}
float DMMotor::getPMAX() const {
  return abs(mappingrange_->pmax);
}
float DMMotor::getVMAX() const {
  return abs(mappingrange_->vmax);
}
float DMMotor::getTMAX() const {
  return abs(mappingrange_->tmax);
}

bool DMMotor::readParamUInt32(DM_RID r, uint32_t& o, uint32_t t) {
  if (!can_) return false;
  CanMsg tx{};
  tx.id = 0x7FF;
  tx.data_length = 8;
  tx.data[0] = slaveId_ & 0xFF;
  tx.data[1] = (slaveId_ >> 8) & 0xFF;
  tx.data[2] = 0x33;
  tx.data[3] = static_cast<uint8_t>(r);
  if (can_->write(tx) < 0) return false;
  unsigned long s = millis();
  while (millis() - s < t) {
    if (can_->available()) {
      CanMsg rx = can_->read();
      if (rx.getStandardId() != masterId_ || rx.data_length < 8 || rx.data[2] != 0x33 || rx.data[3] != static_cast<uint8_t>(r)) continue;
      o = (uint32_t)rx.data[4] | ((uint32_t)rx.data[5] << 8) | ((uint32_t)rx.data[6] << 16) | ((uint32_t)rx.data[7] << 24);
      return true;
    }
  }
  return false;
}
bool DMMotor::readParamFloat(DM_RID r, float& o, uint32_t t) {
  uint32_t raw;
  if (!readParamUInt32(r, raw, t)) return false;
  ::memcpy(&o, &raw, 4);
  return true;
}
bool DMMotor::sendParamUInt32(DM_RID r, uint32_t v, uint32_t t) {
  if (!can_) return false;
  CanMsg tx{};
  tx.id = 0x7FF;
  tx.data_length = 8;
  tx.data[0] = slaveId_ & 0xFF;
  tx.data[1] = (slaveId_ >> 8) & 0xFF;
  tx.data[2] = 0x55;
  tx.data[3] = static_cast<uint8_t>(r);
  tx.data[4] = v & 0xFF;
  tx.data[5] = (v >> 8) & 0xFF;
  tx.data[6] = (v >> 16) & 0xFF;
  tx.data[7] = (v >> 24) & 0xFF;
  if (can_->write(tx) < 0) return false;
  unsigned long s = millis();
  while (millis() - s < t) {
    if (can_->available()) {
      CanMsg rx = can_->read();
      if (rx.getStandardId() != masterId_ || rx.data_length < 8 || rx.data[2] != 0x33 || rx.data[3] != static_cast<uint8_t>(r)) continue;
      uint32_t ret = (uint32_t)rx.data[4] | ((uint32_t)rx.data[5] << 8) | ((uint32_t)rx.data[6] << 16) | ((uint32_t)rx.data[7] << 24);
      bool ok = (ret == v);
      if (ok) {
        float vf;
        ::memcpy(&vf, &ret, 4);
        if (r == DM_RID::DM_RID_PMAX) mappingrange_->pmax = vf;
        if (r == DM_RID::DM_RID_VMAX) mappingrange_->vmax = vf;
        if (r == DM_RID::DM_RID_TMAX) mappingrange_->tmax = vf;
      }
      return ok;
    }
  }
  return false;
}
bool DMMotor::sendParamFloat(DM_RID r, float v, uint32_t t) {
  uint32_t raw;
  ::memcpy(&raw, &v, 4);
  return sendParamUInt32(r, raw, t);
}
bool DMMotor::sendSystemCommand(uint8_t cmd) {
  if (!can_) return false;
  CanMsg tx = {};
  tx.id = slaveId_;
  tx.data_length = 8;
  std::fill(tx.data, tx.data + 7, 0xFF);
  tx.data[7] = cmd;
  return can_->write(tx) >= 0;
}

/**
 * @brief 符号なし整数を、指定された範囲とビット数に基づいて浮動小数点数に変換します。
 * @param x 変換する符号なし整数。
 * @param min_val マッピング範囲の最小値。
 * @param max_val マッピング範囲の最大値。
 * @param bits 整数のビット数。
 * @return float 変換された浮動小数点数。
 */
float DMMotor::uintToFloat(uint16_t x, float min_val, float max_val, int bits) {
  float span = max_val - min_val;
  if (span == 0.0f) return min_val;  // Avoid division by zero
  // 整数値を[0, 1]の範囲に正規化
  float normalized = (float)x / (float)((1 << bits) - 1);
  // 正規化された値を、指定された物理量の範囲にスケール
  return normalized * span + min_val;
}

/**
 * @brief 浮動小数点数を、指定された範囲とビット数に基づいて符号なし整数に変換します。
 * @param x 変換する浮動小数点数。
 * @param min_val マッピング範囲の最小値。
 * @param max_val マッピング範囲の最大値。
 * @param bits 整数のビット数。
 * @return uint16_t 変換された符号なし整数。
 */
uint16_t DMMotor::floatToUint(float x, float min_val, float max_val, int bits) {
  float span = max_val - min_val;
  if (span <= 0.0f) return 0;
  // 値を範囲内にクランプ
  float clamped_x = std::clamp(x, min_val, max_val);
  // 物理量を[0, 1]の範囲に正規化
  float normalized = (clamped_x - min_val) / span;
  // 正規化された値を、指定されたビット数の整数にスケール
  return (uint16_t)(normalized * ((1 << bits) - 1));
}


// --- DMManager Implementations ---
DMManager::DMManager(uint32_t masterId, arduino::HardwareCAN* can_interface) 
  : can_interface_(can_interface), masterId_(masterId) {
  if (can_interface_) {
    propagateCANSettings();
  }
}
void DMManager::setCAN(arduino::HardwareCAN* can_interface) {
  can_interface_ = can_interface;
  propagateCANSettings();
}

void DMManager::registerMotor(uint32_t slaveId, DMMotor* motor) {
  motors_[slaveId] = motor;
  if (can_interface_) {
    motor->setMasterID(masterId_);
    motor->setCAN(can_interface_);
  }
}

void DMManager::propagateCANSettings() {
  if (!can_interface_) return;
  for (auto const& [id, motor] : motors_) {
    motor->setMasterID(masterId_);
    motor->setCAN(can_interface_);
  }
}

void DMManager::update() {
  if (!can_interface_) return;
  while (can_interface_->available()) {
    CanMsg msg = can_interface_->read();
    if (msg.getStandardId() != masterId_) continue;

    uint8_t slaveId = DMMotor::getSlaveIdFromMessage(msg);
    auto it = motors_.find(slaveId);
    if (it != motors_.end()) {
      it->second->processMessage(msg);
    }
  }
}
