#ifndef DM_H
#define DM_H

#if !__has_include(<api/HardwareCAN.h>)
#error "DM.h: include a HardwareCAN provider (e.g. <Arduino_CAN.h>, <ESP32_TWAI.h>, <RP2040PIO_CAN.h>)"
#endif

#include <cstddef>
#include <type_traits>
#include <map>

#include <api/HardwareCAN.h>
#include "DMRegister.h"

// モーターの状態
enum class DM_Status : uint8_t {
  DM_STATUS_DISABLED = 0x0,
  DM_STATUS_ENABLED = 0x1,
  // ... 他の状態を追加
};

// 制御モード
enum class DM_ControlMode : uint32_t {
  DM_CM_MIT = 1,
  DM_CM_POS_VEL = 2,
  DM_CM_VEL = 3,
  DM_CM_POS_FORCE = 4,
};


// --- クラス定義 ---

class DMManager;  // 前方宣言

class DMMotor {
public:
  DMMotor(DMManager* manager, uint32_t slaveId, DM_ControlMode mode = DM_ControlMode::DM_CM_MIT);
  ~DMMotor();

  void initialize();
  void update();

  // --- システムコマンド ---
  bool enable();
  bool disable();
  bool setZeroPoint();

  // --- 制御コマンド ---
  void setControlMode(DM_ControlMode mode);
  bool sendMIT(float position_rad, float velocity_rad_s, float kp, float kd, float torque_ff);
  bool sendPosition(float position_rad, float velocity_limit_rad_s = 10.0f);
  bool sendVelocity(float velocity_rad_s);
  bool sendVelocityRPM(float v);
  bool sendVelocityRPS(float v);

  // --- パラメータアクセス (新しい汎用インターフェース) ---
  template<DM_RID Rid, typename T>
  bool readParam(T& value, uint32_t timeout_ms = 100);

  template<DM_RID Rid, typename T>
  bool writeParam(T value, uint32_t timeout_ms = 100);

  // --- データ取得 ---
  float getPosition() const;
  float getPositionDeg() const;
  float getVelocity() const;
  float getRPM() const;
  float getRPS() const;
  float getTorque() const;
  int8_t getMOSTemp() const;
  int8_t getRotorTemp() const;
  uint32_t getSlaveId() const;
  DM_Status getStatus() const;
  DM_ControlMode getMode();

  // --- 内部利用 (Managerから呼ばれる) ---
  void setMasterID(uint32_t masterId);
  void setCAN(arduino::HardwareCAN* can);
  void processMessage(const CanMsg& msg);
  static uint8_t getSlaveIdFromMessage(const CanMsg& msg);

private:
  // --- 内部構造体 ---
  struct Feedback;
  struct MappingRange;

  // --- メンバー変数 ---
  arduino::HardwareCAN* can_;
  uint32_t masterId_;
  uint32_t slaveId_;
  DM_ControlMode currentMode_;
  bool is_initialized_;
  Feedback* feedback_;
  MappingRange* mappingrange_;

  // --- 低レベル通信 (プライベート化) ---
  bool readParamUInt32(DM_RID r, uint32_t& o, uint32_t t = 100);
  bool readParamFloat(DM_RID r, float& o, uint32_t t = 100);
  bool sendParamUInt32(DM_RID r, uint32_t v, uint32_t t = 100);
  bool sendParamFloat(DM_RID r, float v, uint32_t t = 100);
  bool sendSystemCommand(uint8_t cmd);

  // --- ユーティリティ ---
  float uintToFloat(uint16_t x, float min_val, float max_val, int bits);
  uint16_t floatToUint(float x, float min_val, float max_val, int bits);
  float getPMAX() const;
  float getVMAX() const;
  float getTMAX() const;
};


class DMManager {
public:
  DMManager(uint32_t masterId, arduino::HardwareCAN* can_interface = nullptr);
  void setCAN(arduino::HardwareCAN* can_interface);
  void registerMotor(uint32_t slaveId, DMMotor* motor);
  void update();

private:
  arduino::HardwareCAN* can_interface_;
  uint32_t masterId_;
  std::map<uint32_t, DMMotor*> motors_;

  void propagateCANSettings();
};

// --- テンプレート関数の実装 ---
// ヘッダーに実装を移動することで、コンパイル時のテンプレート解決を可能にする

template<DM_RID Rid, typename T>
bool DMMotor::readParam(T& value, uint32_t timeout_ms) {
  // コンパイル時に型をチェック
  constexpr DM_ParamType expected_type = dm_param_trait<Rid>::type;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(expected_type == DM_ParamType::FLOAT, "Mismatched type for the given RID in readParam. Expected float.");
  } else if constexpr (std::is_same_v<T, uint32_t>) {
    static_assert(expected_type == DM_ParamType::UINT32, "Mismatched type for the given RID in readParam. Expected uint32_t.");
  } else {
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, uint32_t>, "Unsupported type for readParam. Only float and uint32_t are supported.");
  }

  // 実行時の処理
  if constexpr (expected_type == DM_ParamType::FLOAT) {
    return readParamFloat(Rid, reinterpret_cast<float&>(value), timeout_ms);
  } else {  // UINT32
    return readParamUInt32(Rid, reinterpret_cast<uint32_t&>(value), timeout_ms);
  }
}

template<DM_RID Rid, typename T>
bool DMMotor::writeParam(T value, uint32_t timeout_ms) {
  // コンパイル時に型とアクセス権をチェック
  constexpr DM_ParamType expected_type = dm_param_trait<Rid>::type;
  constexpr DM_ParamAccess expected_access = dm_param_trait<Rid>::access;

  static_assert(expected_access == DM_ParamAccess::READ_WRITE, "Write operation is not allowed for this read-only RID.");

  if constexpr (std::is_same_v<T, float>) {
    static_assert(expected_type == DM_ParamType::FLOAT, "Mismatched type for the given RID in writeParam. Expected float.");
  } else if constexpr (std::is_same_v<T, uint32_t>) {
    static_assert(expected_type == DM_ParamType::UINT32, "Mismatched type for the given RID in writeParam. Expected uint32_t.");
  } else {
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, uint32_t>, "Unsupported type for writeParam. Only float and uint32_t are supported.");
  }

  // 実行時の処理
  if constexpr (expected_type == DM_ParamType::FLOAT) {
    return sendParamFloat(Rid, static_cast<float>(value), timeout_ms);
  } else {  // UINT32
    return sendParamUInt32(Rid, static_cast<uint32_t>(value), timeout_ms);
  }
}


#endif  // DM_H
