#ifndef DM_H
#define DM_H

#include <cstdint>
#include <cstddef>
#include <map>
#include <type_traits>

#include <RP2040PIO_CAN.h>

// --- Enums and Structs ---

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

// パラメータのレジスタID (RID)
// !!! 注意: これは仮の定義です。実際のモーター仕様に合わせて修正してください。
enum class DM_RID : uint8_t {
  // Read/Write
  DM_RID_PMAX = 0x01,
  DM_RID_VMAX = 0x02,
  DM_RID_TMAX = 0x03,
  DM_RID_CTRL_MODE = 10,
  DM_RID_KP = 0x10,
  DM_RID_KD = 0x11,

  // Read-Only
  DM_RID_POSITION = 0x20,
  DM_RID_VELOCITY = 0x21,
  DM_RID_TORQUE = 0x22,
  DM_RID_TEMP_MOS = 0x23,
  DM_RID_TEMP_ROTOR = 0x24,
};

// --- パラメータ管理のための新しい定義 ---
// パラメータのアクセス権限
enum class DM_ParamAccess {
  READ_ONLY,
  READ_WRITE
};

// パラメータのデータ型
enum class DM_ParamType {
  FLOAT,
  UINT32
};

// パラメータ情報を格納する構造体
struct DM_ParamInfo {
  DM_RID rid;
  DM_ParamAccess access;
  DM_ParamType type;
  const char* name; // デバッグ用
};

// --- クラス定義 ---

class DMManager; // 前方宣言

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
  template<typename T>
  bool readParam(DM_RID rid, T& value, uint32_t timeout_ms = 100);

  template<typename T>
  bool writeParam(DM_RID rid, T value, uint32_t timeout_ms = 100);

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

// パラメータ定義テーブルの宣言
// C++17 inline static を使えない環境を考慮し、cppファイルで定義
extern const std::map<DM_RID, DM_ParamInfo> dm_param_definitions;

#endif // DM_H