/**
 * @file DM.h
 * @brief DMシリーズ ブラシレスモーターを制御するためのライブラリ
 * @author Your Name
 */
#pragma once

#include <Arduino.h>
#include <map>
#include <cstdint>
#include <api/HardwareCAN.h>

// Forward declarations
class DMMotor;
class DMManager;

/**
 * @enum DM_ControlMode
 * @brief DMMotorの制御モードを定義します。
 */
enum class DM_ControlMode : uint32_t {
  DM_CM_MIT = 1,      ///< MITモード (位置、速度、KP、KD、トルクFF)
  DM_CM_POS_VEL = 2,  ///< 位置・速度制御モード
  DM_CM_VEL = 3,      ///< 速度制御モード
};

/**
 * @enum DM_RID
 * @brief モーターのパラメータを読み書きするためのレジスタIDを定義します。
 */
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
  DM_RID_TMAX = 23,
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
  DM_RID_T_can_br = 35,
  DM_RID_T_sub_ver = 36,
  DM_RID_T_u_off = 50,
  DM_RID_T_v_off = 51,
  DM_RID_T_k1 = 52,
  DM_RID_T_k2 = 53,
  DM_RID_T_m_off = 54,
  DM_RID_T_dir = 55,
  DM_RID_T_p_m = 80,
};

/**
 * @enum DM_Status
 * @brief モーターからのフィードバックに含まれる状態コードを定義します。
 */
enum class DM_Status : uint8_t {
  DM_STATUS_DISABLED = 0x00,
  DM_STATUS_ENABLED = 0x01,
  DM_STATUS_SENSOR_ERROR = 0x05,
  DM_STATUS_PARAM_ERROR = 0x06,
  DM_STATUS_OVER_VOLTAGE = 0x08,
  DM_STATUS_UNDER_VOLTAGE = 0x09,
  DM_STATUS_OVER_CURRENT = 0x0A,
  DM_STATUS_MOS_OVER_TEMP = 0x0B,
  DM_STATUS_COIL_OVER_TEMP = 0x0C,
  DM_STATUS_COMM_LOST = 0x0D,
  DM_STATUS_OVER_LOAD = 0x0E,
};

/**
 * @class DMManager
 * @brief 複数のDMMotorオブジェクトを管理し、CAN通信を中継するクラス。
 */
class DMManager {
public:
  /**
   * @brief DMManagerのコンストラクタ。
   * @param masterId モーターからのフィードバックを受信するマスターID。
   * @param can_interface (オプション) 使用するCANインターフェースのポインタ。
   */
  DMManager(uint32_t masterId, arduino::HardwareCAN* can_interface = nullptr);

  /**
   * @brief CANインターフェースを設定します。
   * @param can_interface 使用するCANインターフェースのポインタ。
   */
  void setCAN(arduino::HardwareCAN* can_interface);

  /**
   * @brief 管理対象のモーターを登録します。
   * @param slaveId 登録するモーターのスレーブID。
   * @param motor 登録するDMMotorオブジェクトのポインタ。
   */
  void registerMotor(uint32_t slaveId, DMMotor* motor);

  /**
   * @brief CANバスをポーリングし、受信したメッセージを適切なモーターに振り分けます。
   * loop関数内で定期的に呼び出す必要があります。
   */
  void update();

  /**
   * @brief 設定されているCANインターフェースを取得します。
   * @return arduino::HardwareCAN* CANインターフェースのポインタ。
   */
  arduino::HardwareCAN* getCanInterface() const;

  /**
   * @brief 設定されているマスターIDを取得します。
   * @return uint32_t マスターID。
   */
  uint32_t getMasterId() const;

private:
  void propagateCANSettings();
  arduino::HardwareCAN* can_interface_;
  uint32_t masterId_;
  std::map<uint32_t, DMMotor*> motors_;
};

/**
 * @class DMMotor
 * @brief 個々のDMシリーズモーターを表現し、制御するためのクラス。
 */
class DMMotor {
public:
  /**
   * @brief DMMotorのコンストラクタ。
   * @param slaveId モーターに設定されたスレーブID。
   * @param mode 初期制御モード。
   * @param manager (オプション) このモーターを管理するDMManagerのポインタ。
   */
  DMMotor(DMManager* manager, uint32_t slaveId, DM_ControlMode mode);

  /**
   * @brief マスターIDを設定します。
   * @param masterId モーターからのフィードバックを受信するマスターID。
   */
  void setMasterID(uint32_t masterId);

  /**
   * @brief CANインターフェースを設定します。
   * @param can 使用するCANインターフェースのポインタ。
   */
  void setCAN(arduino::HardwareCAN* can);

  /**
   * @brief CANメッセージからスレーブIDを抽出します。
   * @param msg CANメッセージ。
   * @return uint8_t 抽出されたスレーブID。
   */
  static uint8_t getSlaveIdFromMessage(const CanMsg& msg);

  /**
   * @brief モーターを初期化します。PMAX, VMAX, TMAXなどのパラメータを読み込みます。
   */
  void initialize();

  /**
   * @brief CANメッセージを処理し、モーターの内部状態を更新します。
   * @param msg 処理するCANメッセージ。
   */
  void processMessage(const CanMsg& msg);

  /**
   * @brief 単体使用時にCANバスをポーリングします。
   */
  void update();

  /**
   * @brief モーターを有効にします（Enable）。
   * @return bool コマンドの送信に成功したか。
   */
  bool enable();

  /**
   * @brief モーターを無効にします（Disable）。
   * @return bool コマンドの送信に成功したか。
   */
  bool disable();

  /**
   * @brief 現在のモーター位置をゼロ点として設定します。
   * @return bool コマンドの送信に成功したか。
   */
  bool setZeroPoint();

  /**
   * @brief 制御モードを設定します。
   * @param mode 設定する制御モード。
   */
  void setControlMode(DM_ControlMode mode);

  /**
   * @brief MITモードでモーターに指令を送信します。
   * @param position_rad 目標位置 (ラジアン)。
   * @param velocity_rad_s 目標速度 (rad/s)。
   * @param kp 位置ゲイン (0.0 - 500.0)。
   * @param kd 速度ゲイン (0.0 - 5.0)。
   * @param torque_ff フィードフォワードトルク (Nm)。
   * @return bool コマンドの送信に成功したか。
   */
  bool sendMIT(float position_rad, float velocity_rad_s, float kp, float kd, float torque_ff);

  /**
   * @brief 位置・速度制御モードでモーターに指令を送信します。
   * @param position_rad 目標位置 (ラジアン)。
   * @param velocity_limit_rad_s 速度制限 (rad/s)。
   * @return bool コマンドの送信に成功したか。
   */
  bool sendPosition(float position_rad, float velocity_limit_rad_s);

  /**
   * @brief 速度制御モードでモーターに指令を送信します (RPM)。
   * @param velocity_rpm 目標速度 (RPM)。
   * @return bool コマンドの送信に成功したか。
   */
  bool sendVelocityRPM(float velocity_rpm);

  /**
   * @brief 速度制御モードでモーターに指令を送信します (RPS)。
   * @param velocity_rps 目標速度 (RPS)。
   * @return bool コマンドの送信に成功したか。
   */
  bool sendVelocityRPS(float velocity_rps);

  /**
   * @brief 速度制御モードでモーターに指令を送信します (rad/s)。
   * @param velocity_rad_s 目標速度 (rad/s)。
   * @return bool コマンドの送信に成功したか。
   */
  bool sendVelocity(float velocity_rad_s);

  /** @brief 現在の位置をラジアン単位で取得します。 @return float 位置 (rad)。 */
  float getPosition() const;
  /** @brief 現在の位置を度単位で取得します。 @return float 位置 (度)。 */
  float getPositionDeg() const;
  /** @brief 現在の速度をrad/s単位で取得します。 @return float 速度 (rad/s)。 */
  float getVelocity() const;
  /** @brief 現在の速度をRPM単位で取得します。 @return float 速度 (RPM)。 */
  float getRPM() const;
  /** @brief 現在の速度をRPS単位で取得します。 @return float 速度 (RPS)。 */
  float getRPS() const;
  /** @brief 現在のトルクをNm単位で取得します。 @return float トルク (Nm)。 */
  float getTorque() const;
  /** @brief 現在のMOSFET温度を取得します。 @return int8_t 温度 (°C)。 */
  int8_t getMOSTemp() const;
  /** @brief 現在のローター温度を取得します。 @return int8_t 温度 (°C)。 */
  int8_t getRotorTemp() const;
  /** @brief 現在の制御モードを取得します。 @return DM_ControlMode 制御モード。 */
  DM_ControlMode getMode();
  /** @brief モーターのスレーブIDを取得します。 @return uint32_t スレーブID。 */
  uint32_t getSlaveId() const;
  /** @brief 現在のモーターの状態を取得します。 @return DM_Status 状態コード。 */
  DM_Status getStatus() const;
  /** @brief 位置の最大値(PMAX)を取得します。 @return float PMAX値。 */
  float getPMAX() const;
  /** @brief 速度の最大値(VMAX)を取得します。 @return float VMAX値。 */
  float getVMAX() const;
  /** @brief トルクの最大値(TMAX)を取得します。 @return float TMAX値。 */
  float getTMAX() const;

private:
  struct Feedback;
  struct MappingRange;

  bool readParamUInt32(DM_RID rid, uint32_t& out, uint32_t timeout_ms = 200);
  bool readParamFloat(DM_RID rid, float& out, uint32_t timeout_ms = 200);
  bool sendParamUInt32(DM_RID rid, uint32_t value_to_write, uint32_t timeout_ms = 200);
  bool sendParamFloat(DM_RID rid, float value_to_write, uint32_t timeout_ms = 200);
  bool sendSystemCommand(uint8_t commandByte);
  static float uintToFloat(uint16_t x, float min_val, float max_val, int bits);
  static uint16_t floatToUint(float x, float min_val, float max_val, int bits);

  arduino::HardwareCAN* can_;
  Feedback* feedback_;
  MappingRange* mappingrange_;
  uint32_t masterId_;
  uint32_t slaveId_;
  DM_ControlMode currentMode_;
  bool is_initialized_;
};
