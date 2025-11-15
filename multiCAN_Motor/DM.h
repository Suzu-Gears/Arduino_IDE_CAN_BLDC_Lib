#pragma once

#include <Arduino.h>
#include <map>
#include <cstdint>
#include <api/HardwareCAN.h>

// Forward declarations
class DMMotor;
class DMManager;

// --- Enums ---
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

// --- DMManager Definition ---
class DMManager {
public:
  DMManager(uint32_t masterId, arduino::HardwareCAN* can_interface = nullptr);
  void setCAN(arduino::HardwareCAN* can_interface);
  void registerMotor(uint32_t slaveId, DMMotor* motor);
  void update();
  arduino::HardwareCAN* getCanInterface() const;
  uint32_t getMasterId() const;
private:
  void propagateCANSettings();
      arduino::HardwareCAN* can_interface_;
      uint32_t masterId_;
      std::map<uint32_t, DMMotor*> motors_;};

// --- DMMotor Definition ---
class DMMotor {
public:
  DMMotor(DMManager* manager, uint32_t slaveId, DM_ControlMode mode);

  void attachCAN(arduino::HardwareCAN* can, uint32_t masterId);

  static uint8_t getSlaveIdFromMessage(const CanMsg& msg);
  void initialize();
  void processMessage(const CanMsg& msg);
  void update();
  bool enable();
  bool disable();
  bool setZeroPoint();
  void setControlMode(DM_ControlMode mode);
  bool sendMIT(float position_rad, float velocity_rad_s, float kp, float kd, float torque_ff);
  bool sendPosition(float position_rad, float velocity_limit_rad_s);
  bool sendVelocityRPM(float velocity_rpm);
  bool sendVelocityRPS(float velocity_rps);
  bool sendVelocity(float velocity_rad_s);
  float getPosition() const;
  float getPositionDeg() const;
  float getVelocity() const;
  float getRPM() const;
  float getRPS() const;
  float getTorque() const;
  int8_t getMOSTemp() const;
  int8_t getRotorTemp() const;
  DM_ControlMode getMode();
  uint32_t getSlaveId() const;
  DM_Status getStatus() const;
  float getPMAX() const;
  float getVMAX() const;
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
