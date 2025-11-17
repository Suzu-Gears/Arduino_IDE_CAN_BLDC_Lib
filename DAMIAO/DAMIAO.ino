#include <RP2040PIO_CAN.h>  //ボードに応じてCANライブラリを変更
#include <CANDemux.h>

//CANライブラリよりも下で呼び出す api/HardwareCAN.hが無いって言われる
#include <C6x0.h>  //https://github.com/tutrc-freshman/TUTRC_ArduinoLib.git
#include "DM.h"    // 作りかけ

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

const uint32_t dm_masterID = 0;
const uint32_t dm_slaveID_A = 9;
const uint32_t dm_slaveID_B = 10;

CANDemux canDemux(&CAN);
VirtualCAN c6x0_vcan = canDemux.createClientWithRange(0x201, 8, 1);
VirtualCAN dm_vcan = canDemux.createClientWithIds({ dm_masterID }, 2);

C6x0 c6x0;
DMManager dmManager(dm_masterID);
DMMotor dmMotorA(&dmManager, dm_slaveID_A, DM_ControlMode::DM_CM_POS_VEL);
DMMotor dmMotorB(&dmManager, dm_slaveID_B, DM_ControlMode::DM_CM_POS_VEL);

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 200;  // ms

void setup() {
  Serial.begin(115200);
  delay(2000);

  // CAN 初期化
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);

  // VirtualCAN を割り当て
  c6x0.setCAN(&c6x0_vcan);
  dmManager.setCAN(&dm_vcan);

  // モーター初期化（例）
  dmMotorA.initialize();
  dmMotorB.initialize();
  dmMotorA.setZeroPoint();
  dmMotorB.setZeroPoint();
  dmMotorA.enable();
  dmMotorB.enable();

  // --- ここからパラメータ読み書きのサンプルコード ---
  Serial.println("\n--- Parameter Read/Write Sample ---");

  // 1. float型パラメータの読み取り (例: KPゲイン)
  float kp_value;
  if (dmMotorA.readParam(DM_RID::DM_RID_KP, kp_value)) {
    Serial.print("Motor A - Initial KP: ");
    Serial.println(kp_value);
  } else {
    Serial.println("Motor A - Failed to read initial KP.");
  }

  // 2. uint32_t型パラメータの読み取り (例: 制御モード)
  uint32_t ctrl_mode_value;
  if (dmMotorA.readParam(DM_RID::DM_RID_CTRL_MODE, ctrl_mode_value)) {
    Serial.print("Motor A - Initial Control Mode (raw): ");
    Serial.println(ctrl_mode_value);
  } else {
    Serial.println("Motor A - Failed to read initial Control Mode.");
  }

  // 3. float型パラメータの書き込み (例: KPゲインを変更)
  float new_kp = 30.0f;
  Serial.print("Motor A - Attempting to set KP to: ");
  Serial.println(new_kp);
  if (dmMotorA.writeParam(DM_RID::DM_RID_KP, new_kp)) {
    Serial.println("Motor A - Successfully set new KP.");
    // 変更が反映されたか確認のため再読み込み
    if (dmMotorA.readParam(DM_RID::DM_RID_KP, kp_value)) {
      Serial.print("Motor A - Confirmed new KP: ");
      Serial.print(kp_value);
    }
  } else {
    Serial.println("Motor A - Failed to set new KP.");
  }

  // 4. 読み取り専用パラメータへの書き込み試行 (例: 現在位置)
  float current_position;
  if (dmMotorA.readParam(DM_RID::DM_RID_POSITION, current_position)) {
    Serial.print("Motor A - Current Position (read-only): ");
    Serial.println(current_position);
  }
  Serial.println("Motor A - Attempting to write to read-only POSITION parameter (should fail)...");
  if (dmMotorA.writeParam(DM_RID::DM_RID_POSITION, 1.23f)) {
    Serial.println("Motor A - ERROR: Successfully wrote to read-only POSITION!");
  } else {
    Serial.println("Motor A - Correctly failed to write to read-only POSITION.");
  }

  Serial.println("--- End of Parameter Read/Write Sample ---\\n");
  // --- ここまでパラメータ読み書きのサンプルコード ---
}

void loop() {
  dmManager.update();

  float target_deg = 180.0f;
  float target_rad = target_deg * PI / 180.0f;

  dmMotorA.sendPosition(target_rad, 10.0f);
  dmMotorB.sendPosition(target_rad, 10.0f);

  delay(3000);

  dmMotorA.sendPosition(-target_rad, 10.0f);
  dmMotorB.sendPosition(-target_rad, 10.0f);

  delay(3000);
}