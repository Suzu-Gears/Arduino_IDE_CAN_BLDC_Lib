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

  // --- ここからパラメータ読み書きのサンプルコード ---
  Serial.println("\n--- Parameter Read/Write Sample ---");

  // 1. uint32_t型パラメータの読み取り (例: 制御モード)
  uint32_t ctrl_mode_value;
  if (dmMotorA.readParam<DMR::CTRL_MODE>(ctrl_mode_value)) {
    Serial.print("Step 1: Motor A - Initial Control Mode (raw): ");
    Serial.println(ctrl_mode_value);
  } else {
    Serial.println("Step 1: Motor A - Failed to read initial Control Mode.");
  }

  // 2. uint32_t型パラメータの書き込み (例: 制御モードを変更)
  uint32_t new_mode = 3;  // DM_CM_VEL
  Serial.print("Step 2: Motor A - Attempting to set Control Mode to: ");
  Serial.println(new_mode);
  if (dmMotorA.writeParam<DMR::CTRL_MODE>(new_mode)) {
    Serial.println("Step 2: Motor A - Successfully sent write command.");
  } else {
    Serial.println("Step 2: Motor A - Failed to send write command.");
  }

  // 3. 書き込んだ値の確認
  if (dmMotorA.readParam<DMR::CTRL_MODE>(ctrl_mode_value)) {
    Serial.print("Step 3: Motor A - Successfully read new Control Mode: ");
    Serial.println(ctrl_mode_value);
  } else {
    Serial.println("Step 3: Motor A - Failed to verify new Control Mode.");
  }

  // 4. 読み取り専用パラメータの読み取り (例: 現在位置)
  float current_position;
  if (dmMotorA.readParam<DMR::POSITION>(current_position)) {
    Serial.print("Step 4: Motor A - Current Position (read-only): ");
    Serial.println(current_position);
  } else {
    Serial.println("Step 4: Motor A - Failed to read current position.");
  }

  // 5. 読み取り専用パラメータへの書き込み試行 (コンパイルエラーの確認)
  Serial.println("Step 5: The following line is commented out as it causes a compile-time error.");
  Serial.println("        (Attempting to write to a read-only parameter)");
  // dmMotorA.writeParam<DMR::POSITION>(1.23f);  // <- この行のコメントを外すとコンパイルエラーになります

  Serial.println("\n--- End of Parameter Read/Write Sample ---\n");
  // --- ここまでパラメータ読み書きのサンプルコード ---

  // モーター初期化（例）
  dmMotorA.initialize();
  dmMotorB.initialize();
  dmMotorA.setZeroPoint();
  dmMotorB.setZeroPoint();
  dmMotorA.enable();
  dmMotorB.enable();
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
