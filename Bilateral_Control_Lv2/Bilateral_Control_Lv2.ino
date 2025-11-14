#include <RP2040PIO_CAN.h>
#include "DM_Motor.h"

const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;
const uint8_t masterId = 0;
const uint8_t slaveId_A = 1;
const uint8_t slaveId_B = 2;

DMMotor motor1(&CAN, masterId, slaveId_A, DM_ControlMode::DM_CM_POS_VEL);
DMMotor motor2(&CAN, masterId, slaveId_B, DM_ControlMode::DM_CM_POS_VEL);

void setup() {
  // Raspberry Pi Pico固有のCAN初期化手順
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);
  // ここまで
  // マイコンに合わせて変更すればArduino UNO R4, ESP32でも動く

  Serial.begin(115200);
  delay(1000);
  motor1.initialize();
  motor2.initialize();
  motor1.setControlMode(DM_CM_MIT);
  motor2.setControlMode(DM_CM_MIT);
  motor1.setZeroPoint();
  motor2.setZeroPoint();
  motor1.enable();
  motor2.enable();
}

void loop() {
  float pos1 = motor1.getPosition();
  float pos2 = motor2.getPosition();
  float vel1 = motor1.getVelocity();
  float vel2 = motor2.getVelocity();

  const float kp = 10.0f;
  const float kd = 2.0f;

  motor1.sendMIT(pos2, 0.0f, kp, kd, 0.0f);
  delay(1);
  motor1.update();
  motor2.sendMIT(pos1, 0.0f, kp, kd, 0.0f);
  delay(1);
  motor2.update();

  // 出力（デバッグ用）
  //  Serial.print("M1 ID:");
  //  Serial.print(motor1.getSlaveId());
  //  Serial.print(" pos:");
  //  Serial.print(pos1, 4);
  //  Serial.print(" vel:");
  //  Serial.print(vel1, 4);
  //  Serial.print("  <-- target:");
  //  Serial.print(pos2, 4);
  //
  //  Serial.print("    |");
  //  Serial.print("    M2 ID:");
  //  Serial.print(motor2.getSlaveId());
  //  Serial.print(" pos:");
  //  Serial.print(pos2, 4);
  //  Serial.print(" vel:");
  //  Serial.print(vel2, 4);
  //  Serial.print("  <-- target:");
  //  Serial.print(pos1, 4);
  //  Serial.println();

  // ループ周期を確保（必要に応じて調整）
  // delay(1);
}
