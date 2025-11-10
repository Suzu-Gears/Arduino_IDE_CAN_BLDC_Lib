#include <RP2040PIO_CAN.h>
#include "DM_Motor.h"

const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;
const uint8_t masterId = 0;
const uint8_t slaveId_A = 1;
const uint8_t slaveId_B = 2;
const uint8_t UserButtonPin = 29;


DM::Motor motor1(&CAN, masterId, slaveId_A, DM::DM_ControlMode::DM_CM_POS_VEL);
DM::Motor motor2(&CAN, masterId, slaveId_B, DM::DM_ControlMode::DM_CM_POS_VEL);

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
  motor1.setControlMode(DM::DM_CM_MIT);
  motor2.setControlMode(DM::DM_CM_MIT);
  motor1.enable();
  motor2.enable();
}

void loop() {
  motor1.sendMIT(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
  delay(1);
  motor1.update();

  motor2.sendMIT(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
  delay(1);
  motor2.update();

  Serial.print("SlaveID: ");
  Serial.print(motor1.getSlaveId());
  Serial.print(", Position_rad: ");
  Serial.print(motor1.getPosition());
  Serial.print(", rad/s: ");
  Serial.print(motor1.getVelocity());
  Serial.print(",  SlaveID: ");
  Serial.print(motor2.getSlaveId());
  Serial.print(", Position_rad: ");
  Serial.print(motor2.getPosition());
  Serial.print(", rad/s: ");
  Serial.print(motor2.getVelocity());
  Serial.println();
}
