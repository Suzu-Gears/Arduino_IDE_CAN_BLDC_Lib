#include <RP2040PIO_CAN.h>
#include "DM_Motor.h"

const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;

const uint8_t masterId = 0;
const uint8_t slaveId = 9;

DMMotor motor1(&CAN, masterId, slaveId, DM_ControlMode::DM_CM_MIT);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  //シリアル接続時のみ
  }
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);
  delay(1000);
  motor1.initialize();
  Serial.print("MasterID: ");
  Serial.print(motor1.getMasterId());
  Serial.print(", SlaveID: ");
  Serial.print(motor1.getSlaveId());
  Serial.print(", Status: ");
  Serial.print(motor1.getStatus());
  Serial.print(", PMAX: ");
  Serial.print(motor1.getPMAX());
  Serial.print(", VMAX: ");
  Serial.print(motor1.getVMAX());
  Serial.print(", TMAX: ");
  Serial.print(motor1.getTMAX());
  Serial.println();
  Serial.print("Ctrl Mode: ");
  Serial.println(motor1.getMode());

  motor1.setControlMode(DM_CM_MIT);
  // motor1.setControlMode(DM_CM_POS_VEL);
  // motor1.setControlMode(DM_CM_VEL);

  delay(10);
  Serial.print("Ctrl Mode: ");
  Serial.println(motor1.getMode());

  motor1.setZeroPoint();
  delay(1000);
  motor1.enable();
}

void loop() {
  motor1.update();

  motor1.sendMIT(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
  // motor1.sendPosition(0.0f, 0.0f);
  // motor1.sendVelocityRPS(0.1f);

  Serial.print("Status: ");
  Serial.print(motor1.getStatus());
  Serial.print(", Position_rad: ");
  Serial.print(motor1.getPosition());
  Serial.print(", PositionDeg: ");
  Serial.print(motor1.getPositionDeg());
  Serial.print(", rad/s: ");
  Serial.print(motor1.getVelocity());
  Serial.print(", RPM: ");
  Serial.print(motor1.getRPM());
  Serial.print(", RPS: ");
  Serial.print(motor1.getRPS());
  Serial.print(", Torque: ");
  Serial.print(motor1.getTorque());
  Serial.print(", MOSTemp: ");
  Serial.print(motor1.getMOSTemp());
  Serial.print(", RotorTemp: ");
  Serial.print(motor1.getRotorTemp());
  Serial.println();
  delay(1);
}
