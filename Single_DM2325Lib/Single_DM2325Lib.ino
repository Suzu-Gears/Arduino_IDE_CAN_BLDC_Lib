#include <RP2040PIO_CAN.h>
#include "DM2325.h"

const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;
const uint8_t masterId = 0;
const uint8_t slaveId = 1;
const uint8_t UserButtonPin = 29;

bool flg = false;

DM::DM2325 motor1(&CAN, masterId, slaveId);

void handleButtonInterrupt() {
  flg = true;
}

void setup() {
  pinMode(UserButtonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(UserButtonPin), handleButtonInterrupt, RISING);

  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial
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
  motor1.setControlMode(DM::DM_CM_VELOCITY);
  delay(10);
  Serial.print("Ctrl Mode: ");
  Serial.println(motor1.getMode());
  while (!flg) { delay(1); }
  motor1.enableMotor();
}

void loop() {
  motor1.update();
  motor1.sendVelocity(60.0f);
  Serial.print("Status: ");
  Serial.print(motor1.getStatus());
  Serial.print(", Position: ");
  Serial.print(motor1.getPositionDeg());
  Serial.print(", RPM: ");
  Serial.print(motor1.getRPM());
  Serial.print(", RPS: ");
  Serial.print(motor1.getRPS());
  Serial.print(", MOSTemp: ");
  Serial.print(motor1.getMOSTemp());
  Serial.print(", RotorTemp: ");
  Serial.print(motor1.getRotorTemp());
  Serial.println();
  delay(1);
}
