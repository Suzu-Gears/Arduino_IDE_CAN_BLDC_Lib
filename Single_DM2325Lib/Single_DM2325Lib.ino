#include <RP2040PIO_CAN.h>
#include "DM2325.h"

const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;
const uint8_t masterId = 0;
const uint8_t slaveId = 1;

DM::DM2325 motor1(&CAN, masterId, slaveId);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial
  }
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);
  delay(500);
  motor1.initialize();
  Serial.print("MasterID: ");
  Serial.print(motor1.getMasterId());
  Serial.print(", SlaveID: ");
  Serial.print(motor1.getSlaveId());
  Serial.print(", PMAX: ");
  Serial.print(motor1.getPMAX());
  Serial.print(", VMAX: ");
  Serial.print(motor1.getVMAX());
  Serial.print(", TMAX: ");
  Serial.print(motor1.getTMAX());
  Serial.println();
  Serial.print("Ctrl Mode: ");
  Serial.println(motor1.getMode());
  motor1.setControlMode(DM::DM_CM_POSITION);
  delay(10);
  Serial.print("Ctrl Mode: ");
  Serial.println(motor1.getMode());

}

void loop() {
  motor1.update();



  
  Serial.print("Position: ");
  Serial.print(motor1.getPositionDeg());
  Serial.print(", RPM: ");
  Serial.print(motor1.getRPM());
  Serial.print(", RPS: ");
  Serial.print(motor1.getRPS());
  Serial.println();
  delay(10);
}
