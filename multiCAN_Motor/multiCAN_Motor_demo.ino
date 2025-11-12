#include <RP2040PIO_CAN.h>

#include "C6x0.h"
#include "CANManager.h"

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

// Single hub reading the physical CAN
CANHub canHub(&CAN);

C6x0 c6x0;

void setup() {
  Serial.begin(115200);

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);

  // Create a client for the C6x0 feedback IDs 0x201..0x201+8
  CANClient* c6x0_client = canHub.createClientWithRange(0x201, 8);
  c6x0.setCAN(c6x0_client);

  // Example for DM motors (one client per masterId):
  // uint32_t dm_master = 0x123;
  // CANClient* dm_client = canHub.createClientWithIds({ dm_master });
  // DM::Motor motor1(dm_client, dm_master, slaveId, DM::DM_ControlMode::DM_CM_VEL);
}

void loop() {
  c6x0.update();

  float kp = 100;
  float rps = c6x0.getRpm(C610_ID_1) / 60.0f;
  float rps_ref = 10;
  float current_ref = kp * (rps_ref - rps);

  c6x0.setCurrent(C610_ID_1, current_ref);

  c6x0.transmit();

  Serial.print("ID: 0x");
  Serial.print(0x201 + C610_ID_1, HEX);
  Serial.print(", Angle: ");
  Serial.print(c6x0.getPosition(C610_ID_1));
  Serial.print(" deg, RPS: ");
  Serial.print(rps);
  Serial.print(", Command_mA: ");
  Serial.print(current_ref);
  Serial.print(", Actual_mA: ");
  Serial.print(c6x0.getCurrent(C610_ID_1));
  Serial.println();

  delay(1);
}
