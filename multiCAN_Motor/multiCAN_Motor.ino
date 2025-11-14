#include <RP2040PIO_CAN.h>

#include "C6x0.h"
#include "CANManager.h"
#include "DM_Motor.h"
#include "DMManager.h"

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

const uint32_t dm_master = 0;
const uint32_t dm_slaveA = 9;  //DMモーターはSlaveID1~8の場合、MITモードの指令のIDが0x201~208となりロボマスモーターののフィードバックと被るので、共存させるなら9より大きいIDを使う
const uint32_t dm_slaveB = 10;

// Single hub reading the physical CAN
CANHub canHub(&CAN);

C6x0 c6x0;
// DM motor and manager instances
DMMotor* dm_motorA = nullptr;
DMMotor* dm_motorB = nullptr;
DMManager* dm_manager = nullptr;

// toggle state for reversing every 3 seconds
bool toggle_sign = false;
unsigned long last_toggle_ms = 0;

void setup() {
  Serial.begin(115200);

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);

  // Create a client for the C6x0 feedback IDs 0x201..0x201+8
  CANClient* c6x0_client = canHub.createClientWithRange(0x201, 8);
  c6x0.setCAN(c6x0_client);

  // Create a client and a manager for DM motors
  CANClient* dm_client = canHub.createClientWithIds({ dm_master });
  dm_manager = new DMManager(dm_client, dm_master);

  // Create motor instances (still need to pass client and masterId for now)
  dm_motorA = new DMMotor(dm_client, dm_master, dm_slaveA, DM_ControlMode::DM_CM_MIT);
  dm_motorB = new DMMotor(dm_client, dm_master, dm_slaveB, DM_ControlMode::DM_CM_MIT);

  // Register motors with the manager
  dm_manager->registerMotor(dm_motorA);
  dm_manager->registerMotor(dm_motorB);

  // Initialize and enable motors
  dm_motorA->initialize();
  dm_motorA->enable();
  dm_motorB->initialize();
  dm_motorB->enable();
  last_toggle_ms = millis();
}

void loop() {
  c6x0.update();

  // The manager handles polling and dispatching messages to the correct motor
  if (dm_manager) {
    dm_manager->update();
  }

  // toggle every 3000 ms
  unsigned long now = millis();
  if (now - last_toggle_ms >= 3000) {
    last_toggle_ms = now;
    toggle_sign = !toggle_sign;
  }

  if (dm_motorA && dm_motorB) {
    // Send commands based on each other's feedback
    float pos1 = dm_motorA->getPosition();
    float pos2 = dm_motorB->getPosition();
    const float kp = 5.0f;
    const float kd = 2.0f;

    dm_motorA->sendMIT(pos2, 0.0f, kp, kd, 0.0f);
    dm_motorB->sendMIT(pos1, 0.0f, kp, kd, 0.0f);
  }

  float kp = 100;
  float rps = c6x0.getRpm(C610_ID_1) / 60.0f;
  float rps_ref = toggle_sign ? -10.0f : 10.0f;
  float current_ref = kp * (rps_ref - rps);

  c6x0.setCurrent(C610_ID_1, current_ref);

  c6x0.transmit();

  Serial.print("ID: 0x");
  Serial.print(0x201 + C610_ID_1, HEX);
  Serial.print(", Angle: ");
  Serial.print(c6x0.getPosition(C610_ID_1));
  Serial.print(" deg, RPS: ");
  Serial.print(rps);
  if (dm_motorA && dm_motorB) {
    Serial.print("\t DM_Status A: ");
    Serial.print(static_cast<int>(dm_motorA->getStatus()));
    Serial.print(", DM_Pos_Deg A: ");
    Serial.print(dm_motorA->getPositionDeg());
    Serial.print(", DM_Vel_RPS A: ");
    Serial.print(dm_motorA->getRPS());
    Serial.print(", DM_Torque A: ");
    Serial.print(dm_motorA->getTorque());

    Serial.print("\t DM_Status B: ");
    Serial.print(static_cast<int>(dm_motorB->getStatus()));
    Serial.print(", DM_Pos_Deg B: ");
    Serial.print(dm_motorB->getPositionDeg());
    Serial.print(", DM_Vel_RPS B: ");
    Serial.print(dm_motorB->getRPS());
    Serial.print(", DM_Torque B: ");
    Serial.print(dm_motorB->getTorque());
  }
  Serial.println();

  delay(1);
}
