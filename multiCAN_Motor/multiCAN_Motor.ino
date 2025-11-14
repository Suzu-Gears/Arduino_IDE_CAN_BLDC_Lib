#include <RP2040PIO_CAN.h>

#include "C6x0.h"
#include "CANManager.h"
#include "DM_Motor.h"

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

const uint32_t dm_master = 0;
const uint32_t dm_slaveA = 9;  //DMモーターはSlaveID1~8の場合、MITモードの指令のIDが0x201~208となりロボマスモーターののフィードバックと被るので、共存させるなら9より大きいIDを使う
const uint32_t dm_slaveB = 10;

// Hardware and Manager objects are now declared globally as values
CANHub canHub(&CAN);
C6x0 c6x0;
DMManager dmManager(&canHub, dm_master);

// Declare motor objects globally, they will auto-register with the manager
DMMotor motorA(&dmManager, dm_slaveA, DM_ControlMode::DM_CM_MIT);
DMMotor motorB(&dmManager, dm_slaveB, DM_ControlMode::DM_CM_MIT);


// toggle state for reversing every 3 seconds
bool toggle_sign = false;
unsigned long last_toggle_ms = 0;

void setup() {
  Serial.begin(115200);

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);

  // Setup C6x0 client
  CANClient* c6x0_client = canHub.createClientWithRange(0x201, 8);
  c6x0.setCAN(c6x0_client);

  // Initialize and enable motors directly
  motorA.initialize();
  motorA.enable();
  motorB.initialize();
  motorB.enable();
  
  last_toggle_ms = millis();
}

void loop() {
  // Update all motor feedbacks
  c6x0.update();
  dmManager.update(); // This will call processMessage on motorA and motorB

  // toggle every 3000 ms
  unsigned long now = millis();
  if (now - last_toggle_ms >= 3000) {
    last_toggle_ms = now;
    toggle_sign = !toggle_sign;
  }

  // Send commands to DM motors directly
  float posA = motorA.getPosition();
  float posB = motorB.getPosition();
  const float kp = 5.0f;
  const float kd = 2.0f;

  motorA.sendMIT(posB, 0.0f, kp, kd, 0.0f);
  motorB.sendMIT(posA, 0.0f, kp, kd, 0.0f);
  
  // Send command to C6x0 motor
  float c6x0_kp = 100;
  float rps = c6x0.getRpm(C610_ID_1) / 60.0f;
  float rps_ref = toggle_sign ? -10.0f : 10.0f;
  float current_ref = c6x0_kp * (rps_ref - rps);

  c6x0.setCurrent(C610_ID_1, current_ref);
  c6x0.transmit();

  // Print feedback
  Serial.print("ID: 0x");
  Serial.print(0x201 + C610_ID_1, HEX);
  Serial.print(", Angle: ");
  Serial.print(c6x0.getPosition(C610_ID_1));
  Serial.print(" deg, RPS: ");
  Serial.print(rps);

  Serial.print("\t DM_Status A: ");
  Serial.print(static_cast<int>(motorA.getStatus()));
  Serial.print(", DM_Pos_Deg A: ");
  Serial.print(motorA.getPositionDeg());
  Serial.print(", DM_Vel_RPS A: ");
  Serial.print(motorA.getRPS());
  Serial.print(", DM_Torque A: ");
  Serial.print(motorA.getTorque());

  Serial.print("\t DM_Status B: ");
  Serial.print(static_cast<int>(motorB.getStatus()));
  Serial.print(", DM_Pos_Deg B: ");
  Serial.print(motorB.getPositionDeg());
  Serial.print(", DM_Vel_RPS B: ");
  Serial.print(motorB.getRPS());
  Serial.print(", DM_Torque B: ");
  Serial.print(motorB.getTorque());
  
  Serial.println();

  delay(1);
}