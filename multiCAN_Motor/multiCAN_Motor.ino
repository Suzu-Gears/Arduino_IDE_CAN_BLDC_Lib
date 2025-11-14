#include <RP2040PIO_CAN.h>

#include "C6x0.h"
#include "CANManager.h"
#include "DM.h"

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

const uint32_t dm_masterID = 0;
const uint32_t dm_slaveID_A = 9;  //DMモーターはSlaveID1~8の場合、速度制御モードの指令のIDが0x201~208となりロボマスモーターののフィードバックと被るので、共存させるなら9より大きいIDを使う
const uint32_t dm_slaveID_B = 10;

CANHub canHub(&CAN);
C6x0 c6x0;
DMManager dmManager;  // Default constructor

DMMotor motorA(&dmManager, dm_slaveID_A, DM_ControlMode::DM_CM_MIT);
DMMotor motorB(&dmManager, dm_slaveID_B, DM_ControlMode::DM_CM_MIT);

bool toggle_sign = false;
unsigned long last_toggle_ms = 0;

unsigned long last_print_ms = 0;
const unsigned long PRINT_INTERVAL_MS = 100;

void setup() {
  Serial.begin(115200);
  delay(1000);

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);

  CANClient* c6x0_client = canHub.createClientWithRange(0x201, 8);
  c6x0.setCAN(c6x0_client);

  CANClient* dm_client = canHub.createClientWithIds({ dm_masterID });
  dmManager.setCAN(dm_client);
  dmManager.setMasterID(dm_masterID);

  motorA.initialize();
  motorA.enable();
  motorA.setZeroPoint();
  motorB.initialize();
  motorB.enable();
  motorB.setZeroPoint();

  last_toggle_ms = millis();
}

void loop() {
  c6x0.update();
  dmManager.update();

  unsigned long now = millis();
  if (now - last_toggle_ms >= 3000) {
    last_toggle_ms = now;
    toggle_sign = !toggle_sign;
  }

  float posA = motorA.getPosition();
  float posB = motorB.getPosition();
  const float kp = 1.0f;
  const float kd = 1.0f;

  motorA.sendMIT(posB, 0.0f, kp, kd, 0.0f);
  motorB.sendMIT(posA, 0.0f, kp, kd, 0.0f);

  float c6x0_kp = 100;
  float rps = c6x0.getRpm(C610_ID_1) / 60.0f;
  float rps_ref = toggle_sign ? -10.0f : 10.0f;
  float current_ref = c6x0_kp * (rps_ref - rps);

  c6x0.setCurrent(C610_ID_1, current_ref);
  c6x0.transmit();

  if (now - last_print_ms >= PRINT_INTERVAL_MS) {
    last_print_ms = now;

    Serial.print("ID: 0x");
    Serial.print(0x201 + C610_ID_1, HEX);
    Serial.print(", Angle: ");
    Serial.print(c6x0.getPosition(C610_ID_1));
    Serial.print(" deg, RPS: ");
    Serial.print(rps);

    Serial.print("\t DM_Status A: ");
    Serial.print(static_cast<int>(motorA.getStatus()));
    Serial.print(", DM_Pos A: ");
    Serial.print(motorA.getPosition());
    Serial.print(", DM_TargetPos A: ");
    Serial.print(posB);
    Serial.print(", DM_Torque A: ");
    Serial.print(motorA.getTorque());

    Serial.print("\t DM_Status B: ");
    Serial.print(static_cast<int>(motorB.getStatus()));
    Serial.print(", DM_Pos B: ");
    Serial.print(motorB.getPosition());
    Serial.print(", DM_TargetPos B: ");
    Serial.print(posA);
    Serial.print(", DM_Torque B: ");
    Serial.print(motorB.getTorque());

    Serial.println();
  }
}
