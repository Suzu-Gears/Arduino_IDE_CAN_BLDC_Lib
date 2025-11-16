#include <RP2040PIO_CAN.h>

#include <C6x0.h>
#include <CANDemux.h>
#include "DM.h"

// 各セグメント(a～g, dp)のピン番号を指定して0～9を順に表示します
// セグメントのピン番号を指定（必要に応じて変更してください）
const int segA = 10;  // a
const int segB = 11;  // b
const int segC = 6;   // c
const int segD = 9;   // d
const int segE = 8;   // e
const int segF = 12;  // f
const int segG = 13;  // g
const int segDP = 7;  // dp（小数点）
const int segPins[7] = { segA, segB, segC, segD, segE, segF, segG };

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

const uint32_t dm_masterID = 0;
const uint32_t dm_slaveID_A = 9;  //DMモーターはSlaveID1~8の場合、速度制御モードの指令のIDが0x201~208となりロボマスモーターののフィードバックと被るので、共存させるなら9より大きいIDを使う
const uint32_t dm_slaveID_B = 10;

CANDemux canHub(&CAN);

CANChannel c6x0_client = canHub.createClientWithRange(0x201, 8, 1);     //ID0x201～0x208を購読し、キューサイズ1でオーバーフロー時に最新を破棄
CANChannel dm_client = canHub.createClientWithIds({ dm_masterID }, 2);  //ID0x0を購読し、キューサイズ4でオーバーフロー時に最新を破棄

C6x0 c6x0;
DMManager dmManager(dm_masterID);
DMMotor motorA(&dmManager, dm_slaveID_A, DM_ControlMode::DM_CM_POS_VEL);
DMMotor motorB(&dmManager, dm_slaveID_B, DM_ControlMode::DM_CM_POS_VEL);

void handleC6x0Overflow() {
  digitalWrite(segDP, HIGH);
}

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

  c6x0_client.setOverflowPolicy(CANChannel::OverflowPolicy::DropOldest);
  c6x0_client.onQueueOverflow(handleC6x0Overflow);
  dm_client.setOverflowPolicy(CANChannel::OverflowPolicy::DropNewest);
  dm_client.onQueueOverflow([]() {
    digitalWrite(segG, HIGH);
  });


  c6x0.setCAN(&c6x0_client);
  dmManager.setCAN(&dm_client);

  motorA.initialize();
  motorB.initialize();
  motorA.setZeroPoint();
  motorB.setZeroPoint();
  motorA.enable();
  motorB.enable();

  // 各セグメントピンを出力に設定
  for (int i = 0; i < 7; i++) {
    pinMode(segPins[i], OUTPUT);
    digitalWrite(segPins[i], LOW);  // 初期状態OFF
  }
  pinMode(segDP, OUTPUT);
  digitalWrite(segDP, LOW);  // dp初期状態OFF
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

  motorA.sendPosition(motorA.getPosition(), 1.0f);
  motorB.sendPosition(motorB.getPosition(), 1.0f);

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
    Serial.print(motorA.getPositionDeg());
    Serial.print(", DM_TargetPos A: ");
    Serial.print(posA);
    Serial.print(", DM_Torque A: ");
    Serial.print(motorA.getTorque());

    Serial.print("\t DM_Status B: ");
    Serial.print(static_cast<int>(motorB.getStatus()));
    Serial.print(", DM_Pos B: ");
    Serial.print(motorB.getPositionDeg());
    Serial.print(", DM_TargetPos B: ");
    Serial.print(posB);
    Serial.print(", DM_Torque B: ");
    Serial.print(motorB.getTorque());

    Serial.println();
  }
}
