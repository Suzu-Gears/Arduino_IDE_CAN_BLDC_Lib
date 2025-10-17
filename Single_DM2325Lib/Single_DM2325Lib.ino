#include <RP2040PIO_CAN.h>
#include "DM2325.h"

// 以下の変数の値を変更して、動作モードを選択します
// 1: MITモード
// 2: 位置制御モード
// 3: 速度制御モード
const int CONTROL_MODE = 1;

// --- CAN設定 ---
const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;

// --- モーター設定 ---
const byte MOTOR_ID = 0x01;
DMMotorController motor(MOTOR_ID, CAN); 

// --- 制御パラメータ ---
const float VELOCITY_LIMIT_RPM = 0.0f;
const float MIT_KP = 0.0f;
const float MIT_KD = 0.0f;

// --- 状態管理用の変数 ---
int motionStep = 0;
unsigned long stepTimer = 0; 

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("--- DMMotorController Library Demo ---");

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  if (CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN bus initialized.");
  } else {
    Serial.println("CAN bus initialization failed!");
    while(1);
  }

  motor.begin(); // モーターを有効化

  motor.playStartupBeep(2000, 100);
  delay(50);
  motor.playStartupBeep(4000, 100);

  motor.setMode((DMMotorController::ControlMode)CONTROL_MODE); 
  Serial.print("Initial mode set to: ");
  Serial.println(CONTROL_MODE);
  
  delay(500);
  Serial.println("Operation started.");

  stepTimer = millis();
  motionStep = 1;
}

void loop() {
  // CONTROL_MODEの値に応じて指令を送信
  switch (CONTROL_MODE) {
    case 1: // ① MITモード
      if (motionStep == 1) {
        motor.setMIT(0.0f, 0.0f, MIT_KP, MIT_KD, 0.0f); // 0度
        if (millis() - stepTimer > 2000) {
          motionStep = 2; // 次のステップへ
          stepTimer = millis();
        }
      } else if (motionStep == 2) {
        motor.setMIT(1.57f, 0.0f, MIT_KP, MIT_KD, 0.0f); // 90度
        if (millis() - stepTimer > 2000) {
          motionStep = 3; // 次のステップへ
          stepTimer = millis();
        }
      } else if (motionStep == 3) {
        motor.setMIT(-1.57f, 0.0f, MIT_KP, MIT_KD, 0.0f); // -90度で停止
      }
      break;

    case 2: // ② 位置制御モード
      if (motionStep == 1) {
        motor.goToPosition(0.0f, VELOCITY_LIMIT_RPM); // 0度
        if (millis() - stepTimer > 2000) {
          motionStep = 2; // 次のステップへ
          stepTimer = millis();
        }
      } else if (motionStep == 2) {
        motor.goToPosition(1.57f, VELOCITY_LIMIT_RPM); // 90度
        if (millis() - stepTimer > 2000) {
          motionStep = 3; // 次のステップへ
          stepTimer = millis();
        }
      } else if (motionStep == 3) {
        motor.goToPosition(1.57f, VELOCITY_LIMIT_RPM); // 90度で停止
      }
      break;

    case 3: // ③ 速度制御モード
      motor.setVelocityRPM(60.0f);
      break;
  }

  // フィードバックデータを読み込んで表示
  MotorFeedback feedback;
  if (motor.readFeedback(feedback)) {
    Serial.print("Step: "); Serial.print(motionStep);
    Serial.print(", Pos: "); Serial.print(feedback.position_deg, 2);
    Serial.print(" deg, Vel: "); Serial.print(feedback.velocity_rpm, 2);
    Serial.print(" RPM, Torque: "); Serial.print(feedback.torque_nm, 3);
    Serial.println(" Nm");
  }
  
  delay(1);
}
