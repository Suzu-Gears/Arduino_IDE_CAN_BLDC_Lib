// ▼▼▼ Select Board ▼▼▼ 使うボードだけコメントアウトを外す
#define USE_BOARD_ARDUINO_R4
// #define USE_BOARD_PICO
// #define USE_BOARD_ESP32

#if defined(USE_BOARD_ARDUINO_R4)
#include <Arduino_CAN.h>

#elif defined(USE_BOARD_ESP32)
#include <ESP32_TWAI.h>  // https://github.com/eyr1n/ESP32_TWAI
const gpio_num_t CAN_TX_PIN = 22;
const gpio_num_t CAN_RX_PIN = 21;

#elif defined(USE_BOARD_PICO)
#include <RP2040PIO_CAN.h>  //https://github.com/eyr1n/RP2040PIO_CAN
const uint32_t CAN_TX_PIN = 0;  //連続してなくてもいい
const uint32_t CAN_RX_PIN = 1;  //GP1とGP3みたいな組み合わせでも動く

#else
#error "ボードが選択されていません。ファイルの先頭で USE_BOARD_... のどれか1つを有効にしてください。"

#endif

const float P_GAIN = 100.0f;              // Pゲイン
const float TARGET_RPS = 10.0f;           // 目標回転数
const int16_t CURRENT_LIMIT_MA = 10000;   // 電流指令値の制限 (mA) (-10A to +10A)
const uint32_t MOTOR_COMMAND_ID = 0x200;  // 指令を送信するCAN ID

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(115200);

#if defined(USE_BOARD_ARDUINO_R4)
  CAN.begin(CanBitRate::BR_1000k);

#elif defined(USE_BOARD_ESP32)
  CAN.begin(CanBitRate::BR_1000k, CAN_TX_PIN, CAN_RX_PIN);

#elif defined(USE_BOARD_PICO)
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);
#endif
}

void loop() {
  while (CAN.available()) {
    CanMsg rxMsg = CAN.read();

    int16_t rotorPositionRaw = (int16_t)(rxMsg.data[0] << 8 | rxMsg.data[1]);
    int16_t rpm = (int16_t)(rxMsg.data[2] << 8 | rxMsg.data[3]);
    int16_t actualTorqueCurrent = (int16_t)(rxMsg.data[4] << 8 | rxMsg.data[5]);
    //data[6] and data[7] are Null

    float rotorDegree = fmap(rotorPositionRaw, 0, 8192.0f, 0, 360.0f);
    float rps = rpm / 60.0f;

    float error = TARGET_RPS - rps;
    float targetCurrent = P_GAIN * error;
    int16_t commandCurrent = constrain(targetCurrent, -CURRENT_LIMIT_MA, CURRENT_LIMIT_MA);

    CanMsg txMsg = {};
    txMsg.id = MOTOR_COMMAND_ID;
    txMsg.data_length = 8;
    txMsg.data[0] = commandCurrent >> 8;  // モーターid1 電流値上位バイト8ビット
    txMsg.data[1] = commandCurrent;       // モーターid1 電流値下位バイト8ビット
    // txMsg.data[2] = commandCurrent >> 8;  // モーターid2 電流値上位バイト8ビット
    // txMsg.data[3] = commandCurrent;       // モーターid2 電流値下位バイト8ビット
    // txMsg.data[4] = commandCurrent >> 8;  // モーターid3 電流値上位バイト8ビット
    // txMsg.data[5] = commandCurrent;       // モーターid3 電流値下位バイト8ビット
    // txMsg.data[6] = commandCurrent >> 8;  // モーターid4 電流値上位バイト8ビット
    // txMsg.data[7] = commandCurrent;       // モーターid4 電流値下位バイト8ビット
    CAN.write(txMsg);

    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 10) {
      lastPrintTime = millis();
      Serial.print("ID: 0x");
      Serial.print(rxMsg.id, HEX);
      Serial.print(", Angle: ");
      Serial.print(rotorDegree);
      Serial.print(" deg, RPS: ");
      Serial.print(rps);
      Serial.print(", Command_mA: ");
      Serial.print(commandCurrent);
      Serial.print(", Actual_mA: ");
      Serial.print(actualTorqueCurrent);
      Serial.println();
    }
  }
  delay(1);
}
