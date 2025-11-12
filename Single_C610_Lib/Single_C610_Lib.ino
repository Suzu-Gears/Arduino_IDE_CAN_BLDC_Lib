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

#include <C6x0.h>  //https://github.com/tutrc-freshman/TUTRC_ArduinoLib.git
                   //CANライブラリよりも下で呼び出す api/HardwareCAN.hが無いって言われる

C6x0 c6x0;

void setup() {
  Serial.begin(115200);
  c6x0.setCAN(&CAN);

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
