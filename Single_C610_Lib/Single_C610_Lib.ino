// これをライブラリフォルダに入れる https://github.com/tutrc-freshman/TUTRC_ArduinoLib.git
// ESPならこれも https://github.com/eyr1n/ESP32_TWAI.git

#include <RP2040PIO_CAN.h>
#include <C6x0.h>

const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;

C6x0 c6x0;

void setup() {
  Serial.begin(115200);
  c6x0.setCAN(&CAN);
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);
}

void loop() {
  c6x0.update();

  float kp = 100;
  float rps = c6x0.getRpm(C610_ID_3) / 60.0f;
  float rps_ref = 10;
  float current_ref = kp * (rps_ref - rps);

  c6x0.setCurrent(C610_ID_3, current_ref);

  c6x0.transmit();

  Serial.print("ID: 0x");
  Serial.print(0x201 + C610_ID_3, HEX);
  Serial.print(", Angle: ");
  Serial.print(c6x0.getPosition(C610_ID_3));
  Serial.print(" deg, RPS: ");
  Serial.print(rps);
  Serial.print(", Command_mA: ");
  Serial.print(current_ref);
  Serial.print(", Actual_mA: ");
  Serial.print(c6x0.getCurrent(C610_ID_3));
  Serial.println();

  delay(1);
}
