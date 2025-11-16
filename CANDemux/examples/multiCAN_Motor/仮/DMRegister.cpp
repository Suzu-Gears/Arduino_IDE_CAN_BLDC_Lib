#include "DMRegister.h"

// Arduino環境外でテストする場合、Serialをダミーで定義
#ifndef ARDUINO
#include <iostream>
#define Serial std::cout
#endif

// コンストラクタで初期値を設定
DMMotorWithGetSet::DMMotorWithGetSet() {
    motor_state_ = {
        .voltageLimit = 24.0f,
        .motorTemperature = 45.5f,
        .canId = 1,
        .serialNumber = 123456789 // シリアル番号は本来モーターから読み込む
    };
}

// --- RW float ---
float DMMotorWithGetSet::getVoltageLimit() const {
    return motor_state_.voltageLimit;
}

void DMMotorWithGetSet::setVoltageLimit(float value) {
    Serial.print("Setting Voltage Limit to ");
    Serial.println(value);
    
    // ここでCAN経由でモーターに値を書き込む処理を呼び出す
    if (write_can_float(DM_RID::VOLTAGE_LIMIT, value)) {
        motor_state_.voltageLimit = value; // 書き込みが成功したら内部状態を更新
        Serial.println("...Done.");
    } else {
        Serial.println("...Failed to write to CAN.");
    }
}

// --- RO float ---
float DMMotorWithGetSet::getMotorTemperature() const {
    // 実際にはCAN経由で定期的に値を取得し、motor_state_を更新する
    return motor_state_.motorTemperature;
}

// --- RW integer ---
uint32_t DMMotorWithGetSet::getCanId() const {
    return motor_state_.canId;
}

void DMMotorWithGetSet::setCanId(uint32_t id) {
    Serial.print("Setting CAN ID to ");
    Serial.println(id);

    // ここでCAN経由でモーターに値を書き込む処理を呼び出す
    if (write_can_uint32(DM_RID::CAN_ID, id)) {
        motor_state_.canId = id; // 内部状態を更新
        Serial.println("...Done.");
    } else {
        Serial.println("...Failed to write to CAN.");
    }
}

// --- RO integer ---
uint32_t DMMotorWithGetSet::getSerialNumber() const {
    // 読み取り専用なので、内部状態を返すだけ
    return motor_state_.serialNumber;
}


// --- 以下はダミーの内部関数 ---
bool DMMotorWithGetSet::write_can_float(DM_RID address, float value) {
    // (ここにCANフレームを送信するコードが入る)
    // 成功したと仮定してtrueを返す
    return true;
}

bool DMMotorWithGetSet::write_can_uint32(DM_RID address, uint32_t value) {
    // (ここにCANフレームを送信するコードが入る)
    // 成功したと仮定してtrueを返す
    return true;
}
