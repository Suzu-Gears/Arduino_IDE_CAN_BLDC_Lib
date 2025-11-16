#ifndef DMREGISTER_H
#define DMREGISTER_H

#include "IDMRegister.h" // マスターリストをインクルード

// 本来は既存のDM.hにあるべきだが、サンプルとしてここに定義
enum class DM_RID : uint8_t {
    VOLTAGE_LIMIT = 0x00,
    MOTOR_TEMP = 0x0B,
    CAN_ID = 0x08,
    SERIAL_NUMBER = 0x0F,
};

// パラメータの値を保持するためのデータ構造体
struct DMRegisterData {
    float voltageLimit;
    float motorTemperature;
    uint32_t canId;
    uint32_t serialNumber;
};

/**
 * @brief IDMRegisterインターフェースを実装する具体的なモータークラス。
 *        既存のDMMotorクラスと区別するため、DMMotorWithGetSetという名前にしています。
 */
class DMMotorWithGetSet : public IDMRegister {
public:
    // コンストラクタ
    DMMotorWithGetSet();

    // --- IDMRegisterで約束した関数の実装宣言 ---
    // overrideキーワードが、マスターリストとのズレを検出します。

    // RW float
    float getVoltageLimit() const override;
    void setVoltageLimit(float value) override;

    // RO float
    float getMotorTemperature() const override;

    // RW integer
    uint32_t getCanId() const override;
    void setCanId(uint32_t id) override;

    // RO integer
    uint32_t getSerialNumber() const override;

private:
    // モーターのパラメータ状態を保持するプライベート変数
    DMRegisterData motor_state_;

    // 内部用のCAN書き込み関数（ダミー）
    // 実際には既存のDM.cppにあるような実装になります。
    bool write_can_float(DM_RID address, float value);
    bool write_can_uint32(DM_RID address, uint32_t value);
};

#endif // DMREGISTER_H
