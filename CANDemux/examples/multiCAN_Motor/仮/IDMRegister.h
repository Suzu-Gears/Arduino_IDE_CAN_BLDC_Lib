#ifndef IDMREGISTER_H
#define IDMREGISTER_H

#include <cstdint>

/**
 * @brief DMMotorクラスが実装すべき関数の仕様を定義するインターフェース。
 * このファイルがマスターリストとして機能します。
 */
class IDMRegister {
public:
    // 仮想デストラクタは必須のお作法です
    virtual ~IDMRegister() = default;

    // --- パターン1: 浮動小数点, 読み書き可能 (RW float) ---
    // 例: 電圧制限値 (Voltage Limit)
    virtual float getVoltageLimit() const = 0;
    virtual void setVoltageLimit(float value) = 0;

    // --- パターン2: 浮動小数点, 読み込み専用 (RO float) ---
    // 例: モーター温度 (Motor Temperature)
    virtual float getMotorTemperature() const = 0;
    // ROなのでセッターは宣言しない

    // --- パターン3: 整数, 読み書き可能 (RW integer) ---
    // 例: CAN ID
    virtual uint32_t getCanId() const = 0;
    virtual void setCanId(uint32_t id) = 0;

    // --- パターン4: 整数, 読み込み専用 (RO integer) ---
    // 例: シリアル番号 (Serial Number)
    virtual uint32_t getSerialNumber() const = 0;
    // ROなのでセッターは宣言しない
};

#endif // IDMREGISTER_H
