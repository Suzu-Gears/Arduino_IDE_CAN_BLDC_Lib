#ifndef DM_REGISTER_H
#define DM_REGISTER_H

#include <cstdint>
#include <map>

// パラメータのレジスタID (RID)
// !!! 注意: これは仮の定義です。実際のモーター仕様に合わせて修正してください。
enum class DM_RID : uint8_t {
  // Read/Write
  DM_RID_PMAX = 0x01,
  DM_RID_VMAX = 0x02,
  DM_RID_TMAX = 0x03,
  DM_RID_CTRL_MODE = 10,
  DM_RID_KP = 0x10,
  DM_RID_KD = 0x11,

  // Read-Only
  DM_RID_POSITION = 0x20,
  DM_RID_VELOCITY = 0x21,
  DM_RID_TORQUE = 0x22,
  DM_RID_TEMP_MOS = 0x23,
  DM_RID_TEMP_ROTOR = 0x24,
};

// --- パラメータ管理のための定義 ---
// パラメータのアクセス権限
enum class DM_ParamAccess {
  READ_ONLY,
  READ_WRITE
};

// パラメータのデータ型
enum class DM_ParamType {
  FLOAT,
  UINT32
};

// パラメータ属性を保持する構造体 (実行時用)
struct DM_ParamInfo {
  DM_ParamAccess access;
  DM_ParamType type;
  const char* name;
};

// --- コンパイル時型チェックのための定義 ---
// DM_RID から DM_ParamType へのコンパイル時マッピング
template<DM_RID Rid>
struct dm_param_trait;

// 特殊化を定義するためのヘルパーマクロ
#define DM_PARAM_TRAIT(RID, PARAM_TYPE, ACCESS_TYPE) \
  template<> struct dm_param_trait<RID> { \
    static constexpr DM_ParamType type = PARAM_TYPE; \
    static constexpr DM_ParamAccess access = ACCESS_TYPE; \
  }

// --- Read/Write Parameters ---
DM_PARAM_TRAIT(DM_RID::DM_RID_PMAX, DM_ParamType::FLOAT, DM_ParamAccess::READ_WRITE);
DM_PARAM_TRAIT(DM_RID::DM_RID_VMAX, DM_ParamType::FLOAT, DM_ParamAccess::READ_WRITE);
DM_PARAM_TRAIT(DM_RID::DM_RID_TMAX, DM_ParamType::FLOAT, DM_ParamAccess::READ_WRITE);
DM_PARAM_TRAIT(DM_RID::DM_RID_CTRL_MODE, DM_ParamType::UINT32, DM_ParamAccess::READ_WRITE);
DM_PARAM_TRAIT(DM_RID::DM_RID_KP, DM_ParamType::FLOAT, DM_ParamAccess::READ_WRITE);
DM_PARAM_TRAIT(DM_RID::DM_RID_KD, DM_ParamType::FLOAT, DM_ParamAccess::READ_WRITE);

// --- Read-Only Parameters ---
DM_PARAM_TRAIT(DM_RID::DM_RID_POSITION, DM_ParamType::FLOAT, DM_ParamAccess::READ_ONLY);
DM_PARAM_TRAIT(DM_RID::DM_RID_VELOCITY, DM_ParamType::FLOAT, DM_ParamAccess::READ_ONLY);
DM_PARAM_TRAIT(DM_RID::DM_RID_TORQUE, DM_ParamType::FLOAT, DM_ParamAccess::READ_ONLY);
DM_PARAM_TRAIT(DM_RID::DM_RID_TEMP_MOS, DM_ParamType::UINT32, DM_ParamAccess::READ_ONLY);
DM_PARAM_TRAIT(DM_RID::DM_RID_TEMP_ROTOR, DM_ParamType::UINT32, DM_ParamAccess::READ_ONLY);

#undef DM_PARAM_TRAIT

// DM_RIDの短縮エイリアス用名前空間
namespace DMR {
constexpr auto PMAX = DM_RID::DM_RID_PMAX;
constexpr auto VMAX = DM_RID::DM_RID_VMAX;
constexpr auto TMAX = DM_RID::DM_RID_TMAX;
constexpr auto CTRL_MODE = DM_RID::DM_RID_CTRL_MODE;
constexpr auto KP = DM_RID::DM_RID_KP;
constexpr auto KD = DM_RID::DM_RID_KD;
constexpr auto POSITION = DM_RID::DM_RID_POSITION;
constexpr auto VELOCITY = DM_RID::DM_RID_VELOCITY;
constexpr auto TORQUE = DM_RID::DM_RID_TORQUE;
constexpr auto TEMP_MOS = DM_RID::DM_RID_TEMP_MOS;
constexpr auto TEMP_ROTOR = DM_RID::DM_RID_TEMP_ROTOR;
}

// パラメータ定義テーブルの宣言 (本体は .cpp ファイル)
extern const std::map<DM_RID, DM_ParamInfo> dm_param_definitions;

#endif // DM_REGISTER_H
