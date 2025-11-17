#include "DMRegister.h"

// =================================================================
// --- パラメータ定義テーブル ---
// =================================================================
// モーターの全パラメータの属性を一元管理します。
// パラメータを追加・変更する場合は、ここを編集してください。
// !!! 注意: RIDは仮の定義です。実際のモーター仕様に合わせてください。
const std::map<DM_RID, DM_ParamInfo> dm_param_definitions = {
  // --- Read/Write Parameters ---
  { DM_RID::DM_RID_PMAX, { DM_ParamAccess::READ_WRITE, DM_ParamType::FLOAT, "P_MAX" } },
  { DM_RID::DM_RID_VMAX, { DM_ParamAccess::READ_WRITE, DM_ParamType::FLOAT, "V_MAX" } },
  { DM_RID::DM_RID_TMAX, { DM_ParamAccess::READ_WRITE, DM_ParamType::FLOAT, "T_MAX" } },
  { DM_RID::DM_RID_CTRL_MODE, { DM_ParamAccess::READ_WRITE, DM_ParamType::UINT32, "CTRL_MODE" } },
  { DM_RID::DM_RID_KP, { DM_ParamAccess::READ_WRITE, DM_ParamType::FLOAT, "KP" } },
  { DM_RID::DM_RID_KD, { DM_ParamAccess::READ_WRITE, DM_ParamType::FLOAT, "KD" } },

  // --- Read-Only Parameters ---
  { DM_RID::DM_RID_POSITION, { DM_ParamAccess::READ_ONLY, DM_ParamType::FLOAT, "POSITION" } },
  { DM_RID::DM_RID_VELOCITY, { DM_ParamAccess::READ_ONLY, DM_ParamType::FLOAT, "VELOCITY" } },
  { DM_RID::DM_RID_TORQUE, { DM_ParamAccess::READ_ONLY, DM_ParamType::FLOAT, "TORQUE" } },
  { DM_RID::DM_RID_TEMP_MOS, { DM_ParamAccess::READ_ONLY, DM_ParamType::UINT32, "TEMP_MOS" } },      // 仮にUINT32
  { DM_RID::DM_RID_TEMP_ROTOR, { DM_ParamAccess::READ_ONLY, DM_ParamType::UINT32, "TEMP_ROTOR" } },  // 仮にUINT32
};
