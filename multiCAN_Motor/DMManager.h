#pragma once

#include <map>
#include "DM_Motor.h"
#include "CANManager.h"

class DMManager {
public:
    DMManager(CANClient* client, uint32_t masterId)
        : client_(client), masterId_(masterId) {}

    void registerMotor(DMMotor* motor) {
        if (motor) {
            registered_motors_[motor->getSlaveId()] = motor;
        }
    }

    void update() {
        if (!client_) return;

        while (client_->available()) {
            CanMsg msg = client_->read();
            
            if (msg.getStandardId() != masterId_) {
                continue;
            }

            // DMMotorから定数を借りてslaveIdを抽出
            uint8_t slaveId_from_feedback = msg.data[0] & DMMotor::FEEDBACK_SLAVE_ID_MASK;

            auto it = registered_motors_.find(slaveId_from_feedback);
            if (it != registered_motors_.end()) {
                // processMessageはmasterIdのチェックを内部で行うので、そのまま渡す
                it->second->processMessage(msg);
            }
        }
    }

private:
    CANClient* client_;
    uint32_t masterId_;
    std::map<uint32_t, DMMotor*> registered_motors_;
};
