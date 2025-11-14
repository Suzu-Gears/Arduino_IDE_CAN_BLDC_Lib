#include "DMManager.h"
#include "DM_Motor.h"

// --- Method Implementations for DMManager ---

DMManager::DMManager(CANHub* hub, uint32_t masterId)
    : masterId_(masterId) {
    if (hub) {
        client_ = hub->createClientWithIds({ masterId_ });
    }
}

void DMManager::registerMotor(uint32_t slaveId, DMMotor* motor) {
    if (motor) {
        motors_[slaveId] = motor;
    }
}

void DMManager::update() {
    if (!client_) return;

    while (client_->available()) {
        CanMsg msg = client_->read();
        
        if (msg.getStandardId() != masterId_) {
            continue;
        }

        uint8_t slaveId = DMMotor::getSlaveIdFromMessage(msg);
        auto it = motors_.find(slaveId);
        if (it != motors_.end()) {
            it->second->processMessage(msg);
        }
    }
}
