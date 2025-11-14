#pragma once

#include <map>
#include "CANManager.h"

// Forward-declare DMMotor to avoid circular dependencies
class DMMotor;

class DMManager {
public:
    // Constructor declaration
    DMManager(CANHub* hub, uint32_t masterId);

    // Method for DMMotor instances to register themselves
    void registerMotor(uint32_t slaveId, DMMotor* motor);

    // Polls CAN messages and dispatches them to the correct motor
    void update();

    // Getters for DMMotor to retrieve necessary info
    CANClient* getCanClient() const { return client_; }
    uint32_t getMasterId() const { return masterId_; }

private:
    CANClient* client_ = nullptr;
    uint32_t masterId_;
    std::map<uint32_t, DMMotor*> motors_; // Manages non-owning pointers to motors
};

