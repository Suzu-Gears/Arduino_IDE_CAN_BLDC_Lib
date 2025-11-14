#pragma once

#include <map>
#include <memory>
#include "DM_Motor.h"
#include "CANManager.h"

class DMManager {
public:
  DMManager(CANHub* hub, uint32_t masterId)
    : masterId_(masterId) {
    if (hub) {
      client_ = hub->createClientWithIds({ masterId_ });
    }
  }

  // Add a motor to be managed. The manager takes ownership.
  void addMotor(uint32_t slaveId, DM_ControlMode mode) {
    if (client_) {
      motors_[slaveId] = std::make_unique<DMMotor>(client_, masterId_, slaveId, mode);
    }
  }

  // Get a reference to a motor. Allows for dot-notation access.
  DMMotor& getMotor(uint32_t slaveId) {
    return *motors_.at(slaveId);
  }

  // Update feedback for all managed motors.
  void update() {
    if (!client_) return;

    while (client_->available()) {
      CanMsg msg = client_->read();

      // Master ID check is already done by the client filter, but for safety:
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

  // Initialize all managed motors.
  void initialize() {
    for (auto const& [slaveId, motor] : motors_) {
      motor->initialize();
    }
  }

  // Enable all managed motors.
  void enable() {
    for (auto const& [slaveId, motor] : motors_) {
      motor->enable();
    }
  }

  // Disable all managed motors.
  void disable() {
    for (auto const& [slaveId, motor] : motors_) {
      motor->disable();
    }
  }

private:
  CANClient* client_ = nullptr;
  uint32_t masterId_;
  std::map<uint32_t, std::unique_ptr<DMMotor>> motors_;
};
