#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace modbus {

class ModbusDevice;

class Modbus : public uart::UARTDevice, public Component {
 public:
  Modbus() = default;

  void loop() override;

  void dump_config() override;

  void register_device(ModbusDevice *device) { this->devices_.push_back(device); }

  float get_setup_priority() const override;

  void send(uint8_t address, uint8_t function, uint16_t start_address, uint16_t register_count);
  void send(uint8_t address, uint8_t function, uint8_t category, uint8_t page, uint8_t index, uint8_t count);

 protected:
  bool parse_modbus_byte_(uint8_t byte);

  std::vector<uint8_t> rx_buffer_;
  uint32_t last_modbus_byte_{0};
  std::vector<ModbusDevice *> devices_;
};

class ModbusDevice {
 public:
  void set_parent(Modbus *parent) { parent_ = parent; }
  void set_address(uint8_t address) { address_ = address; }
  virtual void on_modbus_data(const std::vector<uint8_t> &data) = 0;

  void send(uint8_t function, uint16_t start_address, uint16_t register_count) {
    this->parent_->send(this->address_, function, start_address, register_count);
  }

  void send(uint8_t function, uint8_t category, uint8_t page, uint8_t index, uint8_t count) {
    this->parent_->send(this->address_, function, category, page, index, count);
  }  
  unsigned int calculateCRC(unsigned char *frame, unsigned char bufferSize);
 protected:
  friend Modbus;

  Modbus *parent_;
  uint8_t address_;
};

}  // namespace modbus
}  // namespace esphome
