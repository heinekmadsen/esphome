#include "modbus.h"
#include "esphome/core/log.h"

namespace esphome {
namespace modbus {

static const char *TAG = "modbus";

void Modbus::loop() {
  const uint32_t now = millis();
  if (now - this->last_modbus_byte_ > 50) {
    this->rx_buffer_.clear();
    this->last_modbus_byte_ = now;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->parse_modbus_byte_(byte)) {
      this->last_modbus_byte_ = now;
    } else {
      this->rx_buffer_.clear();
    }
  }
}

uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if ((crc & 0x01) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/// CUSTOM
unsigned int calculateCRC(uint8_t *frame, uint8_t bufferSize)
{
  uint16_t temp = 0xFFFF;
  bool flag;

  for (uint8_t i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (uint8_t j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
      {
        temp ^= 0xA001;
      }
    }
  }

  return temp;
}

// CUSTOM

bool Modbus::parse_modbus_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];

  // Byte 0: modbus address (match all)
  if (at == 0)
    return true;
  uint8_t address = raw[0];

  // Byte 1: Function (msb indicates error)
  if (at == 1)
    return (byte & 0x80) != 0x80;

  // Byte 2: Size (with modbus rtu function code 4/3)
  // See also https://en.wikipedia.org/wiki/Modbus
  if (at == 2)
    return true;

  uint8_t data_len = raw[2];
  // Byte 3..3+data_len-1: Data
  if (at < 3 + data_len)
    return true;

  // Byte 3+data_len: CRC_LO (over all bytes)
  if (at == 3 + data_len)
    return true;
  // Byte 3+len+1: CRC_HI (over all bytes)
  uint16_t computed_crc = crc16(raw, 3 + data_len);
  uint16_t remote_crc = uint16_t(raw[3 + data_len]) | (uint16_t(raw[3 + data_len + 1]) << 8);
  if (computed_crc != remote_crc) {
    ESP_LOGW(TAG, "Modbus CRC Check failed! %02X!=%02X", computed_crc, remote_crc);
    return false;
  }

  std::vector<uint8_t> data(this->rx_buffer_.begin() + 3, this->rx_buffer_.begin() + 3 + data_len);

  bool found = false;
  for (auto *device : this->devices_) {
    if (device->address_ == address) {
      device->on_modbus_data(data);
      found = true;
    }
  }
  if (!found) {
    ESP_LOGW(TAG, "Got Modbus frame from unknown address 0x%02X!", address);
  }

  // return false to reset buffer
  return false;
}

void Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus:");
  this->check_uart_settings(9600, 2);
}
float Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}
void Modbus::send(uint8_t address, uint8_t function, uint16_t start_address, uint16_t register_count) {
  uint8_t frame[8];
  frame[0] = address;
  frame[1] = function;
  frame[2] = start_address >> 8;
  frame[3] = start_address >> 0;
  frame[4] = register_count >> 8;
  frame[5] = register_count >> 0;
  auto crc = crc16(frame, 6);
  frame[6] = crc >> 0;
  frame[7] = crc >> 8;

  this->write_array(frame, 8);
}
void Modbus::send(uint8_t address, uint8_t function, uint8_t category, uint8_t page, uint8_t index, uint8_t count) {
  uint8_t addresstemp = 1; 
  uint8_t frame[8];
  frame[0] = addresstemp;
  frame[1] = function;
  frame[2] = category;
  frame[3] = index;
  frame[4] = page;
  frame[5] = count;
  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;
  ESP_LOGD(TAG,"address: 0x%02X",frame[0]);
  ESP_LOGD(TAG,"function: %02X",frame[1]);
  ESP_LOGD(TAG,"category: %02X",frame[2]);
  ESP_LOGD(TAG,"index: %02X",frame[3]);
  ESP_LOGD(TAG,"page: %02X",frame[4]);
  ESP_LOGD(TAG,"count: %02X",frame[5]);
  ESP_LOGD(TAG,"crc: %02X",frame[6]);
  ESP_LOGD(TAG,"crc: %02X",frame[7]);
  this->write_array(frame, 8);
}

}  // namespace modbus
}  // namespace esphome
