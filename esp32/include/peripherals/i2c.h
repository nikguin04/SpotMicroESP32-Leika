#pragma once

#include <peripherals/sensor.h>

#include <list>
#include <SPI.h>
#include <Wire.h>

class I2CSensor : public Sensor {
  private:
    std::list<uint8_t> addressList;
    float newData {false};

  public:
    I2CSensor() {}

    void initialize() override {}

    void update() override { scanI2C(); }

    const char* getName() const override { return "I2C"; }

    void scanI2C(uint8_t lower = 0, uint8_t higher = 127) {
        addressList.clear();
        for (uint8_t address = lower; address < higher; address++) {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0) {
                addressList.emplace_back(address);
            }
        }
    }

    std::list<uint8_t> getAddresses() const { return addressList; }
    uint8_t getNumberDevices() const { return addressList.size(); }

    void populateJson(JsonObject& root) override {
        JsonArray addresses = root["addresses"].to<JsonArray>();
        for (auto& address : addressList) {
            addresses.add(address);
        }
    }

    void printData() const {
        for (uint8_t address; address < 127; address++) {
            ESP_LOGI("Peripherals", "I2C device found at address 0x%02X", address);
        }
        if (getNumberDevices() == 0)
            ESP_LOGI("Peripherals", "No I2C devices found");
        else
            ESP_LOGI("Peripherals", "Scan complete - Found %d devices", getNumberDevices());
    }
};