#pragma once

#include <peripherals/sensor.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#ifdef USE_BMP
class BMP085Sensor : public Sensor {
  private:
    Adafruit_BMP085_Unified bmp;
    sensors_event_t event;
    bool is_active {false};
    float temperature;
    float pressure;
    float altitude;
    constexpr static float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

  public:
    BMP085Sensor() : bmp(10085) {}

    void initialize() override {
        is_active = bmp.begin();
        if (!is_active) {
            ESP_LOGE("BMP085", "BMP initialize failed");
        }
    }

    void update() override {
        if (!is_active) return;
        acquireI2CLock();
        bmp.getTemperature(&temperature);
        bmp.getEvent(&event);
        pressure = event.pressure;
        altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure);
        releaseI2CLock();
    }

    const char* getName() const override { return "BMP085"; }

    float getTemperature() const { return temperature; }
    float getPressure() const { return pressure; }
    float getAltitude() const { return altitude; }

    void populateJson(JsonObject& root) override {
        root["pressure"] = getPressure();
        root["altitude"] = getAltitude();
        root["bmp_temp"] = getTemperature();
    }

    void printData() const {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" C");
        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.println(" Pa");
    }
};
#endif
