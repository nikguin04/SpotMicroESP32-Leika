#pragma once

#include <peripherals/sensor.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#if USE_MAG
class HMC5883Sensor : public Sensor {
  private:
    Adafruit_HMC5883_Unified mag;
    sensors_event_t event;
    bool is_active {false};
    float heading;

  public:
    HMC5883Sensor() : mag(12345) {}

    void initialize() override {
        is_active = mag.begin();
        if (!is_active) {
            ESP_LOGE("HMC5883", "HMC5883 initialize failed");
        }
    }

    void update() override {
        if (!is_active) return;
        acquireI2CLock();
        mag.getEvent(&event);
        heading = atan2(event.magnetic.y, event.magnetic.x);
        float declinationAngle = 0.22;
        heading += declinationAngle;
        if (heading < 0) heading += 2 * PI;
        if (heading > 2 * PI) heading -= 2 * PI;
        heading *= 180 / M_PI;
        releaseI2CLock();
    }

    const char* getName() const override { return "HMC5883"; }

    float getHeading() const { return heading; }

    void populateJson(JsonObject& root) override { root["heading"] = getHeading(); }

    void printData() const {
        Serial.print("Heading: ");
        Serial.print(heading);
    }
};
#endif
