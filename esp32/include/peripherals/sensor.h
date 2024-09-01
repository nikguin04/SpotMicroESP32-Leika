#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <memory>

#include <ArduinoJson.h>

class Sensor {
  public:
    virtual ~Sensor() = default;
    virtual void initialize() = 0;
    virtual void update() = 0;
    virtual const char* getName() const = 0;
    virtual void populateJson(JsonObject& root) = 0;
};

class DerivedSensor : public Sensor {
  public:
    virtual void updateDerived() = 0;
};

class SensorManager {
  private:
    std::vector<std::unique_ptr<Sensor>> sensors_;
    std::vector<std::unique_ptr<DerivedSensor>> derivedSensors_;

  public:
    template <typename T, typename... Args>
    T& addSensor(Args&&... args) {
        auto sensor = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *sensor;
        sensors_.push_back(std::move(sensor));
        return ref;
    }

    template <typename T, typename... Args>
    T& addDerivedSensor(Args&&... args) {
        auto sensor = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *sensor;
        derivedSensors_.push_back(std::move(sensor));
        return ref;
    }

    template <typename T>
    T* getSensor() {
        for (auto& sensor : sensors_) {
            if (auto ptr = dynamic_cast<T*>(sensor.get())) {
                return ptr;
            }
        }
        for (auto& sensor : derivedSensors_) {
            if (auto ptr = dynamic_cast<T*>(sensor.get())) {
                return ptr;
            }
        }
        return nullptr;
    }

    void initializeAll() {
        for (auto& sensor : sensors_) {
            sensor->initialize();
        }
        for (auto& sensor : derivedSensors_) {
            sensor->initialize();
        }
    }

    void updateAll() {
        for (auto& sensor : sensors_) {
            sensor->update();
        }
        for (auto& sensor : derivedSensors_) {
            sensor->update();
            sensor->updateDerived();
        }
    }

    void printSensorNames() {
        for (const auto& sensor : sensors_) {
            Serial.println(sensor->getName());
        }
        for (const auto& sensor : derivedSensors_) {
            Serial.println(sensor->getName());
        }
    }
};

extern SensorManager sensorManager;