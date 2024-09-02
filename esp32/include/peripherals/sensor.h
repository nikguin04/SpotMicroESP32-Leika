#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <memory>

#include <ArduinoJson.h>
#include <utilities/math_utilities.h>

class I2CSemaphore {
  public:
    static inline SemaphoreHandle_t semaphore;

    static void initialize() { semaphore = xSemaphoreCreateMutex(); }

    static void take() { xSemaphoreTake(semaphore, portMAX_DELAY); }

    static void give() { xSemaphoreGive(semaphore); }
};

class Sensor {
  public:
    virtual ~Sensor() = default;
    virtual void initialize() = 0;
    virtual void update() = 0;
    virtual const char* getName() const = 0;
    virtual void populateJson(JsonObject& root) = 0;

  protected:
    void acquireI2CLock() { I2CSemaphore::take(); }

    void releaseI2CLock() { I2CSemaphore::give(); }
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
    SensorManager() { I2CSemaphore::initialize(); }

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
        I2CSemaphore::take();
        for (auto& sensor : sensors_) {
            sensor->initialize();
        }
        for (auto& sensor : derivedSensors_) {
            sensor->initialize();
        }
        I2CSemaphore::give();
    }

    void updateAll() {
        I2CSemaphore::take();
        for (auto& sensor : sensors_) {
            sensor->update();
        }
        for (auto& sensor : derivedSensors_) {
            sensor->update();
            sensor->updateDerived();
        }
        I2CSemaphore::give();
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