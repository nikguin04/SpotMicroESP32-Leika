#pragma once

#include <peripherals/sensor.h>

#include <MPU6050_6Axis_MotionApps612.h>

#ifdef USE_IMU
class MPU6050Sensor : public Sensor {
  private:
    MPU6050 mpu;
    bool is_active {false};
    uint8_t devStatus {false};
    Quaternion q;
    uint8_t fifoBuffer[64];
    VectorFloat gravity;
    float ypr[3];
    float newData {false};

  public:
    MPU6050Sensor() {}

    void initialize() override {
        mpu.initialize();
        is_active = mpu.testConnection();
        devStatus = mpu.dmpInitialize();
        if (!is_active) {
            Serial.println("Failed to find MPU6050 chip");
            return;
        }
        mpu.setDMPEnabled(true);
        mpu.setI2CMasterModeEnabled(false);
        mpu.setI2CBypassEnabled(true);
        mpu.setSleepEnabled(false);
    }

    void update() override {
        if (!is_active) return;
        newData = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }

    const char* getName() const override { return "MPU6050"; }

    float getYaw() const { return ypr[0]; }
    float getPitch() const { return ypr[1]; }
    float getRoll() const { return ypr[2]; }

    void populateJson(JsonObject& root) override {
        root["x"] = getYaw();
        root["y"] = getPitch();
        root["z"] = getRoll();
    }

    void printData() const {
        Serial.print("Rotation Yaw:");
        Serial.print(getYaw());
        Serial.print("\tPitch: ");
        Serial.print(getPitch());
        Serial.print("\tRoll: ");
        Serial.print(getRoll());
        Serial.println("\t rad");
    }
};
#endif