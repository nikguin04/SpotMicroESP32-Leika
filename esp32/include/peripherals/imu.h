#pragma once

#include <list>
#include <SPI.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <utils/math_utils.h>

#if FT_ENABLED(USE_MPU6050)
#include <MPU6050_6Axis_MotionApps612.h>
#endif

#if FT_ENABLED(USE_ICM20948)
#include "ICM_20948.h" 
#endif

#if FT_ENABLED(USE_BNO055)
#include <Adafruit_BNO055.h>
#endif



struct IMUAnglesMsg {
    float rpy[3] {0, 0, 0};
    float temperature {-1};
    bool success {false};

    friend void toJson(JsonVariant v, IMUAnglesMsg const& a) {
        JsonArray arr = v.to<JsonArray>();
        arr.add(a.rpy[0]);
        arr.add(a.rpy[1]);
        arr.add(a.rpy[2]);
        arr.add(a.temperature);
        arr.add(a.success);
    }

    void fromJson(JsonVariantConst o) {
        JsonArrayConst arr = o.as<JsonArrayConst>();
        rpy[0] = arr[0].as<float>();
        rpy[1] = arr[1].as<float>();
        rpy[2] = arr[2].as<float>();
        temperature = arr[3].as<float>();
        success = arr[4].as<bool>();
    }
};

class IMU {
  public:
    IMU()
#if FT_ENABLED(USE_BNO055)
        : _imu(55, 0x29)
#endif
    {
    }
    bool initialize() {
#if FT_ENABLED(USE_MPU6050)
        _imu.initialize();
        imuMsg.success = _imu.testConnection();
        if (!imuMsg.success) return false;
        devStatus = _imu.dmpInitialize();
        if (devStatus == 0) {
            _imu.setDMPEnabled(false);
            _imu.setDMPConfig1(0x03);
            _imu.setDMPEnabled(true);
            _imu.setI2CMasterModeEnabled(false);
            _imu.setI2CBypassEnabled(true);
            _imu.setSleepEnabled(false);
        } else {
            return false;
        }
#endif
#if FT_ENABLED(USE_BNO055)
        imuMsg.success = _imu.begin();
        if (!imuMsg.success) {
            return false;
        }
        _imu.setExtCrystalUse(true);
#endif
#if FT_ENABLED(USE_ICM20948)
    #if USE_ICM20948_SPIMODE > 0
        _imu.begin(CS_PIN, SPI_PORT);
    #else
        _imu.begin(Wire, 1);
    #endif
    if (_imu.status != ICM_20948_Stat_Ok){ return false; }
    
    _imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    if (_imu.status != ICM_20948_Stat_Ok){ return false; }
    
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm2;
    myFSS.g = dps250;
    _imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (_imu.status != ICM_20948_Stat_Ok){ return false; }
    // TODO: Setup low pass filter config
    _imu.startupMagnetometer();
    if (_imu.status != ICM_20948_Stat_Ok){ return false; }
#endif
        return true;
    }

    bool readIMU() {
        if (!imuMsg.success) return false;
#if FT_ENABLED(USE_MPU6050)
        if (_imu.dmpPacketAvailable()) {
            if (_imu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                _imu.dmpGetQuaternion(&q, fifoBuffer);
                _imu.dmpGetGravity(&gravity, &q);
                _imu.dmpGetYawPitchRoll(imuMsg.rpy, &q, &gravity);
                return true;
            }
        }
        return false;
#endif
#if FT_ENABLED(USE_BNO055)
        sensors_event_t event;
        _imu.getEvent(&event);
        imuMsg.rpy[0] = event.orientation.x;
        imuMsg.rpy[1] = event.orientation.y;
        imuMsg.rpy[2] = event.orientation.z;
#endif
#if FT_ENABLED(USE_ICM20948)
        if (_imu.dataReady())
        {
            _imu.getAGMT();
            imuMsg.rpy[0] = _imu.magX();
            imuMsg.rpy[1] = _imu.magY();
            imuMsg.rpy[2] = _imu.magZ();
        }
#endif
        return true;
    }

    float getTemperature() { return imuMsg.temperature; }

    float getAngleX() { return imuMsg.rpy[2]; }

    float getAngleY() { return imuMsg.rpy[1]; }

    float getAngleZ() { return imuMsg.rpy[0]; }

    bool isActive() { return imuMsg.success; }

    IMUAnglesMsg getIMUAngles() { return imuMsg; }

    void readIMU(JsonObject& root) {
        if (!imuMsg.success) return;
        root["x"] = round2(getAngleX());
        root["y"] = round2(getAngleY());
        root["z"] = round2(getAngleZ());
    }

  private:
#if FT_ENABLED(USE_MPU6050)
    MPU6050 _imu;
    uint8_t devStatus {false};
    Quaternion q;
    uint8_t fifoBuffer[64];
    VectorFloat gravity;
#endif
#if FT_ENABLED(USE_BNO055)
    Adafruit_BNO055 _imu;
#endif
#if FT_ENABLED(USE_ICM20948)
    #if FT_ENABLED(USE_ICM20948_SPIMODE) > 0
        #define SPI_PORT SPI // TODO in periphearals_seetings.h
        #define CS_PIN 2 
        ICM_20948_SPI _imu;
    #else
        //#define WIRE_PORT Wire 
        ICM_20948_I2C _imu;
    #endif
#endif
    IMUAnglesMsg imuMsg;
};