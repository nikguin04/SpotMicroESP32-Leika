#include <peripherals/peripherals.h>

Peripherals::Peripherals(PsychicHttpServer *server, EventSocket *socket)
    : _server(server),
      _socket(socket),
      _httpEndpoint(PeripheralsConfiguration::read, PeripheralsConfiguration::update, this),
      _eventEndpoint(PeripheralsConfiguration::read, PeripheralsConfiguration::update, this, socket,
                     EVENT_CONFIGURATION_SETTINGS),
#if FT_ENABLED(USE_MAG)
      _mag(12345),
#endif
#if FT_ENABLED(USE_BMP)
      _bmp(10085),
#endif
      _fsPersistence(PeripheralsConfiguration::read, PeripheralsConfiguration::update, this, &ESPFS,
                     DEVICE_CONFIG_FILE) {
    _accessMutex = xSemaphoreCreateMutex();
    addUpdateHandler([&](const String &originId) { updatePins(); }, false);
};

void Peripherals::begin() {
    _eventEndpoint.begin();
    _fsPersistence.readFromFS();

    _socket->onEvent(EVENT_I2C_SCAN, [&](JsonObject &root, int originId) {
        scanI2C();
        emitI2C();
    });

    _socket->onSubscribe(EVENT_I2C_SCAN, [&](const String &originId, bool sync) {
        scanI2C();
        emitI2C(originId, sync);
    });

    updatePins();

#if FT_ENABLED(USE_SERVO)
    _pca.begin();
    _pca.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
    _pca.setOscillatorFrequency(FACTORY_SERVO_OSCILLATOR_FREQUENCY);
    _pca.sleep();
    _socket->onEvent(EVENT_SERVO_STATE, [&](JsonObject &root, int originId) {
        _pca_active = root["active"] | false;
        _pca_active ? pcaActivate() : pcaDeactivate();
    });
#endif

#if FT_ENABLED(USE_IMU)
    _imu.initialize();
    imu_success = _imu.testConnection();
    devStatus = _imu.dmpInitialize();
    if (!imu_success) {
        ESP_LOGE("IMUService", "MPU initialize failed");
    }
    _imu.setDMPEnabled(true);
    _imu.setI2CMasterModeEnabled(false);
    _imu.setI2CBypassEnabled(true);
    _imu.setSleepEnabled(false);
#endif
#if FT_ENABLED(USE_MAG)
    mag_success = _mag.begin();
    if (!mag_success) {
        ESP_LOGE("IMUService", "MAG initialize failed");
    }
#endif
#if FT_ENABLED(USE_BMP)
    bmp_success = _bmp.begin();
    if (!bmp_success) {
        ESP_LOGE("IMUService", "BMP initialize failed");
    }
#endif

#if FT_ENABLED(FT_ADS1015) || FT_ENABLED(USE_ADS1115)
    if (!_ads.begin()) {
        ESP_LOGE("Peripherals", "ADS1015/ADS1115 not found");
    }
    _ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
#endif

#if FT_ENABLED(USE_USS)
    _left_sonar = new NewPing(USS_LEFT_PIN, USS_LEFT_PIN, MAX_DISTANCE);
    _right_sonar = new NewPing(USS_RIGHT_PIN, USS_RIGHT_PIN, MAX_DISTANCE);
#endif
};

void Peripherals::loop() {
    EXECUTE_EVERY_N_MS(_updateInterval, {
        beginTransaction();
        updateImu();
        readSonar();
        emitSonar();
        endTransaction();
    });
}

void Peripherals::updatePins() {
    if (i2c_active) {
        Wire.end();
    }

    if (_state.sda != -1 && _state.scl != -1) {
        Wire.begin(_state.sda, _state.scl, _state.frequency);
        i2c_active = true;
    }
}

void Peripherals::emitI2C(const String &originId, bool sync) {
    char output[150];
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    root["sda"] = _state.sda;
    root["scl"] = _state.scl;
    JsonArray addresses = root["addresses"].to<JsonArray>();
    for (auto &address : addressList) {
        addresses.add(address);
    }
    serializeJson(root, output);
    ESP_LOGI("Peripherals", "Emitting I2C scan results, %s %d", originId.c_str(), sync);
    _socket->emit(EVENT_I2C_SCAN, output, originId.c_str(), sync);
}

void Peripherals::scanI2C(uint8_t lower, uint8_t higher) {
    addressList.clear();
    for (uint8_t address = lower; address < higher; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            addressList.emplace_back(address);
            ESP_LOGI("Peripherals", "I2C device found at address 0x%02X", address);
        }
    }
    uint8_t nDevices = addressList.size();
    if (nDevices == 0)
        ESP_LOGI("Peripherals", "No I2C devices found");
    else
        ESP_LOGI("Peripherals", "Scan complete - Found %d devices", nDevices);
}

/* SERVO FUNCTIONS*/
void Peripherals::pcaLerpTo(int index, int value, float t) {
#if FT_ENABLED(USE_SERVO)
    target_pwm[index] = lerp(pwm[index], value, t);
#endif
}

void Peripherals::pcaWrite(int index, int value) {
#if FT_ENABLED(USE_SERVO)
    if (value < 0 || value > 4096) {
        ESP_LOGE("Peripherals", "Invalid PWM value %d for %d :: Valid range 0-4096", value, index);
        return;
    }
    pwm[index] = value;
    target_pwm[index] = value;
    beginTransaction();
    _pca.setPWM(index, 0, value);
    endTransaction();
#endif
}

void Peripherals::pcaWriteAngle(int index, int angle) {
#if FT_ENABLED(USE_SERVO)
    _pca.setPWM(index, 0, 125 + angle * 2);
#endif
}

void Peripherals::pcaWriteAngles(float *angles, int numServos, int offset) {
#if FT_ENABLED(USE_SERVO)
    for (int i = 0; i < numServos; i++) {
        pcaWriteAngle(i + offset, angles[i]);
    }
#endif
}

void Peripherals::pcaActivate() {
#if FT_ENABLED(USE_SERVO)
    if (_pca_active) return;
    _pca_active = true;
    _pca.wakeup();
#endif
}

void Peripherals::pcaDeactivate() {
#if FT_ENABLED(USE_SERVO)
    if (!_pca_active) return;
    _pca_active = false;
    _pca.sleep();
#endif
}

/* ADC FUNCTIONS*/
int16_t Peripherals::readADCVoltage(uint8_t channel) {
    int16_t voltage = -1;
#if FT_ENABLED(FT_ADS1015) || FT_ENABLED(USE_ADS1115)
    float adc0 = _ads.readADC_SingleEnded(channel);
    voltage = _ads.computeVolts(adc0);
#endif
    return voltage;
}

/* IMU FUNCTIONS */
bool Peripherals::readIMU() {
    bool updated = false;
#if FT_ENABLED(USE_IMU)
    if (imu_success) return updated;
    updated = imu_success && _imu.dmpGetCurrentFIFOPacket(fifoBuffer);
    _imu.dmpGetQuaternion(&q, fifoBuffer);
    _imu.dmpGetGravity(&gravity, &q);
    _imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
    return updated;
}

float Peripherals::getTemp() {
    float temp = -1;
#if FT_ENABLED(USE_IMU)
    temp = imu_success ? imu_temperature : -1;
#endif
    return temp;
}

float Peripherals::getAngleX() {
    float angle = 0;
#if FT_ENABLED(USE_IMU)
    angle = imu_success ? ypr[0] * 180 / M_PI : 0;
#endif
    return angle;
}

float Peripherals::getAngleY() {
    float angle = 0;
#if FT_ENABLED(USE_IMU)
    angle = imu_success ? ypr[1] * 180 / M_PI : 0;
#endif
    return angle;
}

float Peripherals::getAngleZ() {
    float angle = 0;
#if FT_ENABLED(USE_IMU)
    angle = imu_success ? ypr[2] * 180 / M_PI : 0;
#endif
    return angle;
}

/* MAG FUNCTIONS */
float Peripherals::getHeading() {
    float heading = 0;
#if FT_ENABLED(USE_MAG)
    sensors_event_t event;
    _mag.getEvent(&event);
    heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = 0.22;
    heading += declinationAngle;
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;
    heading *= 180 / M_PI;
#endif
    return heading;
}

/* BMP FUNCTIONS */
float Peripherals::getAltitude() {
    float altitude = -1;
#if FT_ENABLED(USE_MAG)
    sensors_event_t event;
    _bmp.getEvent(&event);
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    altitude = bmp_success && event.pressure ? _bmp.pressureToAltitude(seaLevelPressure, event.pressure) : -1;
#endif
    return altitude;
}

float Peripherals::getPressure() {
    float pressure = -1;
#if FT_ENABLED(USE_BMP)
    sensors_event_t event;
    _bmp.getEvent(&event);
    pressure = bmp_success && event.pressure ? event.pressure : -1;
#endif
    return pressure;
}

float Peripherals::getTemperature() {
    float temperature = 0;
#if FT_ENABLED(USE_BMP)
    _bmp.getTemperature(&temperature);
    temperature = bmp_success ? temperature : -1;
#endif
    return temperature;
}
void Peripherals::updateImu() {
    doc.clear();
    bool newData = false;
#if FT_ENABLED(USE_IMU)
    newData = imu_success && readIMU();
    if (imu_success) {
        doc["x"] = round2(getAngleX());
        doc["y"] = round2(getAngleY());
        doc["z"] = round2(getAngleZ());
    }
#endif
#if FT_ENABLED(USE_MAG)
    newData = newData || mag_success;
    if (mag_success) {
        doc["heading"] = round2(getHeading());
    }
#endif
#if FT_ENABLED(USE_BMP)
    newData = newData || bmp_success;
    if (bmp_success) {
        doc["pressure"] = round2(getPressure());
        doc["altitude"] = round2(getAltitude());
        doc["bmp_temp"] = round2(getTemperature());
    }
#endif
    if (newData) {
        serializeJson(doc, message);
        _socket->emit(EVENT_IMU, message);
    }
}

void Peripherals::readSonar() {
#if FT_ENABLED(USE_USS)
    _left_distance = _left_sonar->ping_cm();
    delay(50);
    _right_distance = _right_sonar->ping_cm();
#endif
}

void Peripherals::emitSonar() {
#if FT_ENABLED(USE_USS)

    char output[16];
    snprintf(output, sizeof(output), "[%.1f,%.1f]", _left_distance, _right_distance);
    _socket->emit("sonar", output);
#endif
}