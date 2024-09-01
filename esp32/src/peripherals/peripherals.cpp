#include <peripherals/peripherals.h>

Peripherals::Peripherals(PsychicHttpServer *server, EventSocket *socket)
    : _server(server),
      _socket(socket),
      _httpEndpoint(PeripheralsConfiguration::read, PeripheralsConfiguration::update, this),
      _eventEndpoint(PeripheralsConfiguration::read, PeripheralsConfiguration::update, this, socket,
                     EVENT_CONFIGURATION_SETTINGS),
      _fsPersistence(PeripheralsConfiguration::read, PeripheralsConfiguration::update, this, &ESPFS,
                     DEVICE_CONFIG_FILE) {
    _accessMutex = xSemaphoreCreateMutex();
    addUpdateHandler([&](const String &originId) { updatePins(); }, false);
};

void Peripherals::begin() {
    _eventEndpoint.begin();
    _fsPersistence.readFromFS();

    updatePins();

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
    EXECUTE_EVERY_N_MS(200, {
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

/* ADC FUNCTIONS*/
int16_t Peripherals::readADCVoltage(uint8_t channel) {
    int16_t voltage = -1;
#if FT_ENABLED(FT_ADS1015) || FT_ENABLED(USE_ADS1115)
    float adc0 = _ads.readADC_SingleEnded(channel);
    voltage = _ads.computeVolts(adc0);
#endif
    return voltage;
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