#pragma once

#if USE_ADC
class ADS1115Sensor : public Sensor {
  private:
    Adafruit_ADS1115 ads;
    int16_t adc0, adc1, adc2, adc3;
    int16_t voltage0, voltage1, voltage2, voltage3;
    bool is_active {false};

  public:
    ADS1115Sensor() {}

    void initialize() override {
        is_active = ads.begin();
        if (!is_active) {
            ESP_LOGE("ADS1115Sensor", "Failed to initialize ADS1115");
            return;
        }
        ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
    }

    void update() override {
        if (!is_active) return;
        adc0 = ads.readADC_SingleEnded(0);
        adc1 = ads.readADC_SingleEnded(1);
        adc2 = ads.readADC_SingleEnded(2);
        adc3 = ads.readADC_SingleEnded(3);
        voltage0 = ads.computeVolts(adc0);
        voltage1 = ads.computeVolts(adc1);
        voltage2 = ads.computeVolts(adc2);
        voltage3 = ads.computeVolts(adc3);
    }

    const char* getName() const override { return "ADS1115"; }

    int16_t getChannel(uint8_t channel) const {
        switch (channel) {
            case 0: return adc0;
            case 1: return adc1;
            case 2: return adc2;
            case 3: return adc3;
            default: return 0;
        }
    }

    int16_t getVoltage(uint8_t channel) const {
        switch (channel) {
            case 0: return voltage0;
            case 1: return voltage1;
            case 2: return voltage2;
            case 3: return voltage3;
            default: return 0;
        }
    }

    void populateJson(JsonObject& root) const {
        root["voltage0"] = voltage0;
        root["voltage1"] = voltage1;
        root["voltage2"] = voltage2;
        root["voltage3"] = voltage3;
    }

    void printData() const {
        Serial.println("ADS1115 Readings:");
        for (int i = 0; i < 4; i++) {
            Serial.print("ADC");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(getVoltage(i));
        }
    }
};
#endif