#pragma once

#ifdef USE_ADC
class ADS1115Sensor : public Sensor {
  private:
    Adafruit_ADS1115 ads;
    int16_t adc0, adc1, adc2, adc3;
    bool is_active {false};

  public:
    ADS1115Sensor() {}

    void initialize() override {
        is_active = ads.begin();
        if (!is_active) {
            ESP_LOGE("ADS1115Sensor", "Failed to initialize ADS1115");
        }
    }

    void update() override {
        if (!is_active) return;
        adc0 = ads.readADC_SingleEnded(0);
        adc1 = ads.readADC_SingleEnded(1);
        adc2 = ads.readADC_SingleEnded(2);
        adc3 = ads.readADC_SingleEnded(3);
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

    void populateJson(JsonObject& root) const {
        root["adc0"] = adc0;
        root["adc1"] = adc1;
        root["adc2"] = adc2;
        root["adc3"] = adc3;
    }

    void printData() const {
        Serial.println("ADS1115 Readings:");
        for (int i = 0; i < 4; i++) {
            Serial.print("ADC");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(getChannel(i));
        }
    }
};
#endif