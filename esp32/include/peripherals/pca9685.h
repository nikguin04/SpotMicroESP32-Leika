#pragma once

#include <peripherals/sensor.h>

#include <Adafruit_PWMServoDriver.h>

/*
 * Servo Settings
 */
#ifndef FACTORY_SERVO_PWM_FREQUENCY
#define FACTORY_SERVO_PWM_FREQUENCY 50
#endif

#ifndef FACTORY_SERVO_OSCILLATOR_FREQUENCY
#define FACTORY_SERVO_OSCILLATOR_FREQUENCY 27000000
#endif

#ifdef USE_SERVO
class PCA9685Sensor : public Sensor {
  private:
    Adafruit_PWMServoDriver pwm;
    bool is_active {false};
    bool sleeping {true};
    static const uint8_t CHANNEL_COUNT = 16;
    uint16_t lastSetValues[CHANNEL_COUNT];

  public:
    PCA9685Sensor(uint8_t address = 0x40) : pwm(address) { std::fill_n(lastSetValues, CHANNEL_COUNT, 0); }

    void initialize() override {
        pwm.begin();
        pwm.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
        pwm.setOscillatorFrequency(FACTORY_SERVO_OSCILLATOR_FREQUENCY);
        pwm.sleep();
    }

    void update() override {}

    const char* getName() const override { return "PCA9685"; }

    void setPWM(uint8_t channel, uint16_t value) {
        if (value < 0 || value > 4096) {
            ESP_LOGE("PCA9685", "Invalid PWM value %d for %d :: Valid range 0-4096", value, channel);
            return;
        }
        if (channel >= CHANNEL_COUNT) {
            ESP_LOGE("PCA9685", "Invalid channel id %d :: Valid range 0-16", channel);
            return;
        }
        pwm.setPWM(channel, 0, value);
        lastSetValues[channel] = value;
    }

    void deactivate() {
        sleeping = true;
        pwm.sleep();
    }

    void activate() {
        sleeping = false;
        pwm.wakeup();
    }

    bool isActive() const { return sleeping; }

    uint16_t getPWM(uint8_t channel) const { return (channel < CHANNEL_COUNT) ? lastSetValues[channel] : 0; }

    void populateJson(JsonObject& root) {}
};
#endif