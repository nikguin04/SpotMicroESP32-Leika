#ifndef Peripherals_h
#define Peripherals_h

#include <features.h>
#include <event_socket.h>
#include <domain/stateful_service_template.h>
#include <domain/stateful_service_persistence.h>
#include <domain/stateful_service_endpoint.h>
#include <domain/stateful_service_event.h>
#include <utilities/math_utilities.h>
#include <timing.h>

#include <list>
#include <SPI.h>
#include <Wire.h>

#include <MPU6050_6Axis_MotionApps612.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>
#include <NewPing.h>

#define DEVICE_CONFIG_FILE "/config/peripheral.json"
#define EVENT_CONFIGURATION_SETTINGS "peripheralSettings"
#define CONFIGURATION_SETTINGS_PATH "/api/peripheral/settings"

#define EVENT_I2C_SCAN "i2cScan"

#define I2C_INTERVAL 5000
#define MAX_ESP_IMU_SIZE 500
#define EVENT_IMU "imu"
#define EVENT_SERVO_STATE "servoState"

/*
 * Servo Settings
 */
#ifndef FACTORY_SERVO_PWM_FREQUENCY
#define FACTORY_SERVO_PWM_FREQUENCY 50
#endif

#ifndef FACTORY_SERVO_OSCILLATOR_FREQUENCY
#define FACTORY_SERVO_OSCILLATOR_FREQUENCY 27000000
#endif

/*
 * OLED Settings
 */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_RESET -1

/*
 * Ultrasonic Sensor Settings
 */
#define MAX_DISTANCE 200

/*
 * I2C software connection
 */
#ifndef SDA_PIN
#define SDA_PIN SDA
#endif
#ifndef SCL_PIN
#define SCL_PIN SCL
#endif
#ifndef I2C_FREQUENCY
#define I2C_FREQUENCY 100000UL
#endif

class PinConfig {
  public:
    int pin;
    String mode;
    String type;
    String role;

    PinConfig(int p, String m, String t, String r) : pin(p), mode(m), type(t), role(r) {}
};

class PeripheralsConfiguration {
  public:
    int sda = SDA_PIN;
    int scl = SCL_PIN;
    long frequency = I2C_FREQUENCY;
    std::vector<PinConfig> pins;

    static void read(PeripheralsConfiguration &settings, JsonObject &root) {
        root["sda"] = settings.sda;
        root["scl"] = settings.scl;
        root["frequency"] = settings.frequency;
    }

    static StateUpdateResult update(JsonObject &root, PeripheralsConfiguration &settings) {
        settings.sda = root["sda"] | SDA_PIN;
        settings.scl = root["scl"] | SCL_PIN;
        settings.frequency = root["frequency"] | I2C_FREQUENCY;
        return StateUpdateResult::CHANGED;
    };
};

class Peripherals : public StatefulService<PeripheralsConfiguration> {
  public:
    Peripherals(PsychicHttpServer *server, EventSocket *socket);

    void begin();

    void loop();

    void updatePins();

    void emitI2C(const String &originId = "", bool sync = false);

    void scanI2C(uint8_t lower = 1, uint8_t higher = 127);

    /* SERVO FUNCTIONS*/
    void pcaLerpTo(int index, int value, float t);

    void pcaWrite(int index, int value);

    void pcaWriteAngle(int index, int angle);

    void pcaWriteAngles(float *angles, int numServos, int offset = 0);

    void pcaActivate();

    void pcaDeactivate();

    /* ADC FUNCTIONS*/
    int16_t readADCVoltage(uint8_t channel);

    /* IMU FUNCTIONS */
    bool readIMU();

    float getTemp();

    float getAngleX();

    float getAngleY();

    float getAngleZ();

    /* MAG FUNCTIONS */
    float getHeading();

    /* BMP FUNCTIONS */
    float getAltitude();

    float getPressure();

    float getTemperature();

  protected:
    void updateImu();

    void readSonar();

    void emitSonar();

    float leftDistance() { return _left_distance; }
    float rightDistance() { return _right_distance; }

  private:
    PsychicHttpServer *_server;
    EventSocket *_socket;
    HttpEndpoint<PeripheralsConfiguration> _httpEndpoint;
    EventEndpoint<PeripheralsConfiguration> _eventEndpoint;
    FSPersistence<PeripheralsConfiguration> _fsPersistence;

    SemaphoreHandle_t _accessMutex;
    inline void beginTransaction() { xSemaphoreTakeRecursive(_accessMutex, portMAX_DELAY); }

    inline void endTransaction() { xSemaphoreGiveRecursive(_accessMutex); }

    JsonDocument doc;
    char message[MAX_ESP_IMU_SIZE];

#if FT_ENABLED(USE_SERVO)
    Adafruit_PWMServoDriver _pca;
    bool _pca_active {false};
    uint16_t pwm[16] = {0};
    uint16_t target_pwm[16] = {0};
#endif
#if FT_ENABLED(USE_IMU)
    MPU6050 _imu;
    bool imu_success {false};
    uint8_t devStatus {false};
    Quaternion q;
    uint8_t fifoBuffer[64];
    VectorFloat gravity;
    float ypr[3];
    float imu_temperature {-1};
#endif
#if FT_ENABLED(USE_MAG)
    Adafruit_HMC5883_Unified _mag;
    bool mag_success {false};
#endif
#if FT_ENABLED(USE_BMP)
    Adafruit_BMP085_Unified _bmp;
    bool bmp_success {false};
#endif
#if FT_ENABLED(FT_ADS1015)
    Adafruit_ADS1015 _ads;
#endif
#if FT_ENABLED(USE_ADS1115)
    Adafruit_ADS1115 _ads;
#endif
#if FT_ENABLED(USE_USS)
    NewPing *_left_sonar;
    NewPing *_right_sonar;
#endif
    float _left_distance {MAX_DISTANCE};
    float _right_distance {MAX_DISTANCE};

    std::list<uint8_t> addressList;
    bool i2c_active = false;
    unsigned long _updateInterval {I2C_INTERVAL};
};

#endif