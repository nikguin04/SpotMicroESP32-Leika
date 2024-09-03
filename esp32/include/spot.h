#ifndef Spot_h
#define Spot_h

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <features.h>
#include <webserver.h>
#include <task_manager.h>

#include <event_socket.h>

#include <peripherals/led_service.h>
#include <peripherals/servo.h>

#include <peripherals/sensor.h>
#include <peripherals/mpu6050.h>
#include <peripherals/bmp085.h>
#include <peripherals/hmc5883.h>
#include <peripherals/i2c.h>
#include <peripherals/pca9685.h>

#include <motion.h>

namespace spot {

class Spot {
  private:
    PsychicHttpServer *_server;
    WebServer _webserver;
    WiFiService _wifiService;
    APService _apService;
    SystemService _systemService;
    camera::CameraService _cameraService;
    #if USE_WS2812
    LEDService _ledService;
    #endif
    MotionService _motionService;
    EventSocket _socket;
    ServoController _servoController;

    const char *name = "spot-micro";

  protected:
    static void _loopImpl(void *_this) { static_cast<Spot *>(_this)->_loop(); }
    void _loop();
    void startServices();

  public:
    Spot(PsychicHttpServer *server);
    ~Spot();

    void begin();
    void beginAsync();
    void loop();
};

} // namespace spot

#endif
