#include <spot.h>

TaskManager g_task_manager;
SensorManager sensorManager;

namespace spot {
    
NTPService ntpService;

Spot::Spot(PsychicHttpServer *server)
    : _server(server),
      _webserver(server, &_wifiService, &_apService, &_socket, &_systemService, &_cameraService),
      _socket(server),
      _servoController(server, &ESPFS, &_socket),
      _motionService(server, &_socket, &_servoController) {}

Spot::~Spot() {}

void Spot::beginAsync() {
    begin();
    g_task_manager.createTask(this->_loopImpl, "Spot main", 4096, this, 2, NULL);
    g_task_manager.createTask(_motionService._loopImpl, "MotionService", 4096, &_motionService, 3);
}

void Spot::begin() {
    ESPFS.begin(true);
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
    g_task_manager.begin();
    startServices();

#if USE_IMU
    sensorManager.addSensor<MPU6050Sensor>();
#endif

#if USE_BMP
    sensorManager.addSensor<BMP085Sensor>();
#endif

#if USE_ADC
    sensorManager.addSensor<ADS1115Sensor>();
#endif

#if USE_MAG
    sensorManager.addSensor<HMC5883Sensor>();
#endif

#if USE_SERVO
    sensorManager.addSensor<PCA9685Sensor>();
#endif

    sensorManager.addSensor<I2CSensor>();

    sensorManager.initializeAll();
}

void Spot::startServices() {
    _wifiService.begin();
    _wifiService.setupMDNS(name);
    _apService.begin();
    #if USE_NTP
    ntpService.begin();
    #endif
    _cameraService.begin();
    _socket.begin();
    _webserver.begin();
    #if USE_WS2812
    _ledService.begin();
    #endif
    _motionService.begin();
    _servoController.begin();
}

void IRAM_ATTR Spot::_loop() {
    while (true) {
        loop();
        delay(20);
    }
}

void IRAM_ATTR Spot::loop() {
    _webserver.loop();
    _wifiService.loop();
    _apService.loop();
    #if USE_WS2812
    _ledService.loop();
    #endif
}

} // namespace spot