#include <spot.h>

TaskManager g_task_manager;
SensorManager sensorManager;

namespace spot {

Spot::Spot(PsychicHttpServer *server)
    : _server(server),
      _webserver(server, &_wifiService, &_apService, &_socket, &_systemService, &_ntpService, &_cameraService),
      _socket(server),
      _ntpService(server),
      _servoController(server, &ESPFS, &_peripherals, &_socket),
      _peripherals(server, &_socket),
      _motionService(server, &_socket, &_servoController) {}

Spot::~Spot() {}

void Spot::beginAsync() {
    begin();
    g_task_manager.createTask(this->_loopImpl, "Spot main", 4096, this, 2, NULL);
    g_task_manager.createTask(_motionService._loopImpl, "MotionService", 4096, &_motionService, 3);
}

void Spot::begin() {
    ESPFS.begin(true);
    g_task_manager.begin();
    startServices();

#ifdef USE_IMU
    sensorManager.addSensor<MPU6050Sensor>();
#endif

#ifdef USE_BMP
    sensorManager.addSensor<BMP085Sensor>();
#endif

#ifdef USE_ADC
    sensorManager.addSensor<ADS1115Sensor>();
#endif

#ifdef USE_MAG
    sensorManager.addSensor<HMC5883Sensor>();
#endif

#ifdef USE_SERVO
    sensorManager.addSensor<PCA9685Sensor>();
#endif

    sensorManager.addSensor<I2CSensor>();

    sensorManager.initializeAll();
}

void Spot::startServices() {
    _wifiService.begin();
    _wifiService.setupMDNS(name);
    _apService.begin();
    _ntpService.begin();
    _cameraService.begin();
    _socket.begin();
    _webserver.begin();
    _ledService.begin();
    _motionService.begin();
    _peripherals.begin();
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
    _ledService.loop();
    EXECUTE_EVERY_N_MS(200, sensorManager.updateAll());
}

} // namespace spot