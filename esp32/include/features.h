#ifndef Features_h
#define Features_h

#define FT_ENABLED(feature) feature

// ESP32 camera off by default
#ifndef USE_CAMERA
#define USE_CAMERA 0
#endif

// ESP32 IMU on by default
#ifndef USE_IMU
#define USE_IMU 1
#endif

// ESP32 magnetometer on by default
#ifndef USE_MAG
#define USE_MAG 0
#endif

// ESP32 barometer off by default
#ifndef USE_BMP
#define USE_BMP 0
#endif

// ESP32 SONAR off by default
#ifndef USE_USS
#define USE_USS 0
#endif

#endif
