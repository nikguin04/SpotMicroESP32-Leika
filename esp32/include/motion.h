#pragma once

#include <event_socket.h>
#include <task_manager.h>
#include <utilities/kinematics.h>
#include <services/peripherals/servo.h>
#include <utilities/gait_utilities.h>
#include <timing.h>
#include <utilities/math_utilities.h>

#define ANGLES_EVENT "angles"
#define INPUT_EVENT "input"
#define POSITION_EVENT "position"
#define MODE_EVENT "mode"

enum class MOTION_STATE { DEACTIVATED, IDLE, CALIBRATION, REST, STAND, CRAWL, WALK };

class MotionService {
  public:
    MotionService(PsychicHttpServer *server, EventSocket *socket, ServoController *servoController);

    void begin();

    static void _loopImpl(void *_this) { static_cast<MotionService *>(_this)->_loop(); }

  private:
    PsychicHttpServer *_server;
    EventSocket *_socket;
    TaskManager *_taskManager;
    ServoController *_servoController;
    Kinematics kinematics;
    ControllerCommand command = {0, 0, 0, 0, 0, 0, 0, 0};

    friend class GaitState;

    std::unique_ptr<GaitState> crawlGait = std::make_unique<EightPhaseWalkState>();
    std::unique_ptr<GaitState> walkGait = std::make_unique<FourPhaseWalkState>();

    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;
    unsigned long _lastUpdate;
    constexpr static int MotionInterval = 15;

    body_state_t body_state = {0, 0, 0, 0, 0, 0};
    float new_angles[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    static constexpr float dir[12] = {1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1};
    static constexpr float default_feet_positions[4][4] = {
        {1, -1, 0.7, 1}, {1, -1, -0.7, 1}, {-1, -1, 0.7, 1}, {-1, -1, -0.7, 1}};

    float angles[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static constexpr float rest_angles[12] = {0, 90, -145, 0, 90, -145, 0, 90, -145, 0, 90, -145};
    static constexpr float calibration_angles[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    void anglesEvent(JsonObject &root, int originId);

    void positionEvent(JsonObject &root, int originId);

    void handleInput(JsonObject &root, int originId);

    void handleMode(JsonObject &root, int originId);

    void syncAngles(const String &originId = "", bool sync = false);

    bool updateMotion();

    bool update_angles(const float new_angles[12], float angles[12], bool useLerp = true);

    void _loop();
};
