#include <motion.h>

MotionService::MotionService(PsychicHttpServer *server, EventSocket *socket, ServoController *servoController)
    : _server(server), _socket(socket), _servoController(servoController) {}

void MotionService::begin() {
    _socket->onEvent(INPUT_EVENT, [&](JsonObject &root, int originId) { handleInput(root, originId); });

    _socket->onEvent(MODE_EVENT, [&](JsonObject &root, int originId) { handleMode(root, originId); });

    _socket->onEvent(ANGLES_EVENT, [&](JsonObject &root, int originId) { anglesEvent(root, originId); });

    _socket->onEvent(POSITION_EVENT, [&](JsonObject &root, int originId) { positionEvent(root, originId); });

    _socket->onSubscribe(ANGLES_EVENT,
                         std::bind(&MotionService::syncAngles, this, std::placeholders::_1, std::placeholders::_2));

    body_state.updateFeet(default_feet_positions);

    g_task_manager.createTask(this->_loopImpl, "MotionService", 4096, this, 3);
}

void MotionService::anglesEvent(JsonObject &root, int originId) {
    JsonArray array = root["data"].as<JsonArray>();
    for (int i = 0; i < 12; i++) {
        angles[i] = array[i];
    }
    syncAngles(String(originId));
}

void MotionService::positionEvent(JsonObject &root, int originId) {
    JsonArray array = root["data"].as<JsonArray>();
    body_state.omega = array[0];
    body_state.phi = array[1];
    body_state.psi = array[2];
    body_state.xm = array[3];
    body_state.ym = array[4];
    body_state.zm = array[5];
}

void MotionService::handleInput(JsonObject &root, int originId) {
    JsonArray array = root["data"].as<JsonArray>();
    command.lx = array[1];
    command.lx = array[1];
    command.ly = array[2];
    command.rx = array[3];
    command.ry = array[4];
    command.h = array[5];
    command.s = array[6];
    command.s1 = array[7];

    body_state.ym = (command.h + 127.f) * 0.35f / 100;

    switch (motionState) {
        case MOTION_STATE::STAND: {
            body_state.phi = command.rx / 8;
            body_state.psi = command.ry / 8;
            body_state.xm = command.ly / 2 / 100;
            body_state.zm = command.lx / 2 / 100;
            body_state.updateFeet(default_feet_positions);
            break;
        }
    }
}

void MotionService::handleMode(JsonObject &root, int originId) {
    motionState = (MOTION_STATE)root["data"].as<int>();
    ESP_LOGV("MotionService", "Mode %d", motionState);
    char output[2];
    itoa((int)motionState, output, 10);
    motionState == MOTION_STATE::DEACTIVATED ? _servoController->deactivate() : _servoController->activate();
    _socket->emit(MODE_EVENT, output, String(originId).c_str());
}

void MotionService::syncAngles(const String &originId, bool sync) {
    char output[100];
    snprintf(output, sizeof(output), "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]", angles[0],
             angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], angles[7], angles[8], angles[9],
             angles[10], angles[11]);
    _socket->emit(ANGLES_EVENT, output, originId.c_str());
    _servoController->setAngles(angles);
}

bool MotionService::updateMotion() {
    switch (motionState) {
        case MOTION_STATE::DEACTIVATED: return false;
        case MOTION_STATE::IDLE: return false;
        case MOTION_STATE::CALIBRATION: update_angles(calibration_angles, new_angles, false); break;
        case MOTION_STATE::REST: update_angles(rest_angles, new_angles, false); break;
        case MOTION_STATE::STAND: kinematics.calculate_inverse_kinematics(body_state, new_angles); break;
        case MOTION_STATE::CRAWL:
            crawlGait->step(body_state, command);
            kinematics.calculate_inverse_kinematics(body_state, new_angles);
            break;
        case MOTION_STATE::WALK:
            walkGait->step(body_state, command);
            kinematics.calculate_inverse_kinematics(body_state, new_angles);
            break;
    }
    return update_angles(new_angles, angles);
}

bool MotionService::update_angles(const float new_angles[12], float angles[12], bool useLerp) {
    bool updated = false;
    for (int i = 0; i < 12; i++) {
        float new_angle = useLerp ? lerp(angles[i], new_angles[i] * dir[i], 0.3) : new_angles[i] * dir[i];
        if (!isEqual(new_angle, angles[i], 0.1)) {
            angles[i] = new_angle;
            updated = true;
        }
    }
    return updated;
}

void MotionService::_loop() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        if (updateMotion()) syncAngles();
        _servoController->loop();
        vTaskDelayUntil(&xLastWakeTime, MotionInterval / portTICK_PERIOD_MS);
    }
}