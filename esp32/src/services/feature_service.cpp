#include <services/feature_service.h>

static const char *TAG = "FeatureService";

FeaturesService::FeaturesService() {}
FeaturesService::~FeaturesService() {}

void FeaturesService::features(JsonObject &root) {
    root["camera"] = USE_CAMERA;
    root["imu"] = USE_IMU;
    root["mag"] = USE_MAG;
    root["bmp"] = USE_BMP;
    root["sonar"] = USE_USS;
    root["firmware_version"] = APP_VERSION;
    root["firmware_name"] = APP_NAME;
    root["firmware_built_target"] = BUILD_TARGET;
}

esp_err_t FeaturesService::getFeatures(PsychicRequest *request) {
    ESP_LOGV(TAG, "Retrieving features");
    PsychicJsonResponse response = PsychicJsonResponse(request, false);
    JsonObject root = response.getRoot();
    features(root);

    return response.send();
}
