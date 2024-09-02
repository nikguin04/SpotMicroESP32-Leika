#include <peripherals/led_service.h>

LEDService::LEDService() {
    FastLED.addLeds<CHIPSET, WS2812_PIN, COLOR_ORDER>(leds, WS2812_NUM_LEDS).setCorrection(TypicalLEDStrip);
    currentPalette = OceanColors_p;
    currentBlending = LINEARBLEND;
}

LEDService::~LEDService() {}

void LEDService::begin() { loop(); }

void LEDService::loop() {
    EXECUTE_EVERY_N_MS(1000 / 60, {
        if (_brightness >= 200) direction = -5;
        if (_brightness <= 50) direction = 5;
        _brightness += direction;
        if (WiFi.isConnected()) {
            CRGB color = ColorFromPalette(ForestColors_p, 0, _brightness, currentBlending);
            fillColor(color);
        } else {
            CRGB color = ColorFromPalette(OceanColors_p, 0, _brightness, currentBlending);
            fillColor(color);
        }
        FastLED.show();
    });
}

void LEDService::fillColor(CRGB color) {
    for (int i = 0; i < WS2812_NUM_LEDS; ++i) {
        leds[i] = color;
    }
}
