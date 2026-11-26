#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Даташит ESP32-C3 v2.5, таблицы 2-4 и 2-7: GPIO3 доступен для цифрового выхода.
constexpr uint8_t blinkPin = 3;
// Полупериод штатного примера Arduino Blink.
constexpr uint32_t halfPeriodMs = 1000;

void setup() {
    // ESP-IDF v6.1.1, components/esp_stdio/Kconfig: скорость UART консоли по умолчанию.
    Serial.begin(115200);
    pinMode(blinkPin, OUTPUT);
}

static void writeLevel(uint8_t level) {
    // Arduino ESP32 4.0.0, cores/esp32/esp32-hal-misc.c: delay вызывает vTaskDelay.
    const TickType_t tick = xTaskGetTickCount();
    // ESP-IDF v6.1.1, components/esp_timer/include/esp_timer.h: микросекунды от запуска таймера.
    const int64_t before = esp_timer_get_time();
    digitalWrite(blinkPin, level);
    const int64_t after = esp_timer_get_time();
    Serial.printf("Blink GPIO %u %u %lu %lld %lld\n", unsigned(blinkPin), unsigned(level),
                  static_cast<unsigned long>(tick),
                  static_cast<long long>(before), static_cast<long long>(after));
}

void loop() {
    writeLevel(HIGH);
    delay(halfPeriodMs);
    writeLevel(LOW);
    delay(halfPeriodMs);
}
