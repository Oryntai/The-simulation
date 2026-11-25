#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Распайка стенда в README, даташит ESP32-C3 v2.5, табл. 2-1 и 2-4.
 * Допустимые выводы deep sleep: components/soc/esp32c3/include/soc/soc_caps.h. */
#define GPIO_OUT GPIO_NUM_3
#define GPIO_IN GPIO_NUM_10
#define PWM_OUT GPIO_NUM_5
#define I2C_SDA GPIO_NUM_6
#define I2C_SCL GPIO_NUM_7
#define UART_TX GPIO_NUM_0
#define UART_RX GPIO_NUM_1
#define WAKE_GPIO GPIO_NUM_4
#define WAKE_MASK (UINT64_C(1) << WAKE_GPIO)

/* Параметры опыта из README. TRM v1.4, гл. 32: XTAL/((2500/256)*1024) = 4 кГц. */
enum {
    PWM_HZ = 4000,
    DUTY_QUARTER = 256,
    DUTY_THREE_QUARTERS = 768,
    OBSERVE_MS = 20,
    I2C_ADDRESS = 0x50,
    I2C_EMPTY_ADDRESS = 0x51,
    I2C_HZ = 100000,
    I2C_TIMEOUT_MS = 50,
    UART_BAUD = 115200,
    UART_BUFFER_SIZE = 256,
    UART_TIMEOUT_MS = 1000,
    TIMER_WAKE_US = 2000000,
};

/* Метки протокола хранятся в RTC FAST. ESP-IDF components/esp_common/include/esp_attr.h. */
enum sleep_stage { COLD, WAIT_TIMER, WAIT_GPIO };
enum peripheral_failure { FAIL_GPIO = 1, FAIL_LEDC = 2, FAIL_UART = 4, FAIL_I2C = 8 };
RTC_DATA_ATTR static enum sleep_stage stage;
RTC_DATA_ATTR static unsigned failures;

static bool checked(const char *operation, esp_err_t err)
{
    if (err == ESP_OK) {
        return true;
    }
    printf("FIXTURE error %s %s\n", operation, esp_err_to_name(err));
    return false;
}

static bool check_gpio(void)
{
    const gpio_config_t output = {
        .pin_bit_mask = UINT64_C(1) << GPIO_OUT,
        .mode = GPIO_MODE_OUTPUT,
    };
    const gpio_config_t input = {
        .pin_bit_mask = UINT64_C(1) << GPIO_IN,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    if (!checked("gpio preload", gpio_set_level(GPIO_OUT, 0)) ||
        !checked("gpio output", gpio_config(&output)) ||
        !checked("gpio input", gpio_config(&input))) {
        return false;
    }

    /* Последовательность стенда. Низкий уровень при обрыве не скрывает отсутствие единицы. */
    const unsigned levels[] = {0, 1, 0};
    bool ok = true;
    for (size_t i = 0; i < sizeof(levels) / sizeof(levels[0]); ++i) {
        if (!checked("gpio write", gpio_set_level(GPIO_OUT, levels[i]))) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(OBSERVE_MS));
        int actual = gpio_get_level(GPIO_IN);
        printf("FIXTURE gpio out=%u in=%d\n", levels[i], actual);
        ok &= actual == (int)levels[i];
    }
    return ok;
}

static bool check_ledc(void)
{
    /* TRM v1.4, гл. 32. ESP-IDF components/esp_driver_ledc/include/driver/ledc.h. */
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_HZ,
        .clk_cfg = LEDC_USE_XTAL_CLK,
    };
    const ledc_channel_config_t channel = {
        .gpio_num = PWM_OUT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = DUTY_QUARTER,
        .hpoint = 0,
    };
    printf("FIXTURE ledc begin hz=%u duty=%u,%u\n", PWM_HZ, DUTY_QUARTER, DUTY_THREE_QUARTERS);
    if (!checked("ledc timer", ledc_timer_config(&timer)) ||
        !checked("ledc channel", ledc_channel_config(&channel))) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(OBSERVE_MS));
    if (!checked("ledc duty", ledc_set_duty_and_update(channel.speed_mode, channel.channel,
                                                      DUTY_THREE_QUARTERS, channel.hpoint))) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(OBSERVE_MS));
    uint32_t hz = ledc_get_freq(timer.speed_mode, timer.timer_num);
    uint32_t duty = ledc_get_duty(channel.speed_mode, channel.channel);
    printf("FIXTURE ledc readback hz=%" PRIu32 " duty=%" PRIu32 "\n", hz, duty);
    return checked("ledc stop", ledc_stop(channel.speed_mode, channel.channel, 0)) &&
           hz == PWM_HZ && duty == DUTY_THREE_QUARTERS;
}

static bool check_uart(void)
{
    /* TRM v1.4, гл. 26. UART1 независим от UART0, на котором идут сообщения. */
    const uart_config_t config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_XTAL,
    };
    /* Кадр стенда из README: чередование битов и граничные байты. */
    const uint8_t frame[] = {0x55, 0xa6, 0x00, 0xff};
    uint8_t reply[sizeof(frame)];
    if (!checked("uart config", uart_param_config(UART_NUM_1, &config)) ||
        !checked("uart pins", uart_set_pin(UART_NUM_1, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) ||
        !checked("uart install", uart_driver_install(UART_NUM_1, UART_BUFFER_SIZE, 0, 0, NULL, 0))) {
        return false;
    }
    if (!checked("uart rx irq threshold", uart_set_rx_full_threshold(UART_NUM_1, sizeof(frame)))) {
        uart_driver_delete(UART_NUM_1);
        return false;
    }
    int sent = uart_write_bytes(UART_NUM_1, frame, sizeof(frame));
    int received = uart_read_bytes(UART_NUM_1, reply, sizeof(reply), pdMS_TO_TICKS(UART_TIMEOUT_MS));
    printf("FIXTURE uart tx=%d rx=%d", sent, received);
    for (int i = 0; i < received; ++i) {
        printf(" %02x", (unsigned)reply[i]);
    }
    bool match = received == sizeof(reply) && memcmp(frame, reply, sizeof(frame)) == 0;
    printf(" %s\n", match ? "match" : "fail");
    return checked("uart delete", uart_driver_delete(UART_NUM_1)) && sent == sizeof(frame) && match;
}

static bool check_i2c(void)
{
    /* TRM v1.4, гл. 28; components/esp_driver_i2c/include/driver/i2c_master.h. */
    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    const i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDRESS,
        .scl_speed_hz = I2C_HZ,
    };
    i2c_master_bus_handle_t bus;
    printf("FIXTURE i2c begin hz=%u address=0x%02x\n", I2C_HZ, I2C_ADDRESS);
    if (!checked("i2c bus", i2c_new_master_bus(&config, &bus))) {
        return false;
    }
    esp_err_t ack = i2c_master_probe(bus, I2C_ADDRESS, I2C_TIMEOUT_MS);
    printf("FIXTURE i2c probe 0x%02x %s\n", I2C_ADDRESS, esp_err_to_name(ack));
    esp_err_t nack = i2c_master_probe(bus, I2C_EMPTY_ADDRESS, I2C_TIMEOUT_MS);
    printf("FIXTURE i2c probe 0x%02x %s\n", I2C_EMPTY_ADDRESS, esp_err_to_name(nack));
    bool payload_ok = false;
    i2c_master_dev_handle_t target;
    if (ack == ESP_OK && checked("i2c device", i2c_master_bus_add_device(bus, &device, &target))) {
        /* Контракт ведомого из README: выбор регистра и чтение через repeated START. */
        const uint8_t reg = 0x10;
        const uint8_t expected[] = {0x5a, 0xa5};
        uint8_t data[sizeof(expected)];
        if (checked("i2c combined", i2c_master_transmit_receive(target, &reg, sizeof(reg),
                                                              data, sizeof(data), I2C_TIMEOUT_MS))) {
            printf("FIXTURE i2c data %02x %02x\n", (unsigned)data[0], (unsigned)data[1]);
            payload_ok = memcmp(data, expected, sizeof(data)) == 0;
        }
        payload_ok = checked("i2c remove", i2c_master_bus_rm_device(target)) && payload_ok;
    }
    return checked("i2c delete", i2c_del_master_bus(bus)) && ack == ESP_OK &&
           nack == ESP_ERR_NOT_FOUND && payload_ok;
}

static void enter_sleep(enum sleep_stage next)
{
    stage = next;
    fflush(stdout);
    /* ESP-IDF components/esp_hw_support/include/esp_sleep.h: возврат означает отказ входа. */
    esp_err_t err = esp_deep_sleep_try_to_start();
    stage = COLD;
    printf("FIXTURE sleep rejected %s\n", esp_err_to_name(err));
}

void app_main(void)
{
    const esp_reset_reason_t reset = esp_reset_reason();
    const uint32_t causes = esp_sleep_get_wakeup_causes();
    /* ESP-IDF sleep_modes.c, esp_sleep_get_wakeup_causes: при обычном старте
     * выставлен бит ESP_SLEEP_WAKEUP_UNDEFINED. */
    if (reset == ESP_RST_POWERON && stage == COLD &&
        causes == (UINT32_C(1) << ESP_SLEEP_WAKEUP_UNDEFINED)) {
        puts("FIXTURE cold");
        failures = 0;
        if (!check_gpio()) {
            failures |= FAIL_GPIO;
        }
        if (!check_uart()) {
            failures |= FAIL_UART;
        }
        if (!check_ledc()) {
            failures |= FAIL_LEDC;
        }
        if (!check_i2c()) {
            failures |= FAIL_I2C;
        }
        if (checked("wake timer", esp_sleep_enable_timer_wakeup(TIMER_WAKE_US))) {
            printf("FIXTURE sleep timer us=%u failures=0x%x\n", TIMER_WAKE_US, failures);
            enter_sleep(WAIT_TIMER);
        }
    } else if (reset == ESP_RST_DEEPSLEEP && stage == WAIT_TIMER &&
               causes == (UINT32_C(1) << ESP_SLEEP_WAKEUP_TIMER)) {
        puts("FIXTURE wake timer");
        const gpio_config_t input = { .pin_bit_mask = WAKE_MASK, .mode = GPIO_MODE_INPUT };
        if (!checked("wake gpio input", gpio_config(&input))) {
            return;
        }
        if (gpio_get_level(WAKE_GPIO) != 1) {
            puts("FIXTURE wake gpio already active");
            return;
        }
        if (checked("wake gpio", esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
                        WAKE_MASK, ESP_GPIO_WAKEUP_GPIO_LOW))) {
            printf("FIXTURE sleep gpio pin=%d level=0\n", WAKE_GPIO);
            enter_sleep(WAIT_GPIO);
        }
    } else if (reset == ESP_RST_DEEPSLEEP && stage == WAIT_GPIO &&
               causes == (UINT32_C(1) << ESP_SLEEP_WAKEUP_GPIO)) {
        /* На C3 номера в маске RTC IO совпадают с GPIO. ESP-IDF sleep_modes.c,
         * esp_sleep_wakeup_io_bit2num и components/soc/esp32c3/include/soc/soc_caps.h. */
        const uint64_t pins = esp_sleep_get_gpio_wakeup_status();
        printf("FIXTURE wake gpio mask=0x%" PRIx64 "\n", pins);
        if (pins == WAKE_MASK && failures == 0) {
            puts("FIXTURE done");
        } else {
            printf("FIXTURE failed mask=0x%x\n", failures);
        }
        stage = COLD;
    } else {
        printf("FIXTURE unexpected boot reset=%d stage=%d causes=0x%" PRIx32 "\n", reset, stage, causes);
    }
}
