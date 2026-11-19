#include "servo_motor.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "SERVO";

// Estructura interna
typedef struct servo_dev_t {
    uint8_t gpio;
    ledc_channel_t channel;
} servo_dev_t;

// Estado interno
static bool timer_initialized = false;

// Inicializar timer (una sola vez)
static void servo_timer_init_once(void)
{
    if (timer_initialized) return;

    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_SPEED_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_DUTY_RESOLUTION,
        .freq_hz         = SERVO_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };

    esp_err_t ret = ledc_timer_config(&timer);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Timer ya configurado, se reutiliza");
    }

    timer_initialized = true;
}

// Crear servo
esp_err_t servo_create(servo_handle_t *handle, uint8_t gpio, ledc_channel_t channel)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    servo_timer_init_once();

    servo_dev_t *dev = calloc(1, sizeof(servo_dev_t));
    if (!dev) return ESP_ERR_NO_MEM;

    dev->gpio = gpio;
    dev->channel = channel;

    ledc_channel_config_t ch = {
        .gpio_num   = gpio,
        .speed_mode = SERVO_SPEED_MODE,
        .channel    = channel,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    *handle = dev;
    return ESP_OK;
}

// Set pulso
esp_err_t servo_set_pulse_us(servo_handle_t handle, uint32_t pulsewidth)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    if (pulsewidth < SERVO_MIN_PULSEWIDTH_US)
        pulsewidth = SERVO_MIN_PULSEWIDTH_US;

    if (pulsewidth > SERVO_MAX_PULSEWIDTH_US)
        pulsewidth = SERVO_MAX_PULSEWIDTH_US;

    uint32_t max_duty = (1 << SERVO_DUTY_RESOLUTION);
    uint32_t duty = (pulsewidth * max_duty) / 20000;

    ledc_set_duty(SERVO_SPEED_MODE, handle->channel, duty);
    ledc_update_duty(SERVO_SPEED_MODE, handle->channel);

    return ESP_OK;
}

// Set ángulo
esp_err_t servo_set_angle(servo_handle_t handle, int angle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    if (angle < 0) angle = 0;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;

    uint32_t pulse = SERVO_MIN_PULSEWIDTH_US +
        ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / SERVO_MAX_DEGREE;

    return servo_set_pulse_us(handle, pulse);
}

// Eliminar servo
esp_err_t servo_delete(servo_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;

    free(handle);
    return ESP_OK;
}