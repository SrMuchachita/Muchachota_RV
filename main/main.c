#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_err.h"

/* ============================================================
 * CONFIGURACIÓN GENERAL
 * ============================================================ */

// Pines
#define GPIO_INH          4
#define GPIO_PWM_A        5
#define GPIO_PWM_B        6

// PWM
#define PWM_FREQ_HZ        10000        // 10 kHz
#define PWM_RESOLUTION_HZ  10000000     // 10 MHz → 1 tick = 100 ns
#define PWM_PERIOD_TICKS  (PWM_RESOLUTION_HZ / PWM_FREQ_HZ) // 1000 ticks

static const char *TAG = "MOTOR";

/* ============================================================
 * HANDLES MCPWM
 * ============================================================ */
static mcpwm_timer_handle_t timer;
static mcpwm_oper_handle_t  oper;
static mcpwm_cmpr_handle_t  comparator_a;
static mcpwm_cmpr_handle_t  comparator_b;
static mcpwm_gen_handle_t   gen_a;
static mcpwm_gen_handle_t   gen_b;

/* ============================================================
 * ENUM DIRECCIÓN
 * ============================================================ */
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} motor_dir_t;

static motor_dir_t current_dir = MOTOR_STOP;
static uint16_t current_speed = 0;

/* ============================================================
 * PROTOTIPOS
 * ============================================================ */
static void motor_hw_init(void);
static void motor_set_speed(uint16_t duty_0_1000);
static void motor_set_direction(motor_dir_t dir);
static void motor_enable(bool en);

/* ============================================================
 * APP MAIN
 * ============================================================ */
void app_main(void)
{
    motor_hw_init();
    motor_enable(true);

    while (1) {

        ESP_LOGI(TAG, "Forward 30%%");
        motor_set_direction(MOTOR_FORWARD);
        motor_set_speed(300);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Forward 70%%");
        motor_set_speed(700);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Forward 90%%");
        motor_set_speed(900);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Forward 100%%");
        motor_set_speed(1000);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Stop");
        motor_set_direction(MOTOR_STOP);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Reverse 30%%");
        motor_set_direction(MOTOR_REVERSE);
        motor_set_speed(300);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Reverse 70%%");
        motor_set_speed(700);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Reverse 90%%");
        motor_set_speed(900);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Reverse 100%%");
        motor_set_speed(1000);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Stop");
        motor_set_direction(MOTOR_STOP);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ============================================================
 * INICIALIZACIÓN HARDWARE
 * ============================================================ */
static void motor_hw_init(void)
{
    /* ---------- INH ---------- */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_INH),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_INH, 0);

    /* ---------- TIMER ---------- */
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .period_ticks = PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    /* ---------- OPERATOR ---------- */
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    /* ---------- COMPARADORES ---------- */
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &comparator_a));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &comparator_b));

    /* ---------- GENERADORES ---------- */
    mcpwm_generator_config_t gen_cfg = {0};

    gen_cfg.gen_gpio_num = GPIO_PWM_A;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen_a));

    gen_cfg.gen_gpio_num = GPIO_PWM_B;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen_b));

    /* ---------- PWM BASE GEN A ---------- */
    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_timer_event(
            gen_a,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_compare_event(
            gen_a,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                comparator_a,
                MCPWM_GEN_ACTION_LOW)));

    /* ---------- PWM BASE GEN B ---------- */
    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_timer_event(
            gen_b,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(
        mcpwm_generator_set_action_on_compare_event(
            gen_b,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                comparator_b,
                MCPWM_GEN_ACTION_LOW)));

    /* ---------- ARRANQUE ---------- */
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Motor HW inicializado");
}

/* ============================================================
 * VELOCIDAD (0–1000)
 * ============================================================ */
static void motor_set_speed(uint16_t duty_0_1000)
{
    if (duty_0_1000 > 1000) duty_0_1000 = 1000;
    current_speed = duty_0_1000;

    uint32_t duty_ticks =
        (PWM_PERIOD_TICKS * duty_0_1000) / 1000;

    switch (current_dir) {

    case MOTOR_FORWARD:
        mcpwm_comparator_set_compare_value(comparator_a, duty_ticks);
        mcpwm_comparator_set_compare_value(comparator_b, 0);
        break;

    case MOTOR_REVERSE:
        mcpwm_comparator_set_compare_value(comparator_a, 0);
        mcpwm_comparator_set_compare_value(comparator_b, duty_ticks);
        break;

    case MOTOR_STOP:
    default:
        mcpwm_comparator_set_compare_value(comparator_a, 0);
        mcpwm_comparator_set_compare_value(comparator_b, 0);
        break;
    }
}

/* ============================================================
 * DIRECCIÓN
 * ============================================================ */
static void motor_set_direction(motor_dir_t dir)
{
    current_dir = dir;
    motor_set_speed(current_speed);
}

/* ============================================================
 * ENABLE PUENTE
 * ============================================================ */
static void motor_enable(bool en)
{
    gpio_set_level(GPIO_INH, en ? 1 : 0);
}
