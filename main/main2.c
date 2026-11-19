// ============================================================
// RD90 Pro - Firmware ESP32-S3
// Versión corregida - Abril 2026
//
// Correcciones aplicadas:
//  [FIX-1] current_dir inicializada en MOTOR_STOP (motores se iban solos)
//  [FIX-2] last_motor_activity_time inicializada con tiempo real al arranque
//  [FIX-3] Ángulos IMU sin escala x100, guardados directamente en grados
//  [FIX-4] DOUT0 inicializado en vGpioInit
//  [FIX-5] vTaskImuMonitor con un solo vTaskDelay al final del ciclo
//  [FIX-6] Validación de errores en vI2cInit
//  [FIX-7] motorDcControl marcada como función de test, no se llama en producción
//  [FIX-8] Watchdog de motores separado y limpio al inicio del loop
//  [FIX-9] Timeout en wait_for_sensor: while(timeout-- > 0) en lugar de while(timeout--)
//  [FIX-10] Motor atorado: agregar target_vel=0 cuando cmd==MOTOR_STOP para rampa suave
//  [FIX-11] Filtro complementario: acelerómetro + giroscopio para ángulos estables
//  [FIX-12] Calibración de ángulos IMU: offset referenceable activable por código
//  [FIX-13] Lectura de temperatura cada 500ms en lugar de 100ms para evitar errores I2C
//  [FIX-14] Delays después de encender IMU + aumentar periodo de lectura a 200ms
//  [FIX-15] Calibración automática al inicio + logs de ángulos cada 1 segundo en serial
//  [FIX-17] Logs de dirección y velocidad de motores cada 2 segundos en serial
//  [FIX-18] Prevenir underflow en rampa suave: verificar que current_vel >= RAMP_STEP
// ============================================================

//ANSI C
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// ── Interruptor de compilacion: WiFi + OTA ───────────────────────────────────
// En 0 NO se compila nada de WiFi/OTA (ahorra flash; es el estado actual).
// Para reactivar: poner en 1 Y volver a anadir esp_wifi, esp_netif y ota_http
// en REQUIRES de main/CMakeLists.txt.
#define WIFI_OTA_ENABLED 0

#include "nvs_flash.h"       // NVS: lo usa NimBLE (bonding) y el arranque; NO depende de WiFi
#include "esp_event.h"       // event loop base (se mantiene; no arrastra WiFi)
#if WIFI_OTA_ENABLED
#include "esp_wifi.h"
#include "esp_netif.h"
#include "ota_http.h"
#endif

// BLE (NimBLE) - control opcional, ver BLE_CONTROL_ENABLED
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_hs_mbuf.h"          // ble_hs_mbuf_from_flat / to_flat (OTA por BLE)
#include "esp_ota_ops.h"               // OTA por BLE (esp_ota_*)
#include "esp_system.h"                // esp_restart

// ESP-IDF Drivers
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

// Librerias externas
#include "esp_modbus_slave.h"
#include "mbcontroller.h"
#include "modbus_params.h"
#include "ina260.h"
#include "icm42670.h"
#include "servo_motor.h"


//*****************************************WIFI + OTA*********************************************************/
// NOTA: el interruptor WIFI_OTA_ENABLED ahora se define ARRIBA, antes de los #include,
// para poder excluir del build los headers de WiFi cuando esta en 0. En 0 = desactivado.

//*****************************************BLE CONTROL (TEST DE LABORATORIO)**************************************/
// Control extra por BLE, pensado para pruebas de corto alcance en el taller con una GUI
// Python (bleak). No reemplaza ni modifica el Modbus RTU: escribe en las mismas variables
// (mb_holding_reg / mb_coil_reg) y avisa con los mismos event bits, asi que la logica de
// motores/servos/salidas no se toca. Poner en 0 para desactivar del todo (ni se compila).
#define BLE_CONTROL_ENABLED 1
#define BLE_DEVICE_NAME     "RD90R-260033"  /* <- Cambiar por el numero serial del robot */
// Servicio OTA por BLE (subir firmware .bin desde ble_ota_gui.py). Requiere
// BLE_CONTROL_ENABLED=1. Poner en 0 para quitar solo el OTA y dejar el control.
#define BLE_OTA_ENABLED     1

#define WIFI_SSID        "WTP TALLER"       /* <- Cambiar */
#define WIFI_PASS        "24012024"         /* <- Cambiar */
#define OTA_VERSION_URL  "https://raw.githubusercontent.com/SrMuchachita/Muchachota_R/main/version.json"
#define OTA_FIRMWARE_URL "https://github.com/SrMuchachita/Muchachota_R/releases/latest/download/WP_R_V9_UP.bin"
#define OTA_CHECK        20                   /* segundos entre chequeos */

//*****************************************RECURSOS DE MODULO MAIN************************************************/
#define TASK_SIZE            (6 * 1024)
#define TAG                  "RD90Pro"


//*****************************************DEFINICIONES EQUIPO*********************************************************/
#define MB_SLAVE_ADDR        (0x0A)
#define MB_SLAVE_BAUD        (115200)
#define MB_FIRMWARE_REVISION (0x0100)
#define DEVICE_SERIAL_PREFIX "RD90"
#define DEVICE_SERIAL_SUFFIX "260033"    // <- Cambiar por el numero serial del robot
/*Formato: RD90-262501 (26 = año 2026, 0001 = robot número)*/

// Modelo del robot: 0=RD80, 1=RD90, 2=RD100. Se manda como número simple por Modbus
// (mb_input_reg.model) para que la consola no tenga que parsear texto.
#define ROBOT_MODEL_RD80   0
#define ROBOT_MODEL_RD90   1
#define ROBOT_MODEL_RD100  2
#define ROBOT_MODEL        ROBOT_MODEL_RD90

//*****************************************DEFINICIONES SYSTEM MODE**************************************************/
#define MB_SYSTEM_START      (0x07D0)
#define MB_SYSTEM_STOP       (0x03E8)


//*****************************************DEFINICIONES GPIO*********************************************************/
#define DOUT2   39
#define DOUT3   38
#define DOUT_MASK_CONFIG     ((1ULL << DOUT2)|(1ULL << DOUT3))

#define LED_IO45   45
#define LED_IO46   46
#define LED_IO_MASK_CONFIG   ((1ULL << LED_IO45)|(1ULL << LED_IO46))
// Ambos LEDs arrancan apagados; se controlan por Modbus (coils out4/out5)


//*****************************************PINOUT MOTORES DC ******************************************************/
#define M0INH   3
#define M0PWMA  10
#define M0PWMB  9

#define M1INH   4
#define M1PWMA  7
#define M1PWMB  5

#define M2INH   8
#define M2PWMA  17
#define M2PWMB  18

#define M3INH   16
#define M3PWMA  15
#define M3PWMB  6

/*
#define PWM_FREQ_HZ        10000
#define PWM_RESOLUTION_HZ  10000000
#define PWM_PERIOD_TICKS   (PWM_RESOLUTION_HZ / PWM_FREQ_HZ)
#define MOTOR_COUNT        (4)
#define MOTOR_MAX_PWM      1000
#define RAMP_STEP          10
#define TASK_PERIOD_MS     20*/

// PRUEBA de 1000Hz revertida — no mejoró la velocidad, se descarta la frecuencia PWM como causa
#define PWM_FREQ_HZ        20000
#define PWM_RESOLUTION_HZ  10000000  // ← no cambia
#define PWM_PERIOD_TICKS   (PWM_RESOLUTION_HZ / PWM_FREQ_HZ)  // ← no cambia (se calcula solo)
#define MOTOR_COUNT        (4)
#define MOTOR_MAX_PWM      1000     // ← no cambia
// #define RAMP_STEP       10       // ← ORIGINAL: rampa de 2s. Descomentar esta línea y comentar la de abajo para regresar
#define RAMP_STEP          500       // ← ACTUAL: rampa de ~0.5s
#define TASK_PERIOD_MS     20        // ← no cambia
/*500 / 10 = 50 pasos × 20ms = solo 1 segundo*/

//*****************************************PINOUT SERVOS******************************************************/
#define SERVO1_CH0_PIN   12
#define SERVO2_CH1_PIN   11
#define SERVO3_CH3_PIN      41   // SRV3 salida A
#define SERVO3B_CH4_PIN     40   // SRV3 salida B (espejo), antes DOUT1

// Grados maximos independiente por servo (los 3 son de 270° por ahora)
#define SERVO1_MAX_DEGREE   270
#define SERVO2_MAX_DEGREE   270
#define SERVO3_MAX_DEGREE   270

// Rango de pulso independiente por servo (us). Ajustar aqui si algun servo
// necesita un rango distinto al estandar 500-2500us para cubrir sus 270°.
#define SERVO1_MIN_PULSEWIDTH_US  500
#define SERVO1_MAX_PULSEWIDTH_US  2500

#define SERVO2_MIN_PULSEWIDTH_US  500
#define SERVO2_MAX_PULSEWIDTH_US  2500

#define SERVO3_MIN_PULSEWIDTH_US  500
#define SERVO3_MAX_PULSEWIDTH_US  2500

#define SERVO3B_MIN_PULSEWIDTH_US  500
#define SERVO3B_MAX_PULSEWIDTH_US  2500

// SRV3 mueve 2 servos fisicos a la vez: la salida A sigue el angulo tal cual,
// la B va en ESPEJO alrededor de estos puntos de inicio (misma logica que la
// prueba servo_s3_srv3, con los pines que ya tenemos en este circuito):
//     angulo_B = SRV3B_HOME_B + SRV3B_HOME_A - angulo_A
#define SRV3B_HOME_A   270
#define SRV3B_HOME_B   0


//*****************************************PINOUT PWM LED******************************************************/
#define LED_PIN          13
#define LED_FREQ_HZ      5000


//*****************************************DEFINICIONES I2C*********************************************************/
#define INA_I2C_PORT                    I2C_NUM_0
#define INA_I2C_SDA_PIN                 (48)
#define INA_I2C_SCL_PIN                 (47)

#define IMU_I2C_PORT                    I2C_NUM_1
#define IMU_I2C_SDA_PIN                 (14)
#define IMU_I2C_SCL_PIN                 (21)

#define IMU_DEV_ADDR                    (0b1101000)
#define IMU_DEV_FREQ_Hz                 100000

#define INA_DEV_ADDR                    (0b1000000)
#define INA_DEV_FREQ_Hz                 100000


//*****************************************DEFINICIONES MODBUS*****************************************************/
#define MB_PORT_NUM           UART_NUM_2
#define MB_UART_TXD           (1)
#define MB_UART_RXD           (2)
#define MB_UART_DIR           (42)
#define UART_BUFFER_SIZE      (512)

#define MB_REG_COILS_START    (0x0000)
#define MB_REG_HOLDING_START  (0x0000)
#define MB_REG_INPUT_START    (0x0000)

#define MB_READ_MASK          (MB_EVENT_HOLDING_REG_RD | MB_EVENT_COILS_RD | MB_EVENT_INPUT_REG_RD)
#define MB_WRITE_MASK         (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK    (MB_READ_MASK | MB_WRITE_MASK)


//*****************************************BITS DE EVENTOS*****************************************************/
#define MB_REG_COIL0_WR_BIT       BIT0
#define MB_REG_COIL1_WR_BIT       BIT1
#define MB_REG_COIL2_WR_BIT       BIT2
#define MB_REG_COIL3_WR_BIT       BIT3
#define MB_REG_COIL4_WR_BIT       BIT4
#define MB_REG_COIL5_WR_BIT       BIT5
#define MB_REG_COILS_MASK         (MB_REG_COIL0_WR_BIT | MB_REG_COIL1_WR_BIT | MB_REG_COIL2_WR_BIT | MB_REG_COIL3_WR_BIT | MB_REG_COIL4_WR_BIT | MB_REG_COIL5_WR_BIT)

#define MB_REG_HOLDING_MOTOR_BIT  BIT0
#define MB_REG_HOLDING_SERVO_BIT  BIT1
#define MB_REG_HOLDING_LED_BIT    BIT2


//*****************************************TIMEOUT MOTORES*****************************************************/
// [FIX-8] Timeout claramente definido y separado
#define MODBUS_TIMEOUT_MS           3000    // 3 segundos sin actividad -> stop motores
#define MODBUS_INACTIVITY_RESET_MS  (5 * 60 * 1000)  // 5 min sin actividad -> restart


//******************************* ESTRUCTURAS PARA CONTROL DE MOTOR *****************************************/
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_RIGHT,
    MOTOR_LEFT,
} motor_dir_t;

typedef struct {
    gpio_num_t inh;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmp_a;
    mcpwm_cmpr_handle_t cmp_b;
    mcpwm_gen_handle_t  gen_a;
    mcpwm_gen_handle_t  gen_b;
} motor_t;

static mcpwm_timer_handle_t timer0;
static mcpwm_timer_handle_t timer1;
static motor_t motors[MOTOR_COUNT];


//****************************************PROTOTIPOS DE FUNCIONES APP*********************************************/
void vGpioInit(void);
void vI2cInit(void);
void vHardwareInit(void);
void vModbusSlaveInit(void);
void vMotorsDcInit(void);
void vServosInit(void);
void vLedPwmInit(void);


//**************************************PROTOTIPOS DE FUNCIONES DE UTIL*********************************************/
void motorDcSet(uint8_t id, motor_dir_t dir, uint16_t speed);
void motorDcControl_TEST(void);   // [FIX-7] Renombrada como TEST, no usar en producción
void setServoAngles(void);
void wait_for_sensor(ina260_t *dev);
void vModbusSlaveControlStop(void);

// [FIX-12] Función para calibración de ángulos IMU
void imu_calibrate_reference(float pitch, float roll);
void imu_calibration_enable(bool enable);


//**************************************PROTOTIPOS DE FUNCIONES TAREAS*******************************************/
void vTaskModbusEvents  (void *pvParameters);
void vTaskMotorsControl (void *pvParameters);
void vTaskServosControl (void *pvParameters);
void vTaskPowerMonitor  (void *pvParameters);
void vTaskImuMonitor    (void *pvParameters);
void vTaskDigitalOutput (void *pvParameters);
void vTaskLedControl    (void *pvParameters);
void vTaskModbusWatchdog(void *pvParameters);


//************************************** RECURSOS DE FREERTOS ****************************************************/
EventGroupHandle_t xEventGroupCoilsReg   = NULL;
EventGroupHandle_t xEventGroupHoldingReg = NULL;


//************************************** VARIABLES GLOBALES ****************************************************/
static void         *mb_slave_handle             = NULL;
static portMUX_TYPE  paramLock                   = portMUX_INITIALIZER_UNLOCKED;
static int64_t       last_modbus_activity_time   = 0;
static int64_t       last_motor_activity_time    = 0;

// [FIX-12] Variables para calibración de ángulos IMU
static float imu_calibration_offset_pitch = 0.0f;
static float imu_calibration_offset_roll = 0.0f;
static bool imu_calibration_enabled = false;

static char device_serial[16] = {0};

i2c_master_bus_handle_t i2c1_handle   = NULL;
icm42670_handle_t dev_icm42670_handle = NULL;
ina260_t dev_ina260_handle            = {0};

servo_handle_t servo1;
servo_handle_t servo2;
servo_handle_t servo3;
servo_handle_t servo3b;


//**************************************WIFI + OTA****************************************************/

#if WIFI_OTA_ENABLED
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi conectado. IP: " IPSTR, IP2STR(&e->ip_info.ip));
        ota_http_notify_connected();
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_cfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Conectando a '%s'...", WIFI_SSID);
}
#endif

//**************************************BLE CONTROL (TEST DE LABORATORIO)*****************************************/
// Servidor GATT NimBLE para controlar el robot desde una GUI Python (bleak) a corta
// distancia. Es puramente aditivo: escribe en las mismas mb_holding_reg / mb_coil_reg
// y avisa con los mismos event bits que ya usa el Modbus RTU, asi que la logica de
// motores/servos/salidas no cambia en nada. Para apagarlo del todo (ni se compila),
// poner BLE_CONTROL_ENABLED en 0 mas arriba.
#if BLE_CONTROL_ENABLED

// UUIDs propios de 128 bits (no son de ningun estandar BLE, solo para este proyecto).
static const ble_uuid128_t ble_svc_uuid =
    BLE_UUID128_INIT(0x5a, 0x90, 0x52, 0x44, 0x39, 0x30, 0x43, 0x54,
                      0x52, 0x4c, 0x2d, 0x42, 0x4c, 0x45, 0x00, 0x01);

// Escritura de "registro" (igual direccionamiento que Modbus holding, FC06):
// payload = [addr(1) | valor_lo(1) | valor_hi(1)], valor en little-endian.
// addr: 0=motor_cmd 1=motor_vel 2=srv1_angle 3=srv2_angle 4=led_pwm 5=srv3_angle
static const ble_uuid128_t ble_chr_reg_uuid =
    BLE_UUID128_INIT(0x5a, 0x90, 0x52, 0x44, 0x39, 0x30, 0x43, 0x54,
                      0x52, 0x4c, 0x2d, 0x42, 0x4c, 0x45, 0x00, 0x02);

// Escritura de coil (igual direccionamiento que Modbus coils, FC05):
// payload = [addr(1) | valor(1, 0 o 1)]. addr: 0=out0 1=out1 2=out2 3=out3 4=RL1 5=RL2
static const ble_uuid128_t ble_chr_coil_uuid =
    BLE_UUID128_INIT(0x5a, 0x90, 0x52, 0x44, 0x39, 0x30, 0x43, 0x54,
                      0x52, 0x4c, 0x2d, 0x42, 0x4c, 0x45, 0x00, 0x03);

// Lectura/notificacion de sensores: 6 x int16 little-endian =
// [pitch | roll | temp | voltage | current | power] (mismo orden que input regs 0x0000-0x0005)
static const ble_uuid128_t ble_chr_sensor_uuid =
    BLE_UUID128_INIT(0x5a, 0x90, 0x52, 0x44, 0x39, 0x30, 0x43, 0x54,
                      0x52, 0x4c, 0x2d, 0x42, 0x4c, 0x45, 0x00, 0x04);

static uint16_t ble_chr_reg_val_handle;
static uint16_t ble_chr_coil_val_handle;
static uint16_t ble_chr_sensor_val_handle;

static uint8_t  ble_own_addr_type;
static uint16_t ble_sensor_conn_handle = 0;
static bool     ble_sensor_subscribed  = false;

#if BLE_OTA_ENABLED
// ── OTA por BLE ──────────────────────────────────────────────────────────────
// Servicio GATT para recibir un firmware .bin desde el GUI Python (ble_ota_gui.py)
// y escribirlo con esp_ota_*. UUIDs y protocolo DEBEN calzar con ese GUI:
//   SERVICE 1000454c-..  DATA 1100454c-.. (write)  CONTROL 1200454c-.. (write+notify)
static const ble_uuid128_t ble_ota_svc_uuid =
    BLE_UUID128_INIT(0x5a, 0x90, 0x52, 0x44, 0x39, 0x30, 0x43, 0x54,
                      0x52, 0x4c, 0x2d, 0x42, 0x4c, 0x45, 0x00, 0x10);
static const ble_uuid128_t ble_ota_data_uuid =
    BLE_UUID128_INIT(0x5a, 0x90, 0x52, 0x44, 0x39, 0x30, 0x43, 0x54,
                      0x52, 0x4c, 0x2d, 0x42, 0x4c, 0x45, 0x00, 0x11);
static const ble_uuid128_t ble_ota_ctrl_uuid =
    BLE_UUID128_INIT(0x5a, 0x90, 0x52, 0x44, 0x39, 0x30, 0x43, 0x54,
                      0x52, 0x4c, 0x2d, 0x42, 0x4c, 0x45, 0x00, 0x12);

static uint16_t ble_ota_data_val_handle;
static uint16_t ble_ota_ctrl_val_handle;
static uint16_t ble_ota_conn_handle     = 0;
static bool     ble_ota_ctrl_subscribed = false;

// Opcodes del protocolo (identicos a ble_ota_gui.py)
#define OTA_CMD_START  0x01
#define OTA_CMD_END    0x02
#define OTA_CMD_ABORT  0x03
#define OTA_ST_READY   0x10
#define OTA_ST_DONE    0x11
#define OTA_ST_ERROR   0x12
#define OTA_ST_PROG    0x13
// Detalle que acompana a OTA_ST_ERROR
#define OTA_ERR_NO_PART    1
#define OTA_ERR_BEGIN      2
#define OTA_ERR_NOT_READY  3
#define OTA_ERR_WRITE      4
#define OTA_ERR_END        5
#define OTA_ERR_BOOT       6

static esp_ota_handle_t       ota_handle      = 0;
static const esp_partition_t *ota_part        = NULL;
static bool                   ota_in_progress = false;
static uint32_t               ota_total       = 0;
static uint32_t               ota_received    = 0;
static uint32_t               ota_last_prog   = 0;
#endif // BLE_OTA_ENABLED

void ble_store_config_init(void);

static int ble_gap_event_handler(struct ble_gap_event *event, void *arg);

static void ble_start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE: error en adv_set_fields rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(ble_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                            ble_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE: error en adv_start rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE: advertising como '%s'", name);
}

static int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE: conexion %s; status=%d",
                 event->connect.status == 0 ? "establecida" : "fallida",
                 event->connect.status);
        if (event->connect.status != 0) {
            ble_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE: desconectado; reason=%d", event->disconnect.reason);
        ble_sensor_subscribed = false;
#if BLE_OTA_ENABLED
        ble_ota_ctrl_subscribed = false;
        if (ota_in_progress) {
            esp_ota_abort(ota_handle);
            ota_in_progress = false;
            ESP_LOGW(TAG, "BLE OTA: abortado por desconexion");
        }
#endif
        ble_start_advertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == ble_chr_sensor_val_handle) {
            ble_sensor_conn_handle = event->subscribe.conn_handle;
            ble_sensor_subscribed  = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "BLE: sensores %s", ble_sensor_subscribed ? "suscrito" : "desuscrito");
        }
#if BLE_OTA_ENABLED
        if (event->subscribe.attr_handle == ble_ota_ctrl_val_handle) {
            ble_ota_conn_handle     = event->subscribe.conn_handle;
            ble_ota_ctrl_subscribed = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "BLE: OTA ctrl %s", ble_ota_ctrl_subscribed ? "suscrito" : "desuscrito");
        }
#endif
        return 0;

    default:
        return 0;
    }
}

// Aplica una escritura de "registro" con el mismo direccionamiento/efecto que ya
// tiene el handler de Modbus holding (MB_EVENT_HOLDING_REG_WR) en vTaskModbusEvents.
static void ble_apply_reg_write(uint8_t addr, uint16_t value)
{
    int64_t now = esp_timer_get_time() / 1000;

    portENTER_CRITICAL(&paramLock);
    last_modbus_activity_time = now;

    switch (addr) {
        case 0: mb_holding_reg.motor_cmd  = value; last_motor_activity_time = now; break;
        case 1: mb_holding_reg.motor_vel  = value; last_motor_activity_time = now; break;
        case 2: mb_holding_reg.srv1_angle = value; break;
        case 3: mb_holding_reg.srv2_angle = value; break;
        case 4: mb_holding_reg.led_pwm    = value; break;
        case 5: mb_holding_reg.srv3_angle = value; break;
        default: break;  // 6 = fw_version (solo lectura); fuera de rango se ignora
    }
    portEXIT_CRITICAL(&paramLock);

    if (addr == 2 || addr == 3 || addr == 5) {
        xEventGroupSetBits(xEventGroupHoldingReg, MB_REG_HOLDING_SERVO_BIT);
    } else if (addr == 4) {
        xEventGroupSetBits(xEventGroupHoldingReg, MB_REG_HOLDING_LED_BIT);
    }
}

// Aplica una escritura de coil con el mismo direccionamiento/efecto que ya tiene
// el handler de Modbus coils (MB_EVENT_COILS_WR) en vTaskModbusEvents.
static void ble_apply_coil_write(uint8_t addr, uint8_t value)
{
    if (addr > 5) return;

    static const EventBits_t coil_bits[6] = {
        MB_REG_COIL0_WR_BIT, MB_REG_COIL1_WR_BIT, MB_REG_COIL2_WR_BIT,
        MB_REG_COIL3_WR_BIT, MB_REG_COIL4_WR_BIT, MB_REG_COIL5_WR_BIT,
    };

    portENTER_CRITICAL(&paramLock);
    last_modbus_activity_time = esp_timer_get_time() / 1000;
    switch (addr) {
        case 0: mb_coil_reg.out0 = value ? 1 : 0; break;
        case 1: mb_coil_reg.out1 = value ? 1 : 0; break;
        case 2: mb_coil_reg.out2 = value ? 1 : 0; break;
        case 3: mb_coil_reg.out3 = value ? 1 : 0; break;
        case 4: mb_coil_reg.rl1  = value ? 1 : 0; break;
        case 5: mb_coil_reg.rl2  = value ? 1 : 0; break;
    }
    portEXIT_CRITICAL(&paramLock);

    xEventGroupSetBits(xEventGroupCoilsReg, coil_bits[addr]);
}

static int ble_chr_reg_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    if (ctxt->om->om_len != 3) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t  addr  = ctxt->om->om_data[0];
    uint16_t value = (uint16_t)ctxt->om->om_data[1] | ((uint16_t)ctxt->om->om_data[2] << 8);
    ble_apply_reg_write(addr, value);
    return 0;
}

static int ble_chr_coil_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    if (ctxt->om->om_len != 2) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    ble_apply_coil_write(ctxt->om->om_data[0], ctxt->om->om_data[1]);
    return 0;
}

static int ble_chr_sensor_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;

    int16_t  vals_s16[3];
    uint16_t vals_u16[3];
    portENTER_CRITICAL(&paramLock);
    vals_s16[0] = mb_input_reg.imu_pitch;
    vals_s16[1] = mb_input_reg.imu_roll;
    vals_s16[2] = mb_input_reg.imu_temp;
    vals_u16[0] = mb_input_reg.ina_voltage;
    vals_u16[1] = mb_input_reg.ina_current;
    vals_u16[2] = mb_input_reg.ina_power;
    portEXIT_CRITICAL(&paramLock);

    uint8_t buf[12];
    memcpy(&buf[0], vals_s16, 6);
    memcpy(&buf[6], vals_u16, 6);

    int rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

#if BLE_OTA_ENABLED
// Envia un estado al GUI por notificacion de la caracteristica CONTROL.
static void ble_ota_notify(uint8_t code, const void *extra, uint8_t extra_len)
{
    if (!ble_ota_ctrl_subscribed) return;

    uint8_t buf[8];
    buf[0] = code;
    if (extra && extra_len) {
        if (extra_len > (uint8_t)(sizeof(buf) - 1)) extra_len = sizeof(buf) - 1;
        memcpy(&buf[1], extra, extra_len);
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, 1 + extra_len);
    if (om) {
        ble_gatts_notify_custom(ble_ota_conn_handle, ble_ota_ctrl_val_handle, om);
    }
}

// Reinicia poco despues de DONE, para dar tiempo a que salga la notificacion.
static void ble_ota_reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(800));
    ESP_LOGI(TAG, "BLE OTA: reiniciando al firmware nuevo...");
    esp_restart();
}

// CONTROL: recibe comandos START / END / ABORT y responde por notificacion.
static int ble_ota_ctrl_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

    uint8_t  cmd[8];
    uint16_t n = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, cmd, sizeof(cmd), &n) != 0 || n < 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    switch (cmd[0]) {
    case OTA_CMD_START: {
        if (n < 5) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        ota_total = (uint32_t)cmd[1]        | ((uint32_t)cmd[2] << 8) |
                   ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 24);

        if (ota_in_progress) { esp_ota_abort(ota_handle); ota_in_progress = false; }

        ota_part = esp_ota_get_next_update_partition(NULL);
        if (!ota_part) {
            uint8_t e = OTA_ERR_NO_PART; ble_ota_notify(OTA_ST_ERROR, &e, 1);
            ESP_LOGE(TAG, "BLE OTA: sin particion de destino");
            return 0;
        }
        esp_err_t err = esp_ota_begin(ota_part, ota_total, &ota_handle);
        if (err != ESP_OK) {
            uint8_t e = OTA_ERR_BEGIN; ble_ota_notify(OTA_ST_ERROR, &e, 1);
            ESP_LOGE(TAG, "BLE OTA: esp_ota_begin fallo: %s", esp_err_to_name(err));
            return 0;
        }
        ota_received  = 0;
        ota_last_prog = 0;
        ota_in_progress = true;

        // Pedir intervalo de conexion corto + supervision larga: acelera la
        // transferencia y la hace mas robusta a pausas por escritura en flash.
        // (El central -Windows- puede aceptarlo o no; si no, se queda como estaba.)
        struct ble_gap_upd_params cp = {
            .itvl_min            = 6,    // 6  * 1.25ms = 7.5 ms
            .itvl_max            = 12,   // 12 * 1.25ms = 15  ms
            .latency             = 0,
            .supervision_timeout = 600,  // 600 * 10ms  = 6   s
        };
        ble_gap_update_params(ble_ota_conn_handle, &cp);

        ESP_LOGI(TAG, "BLE OTA: inicio %u bytes -> particion '%s'",
                 (unsigned)ota_total, ota_part->label);
        ble_ota_notify(OTA_ST_READY, NULL, 0);
        return 0;
    }
    case OTA_CMD_END: {
        if (!ota_in_progress) {
            uint8_t e = OTA_ERR_NOT_READY; ble_ota_notify(OTA_ST_ERROR, &e, 1);
            return 0;
        }
        ota_in_progress = false;
        esp_err_t err = esp_ota_end(ota_handle);
        if (err != ESP_OK) {
            uint8_t e = OTA_ERR_END; ble_ota_notify(OTA_ST_ERROR, &e, 1);
            ESP_LOGE(TAG, "BLE OTA: esp_ota_end fallo: %s", esp_err_to_name(err));
            return 0;
        }
        err = esp_ota_set_boot_partition(ota_part);
        if (err != ESP_OK) {
            uint8_t e = OTA_ERR_BOOT; ble_ota_notify(OTA_ST_ERROR, &e, 1);
            ESP_LOGE(TAG, "BLE OTA: set_boot_partition fallo: %s", esp_err_to_name(err));
            return 0;
        }
        ESP_LOGI(TAG, "BLE OTA: completo (%u bytes). Reiniciando.", (unsigned)ota_received);
        ble_ota_notify(OTA_ST_DONE, NULL, 0);
        xTaskCreate(ble_ota_reboot_task, "OtaReboot", 2048, NULL, 5, NULL);
        return 0;
    }
    case OTA_CMD_ABORT:
        if (ota_in_progress) {
            esp_ota_abort(ota_handle);
            ota_in_progress = false;
            ESP_LOGW(TAG, "BLE OTA: abortado por el GUI");
        }
        return 0;
    default:
        return 0;
    }
}

// DATA: recibe trozos del .bin y los escribe en flash (modo confiable = con respuesta).
static int ble_ota_data_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    if (!ota_in_progress) return BLE_ATT_ERR_UNLIKELY;

    static uint8_t ota_buf[600];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, ota_buf, sizeof(ota_buf), &len) != 0) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (len == 0) return 0;

    esp_err_t err = esp_ota_write(ota_handle, ota_buf, len);
    if (err != ESP_OK) {
        uint8_t e = OTA_ERR_WRITE; ble_ota_notify(OTA_ST_ERROR, &e, 1);
        esp_ota_abort(ota_handle);
        ota_in_progress = false;
        ESP_LOGE(TAG, "BLE OTA: esp_ota_write fallo: %s", esp_err_to_name(err));
        return BLE_ATT_ERR_UNLIKELY;
    }
    ota_received += len;

    // Progreso informativo cada ~16 KB (o al final)
    if (ota_received - ota_last_prog >= 16384 || ota_received >= ota_total) {
        ota_last_prog = ota_received;
        uint8_t p[4] = {
            (uint8_t)(ota_received),        (uint8_t)(ota_received >> 8),
            (uint8_t)(ota_received >> 16),  (uint8_t)(ota_received >> 24),
        };
        ble_ota_notify(OTA_ST_PROG, p, 4);
    }
    return 0;
}
#endif // BLE_OTA_ENABLED

static const struct ble_gatt_svc_def ble_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &ble_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &ble_chr_reg_uuid.u,
                .access_cb  = ble_chr_reg_access_cb,
                .flags      = BLE_GATT_CHR_F_WRITE,
                .val_handle = &ble_chr_reg_val_handle,
            }, {
                .uuid       = &ble_chr_coil_uuid.u,
                .access_cb  = ble_chr_coil_access_cb,
                .flags      = BLE_GATT_CHR_F_WRITE,
                .val_handle = &ble_chr_coil_val_handle,
            }, {
                .uuid       = &ble_chr_sensor_uuid.u,
                .access_cb  = ble_chr_sensor_access_cb,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_chr_sensor_val_handle,
            }, {
                0,
            }
        },
    },
#if BLE_OTA_ENABLED
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &ble_ota_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &ble_ota_data_uuid.u,
                .access_cb  = ble_ota_data_access_cb,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &ble_ota_data_val_handle,
            }, {
                .uuid       = &ble_ota_ctrl_uuid.u,
                .access_cb  = ble_ota_ctrl_access_cb,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &ble_ota_ctrl_val_handle,
            }, {
                0,
            }
        },
    },
#endif
    {
        0,
    },
};

static void ble_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)ctxt;
    (void)arg;
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE: stack reiniciado, reason=%d", reason);
}

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE: sin direccion disponible");
        return;
    }
    rc = ble_hs_id_infer_auto(0, &ble_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE: error infiriendo tipo de direccion rc=%d", rc);
        return;
    }
    ble_start_advertising();
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE: host task iniciada");
    nimble_port_run();  // no retorna hasta nimble_port_stop()
    vTaskDelete(NULL);
}

static void ble_sensor_notify_task(void *param)
{
    while (1) {
        if (ble_sensor_subscribed) {
            ble_gatts_notify(ble_sensor_conn_handle, ble_chr_sensor_val_handle);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static void ble_control_init(void)
{
    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "BLE: nimble_port_init fallo rc=%d", rc);
        return;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(ble_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "BLE: count_cfg fallo rc=%d", rc); return; }
    rc = ble_gatts_add_svcs(ble_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "BLE: add_svcs fallo rc=%d", rc); return; }

    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0) { ESP_LOGE(TAG, "BLE: device_name_set fallo rc=%d", rc); return; }

    ble_hs_cfg.reset_cb          = ble_on_reset;
    ble_hs_cfg.sync_cb           = ble_on_sync;
    ble_hs_cfg.gatts_register_cb = ble_gatt_register_cb;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

    ble_store_config_init();

    xTaskCreate(ble_host_task,          "BleHost",   TASK_SIZE, NULL, 4, NULL);
    xTaskCreate(ble_sensor_notify_task, "BleSensor", TASK_SIZE, NULL, 3, NULL);

    ESP_LOGI(TAG, "BLE: control habilitado, nombre '%s'", BLE_DEVICE_NAME);
}

#endif // BLE_CONTROL_ENABLED

//**************************************APP MAIN******************************************************/
void app_main(void)
{
    /* NVS — obligatorio antes de WiFi */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

#if WIFI_OTA_ENABLED
    /* WiFi + OTA en segundo plano */
    wifi_init();
    ota_http_config_t ota_cfg = {
        .version_url        = OTA_VERSION_URL,
        .firmware_url       = OTA_FIRMWARE_URL,
        .check_interval_sec = OTA_CHECK,
    };
    ota_http_start(&ota_cfg);
#else
    ESP_LOGW(TAG, "WiFi + OTA DESACTIVADO (WIFI_OTA_ENABLED=0)");
#endif

    snprintf(device_serial, sizeof(device_serial), DEVICE_SERIAL_PREFIX "-" DEVICE_SERIAL_SUFFIX);
    ESP_LOGI(TAG, "Serial: %s", device_serial);

    // Cargar serial y modelo en mb_input_reg ANTES de vHardwareInit(): ahí adentro
    // vModbusSlaveInit() registra este mismo bloque y arranca el esclavo Modbus, así
    // que si se cargaran después quedaba una ventana donde el esclavo ya respondía
    // pero con serial/model todavía en cero (valor por defecto de la struct estática).
    for (int i = 0; i < 6; i++) {
        mb_input_reg.serial[i] = ((uint16_t)(uint8_t)device_serial[i * 2] << 8)
                                | (uint8_t)device_serial[i * 2 + 1];
    }

    mb_input_reg.model = ROBOT_MODEL;
    ESP_LOGI(TAG, "Modelo: %d (0=RD80, 1=RD90, 2=RD100)", ROBOT_MODEL);

    vHardwareInit();

    xEventGroupHoldingReg = xEventGroupCreate();
    if (xEventGroupHoldingReg == NULL) {
        ESP_LOGE(TAG, "Error al crear xEventGroupHoldingReg");
        return;
    }

    xEventGroupCoilsReg = xEventGroupCreate();
    if (xEventGroupCoilsReg == NULL) {
        ESP_LOGE(TAG, "Error al crear xEventGroupCoilsReg");
        return;
    }

    xTaskCreate(vTaskModbusEvents,   "ModbusRTU",  TASK_SIZE, NULL, 6, NULL);
    xTaskCreate(vTaskMotorsControl,  "MotoresDC",  TASK_SIZE, NULL, 5, NULL);
    xTaskCreate(vTaskServosControl,  "ServoMotor", TASK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskLedControl,     "LedCtrlPwm", TASK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskPowerMonitor,   "PowerMon",   TASK_SIZE, NULL, 5, NULL);
    xTaskCreate(vTaskImuMonitor,     "ImuMonitor", TASK_SIZE, NULL, 4, NULL);
    xTaskCreate(vTaskDigitalOutput,  "DigitalOut", TASK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskModbusWatchdog, "MbWatchdog", TASK_SIZE, NULL, 7, NULL);
}


/* ============================================================
 * TAREAS FREERTOS
 * ============================================================ */

void vTaskModbusEvents(void *pvParameters)
{
    mb_param_info_t regInfo;

    while (1) {
        (void)mbc_slave_check_event(mb_slave_handle, MB_READ_WRITE_MASK);

        if (ESP_OK != mbc_slave_get_param_info(mb_slave_handle, &regInfo, 10)) {
            ESP_LOGE(TAG, "Get parameter information -> ESP_ERR_TIMEOUT");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Registrar actividad Modbus
        portENTER_CRITICAL(&paramLock);
        last_modbus_activity_time = esp_timer_get_time() / 1000;
        portEXIT_CRITICAL(&paramLock);

        switch (regInfo.type)
        {
            case MB_EVENT_INPUT_REG_RD:
            break;

            case MB_EVENT_HOLDING_REG_WR:
                // Registros 0-1: Motor
                if (regInfo.mb_offset <= 1) {
                    // [FIX-2] Actualizar tiempo de actividad de motores
                    portENTER_CRITICAL(&paramLock);
                    last_motor_activity_time = esp_timer_get_time() / 1000;
                    portEXIT_CRITICAL(&paramLock);
                }
                // Registros 2-3-5: Servos
                else if (regInfo.mb_offset <= 3 || regInfo.mb_offset == 5) {
                    xEventGroupSetBits(xEventGroupHoldingReg, MB_REG_HOLDING_SERVO_BIT);
                }
                // Registro 4: LED
                else if (regInfo.mb_offset == 4) {
                    xEventGroupSetBits(xEventGroupHoldingReg, MB_REG_HOLDING_LED_BIT);
                }
            break;

            case MB_EVENT_HOLDING_REG_RD:
            break;

            case MB_EVENT_COILS_WR:
                if (regInfo.size == 1) {
                    if (regInfo.mb_offset == 0) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL0_WR_BIT);
                    } else if (regInfo.mb_offset == 1) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL1_WR_BIT);
                    } else if (regInfo.mb_offset == 2) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL2_WR_BIT);
                    } else if (regInfo.mb_offset == 3) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL3_WR_BIT);
                    } else if (regInfo.mb_offset == 4) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL4_WR_BIT);
                    } else if (regInfo.mb_offset == 5) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL5_WR_BIT);
                    } else {
                        ESP_LOGE(TAG, "Error de solicitud de escritura Coil");
                    }
                }
            break;

            default:
                ESP_LOGE(TAG, "Evento desconocido: %d", regInfo.type);
            break;
        }
    }
}


void vTaskMotorsControl(void *pvParameters)
{
    motor_dir_t cmd = MOTOR_STOP;

    // [FIX-1] current_dir inicializada en MOTOR_STOP, no MOTOR_FORWARD
    static motor_dir_t current_dir = MOTOR_STOP;

    uint16_t target_vel  = 0;
    static uint16_t current_vel = 0;

    // [FIX-2] Inicializar last_motor_activity_time con tiempo real al arranque
    // Así el watchdog no dispara inmediatamente
    portENTER_CRITICAL(&paramLock);
    last_motor_activity_time = esp_timer_get_time() / 1000;
    portEXIT_CRITICAL(&paramLock);

    // Estado seguro al arranque
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motorDcSet(i, MOTOR_STOP, 0);
    }

    while (1)
    {
        // =========================
        // [FIX-8] WATCHDOG separado y limpio al inicio del loop
        // =========================
        int64_t now = esp_timer_get_time() / 1000;
        int64_t last_activity_snapshot;

        portENTER_CRITICAL(&paramLock);
        last_activity_snapshot = last_motor_activity_time;
        portEXIT_CRITICAL(&paramLock);

        bool watchdog_triggered = (now - last_activity_snapshot) > MODBUS_TIMEOUT_MS;

        // =========================
        // Leer Modbus
        // =========================
        portENTER_CRITICAL(&paramLock);
        cmd        = (motor_dir_t)mb_holding_reg.motor_cmd;
        target_vel = mb_holding_reg.motor_vel;
        portEXIT_CRITICAL(&paramLock);

        // =========================
        // Aplicar watchdog
        // =========================
        if (watchdog_triggered) {
            cmd        = MOTOR_STOP;
            target_vel = 0;
            // Solo loguear una vez cuando dispara, no en cada ciclo
            static bool watchdog_logged = false;
            if (!watchdog_logged) {
                ESP_LOGW(TAG, "WATCHDOG MOTORES: sin actividad > %d ms -> STOP", MODBUS_TIMEOUT_MS);
                watchdog_logged = true;
            }
        } else {
            // Reset del flag de log cuando hay actividad normal
            static bool watchdog_logged = false;
            watchdog_logged = false;
        }

        // =========================
        // VALIDACIONES
        // =========================
        if (target_vel > MOTOR_MAX_PWM) {
            target_vel = MOTOR_MAX_PWM;
        }

        if (cmd > MOTOR_LEFT) {
            cmd = MOTOR_STOP;
        }

        // =========================
        // CAMBIO DE DIRECCIÓN SEGURO
        // =========================
        if ((cmd != current_dir) && (cmd != MOTOR_STOP))
        {
            // Primero frenar completamente antes de cambiar dirección
            target_vel = 0;

            if (current_vel == 0) {
                current_dir = cmd;
            } else {
                // Mantener dirección actual mientras frena
                cmd = current_dir;
            }
        }
        else if (cmd == MOTOR_STOP)
        {
            // [FIX-10] Poner target_vel a 0 para que la rampa suave disminuya gradualmente
            // Sin esto, la rampa nunca sabe que debe frenar y el motor se queda atorado
            target_vel = 0;
            
            // Si se pide stop explícito, actualizar dirección también
            if (current_vel == 0) {
                current_dir = MOTOR_STOP;
            }
        }
        else
        {
            current_dir = cmd;
        }

        // =========================
        // RAMPA SUAVE
        // =========================
        if (current_vel < target_vel) {
            current_vel += RAMP_STEP;
        } else if (current_vel > target_vel) {
            // [FIX-18] Prevenir underflow: si current_vel < RAMP_STEP, ir directamente a target_vel
            // Antes: 5 - 10 = 65531 (underflow en uint16_t) -> robot se atoraba con valores enormes
            if (current_vel >= RAMP_STEP) {
                current_vel -= RAMP_STEP;
            } else {
                current_vel = target_vel;  // Llegar al target sin underflow
            }
        }

        // Evitar oscilación alrededor del target
        if (abs((int)current_vel - (int)target_vel) < RAMP_STEP) {
            current_vel = target_vel;
        }

        // =========================
        // CONTROL DE MOTORES
        // =========================
        // Solo detener cuando la rampa ya llegó a 0
        // NO usar cmd == MOTOR_STOP aquí, eso cortaría la rampa suave
        if (current_vel == 0)
        {
            for (int i = 0; i < MOTOR_COUNT; i++) {
                motorDcSet(i, MOTOR_STOP, 0);
            }
        }
        else
        {
            switch (current_dir)
            {
                case MOTOR_FORWARD:
                    for (int i = 0; i < MOTOR_COUNT; i++) {
                        motorDcSet(i, MOTOR_FORWARD, current_vel);
                    }
                    break;

                case MOTOR_REVERSE:
                    for (int i = 0; i < MOTOR_COUNT; i++) {
                        motorDcSet(i, MOTOR_REVERSE, current_vel);
                    }
                    break;

                case MOTOR_RIGHT:
                    motorDcSet(0, MOTOR_FORWARD, current_vel);
                    motorDcSet(1, MOTOR_REVERSE, current_vel);
                    motorDcSet(2, MOTOR_FORWARD, current_vel);
                    motorDcSet(3, MOTOR_REVERSE, current_vel);
                    break;

                case MOTOR_LEFT:
                    motorDcSet(0, MOTOR_REVERSE, current_vel);
                    motorDcSet(1, MOTOR_FORWARD, current_vel);
                    motorDcSet(2, MOTOR_REVERSE, current_vel);
                    motorDcSet(3, MOTOR_FORWARD, current_vel);
                    break;

                default:
                    // Cualquier estado desconocido -> seguro
                    for (int i = 0; i < MOTOR_COUNT; i++) {
                        motorDcSet(i, MOTOR_STOP, 0);
                    }
                    break;
            }
        }

        // [FIX-17] Log de estado de motores cada 2 segundos (10 ciclos × 20ms)
        static int motor_log_counter = 0;
        if (motor_log_counter++ >= 10) {
            motor_log_counter = 0;
            const char* dir_names[] = {"STOP", "FORWARD", "REVERSE", "RIGHT", "LEFT"};
            ESP_LOGI(TAG, "MOTOR: dir=%s vel_actual=%d vel_target=%d (PWM: %d/1000)",
                     dir_names[current_dir], current_vel, target_vel, current_vel);
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}


void vTaskServosControl(void *pvParameters)
{
    uint16_t angle1, angle2, angle3;

    while (1)
    {
        xEventGroupWaitBits(xEventGroupHoldingReg,
                            MB_REG_HOLDING_SERVO_BIT,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        portENTER_CRITICAL(&paramLock);
        angle1 = mb_holding_reg.srv1_angle;
        angle2 = mb_holding_reg.srv2_angle;
        angle3 = mb_holding_reg.srv3_angle;
        portEXIT_CRITICAL(&paramLock);

        if (angle1 > SERVO1_MAX_DEGREE) angle1 = SERVO1_MAX_DEGREE;
        if (angle2 > SERVO2_MAX_DEGREE) angle2 = SERVO2_MAX_DEGREE;
        if (angle3 > SERVO3_MAX_DEGREE) angle3 = SERVO3_MAX_DEGREE;

        // SRV3 mueve 2 servos: B va en espejo del angulo de A alrededor de
        // SRV3B_HOME_A/SRV3B_HOME_B (ver definicion de las constantes arriba).
        int32_t angle3b = SRV3B_HOME_B + SRV3B_HOME_A - (int32_t)angle3;
        if (angle3b < 0) angle3b = 0;
        if (angle3b > SERVO3_MAX_DEGREE) angle3b = SERVO3_MAX_DEGREE;

        // Cada servo tiene su propio rango de grados y de pulso, independiente de los otros
        // servo_set_angle() asume 180° máx, por eso se calcula el pulso directo para cada uno
        uint32_t pulse1 = SERVO1_MIN_PULSEWIDTH_US +
            ((SERVO1_MAX_PULSEWIDTH_US - SERVO1_MIN_PULSEWIDTH_US) * angle1) / SERVO1_MAX_DEGREE;
        uint32_t pulse2 = SERVO2_MIN_PULSEWIDTH_US +
            ((SERVO2_MAX_PULSEWIDTH_US - SERVO2_MIN_PULSEWIDTH_US) * angle2) / SERVO2_MAX_DEGREE;
        uint32_t pulse3 = SERVO3_MIN_PULSEWIDTH_US +
            ((SERVO3_MAX_PULSEWIDTH_US - SERVO3_MIN_PULSEWIDTH_US) * angle3) / SERVO3_MAX_DEGREE;
        uint32_t pulse3b = SERVO3B_MIN_PULSEWIDTH_US +
            ((SERVO3B_MAX_PULSEWIDTH_US - SERVO3B_MIN_PULSEWIDTH_US) * (uint32_t)angle3b) / SERVO3_MAX_DEGREE;

        servo_set_pulse_us(servo1, pulse1);
        servo_set_pulse_us(servo2, pulse2);
        servo_set_pulse_us(servo3, pulse3);
        servo_set_pulse_us(servo3b, pulse3b);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void vTaskLedControl(void *pvParameters)
{
    uint16_t duty = 0;

    while (1)
    {
        xEventGroupWaitBits(xEventGroupHoldingReg,
                            MB_REG_HOLDING_LED_BIT,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        portENTER_CRITICAL(&paramLock);
        duty = mb_holding_reg.led_pwm;
        portEXIT_CRITICAL(&paramLock);

        if (duty > 1023) duty = 1023;

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void vTaskPowerMonitor(void *pvParameters)
{
    float voltage, current, power;
    wait_for_sensor(&dev_ina260_handle);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(200));

        if (ESP_OK != ina260_trigger(&dev_ina260_handle)) {
            ESP_LOGE(TAG, "Error en la configuracion de un solo disparo");
            continue;
        }

        wait_for_sensor(&dev_ina260_handle);

        if (ESP_OK != ina260_get_bus_voltage(&dev_ina260_handle, &voltage)) {
            ESP_LOGE(TAG, "Error en la lectura de voltaje");
            continue;
        }

        if (ESP_OK != ina260_get_current(&dev_ina260_handle, &current)) {
            ESP_LOGE(TAG, "Error en la lectura de corriente");
            continue;
        }

        if (ESP_OK != ina260_get_power(&dev_ina260_handle, &power)) {
            ESP_LOGE(TAG, "Error en la lectura de potencia");
            continue;
        }

        portENTER_CRITICAL(&paramLock);
        mb_input_reg.ina_voltage = (uint16_t)(voltage * 1000.0f); // mV
        mb_input_reg.ina_current = (uint16_t)(current * 1000.0f); // mA
        mb_input_reg.ina_power   = (uint16_t)(power   * 1000.0f); // mW
        portEXIT_CRITICAL(&paramLock);
    }
}


void vTaskImuMonitor(void *pvParameters)
{
    // ══════════════════════════════════════════════════════════════════
    // [FIX-28] FILTRO MAHONY (quaternion) — reemplaza el filtro complementario
    // de ángulos sueltos (FIX-11...FIX-26). Es el mismo algoritmo (Mahony &
    // Hamel) que usan ArduPilot/Betaflight para estimar orientación: sin las
    // singularidades de atan2 que tenían las fórmulas anteriores, y con
    // corrección automática del bias del giroscopio (deriva a largo plazo),
    // que el filtro anterior no tenía.
    //
    // OJO: el remapeo de ejes de más abajo es la mejor estimación para el
    // montaje físico de este sensor. A diferencia del filtro anterior (que
    // usaba 2 fórmulas de ángulo independientes y toleraba cualquier
    // permutación/signo arbitrario de ejes), un filtro de quaternion necesita
    // una rotación 3D válida real. Es posible que algún eje salga invertido
    // al probar en hardware — si pasa, corregir el signo correspondiente
    // en el bloque "REMAPEO DE EJES" de más abajo (una sola línea cada vez).
    // ══════════════════════════════════════════════════════════════════

    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    static float integral_fb_x = 0.0f, integral_fb_y = 0.0f, integral_fb_z = 0.0f;
    static bool quat_init = false;
    const float MAHONY_TWO_KP = 1.0f;    // ganancia proporcional (corrección hacia la gravedad medida)
    const float MAHONY_TWO_KI = 0.004f;  // ganancia integral (corrige el bias del giroscopio solo, con el tiempo)

    // [FIX-27] DT medido en tiempo real en vez de asumir 200ms fijos. El ciclo real
    // varía: cada 2 ciclos lee temperatura extra por I2C, a veces hay reintentos.
    static int64_t last_sample_us = 0;

    // [FIX-13] Lectura de temperatura menos frecuente para evitar errores I2C
    static int temp_read_counter = 0;
    const int TEMP_READ_INTERVAL = 2;  // cada 2 ciclos (200ms * 2 = 400ms aprox)

    // [FIX-19] Auto-calibración de punto cero: a los 5s de arrancar, fija el pitch/roll
    // actual como nueva referencia (0°,0°) y a partir de ahí se reportan variaciones relativas
    static bool auto_zero_done = false;
    static int auto_zero_counter = 0;
    const int AUTO_ZERO_CYCLES = 25;  // 25 ciclos * 200ms = 5 segundos

    icm42670_value_t acc;
    icm42670_value_t gyro;
    float temperature = 0.0f;

    while (1)
    {
        // Leer acelerómetro
        if (ESP_OK != icm42670_get_acce_value(dev_icm42670_handle, &acc)) {
            ESP_LOGE(TAG, "Error lectura acelerómetro");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Leer giroscopio
        if (ESP_OK != icm42670_get_gyro_value(dev_icm42670_handle, &gyro)) {
            ESP_LOGE(TAG, "Error lectura giroscopio");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // [FIX-27] DT real desde la última muestra válida (en vez de 200ms fijos)
        int64_t now_us = esp_timer_get_time();
        float DT = (last_sample_us == 0) ? 0.2f : (now_us - last_sample_us) / 1000000.0f;
        last_sample_us = now_us;
        if (DT > 0.5f) {
            DT = 0.5f;  // ciclo anormalmente largo (ej. reintentos I2C): evitar un salto falso al integrar el giro
        }

        // [FIX-13] Leer temperatura cada 2 ciclos (400ms aprox) en lugar de cada ciclo
        if (temp_read_counter++ >= TEMP_READ_INTERVAL) {
            temp_read_counter = 0;
            if (ESP_OK != icm42670_get_temp_value(dev_icm42670_handle, &temperature)) {
                ESP_LOGW(TAG, "Error lectura temperatura IMU (intentando de nuevo en 500ms)");
                // No es crítico, continuamos sin abortar el ciclo
            }
        }

        // ── [FIX-28] REMAPEO DE EJES (rotación 3D del montaje físico) ──────
        // Mejor estimación inicial para este montaje. Si algún eje sale
        // invertido al probar en el robot, cambiar el signo correspondiente
        // ACÁ únicamente (no hace falta tocar el resto del filtro).
        float ax = -acc.x;
        float ay =  acc.y;
        float az = -acc.z;
        float gx_dps = -gyro.x;  // °/s
        float gy_dps =  gyro.y;
        float gz_dps = -gyro.z;
        // ────────────────────────────────────────────────────────────────

        float gx = gx_dps * (float)M_PI / 180.0f;  // rad/s, lo que necesita el filtro
        float gy = gy_dps * (float)M_PI / 180.0f;
        float gz = gz_dps * (float)M_PI / 180.0f;

        // [FIX-23 adaptado] Si la magnitud total del acelerómetro se aleja de 1g,
        // hay aceleración lineal real mezclada (no solo gravedad, típico al girar
        // rápido con la mano): se reduce la ganancia proporcional para no corregir
        // ese ciclo con un dato de gravedad contaminado.
        float acc_mag = sqrtf(ax * ax + ay * ay + az * az);
        float acc_dev = fabsf(acc_mag - 1.0f);
        float two_kp = (acc_dev > 0.10f) ? (MAHONY_TWO_KP * 0.05f) : MAHONY_TWO_KP;

        if (!quat_init) {
            // Inicializar el quaternion con la primera lectura del acelerómetro,
            // así arranca ya orientado en vez de converger desde "plano" (0°,0°).
            quat_init = true;
            float roll0  = atan2f(ay, az);
            float pitch0 = atan2f(-ax, sqrtf(ay * ay + az * az));
            float cr = cosf(roll0 * 0.5f),  sr = sinf(roll0 * 0.5f);
            float cp = cosf(pitch0 * 0.5f), sp = sinf(pitch0 * 0.5f);
            q0 = cr * cp;
            q1 = sr * cp;
            q2 = cr * sp;
            q3 = -sr * sp;
            ESP_LOGI(TAG, "IMU (Mahony) inicializado: pitch=%.2f° roll=%.2f°",
                     pitch0 * 180.0f / (float)M_PI, roll0 * 180.0f / (float)M_PI);
        } else if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
            // ── Filtro Mahony (algoritmo estándar AHRS, Mahony & Hamel 2008) ──
            float recip_norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
            float nax = ax * recip_norm, nay = ay * recip_norm, naz = az * recip_norm;

            // Dirección de gravedad estimada a partir del quaternion actual
            float halfvx = q1 * q3 - q0 * q2;
            float halfvy = q0 * q1 + q2 * q3;
            float halfvz = q0 * q0 - 0.5f + q3 * q3;

            // Error = producto cruz entre la gravedad estimada y la medida
            float halfex = (nay * halfvz - naz * halfvy);
            float halfey = (naz * halfvx - nax * halfvz);
            float halfez = (nax * halfvy - nay * halfvx);

            // Realimentación integral: corrige el bias del giroscopio con el tiempo
            if (MAHONY_TWO_KI > 0.0f) {
                integral_fb_x += MAHONY_TWO_KI * halfex * DT;
                integral_fb_y += MAHONY_TWO_KI * halfey * DT;
                integral_fb_z += MAHONY_TWO_KI * halfez * DT;
                gx += integral_fb_x;
                gy += integral_fb_y;
                gz += integral_fb_z;
            }

            // Realimentación proporcional
            gx += two_kp * halfex;
            gy += two_kp * halfey;
            gz += two_kp * halfez;

            // Integrar la tasa de cambio del quaternion
            gx *= 0.5f * DT;
            gy *= 0.5f * DT;
            gz *= 0.5f * DT;
            float qa = q0, qb = q1, qc = q2;
            q0 += (-qb * gx - qc * gy - q3 * gz);
            q1 += (qa * gx + qc * gz - q3 * gy);
            q2 += (qa * gy - qb * gz + q3 * gx);
            q3 += (qa * gz + qb * gy - qc * gx);

            recip_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
            q0 *= recip_norm;
            q1 *= recip_norm;
            q2 *= recip_norm;
            q3 *= recip_norm;
        }

        // Quaternion -> ángulos de Euler (grados)
        float filtered_roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                     1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / (float)M_PI;
        float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
        if (sin_pitch > 1.0f)  sin_pitch = 1.0f;
        if (sin_pitch < -1.0f) sin_pitch = -1.0f;
        float filtered_pitch = asinf(sin_pitch) * 180.0f / (float)M_PI;

        // [FIX-19] Auto-calibración de punto cero, una sola vez, 5s después de arrancar
        if (!auto_zero_done && ++auto_zero_counter >= AUTO_ZERO_CYCLES) {
            auto_zero_done = true;
            imu_calibrate_reference(filtered_pitch, filtered_roll);
            imu_calibration_enable(true);
            ESP_LOGI(TAG, "IMU auto-calibración de punto cero ejecutada (5s desde el arranque)");
        }

        // [FIX-12] Aplicar calibración: restar offsets si está habilitada
        float final_pitch = filtered_pitch;
        float final_roll = filtered_roll;
        if (imu_calibration_enabled) {
            final_pitch = filtered_pitch - imu_calibration_offset_pitch;
            final_roll = filtered_roll - imu_calibration_offset_roll;
        }

        // Actualizar registros Modbus de forma atómica
        // Nota: el maestro Modbus (control) debe leer estos valores y dividir
        // por 100 para obtener los grados reales con 2 decimales de precisión.
        // Ejemplo: si imu_pitch = 4550 -> ángulo real = 45.50°
        // [FIX-28] Con la rotación 3D del Mahony, pitch/roll deberían salir bien
        // asignados directamente (el filtro anterior necesitaba intercambiarlos
        // a la salida). Si igual salen cruzados al probar, intercambiar acá:
        // mb_input_reg.imu_pitch = final_roll; mb_input_reg.imu_roll = final_pitch;
        portENTER_CRITICAL(&paramLock);
        mb_input_reg.imu_pitch = (int16_t)(final_pitch * 100.0f);
        mb_input_reg.imu_roll  = (int16_t)(final_roll  * 100.0f);
        mb_input_reg.imu_temp  = (int16_t)(temperature * 100.0f);
        portEXIT_CRITICAL(&paramLock);

        // Log de ángulos en cada ciclo (cada 200ms) para monitoreo en serial en tiempo real
        ESP_LOGI(TAG, "[%s] IMU: pitch=%.2f° roll=%.2f° temp=%.1f°C",
                 device_serial, final_pitch, final_roll, temperature);

        // El sensor ICM42670 necesita este intervalo para estabilizarse entre lecturas
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


void vTaskDigitalOutput(void *pvParameters)
{
    EventBits_t uxBits = 0;

    while (1)
    {
        uxBits = xEventGroupWaitBits(xEventGroupCoilsReg,
                                     MB_REG_COILS_MASK,
                                     pdFALSE,
                                     pdFALSE,
                                     portMAX_DELAY);

        if (uxBits & MB_REG_COIL2_WR_BIT) {
            // out2 ya NO controla el GPIO39 (DOUT2): ese pin es ahora exclusivo de RL1.
            // El coil out2 queda reservado sin GPIO (como out0/out1).
            xEventGroupClearBits(xEventGroupCoilsReg, MB_REG_COIL2_WR_BIT);
        }
        if (uxBits & MB_REG_COIL3_WR_BIT) {
            portENTER_CRITICAL(&paramLock);
            gpio_set_level(DOUT3, (uint32_t)mb_coil_reg.out3);
            portEXIT_CRITICAL(&paramLock);
            xEventGroupClearBits(xEventGroupCoilsReg, MB_REG_COIL3_WR_BIT);
        }
        if (uxBits & MB_REG_COIL4_WR_BIT) {
            portENTER_CRITICAL(&paramLock);
            gpio_set_level(LED_IO45, (uint32_t)mb_coil_reg.rl1);
            gpio_set_level(DOUT3, (uint32_t)mb_coil_reg.rl1);  // DOUT3 (GPIO38) acompaña a RL1/LED_IO45
            gpio_set_level(DOUT2, (uint32_t)mb_coil_reg.rl1);  // DOUT2 (GPIO39) tercera salida que acompaña a RL1
            portEXIT_CRITICAL(&paramLock);
            xEventGroupClearBits(xEventGroupCoilsReg, MB_REG_COIL4_WR_BIT);
        }
        if (uxBits & MB_REG_COIL5_WR_BIT) {
            portENTER_CRITICAL(&paramLock);
            gpio_set_level(LED_IO46, (uint32_t)mb_coil_reg.rl2);
            portEXIT_CRITICAL(&paramLock);
            xEventGroupClearBits(xEventGroupCoilsReg, MB_REG_COIL5_WR_BIT);
        }
    }
}


void vTaskModbusWatchdog(void *pvParameters)
{
    const uint32_t WATCHDOG_CHECK_PERIOD_MS = 1000;

    int64_t now       = 0;
    int64_t last_time = 0;

    // Inicializar con tiempo real al arranque
    portENTER_CRITICAL(&paramLock);
    last_modbus_activity_time = esp_timer_get_time() / 1000;
    portEXIT_CRITICAL(&paramLock);

    while (1)
    {
        now = esp_timer_get_time() / 1000;

        portENTER_CRITICAL(&paramLock);
        last_time = last_modbus_activity_time;
        portEXIT_CRITICAL(&paramLock);

        bool skip_inactivity_reset = false;
#if BLE_OTA_ENABLED
        // Durante un OTA por BLE no hay trafico Modbus: no reiniciar o se corta la subida.
        skip_inactivity_reset = ota_in_progress;
#endif
        if (!skip_inactivity_reset && (now - last_time) > MODBUS_INACTIVITY_RESET_MS)
        {
            ESP_LOGE(TAG, "MODBUS INACTIVITY 5min -> SYSTEM RESTART");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        }

        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS));
    }
}


//*************************************************************************************
//***************************** FUNCIONES DE UTILIDAD *********************************
//*************************************************************************************

void motorDcSet(uint8_t id, motor_dir_t dir, uint16_t speed)
{
    if (id >= MOTOR_COUNT) return;
    if (speed > MOTOR_MAX_PWM) speed = MOTOR_MAX_PWM;

    uint32_t duty_ticks = (PWM_PERIOD_TICKS * speed) / MOTOR_MAX_PWM;

    switch (dir)
    {
        case MOTOR_FORWARD:
            mcpwm_comparator_set_compare_value(motors[id].cmp_a, duty_ticks);
            mcpwm_comparator_set_compare_value(motors[id].cmp_b, 0);
            break;

        case MOTOR_REVERSE:
            mcpwm_comparator_set_compare_value(motors[id].cmp_a, 0);
            mcpwm_comparator_set_compare_value(motors[id].cmp_b, duty_ticks);
            break;

        case MOTOR_STOP:
        default:
            mcpwm_comparator_set_compare_value(motors[id].cmp_a, 0);
            mcpwm_comparator_set_compare_value(motors[id].cmp_b, 0);
            break;
    }
}


// [FIX-7] Función de TEST - NO llamar en producción
// Solo para verificación del hardware en banco de pruebas
void motorDcControl_TEST(void)
{
    ESP_LOGW(TAG, "motorDcControl_TEST: MODO TEST ACTIVO - solo usar en banco");

    for (uint16_t speed = 0; speed <= 1000; speed += 10) {
        for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_REVERSE, speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (int speed = 1000; speed >= 0; speed -= 10) {
        for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_REVERSE, speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    for (uint16_t speed = 0; speed <= 1000; speed += 10) {
        for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_FORWARD, speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (int speed = 1000; speed >= 0; speed -= 10) {
        for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_FORWARD, speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_STOP, 0);

    ESP_LOGW(TAG, "motorDcControl_TEST: TEST FINALIZADO");
}


void wait_for_sensor(ina260_t *dev)
{
    bool ready = false, alert = false, overflow = false;
    int timeout = 100;

    // [FIX-9] Cambiar while(timeout--) por while(timeout-- > 0) para evitar timeout negativo
    // El problema anterior: timeout-- devuelve el valor anterior, así que el loop solo se ejecutaba 99 veces
    // y timeout terminaba en -1, nunca pasando la condición <= 0
    while (timeout-- > 0) {
        // [FIX-25] Un glitch de I2C pasajero (frecuente con este sensor) hacía abortar
        // todo el firmware vía ESP_ERROR_CHECK. Ahora se trata como "todavía no listo"
        // y se reintenta dentro del mismo timeout, sin tumbar el equipo.
        esp_err_t ret = ina260_get_status(dev, &ready, &alert, &overflow);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "wait_for_sensor: error leyendo estado INA260: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (alert)    ESP_LOGW(TAG, "INA260 ALERT!");
        if (overflow) ESP_LOGW(TAG, "INA260 OVERFLOW!");
        if (ready)    break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (timeout <= 0) {
        ESP_LOGE(TAG, "wait_for_sensor: timeout esperando INA260");
    }
}


// [FIX-12] Calibración IMU: Guarda los ángulos actuales como referencia (0,0)
// Ejemplo: Si el dispositivo está inclinado 15° al prenderse, al calibrar
// esos 15° se marcan como 0°, y a partir de ahí se miden relativos a esa posición
void imu_calibrate_reference(float pitch, float roll)
{
    portENTER_CRITICAL(&paramLock);
    imu_calibration_offset_pitch = pitch;
    imu_calibration_offset_roll = roll;
    portEXIT_CRITICAL(&paramLock);
    
    ESP_LOGI(TAG, "IMU Calibración guardada: pitch_offset=%.2f°, roll_offset=%.2f°", 
             pitch, roll);
}


// [FIX-12] Habilitar o deshabilitar la calibración IMU
void imu_calibration_enable(bool enable)
{
    imu_calibration_enabled = enable;
    if (enable) {
        ESP_LOGI(TAG, "IMU Calibración HABILITADA - midiendo relativos a referencia");
    } else {
        ESP_LOGI(TAG, "IMU Calibración DESHABILITADA - midiendo ángulos absolutos");
    }
}


//*************************************************************************************
//***************************** CONFIGURACION DE HARDWARE *****************************
//*************************************************************************************

void vHardwareInit(void)
{
    vGpioInit();
    vMotorsDcInit();
    vServosInit();
    vLedPwmInit();
    vI2cInit();
    vModbusSlaveInit();
#if BLE_CONTROL_ENABLED
    ble_control_init();
#else
    ESP_LOGW(TAG, "BLE DESACTIVADO (BLE_CONTROL_ENABLED=0)");
#endif
    ESP_LOGW(TAG, "Configuracion de drivers ESP-IDF exitosa");
}


void vGpioInit(void)
{
    gpio_config_t gpioOutputConfig = {
        .pin_bit_mask = DOUT_MASK_CONFIG,
        .mode         = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpioOutputConfig);

    gpio_set_level(DOUT2, 0);
    gpio_set_level(DOUT3, 0);

    // LEDs en IO45 e IO46: habilitados, apagados al arrancar (se controlan por Modbus)
    gpio_config_t gpioLedConfig = {
        .pin_bit_mask = LED_IO_MASK_CONFIG,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpioLedConfig);

    gpio_set_level(LED_IO45, 0);
    gpio_set_level(LED_IO46, 0);
}


void vServosInit(void)
{
    servo_create(&servo1, SERVO1_CH0_PIN, LEDC_CHANNEL_0);
    servo_create(&servo2, SERVO2_CH1_PIN, LEDC_CHANNEL_1);
    servo_create(&servo3, SERVO3_CH3_PIN, LEDC_CHANNEL_3);
    servo_create(&servo3b, SERVO3B_CH4_PIN, LEDC_CHANNEL_4);

    // Posicion de arranque de SRV3: A en 0 grados (pulso minimo), B en espejo = 270
    // grados (pulso maximo) con los HOME actuales (SRV3B_HOME_A/SRV3B_HOME_B). Se
    // mantiene el pulso ~300ms para que los servos la registren antes de que llegue
    // el primer comando Modbus (misma logica que la prueba servo_s3_srv3).
    uint32_t pulse3_start  = SERVO3_MIN_PULSEWIDTH_US;
    uint32_t pulse3b_start = SERVO3B_MAX_PULSEWIDTH_US;
    for (int rep = 0; rep < 15; rep++) {
        servo_set_pulse_us(servo3, pulse3_start);
        servo_set_pulse_us(servo3b, pulse3b_start);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void vLedPwmInit(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = LED_FREQ_HZ,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = LED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_2,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0
    };
    ledc_channel_config(&channel);
}


void vMotorsDcInit(void)
{
    gpio_config_t gpio_inh = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << M0INH) |
            (1ULL << M1INH) |
            (1ULL << M2INH) |
            (1ULL << M3INH)
    };
    gpio_config(&gpio_inh);

    mcpwm_timer_config_t tcfg = {
        .resolution_hz = PWM_RESOLUTION_HZ,
        .period_ticks  = PWM_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP
    };

    tcfg.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &timer0));

    tcfg.group_id = 1;
    ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &timer1));

    gpio_num_t inh_pins[MOTOR_COUNT] = {M0INH,  M1INH,  M2INH,  M3INH};
    gpio_num_t pwm_a[MOTOR_COUNT]    = {M0PWMA, M1PWMA, M2PWMA, M3PWMA};
    gpio_num_t pwm_b[MOTOR_COUNT]    = {M0PWMB, M1PWMB, M2PWMB, M3PWMB};

    for (int i = 0; i < MOTOR_COUNT; i++) {

        motors[i].inh = inh_pins[i];

        mcpwm_operator_config_t ocfg = {
            .group_id = (i < 3) ? 0 : 1
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &motors[i].oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motors[i].oper, (i < 3) ? timer0 : timer1));

        mcpwm_comparator_config_t ccfg = {
            .flags.update_cmp_on_tez = true
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(motors[i].oper, &ccfg, &motors[i].cmp_a));
        ESP_ERROR_CHECK(mcpwm_new_comparator(motors[i].oper, &ccfg, &motors[i].cmp_b));

        mcpwm_generator_config_t gcfg = {0};

        gcfg.gen_gpio_num = pwm_a[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].oper, &gcfg, &motors[i].gen_a));

        gcfg.gen_gpio_num = pwm_b[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].oper, &gcfg, &motors[i].gen_b));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motors[i].gen_a,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motors[i].gen_a,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                motors[i].cmp_a,
                MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motors[i].gen_b,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motors[i].gen_b,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                motors[i].cmp_b,
                MCPWM_GEN_ACTION_LOW)));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer0));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer1));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP));

    for (int i = 0; i < MOTOR_COUNT; i++) {
        gpio_set_level(motors[i].inh, 1);
    }

    ESP_LOGI(TAG, "Motores DC inicializados");
}


void vI2cInit(void)
{
    esp_err_t ret;

    // ==== Configuracion ICM-42670 (IMU) ====
    const i2c_master_bus_config_t i2c1_bus_config = {
        .i2c_port   = IMU_I2C_PORT,
        .sda_io_num = IMU_I2C_SDA_PIN,
        .scl_io_num = IMU_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };

    // [FIX-6] Validar inicialización del bus I2C
    ret = i2c_new_master_bus(&i2c1_bus_config, &i2c1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando bus I2C1 (IMU): %s", esp_err_to_name(ret));
        return;
    }

    // [FIX-6] Validar creación del device ICM-42670
    ret = icm42670_create(i2c1_handle, ICM42670_I2C_ADDRESS, &dev_icm42670_handle);
    if (ret != ESP_OK || dev_icm42670_handle == NULL) {
        ESP_LOGE(TAG, "Error creando device ICM-42670: %s", esp_err_to_name(ret));
        return;
    }

    const icm42670_cfg_t dev_imu_cfg = {
        .acce_fs  = ACCE_FS_2G,
        .acce_odr = ACCE_ODR_400HZ,
        .gyro_fs  = GYRO_FS_2000DPS,
        .gyro_odr = GYRO_ODR_400HZ,
    };

    ret = icm42670_config(dev_icm42670_handle, &dev_imu_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando ICM-42670: %s", esp_err_to_name(ret));
        return;
    }

    // ==== Configuracion INA260 ====
    i2cdev_init();
    memset(&dev_ina260_handle, 0, sizeof(ina260_t));

    ret = ina260_init_desc(&dev_ina260_handle, INA_DEV_ADDR, INA_I2C_PORT, INA_I2C_SDA_PIN, INA_I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando descriptor INA260: %s", esp_err_to_name(ret));
        return;
    }

    ret = ina260_init(&dev_ina260_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando INA260: %s", esp_err_to_name(ret));
        return;
    }

    ret = ina260_set_config(&dev_ina260_handle,
                            INA260_MODE_TRIG_SHUNT_BUS,
                            INA260_AVG_128,
                            INA260_CT_1100,
                            INA260_CT_1100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando INA260: %s", esp_err_to_name(ret));
        return;
    }

    // Encender acelerómetro y giroscopio en modo low noise
    ret = icm42670_acce_set_pwr(dev_icm42670_handle, ACCE_PWR_LOWNOISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error encendiendo acelerómetro ICM-42670");
        return;
    }
    
    // [FIX-14] Agregar delay después de encender acelerómetro 
    // El sensor necesita estabilizarse antes de leer valores
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = icm42670_gyro_set_pwr(dev_icm42670_handle, GYRO_PWR_LOWNOISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error encendiendo giroscopio ICM-42670");
        return;
    }
    
    // [FIX-14] Agregar delay después de encender giroscopio
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "I2C inicializado correctamente (IMU + INA260)");
}


void vModbusSlaveInit(void)
{
    esp_err_t ret;

    mb_communication_info_t mb_com_config = {
        .ser_opts.mode      = MB_RTU,
        .ser_opts.uid       = MB_SLAVE_ADDR,
        .ser_opts.port      = MB_PORT_NUM,
        .ser_opts.baudrate  = MB_SLAVE_BAUD,
        .ser_opts.parity    = MB_PARITY_NONE,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1
    };

    ret = mbc_slave_create_serial(&mb_com_config, &mb_slave_handle);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Modbus Slave: fallo en la inicializacion");
        return;
    }

    mb_register_area_descriptor_t reg_area;

    reg_area.type         = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_COILS_START;
    reg_area.address      = (void*)&mb_coil_reg;
    reg_area.size         = sizeof(mb_coil_reg);
    ret = mbc_slave_set_descriptor(mb_slave_handle, reg_area);
    if (ESP_OK != ret) ESP_LOGE(TAG, "Register MB_PARAM_COIL fail");

    reg_area.type         = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START;
    reg_area.address      = (void*)&mb_holding_reg;
    reg_area.size         = sizeof(mb_holding_reg);
    ret = mbc_slave_set_descriptor(mb_slave_handle, reg_area);
    if (ESP_OK != ret) ESP_LOGE(TAG, "Register MB_PARAM_HOLDING fail");

    reg_area.type         = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START;
    reg_area.address      = (void*)&mb_input_reg;
    reg_area.size         = sizeof(mb_input_reg);
    ret = mbc_slave_set_descriptor(mb_slave_handle, reg_area);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Register MB_PARAM_INPUT fail");
        return;
    }

    vModbusSlaveControlStop();

    ret = mbc_slave_start(mb_slave_handle);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "mb controller start fail");
        return;
    }

    ret = uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_DIR, UART_PIN_NO_CHANGE);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "uart_set_pin failure");
        return;
    }

    ret = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "uart_set_mode failure");
        return;
    }

    ESP_LOGI(TAG, "Modbus RTU Stack inicializado!");
}


void vModbusSlaveControlStop(void)
{
    gpio_set_level(DOUT2, 0);
    gpio_set_level(DOUT3, 0);
    gpio_set_level(LED_IO45, 0);
    gpio_set_level(LED_IO46, 0);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motorDcSet(i, MOTOR_STOP, 0);
    }

    portENTER_CRITICAL(&paramLock);
    mb_holding_reg.motor_cmd  = MOTOR_STOP;
    mb_holding_reg.motor_vel  = 0;
    mb_holding_reg.srv1_angle = 0;
    mb_holding_reg.srv2_angle = 0;
    mb_holding_reg.led_pwm    = 0;
    mb_holding_reg.srv3_angle = 0;
    mb_holding_reg.fw_version = MB_FIRMWARE_REVISION;
    mb_input_reg.imu_pitch    = 0;
    mb_input_reg.imu_roll     = 0;
    mb_input_reg.imu_temp     = 0;
    mb_input_reg.ina_current  = 0;
    mb_input_reg.ina_voltage  = 0;
    mb_input_reg.ina_power    = 0;
    mb_coil_reg.out2          = 0;
    mb_coil_reg.out3          = 0;
    mb_coil_reg.rl1           = 0;
    mb_coil_reg.rl2           = 0;
    portEXIT_CRITICAL(&paramLock);

    ESP_LOGI(TAG, "Slave Modbus Stop Init");
}