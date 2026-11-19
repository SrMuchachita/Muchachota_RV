
#ifndef _DEVICE_PARAMS
#define _DEVICE_PARAMS

#include <stdint.h>

// Coil
#pragma pack(push, 1)
typedef struct {
    uint8_t out0 :1;  // 00001 - 0x0000
    uint8_t out1 :1;  // 00002 - 0x0001
    uint8_t out2 :1;  // 00003 - 0x0002
    uint8_t out3 :1;  // 00004 - 0x0003
    uint8_t rl1  :1;  // 00005 - 0x0004  RL1 (por ahora LED en IO45, luego relé)
    uint8_t rl2  :1;  // 00006 - 0x0005  RL2 (por ahora LED en IO46, luego relé)
    uint8_t      :2;
} mb_coil_reg_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint16_t motor_cmd;     // 40001 - 0x0000
    uint16_t motor_vel;     // 40002 - 0x0001
    uint16_t srv1_angle;    // 40003 - 0x0002
    uint16_t srv2_angle;    // 40004 - 0x0003
    uint16_t led_pwm;       // 40005 - 0x0004
    uint16_t srv3_angle;    // 40006 - 0x0005  (servo 270°: rango 0-270; mueve 2 servos en espejo, ver SRV3B_HOME_A/B en main2.c)
    uint16_t fw_version;    // 40007 - 0x0006
} mb_holding_reg_t;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct
{
    int16_t  imu_pitch;     // 30001 - 0x0000
    int16_t  imu_roll;      // 30002 - 0x0001
    int16_t  imu_temp;      // 30003 - 0x0002
    uint16_t ina_voltage;   // 30004 - 0x0003
    uint16_t ina_current;   // 30005 - 0x0004
    uint16_t ina_power;     // 30006 - 0x0005
    uint16_t serial[6];     // 30007-30012 - 0x0006-0x000B  "RD90-XXXXXX" (2 ASCII chars por registro)
    uint16_t model;         // 30013 - 0x000C  0=RD80, 1=RD90, 2=RD100
} mb_input_reg_t;
#pragma pack(pop)

//Memoria para registros MB
extern mb_coil_reg_t     mb_coil_reg;
extern mb_input_reg_t    mb_input_reg;
extern mb_holding_reg_t  mb_holding_reg;

#endif 
