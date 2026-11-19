# Tablas Modbus — RD90 Pro (WP_R_V9_UP)

Mapa de registros Modbus RTU que expone el firmware (esclavo dirección `0x0A`, 115200 bps, 8N1).
Fuente: `components/modbus_params/include/modbus_params.h` y su inicialización en `main/main2.c`.

> Este archivo se debe actualizar cada vez que se agregue, quite o modifique un registro Modbus,
> para mantener un historial de los cambios al mapa de registros.

## Input Registers — lectura, FC04 (lo que el robot informa a la consola)

| Registro | Dirección (0x) | Campo | Tipo | Escala / formato |
|---|---|---|---|---|
| 30001 | 0x0000 | `imu_pitch` | int16 | °×100 (ej. 4550 → 45.50°) |
| 30002 | 0x0001 | `imu_roll` | int16 | °×100 |
| 30003 | 0x0002 | `imu_temp` | int16 | °C×100 |
| 30004 | 0x0003 | `ina_voltage` | uint16 | mV |
| 30005 | 0x0004 | `ina_current` | uint16 | mA |
| 30006 | 0x0005 | `ina_power` | uint16 | mW |
| 30007–30012 | 0x0006–0x000B | `serial[6]` | uint16×6 | texto ASCII `"RD90-260009"`, 2 chars por registro |
| 30013 | 0x000C | `model` | uint16 | 0=RD80, 1=RD90, 2=RD100 |

## Holding Registers — lectura/escritura, FC03/FC06 (comandos de la consola al robot)

| Registro | Dirección (0x) | Campo | Rango |
|---|---|---|---|
| 40001 | 0x0000 | `motor_cmd` | 0=STOP, 1=FWD, 2=REV, 3=RIGHT, 4=LEFT |
| 40002 | 0x0001 | `motor_vel` | 0–1000 |
| 40003 | 0x0002 | `srv1_angle` | 0–180° |
| 40004 | 0x0003 | `srv2_angle` | 0–180° |
| 40005 | 0x0004 | `led_pwm` | 0–1023 |
| 40006 | 0x0005 | `srv3_angle` | 0–270° (mueve 2 servos a la vez, ver nota abajo) |
| 40007 | 0x0006 | `fw_version` | valor fijo `0x0100` |

**Nota — SRV3 mueve 2 servos físicos:** `srv3_angle` controla la salida A (GPIO41, `LEDC_CHANNEL_3`,
igual que antes) y además la salida B (GPIO40, `LEDC_CHANNEL_4`), que se mueve en **espejo** alrededor
de puntos de inicio fijos: `angulo_B = SRV3B_HOME_B + SRV3B_HOME_A - angulo_A` (constantes en `main2.c`,
por defecto `SRV3B_HOME_A=270`, `SRV3B_HOME_B=0`). No existe un registro Modbus separado para la
segunda salida — las dos se mueven siempre juntas con el mismo comando.

## Coils — lectura/escritura, FC01/FC05 (comandos ON/OFF de la consola al robot)

| Coil | Dirección (0x) | Campo |
|---|---|---|
| 00001 | 0x0000 | `out0` (reservado, sin GPIO asignado — pin movido a servo 3, salida A) |
| 00002 | 0x0001 | `out1` (reservado, sin GPIO asignado — pin movido a servo 3, salida B / espejo) |
| 00003 | 0x0002 | `out2` — reservado, ya sin GPIO (el GPIO39 pasó a ser exclusivo de `RL1`) |
| 00004 | 0x0003 | `out3` (GPIO38) — también se activa junto con `RL1` (ver nota abajo) |
| 00005 | 0x0004 | `RL1` — LED en GPIO IO45 (pendiente moverlo a pin de relé); al escribirla también enciende/apaga `DOUT3` (GPIO38) y `DOUT2` (GPIO39) |
| 00006 | 0x0005 | `RL2` — por ahora LED en GPIO IO46 (pendiente moverlo a pin de relé) |

## Control BLE (opcional, no reemplaza el Modbus RTU)

Servidor GATT NimBLE agregado en `main2.c` (`BLE_CONTROL_ENABLED`, por defecto `1`; poner en `0`
desactiva el BLE por completo, ni se compila). Pensado para pruebas de corto alcance en el taller
con una GUI Python (`bleak`). No modifica nada del Modbus RTU: escribe en las mismas variables
(`mb_holding_reg` / `mb_coil_reg`) y usa los mismos event bits, así que la lógica de
motores/servos/salidas es idéntica para ambos transportes. Nombre del dispositivo: `RD90PRO-BLE`.

Service UUID: `0100454c-422d-4c52-5443-30394452905a`

| Characteristic UUID | Tipo | Payload | Equivalente Modbus |
|---|---|---|---|
| `0200454c-...-905a` | write | `[addr(1)\|valor_lo(1)\|valor_hi(1)]` LE | Holding FC06 — addr 0-5 = mismas direcciones 0x0000-0x0005 de la tabla de arriba |
| `0300454c-...-905a` | write | `[addr(1)\|valor(1, 0/1)]` | Coils FC05 — addr 0-5 = mismas direcciones 0x0000-0x0005 de la tabla de coils |
| `0400454c-...-905a` | read + notify | 6 x int16 LE: `[pitch\|roll\|temp\|voltage\|current\|power]` | Input regs 0x0000-0x0005 |

GUI de referencia: `RD90PRO_V9_CONSOLA_VIRTUAL.py` (selector de modo Modbus/BLE agregado).

## Historial de cambios

| Fecha | Cambio |
|---|---|
| 2026-10-22 | La coil `RL1` (00005 / 0x0004) ahora mueve **tres** salidas a la vez: `LED_IO45` (GPIO45), `DOUT3` (GPIO38) y `DOUT2` (GPIO39), con el mismo valor y en el mismo instante (`main2.c`, handler de `MB_REG_COIL4_WR_BIT`). El `GPIO39` pasó a ser **exclusivo de RL1**: la coil `out2` (00003 / 0x0002) ya no controla ese pin y queda reservada sin GPIO (como `out0`/`out1`). `DOUT3` (GPIO38) sí sigue compartido entre `out3` y `RL1`. |
| 2026-10-17 | Se agrega control opcional por BLE (NimBLE, `BLE_CONTROL_ENABLED` en `main2.c`) como transporte extra para pruebas de laboratorio — ver sección arriba. Reutiliza el mismo mapa de registros/coils, ningún registro Modbus cambió. Se actualiza `RD90PRO_V9_CONSOLA_VIRTUAL.py` con selector de modo y panel de coils (`out2`, `out3`, `RL1`, `RL2`) que antes no estaban expuestos en esa GUI. |
| 2026-10-12 | La coil `RL1` (00005 / 0x0004) ahora también enciende/apaga `DOUT3` (GPIO38) junto con `LED_IO45` — mismo valor, mismo instante (`main2.c`, handler de `MB_REG_COIL4_WR_BIT`). `out3` (00004) sigue existiendo como coil independiente; el último que escriba gana. |
| 2026-10-05 | El pin GPIO40, que antes era `DOUT1` (coil 00002 / `out1`), se reasigna a una segunda salida física del servo 3 (`SERVO3B_CH4_PIN`, `LEDC_CHANNEL_4`): `srv3_angle` (40006) mueve ambos servos a la vez, el segundo en espejo (ver nota arriba). El coil `out1` queda reservado sin GPIO (igual que `out0`); solo quedan como salidas digitales reales `DOUT2` (GPIO39) y `DOUT3` (GPIO38). |
| 2026-08-18 | Se agregan coils `RL1` (00005 / 0x0004) y `RL2` (00006 / 0x0005) — pensados para relés, de momento cableados a los LEDs de IO45/IO46 solo para verificar visualmente que el comando llega. Ambos arrancan **apagados** (antes IO45 encendía solo al inicio por firmware). Implementado también en la consola `WP_C_V9_UP` (FC05 Write Single Coil, `HMI_REG_RL1`=0x24, `HMI_REG_RL2`=0x26). |
| 2026-08-11 | Se agrega `model` (30013 / 0x000C) — código numérico de modelo de robot (0=RD80, 1=RD90, 2=RD100). |
| 2026-08-11 | `serial[6]` cambia de formato: prefijo `"RD90R-"` → `"RD90-"` (se quita la R). |
