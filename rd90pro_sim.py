#!/usr/bin/env python3
"""
RD90 Pro - Simulador de Robot via RS485-USB
Emula el firmware ESP32-S3 como esclavo Modbus RTU.

El maestro Modbus (tu control) se conecta por RS485-USB a la PC
y este script responde exactamente igual que el robot real.

Requisitos:
    pip install pymodbus pyserial colorama

Uso:
    python rd90pro_sim.py --port COM3
"""

import time
import math
import random
import threading
import argparse
import sys

try:
    import colorama
    colorama.init()
except ImportError:
    pass  # Los colores ANSI igual funcionan en Windows Terminal / PowerShell moderno

try:
    from pymodbus.server import StartSerialServer
    from pymodbus.datastore import (
        ModbusSequentialDataBlock,
        ModbusSlaveContext,
        ModbusServerContext,
    )
    from pymodbus.framer import ModbusRtuFramer
except ImportError:
    print("ERROR: Instala las dependencias con:  pip install pymodbus pyserial colorama")
    sys.exit(1)


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURACIÓN DEL EQUIPO  (igual que los #define en main2.c)
# ══════════════════════════════════════════════════════════════════════════════
DEVICE_SERIAL_PREFIX  = "RD90R"
DEVICE_SERIAL_SUFFIX  = "SAPEEE"    # <- mismo define que en main2.c línea 69
DEVICE_SERIAL         = f"{DEVICE_SERIAL_PREFIX}-{DEVICE_SERIAL_SUFFIX}"

MB_SLAVE_ADDR         = 0x0A
MB_FIRMWARE_REVISION  = 0x0100
MOTOR_MAX_PWM         = 500
MODBUS_TIMEOUT_MS     = 3000
MODBUS_INACTIVITY_MS  = 5 * 60 * 1000   # 5 minutos

MOTOR_NAMES = ["STOP", "FORWARD", "REVERSE", "RIGHT", "LEFT"]
TAG = "RD90Pro"


# ══════════════════════════════════════════════════════════════════════════════
#  LOGGER  (imita ESP_LOGI / ESP_LOGW / ESP_LOGE del firmware)
# ══════════════════════════════════════════════════════════════════════════════
_t0 = time.time()

def log(level: str, msg: str):
    ms    = int((time.time() - _t0) * 1000)
    color = {"I": "\033[32m", "W": "\033[33m", "E": "\033[31m"}.get(level, "")
    print(f"{color}{level} ({ms:7d}) {TAG}: {msg}\033[0m", flush=True)


# ══════════════════════════════════════════════════════════════════════════════
#  MODBUS DATASTORE
# ══════════════════════════════════════════════════════════════════════════════

def pack_serial(serial_str: str) -> list[int]:
    """Empaqueta el serial ASCII en 6 registros uint16 (2 chars por registro)."""
    regs = []
    for i in range(6):
        hi = ord(serial_str[i * 2])     if i * 2     < len(serial_str) else 0
        lo = ord(serial_str[i * 2 + 1]) if i * 2 + 1 < len(serial_str) else 0
        regs.append((hi << 8) | lo)
    return regs

# Input registers  30001-30012  (FC04)
#   [0] imu_pitch  [1] imu_roll  [2] imu_temp
#   [3] ina_volt   [4] ina_curr  [5] ina_power
#   [6-11] serial "RD90R-SAPEEE"
_ir = ModbusSequentialDataBlock(0, [0] * 6 + pack_serial(DEVICE_SERIAL))

# Holding registers  40001-40006  (FC03/FC06)
#   [0] motor_cmd  [1] motor_vel  [2] srv1_angle  [3] srv2_angle
#   [4] led_pwm    [5] fw_version
_hr = ModbusSequentialDataBlock(0, [0, 0, 0, 0, 0, MB_FIRMWARE_REVISION])

# Coils  00001-00004  (FC01/FC05)
#   [0] out0  [1] out1  [2] out2  [3] out3
_co = ModbusSequentialDataBlock(0, [0] * 4)

slave_ctx  = ModbusSlaveContext(di=ModbusSequentialDataBlock(0, [0]*4),
                                co=_co, hr=_hr, ir=_ir)
server_ctx = ModbusServerContext(slaves={MB_SLAVE_ADDR: slave_ctx}, single=False)


# ══════════════════════════════════════════════════════════════════════════════
#  SIMULACIÓN DE SENSORES
# ══════════════════════════════════════════════════════════════════════════════

def simulate_imu(t: float, motor_cmd: int, motor_vel: int):
    """Pitch/roll simulados con reacción al movimiento + ruido."""
    vel_f = motor_vel / MOTOR_MAX_PWM
    n     = random.gauss(0, 0.08)
    if   motor_cmd == 1: pitch, roll =  -5.0 * vel_f + n,  n
    elif motor_cmd == 2: pitch, roll =   5.0 * vel_f + n,  n
    elif motor_cmd == 3: pitch, roll =   n,              -3.0 * vel_f + n
    elif motor_cmd == 4: pitch, roll =   n,               3.0 * vel_f + n
    else:
        pitch = math.sin(t * 0.10) * 0.3 + n
        roll  = math.cos(t * 0.13) * 0.2 + n
    return pitch, roll, 28.5 + random.gauss(0, 0.1)


def simulate_ina(motor_cmd: int, motor_vel: int):
    """Voltaje/corriente/potencia simulados según carga de motores."""
    v = 11.8 + random.gauss(0, 0.05)
    i = 0.3 + (motor_vel / MOTOR_MAX_PWM * 3.5 * 4 if motor_cmd else 0) + random.gauss(0, 0.05)
    i = max(0.0, i)
    return v, i, v * i


def to_reg16(value: float) -> int:
    """Convierte float a uint16 con signo en complemento a 2 (para pitch/roll negativos)."""
    return int(value) & 0xFFFF


# ══════════════════════════════════════════════════════════════════════════════
#  LOOP DE SIMULACIÓN  (corre en hilo separado al servidor Modbus)
# ══════════════════════════════════════════════════════════════════════════════

def simulation_loop(port: str):
    filtered_pitch = 0.0
    filtered_roll  = 0.0
    first_read     = True
    init_counter   = 0
    imu_log_cnt    = 0
    t0             = time.time()

    now_ms            = time.time() * 1000
    last_motor_ms     = now_ms
    last_modbus_ms    = now_ms
    watchdog_logged   = False
    inactivity_logged = False
    prev_hr           = [0, 0, 0, 0, 0, MB_FIRMWARE_REVISION]

    # ── Logs de arranque (igual que el firmware real) ─────────────────────────
    log("I", f"Serial: {DEVICE_SERIAL}")
    log("I", "Motores DC inicializados")
    log("I", "I2C inicializado correctamente (IMU + INA260)")
    log("I", "Modbus RTU Stack inicializado!")
    log("I", "Slave Modbus Stop Init")
    log("W", "Configuracion de drivers ESP-IDF exitosa")
    print()
    log("I", f"Simulador escuchando en {port} @ 115200 baud  |  Esclavo ID: 0x{MB_SLAVE_ADDR:02X}")
    print()

    while True:
        time.sleep(0.2)
        t      = time.time() - t0
        now_ms = time.time() * 1000

        # ── Leer registros (lo que el maestro Modbus haya escrito) ────────────
        hr = server_ctx[MB_SLAVE_ADDR].getValues(3, 0, 6)
        co = server_ctx[MB_SLAVE_ADDR].getValues(1, 0, 4)

        motor_cmd = hr[0] if hr[0] <= 4 else 0
        motor_vel = min(hr[1], MOTOR_MAX_PWM)
        srv1      = min(hr[2], 180)
        srv2      = min(hr[3], 180)
        led       = min(hr[4], 1023)

        # ── Detectar escrituras del maestro ───────────────────────────────────
        if list(hr) != prev_hr:
            last_modbus_ms    = now_ms
            inactivity_logged = False
            if hr[0] != prev_hr[0] or hr[1] != prev_hr[1]:
                last_motor_ms = now_ms
            prev_hr = list(hr)

        # ── Watchdog motores (3 s sin actividad -> STOP) ──────────────────────
        if (now_ms - last_motor_ms) > MODBUS_TIMEOUT_MS:
            motor_cmd = 0
            motor_vel = 0
            if not watchdog_logged:
                log("W", f"WATCHDOG MOTORES: sin actividad > {MODBUS_TIMEOUT_MS} ms -> STOP")
                watchdog_logged = True
        else:
            watchdog_logged = False

        # ── Watchdog inactividad total (5 min -> restart simulado) ────────────
        if (now_ms - last_modbus_ms) > MODBUS_INACTIVITY_MS:
            if not inactivity_logged:
                log("E", "MODBUS INACTIVITY 5min -> SYSTEM RESTART")
                inactivity_logged = True
            last_modbus_ms = now_ms  # simula reinicio

        # ── Filtro complementario IMU ─────────────────────────────────────────
        pitch_raw, roll_raw, temp = simulate_imu(t, motor_cmd, motor_vel)

        if first_read and init_counter < 10:
            filtered_pitch = pitch_raw
            filtered_roll  = roll_raw
            init_counter  += 1
            if init_counter == 1:
                log("I", "IMU inicializando filtro con acelerómetro...")
        elif first_read:
            first_read = False
            log("I", f"IMU calibración automática completada. "
                     f"Pitch={filtered_pitch:.2f}° Roll={filtered_roll:.2f}°")

        ALPHA          = 0.80
        filtered_pitch = ALPHA * filtered_pitch + (1 - ALPHA) * pitch_raw
        filtered_roll  = ALPHA * filtered_roll  + (1 - ALPHA) * roll_raw

        # ── INA260 ────────────────────────────────────────────────────────────
        voltage, current, power = simulate_ina(motor_cmd, motor_vel)

        # ── Actualizar input registers (lo que el maestro leerá con FC04) ─────
        server_ctx[MB_SLAVE_ADDR].setValues(4, 0, [
            to_reg16(filtered_pitch * 100),
            to_reg16(filtered_roll  * 100),
            to_reg16(temp           * 100),
            int(voltage * 1000),
            int(current * 1000),
            int(power   * 1000),
        ])

        # ── Logs consola cada ~1 segundo (5 ciclos × 200 ms) ─────────────────
        imu_log_cnt += 1
        if imu_log_cnt >= 5:
            imu_log_cnt = 0

            log("I", f"[{DEVICE_SERIAL}] IMU: "
                     f"pitch={filtered_pitch:.2f}°  "
                     f"roll={filtered_roll:.2f}°  "
                     f"temp={temp:.1f}°C")

            dir_name = MOTOR_NAMES[motor_cmd] if motor_cmd < len(MOTOR_NAMES) else "?"
            if motor_cmd != 0:
                log("I", f"MOTOR: dir={dir_name}  vel={motor_vel}/{MOTOR_MAX_PWM}  |  "
                         f"Servos: srv1={srv1}°  srv2={srv2}°  |  LED: {led}/1023")

            log("I", f"INA260: {voltage:.2f} V  {current*1000:.0f} mA  {power:.2f} W  |  "
                     f"DOUT: out0={co[0]}  out1={co[1]}  out2={co[2]}  out3={co[3]}")
            print()


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="RD90 Pro - Simulador de robot via RS485-USB"
    )
    parser.add_argument(
        "--port", default="COM3",
        help="Puerto COM del adaptador RS485-USB (ej: COM3, COM4, /dev/ttyUSB0)"
    )
    args = parser.parse_args()

    # Hilo de simulación (actualiza datos y muestra logs)
    sim = threading.Thread(target=simulation_loop, args=(args.port,), daemon=True)
    sim.start()

    # Servidor Modbus RTU (bloqueante — responde al maestro RS485)
    try:
        StartSerialServer(
            context=server_ctx,
            framer=ModbusRtuFramer,
            port=args.port,
            baudrate=115200,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.005,
        )
    except KeyboardInterrupt:
        print()
        log("W", "Simulador detenido por el usuario")
    except Exception as e:
        log("E", f"Error al abrir {args.port}: {e}")
        log("E", "Verifica que el adaptador RS485-USB esté conectado y el puerto sea correcto")
        sys.exit(1)


if __name__ == "__main__":
    main()
