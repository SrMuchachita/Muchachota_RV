#!/usr/bin/env python3
"""
RD90 Pro - Actualizador de firmware por BLE (OTA)
==================================================

GUI para subir un firmware compilado (.bin) al robot ESP32-S3 por Bluetooth LE,
sin cable. Eliges el archivo build/WP_R_V9_UP.bin, conectas al robot y lo subes.

IMPORTANTE: este GUI es solo la mitad PC del sistema. Para que funcione, el
firmware del robot debe exponer el servicio GATT de OTA descrito abajo (eso es la
"parte 2" de la Fase B, todavia por implementar). Mientras ese servicio no exista
en el robot, el GUI conecta y escanea, pero al subir dira que no encuentra el
servicio OTA. Los UUID y el protocolo de aqui son el contrato que el firmware debe
cumplir EXACTAMENTE.

--------------------------------------------------------------------------------
PROTOCOLO OTA-BLE (contrato PC <-> firmware)
--------------------------------------------------------------------------------
Servicio OTA (128-bit, mismo espacio de UUID vendor que el control BLE):
    SERVICE : 1000454c-422d-4c52-5443-30394452905a
    DATA    : 1100454c-422d-4c52-5443-30394452905a   (Write / Write-No-Response)
    CONTROL : 1200454c-422d-4c52-5443-30394452905a   (Write + Notify)

Equivalente en C para el firmware (NimBLE, orden de bytes little-endian):
    // SERVICE  -> ...00,0x10
    BLE_UUID128_INIT(0x5a,0x90,0x52,0x44,0x39,0x30,0x43,0x54,
                     0x52,0x4c,0x2d,0x42,0x4c,0x45,0x00,0x10)
    // DATA     -> ...00,0x11
    BLE_UUID128_INIT(0x5a,0x90,0x52,0x44,0x39,0x30,0x43,0x54,
                     0x52,0x4c,0x2d,0x42,0x4c,0x45,0x00,0x11)
    // CONTROL  -> ...00,0x12
    BLE_UUID128_INIT(0x5a,0x90,0x52,0x44,0x39,0x30,0x43,0x54,
                     0x52,0x4c,0x2d,0x42,0x4c,0x45,0x00,0x12)

Comandos del PC -> robot (se escriben en CONTROL):
    START : [0x01][size:uint32 LE]   -> robot hace esp_ota_begin en la particion
                                         siguiente; responde READY o ERROR.
    END   : [0x02][crc32:uint32 LE]  -> robot valida/termina (esp_ota_end +
                                         set_boot_partition); responde DONE y
                                         reinicia. El CRC32 es del .bin completo;
                                         el firmware puede verificarlo u omitirlo.
    ABORT : [0x03]                   -> robot cancela (esp_ota_abort).

Datos del firmware: se escriben en DATA como bytes crudos, en orden. El robot los
pasa tal cual a esp_ota_write(). Tamano de cada trozo = (MTU - 3) bytes.

Estados del robot -> PC (llegan por notificacion de CONTROL):
    READY : [0x10]                   -> START aceptado, listo para recibir.
    DONE  : [0x11]                   -> OTA completo, reiniciando.
    ERROR : [0x12][code:uint8]       -> fallo (code = detalle, libre).
    PROG  : [0x13][bytes:uint32 LE]  -> (opcional) bytes recibidos hasta ahora.

--------------------------------------------------------------------------------
Requisitos:
    pip install bleak
Uso:
    python ble_ota_gui.py
"""

import os
import time
import queue
import struct
import zlib
import asyncio
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    from bleak import BleakScanner, BleakClient
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False

# ── UUIDs del servicio OTA (ver cabecera) ────────────────────────────────────
OTA_SVC_UUID  = "1000454c-422d-4c52-5443-30394452905a"
OTA_DATA_UUID = "1100454c-422d-4c52-5443-30394452905a"
OTA_CTRL_UUID = "1200454c-422d-4c52-5443-30394452905a"

DEVICE_NAME_HINT = "RD90PRO-BLE"   # nombre que anuncia el robot (BLE_DEVICE_NAME)

# ── Opcodes del protocolo ────────────────────────────────────────────────────
CMD_START, CMD_END, CMD_ABORT = 0x01, 0x02, 0x03
ST_READY,  ST_DONE, ST_ERROR, ST_PROG = 0x10, 0x11, 0x12, 0x13

# ── Tema (mismo estilo que imu_monitor_gui.py) ───────────────────────────────
BG    = "#0D1117"
CARD  = "#070809"
CARD2 = "#0C0D0F"
AMBER = "#FFD000"
GREEN = "#39FF6A"
RED   = "#FF4C4C"
WHITE = "#FFF5D0"
GRAY  = "#8B7A45"

FT = ("Consolas", 16, "bold")
FS = ("Consolas", 12, "bold")
FB = ("Consolas", 11, "bold")
FL = ("Consolas", 10)


# ═════════════════════════════════════════════════════════════════════════════
#  Trabajador BLE: corre un loop asyncio en su propio hilo. La GUI (tkinter, que
#  es sincrona) le encarga corrutinas y recibe resultados por una cola.
# ═════════════════════════════════════════════════════════════════════════════
class BleWorker:
    def __init__(self, ui_queue: queue.Queue):
        self.q = ui_queue
        self.client = None
        self._abort = False
        # Eventos de sincronizacion (se crean dentro del loop, en do_ota)
        self._ready = None
        self._done = None
        self._error_ev = None
        self._err_code = None
        # Loop asyncio en hilo aparte
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def submit(self, coro):
        """Encola una corrutina en el loop BLE desde el hilo de la GUI."""
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def log(self, msg):
        self.q.put(("log", msg))

    # ── Escaneo ──────────────────────────────────────────────────────────────
    async def scan(self, timeout=6.0):
        if not BLEAK_AVAILABLE:
            self.log("bleak no esta instalado. Corre: pip install bleak")
            return
        self.log(f"Escaneando {timeout:.0f} s...")
        try:
            devs = await BleakScanner.discover(timeout=timeout)
        except Exception as e:
            self.log(f"Error al escanear: {e}")
            return
        items = []
        for d in devs:
            items.append((d.name or "(sin nombre)", d.address))
        # Primero los que parecen ser el robot
        items.sort(key=lambda x: (DEVICE_NAME_HINT.lower() not in (x[0] or "").lower(),
                                   x[0].lower()))
        self.q.put(("devices", items))
        self.log(f"Encontrados {len(items)} dispositivos.")

    # ── Conexion ───────────────────────────────────────────────────────────
    def _on_disconnect(self, _client):
        self.q.put(("connected", False))
        self.log("Desconectado del dispositivo.")

    async def connect(self, address):
        try:
            self.client = BleakClient(address, disconnected_callback=self._on_disconnect)
            await self.client.connect()
            mtu = getattr(self.client, "mtu_size", "?")
            self.q.put(("connected", True))
            self.log(f"Conectado a {address}  (MTU={mtu})")
        except Exception as e:
            self.client = None
            self.q.put(("connected", False))
            self.log(f"Fallo al conectar: {e}")

    async def disconnect(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
        except Exception as e:
            self.log(f"Error al desconectar: {e}")
        finally:
            self.q.put(("connected", False))

    def request_abort(self):
        self._abort = True

    # ── Notificaciones del robot ─────────────────────────────────────────────
    def _on_ctrl(self, _sender, data: bytearray):
        if not data:
            return
        code = data[0]
        if code == ST_READY:
            if self._ready:
                self._ready.set()
        elif code == ST_DONE:
            if self._done:
                self._done.set()
        elif code == ST_ERROR:
            self._err_code = data[1] if len(data) > 1 else 0
            if self._error_ev:
                self._error_ev.set()
            self.log(f"El robot reporto ERROR (code={self._err_code}).")
        elif code == ST_PROG:
            if len(data) >= 5:
                n = struct.unpack("<I", data[1:5])[0]
                self.log(f"Robot confirmo {n} bytes escritos.")

    async def _await(self, ev, timeout):
        """Espera 'ev' o un ERROR, lo que ocurra primero."""
        t_ok = asyncio.create_task(ev.wait())
        t_err = asyncio.create_task(self._error_ev.wait())
        done, pending = await asyncio.wait({t_ok, t_err}, timeout=timeout,
                                           return_when=asyncio.FIRST_COMPLETED)
        for t in pending:
            t.cancel()
        if self._error_ev.is_set():
            raise RuntimeError(f"el robot reporto error (code={self._err_code})")
        if not ev.is_set():
            raise TimeoutError("sin respuesta del robot (timeout)")

    # ── Secuencia OTA completa ───────────────────────────────────────────────
    async def do_ota(self, path, use_response):
        if not (self.client and self.client.is_connected):
            self.log("No estas conectado a ningun robot.")
            self.q.put(("ota_done", (False, "sin conexion")))
            return

        # Verificar que el servicio OTA exista en el robot
        try:
            svcs = self.client.services
            has_ota = any(c.uuid.lower() == OTA_DATA_UUID
                          for s in svcs for c in s.characteristics)
        except Exception:
            has_ota = True  # si no se pudo listar, intentamos igual
        if not has_ota:
            self.log("Este robot NO expone el servicio OTA-BLE todavia.")
            self.log("Falta implementar el servicio GATT de OTA en el firmware (Fase B parte 2).")
            self.q.put(("ota_done", (False, "firmware sin servicio OTA")))
            return

        try:
            with open(path, "rb") as f:
                blob = f.read()
        except Exception as e:
            self.log(f"No se pudo leer el .bin: {e}")
            self.q.put(("ota_done", (False, "archivo ilegible")))
            return

        size = len(blob)
        crc = zlib.crc32(blob) & 0xFFFFFFFF
        self.log(f"Firmware: {os.path.basename(path)}  |  {size} bytes  |  CRC32=0x{crc:08x}")

        # Preparar eventos (en el contexto del loop)
        self._ready = asyncio.Event()
        self._done = asyncio.Event()
        self._error_ev = asyncio.Event()
        self._err_code = None
        self._abort = False

        try:
            await self.client.start_notify(OTA_CTRL_UUID, self._on_ctrl)

            # 1) START
            await self.client.write_gatt_char(
                OTA_CTRL_UUID, bytes([CMD_START]) + struct.pack("<I", size), response=True)
            self.log("START enviado. Esperando READY del robot (borra flash, puede tardar)...")
            await self._await(self._ready, timeout=30)
            self.log("Robot listo. Transmitiendo firmware...")

            # 2) DATA en trozos
            mtu = getattr(self.client, "mtu_size", 23) or 23
            chunk = max(mtu - 3, 20)
            self.log(f"Tamano de trozo: {chunk} bytes  (modo {'confiable' if use_response else 'rapido'})")

            sent = 0
            i = 0
            t0 = time.time()
            last_ui = 0.0
            while sent < size:
                if self._abort:
                    await self.client.write_gatt_char(OTA_CTRL_UUID, bytes([CMD_ABORT]), response=True)
                    self.log("OTA cancelado por el usuario.")
                    self.q.put(("ota_done", (False, "cancelado")))
                    await self._safe_stop_notify()
                    return

                part = blob[sent:sent + chunk]
                await self.client.write_gatt_char(OTA_DATA_UUID, part, response=use_response)
                sent += len(part)
                i += 1

                # En modo rapido (sin respuesta) damos un respiro al stack
                if not use_response and (i % 20 == 0):
                    await asyncio.sleep(0.003)

                now = time.time()
                if now - last_ui > 0.1 or sent >= size:
                    speed = sent / max(now - t0, 1e-3)
                    self.q.put(("progress", (sent, size, speed)))
                    last_ui = now

            # 3) END
            await self.client.write_gatt_char(
                OTA_CTRL_UUID, bytes([CMD_END]) + struct.pack("<I", crc), response=True)
            self.log("END enviado. Esperando confirmacion (DONE)...")
            try:
                await self._await(self._done, timeout=20)
                self.log("OK. OTA completo. El robot se esta reiniciando.")
                self.q.put(("ota_done", (True, "completo")))
            except Exception as e:
                # El robot puede reiniciar y cortar el enlace justo en DONE.
                self.log(f"Sin confirmacion clara ({e}). Si el robot reinicio, probablemente OK.")
                self.q.put(("ota_done", (True, "fin - verifica la version")))

            await self._safe_stop_notify()

        except Exception as e:
            self.log(f"Error durante el OTA: {e}")
            self.q.put(("ota_done", (False, str(e))))
            await self._safe_stop_notify()

    async def _safe_stop_notify(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.stop_notify(OTA_CTRL_UUID)
        except Exception:
            pass


# ═════════════════════════════════════════════════════════════════════════════
#  GUI tkinter
# ═════════════════════════════════════════════════════════════════════════════
class OtaGui:
    def __init__(self, root):
        self.root = root
        self.q = queue.Queue()
        self.worker = BleWorker(self.q)

        self.devices = []          # [(name, address), ...]
        self.bin_path = None
        self.connected = False
        self.busy = False          # True durante escaneo/conexion/ota
        self.t_start = None        # inicio de la subida (para cronometro)

        root.title("RD90 Pro - OTA por BLE")
        root.configure(bg=BG)
        root.geometry("680x620")
        root.minsize(620, 560)

        self._build_styles()
        self._build_widgets()

        if not BLEAK_AVAILABLE:
            self._log("bleak NO esta instalado. Corre:  pip install bleak")
            messagebox.showwarning("Falta dependencia",
                                   "Necesitas instalar bleak:\n\n    pip install bleak")

        self.root.after(50, self._poll_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Estilos ttk ────────────────────────────────────────────────────────
    def _build_styles(self):
        st = ttk.Style()
        try:
            st.theme_use("clam")
        except Exception:
            pass
        st.configure("RD.Horizontal.TProgressbar",
                     troughcolor=CARD2, background=GREEN, bordercolor=CARD2,
                     lightcolor=GREEN, darkcolor=GREEN, thickness=22)
        st.configure("RD.TCombobox",
                     fieldbackground=CARD2, background=CARD2, foreground=WHITE,
                     arrowcolor=AMBER, selectbackground=CARD2, selectforeground=WHITE)

    def _btn(self, parent, text, cmd, color=AMBER):
        return tk.Button(parent, text=text, command=cmd,
                         bg=CARD2, fg=color, activebackground=CARD,
                         activeforeground=WHITE, font=FB, relief="flat",
                         bd=0, highlightthickness=1, highlightbackground=GRAY,
                         padx=10, pady=6, cursor="hand2")

    def _lbl(self, parent, text, fg=WHITE, font=FL):
        return tk.Label(parent, text=text, bg=BG, fg=fg, font=font)

    # ── Construccion de widgets ──────────────────────────────────────────────
    def _build_widgets(self):
        self._lbl(self.root, "ACTUALIZADOR DE FIRMWARE POR BLE", AMBER, FT).pack(pady=(14, 2))
        self._lbl(self.root, "RD90 Pro  ·  ESP32-S3", GRAY, FL).pack(pady=(0, 10))

        # --- Fila 1: escaneo + dispositivo ---
        f1 = tk.Frame(self.root, bg=BG); f1.pack(fill="x", padx=16, pady=4)
        self.btn_scan = self._btn(f1, "Buscar", self._on_scan)
        self.btn_scan.pack(side="left")
        self.cmb_dev = ttk.Combobox(f1, style="RD.TCombobox", state="readonly",
                                    font=FL, width=40)
        self.cmb_dev.pack(side="left", fill="x", expand=True, padx=8)

        # --- Fila 2: conectar + estado ---
        f2 = tk.Frame(self.root, bg=BG); f2.pack(fill="x", padx=16, pady=4)
        self.btn_conn = self._btn(f2, "Conectar", self._on_connect)
        self.btn_conn.pack(side="left")
        self.lbl_state = self._lbl(f2, "● Desconectado", RED, FS)
        self.lbl_state.pack(side="left", padx=12)

        # --- Fila 3: archivo .bin ---
        f3 = tk.Frame(self.root, bg=BG); f3.pack(fill="x", padx=16, pady=(12, 4))
        self._btn(f3, "Seleccionar .bin", self._on_pick).pack(side="left")
        self.lbl_file = self._lbl(f3, "(ningun archivo)", GRAY, FL)
        self.lbl_file.pack(side="left", padx=10)

        # --- Fila 4: modo de escritura ---
        f4 = tk.Frame(self.root, bg=BG); f4.pack(fill="x", padx=16, pady=4)
        self._lbl(f4, "Modo:", WHITE, FL).pack(side="left")
        self.cmb_mode = ttk.Combobox(f4, style="RD.TCombobox", state="readonly",
                                     font=FL, width=34,
                                     values=["Confiable (con respuesta) - recomendado",
                                             "Rapido (sin respuesta) - experimental"])
        self.cmb_mode.current(0)
        self.cmb_mode.pack(side="left", padx=8)

        # --- Barra de progreso ---
        f5 = tk.Frame(self.root, bg=BG); f5.pack(fill="x", padx=16, pady=(12, 2))
        self.pbar = ttk.Progressbar(f5, style="RD.Horizontal.TProgressbar",
                                    orient="horizontal", mode="determinate", maximum=100)
        self.pbar.pack(fill="x")
        self.lbl_prog = self._lbl(self.root, "0%   ·   0.0 KB/s", WHITE, FS)
        self.lbl_prog.pack(pady=2)
        self.lbl_time = self._lbl(self.root, "Tiempo:  0:00", AMBER, FS)
        self.lbl_time.pack(pady=(0, 2))

        # --- Botones de accion ---
        f6 = tk.Frame(self.root, bg=BG); f6.pack(fill="x", padx=16, pady=8)
        self.btn_update = self._btn(f6, "  ACTUALIZAR  ", self._on_update, GREEN)
        self.btn_update.pack(side="left")
        self.btn_cancel = self._btn(f6, "Cancelar", self._on_cancel, RED)
        self.btn_cancel.pack(side="left", padx=8)

        # --- Log ---
        self._lbl(self.root, "Registro", GRAY, FL).pack(anchor="w", padx=16)
        lf = tk.Frame(self.root, bg=CARD, highlightthickness=1, highlightbackground=GRAY)
        lf.pack(fill="both", expand=True, padx=16, pady=(0, 14))
        self.txt = tk.Text(lf, bg=CARD, fg=WHITE, font=("Consolas", 9),
                           relief="flat", bd=0, wrap="word", height=10)
        self.txt.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        sb = tk.Scrollbar(lf, command=self.txt.yview)
        sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set, state="disabled")

        self._refresh_buttons()

    # ── Acciones de la GUI ───────────────────────────────────────────────────
    def _on_scan(self):
        if self.busy:
            return
        self._set_busy(True, "Escaneando...")
        self.worker.submit(self.worker.scan())

    def _on_connect(self):
        if self.connected:
            self.worker.submit(self.worker.disconnect())
            return
        addr = self._selected_address()
        if not addr:
            messagebox.showinfo("Selecciona un dispositivo",
                                "Primero busca y elige el robot en la lista.")
            return
        self._set_busy(True, "Conectando...")
        self.worker.submit(self.worker.connect(addr))

    def _on_pick(self):
        path = filedialog.askopenfilename(
            title="Elige el firmware compilado",
            filetypes=[("Firmware ESP32 (*.bin)", "*.bin"), ("Todos", "*.*")],
            initialdir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "build"))
        if path:
            self.bin_path = path
            size = os.path.getsize(path)
            self.lbl_file.config(text=f"{os.path.basename(path)}  ({size} bytes)", fg=WHITE)
            self._log(f"Archivo seleccionado: {path}")
        self._refresh_buttons()

    def _on_update(self):
        if self.busy or not self.connected or not self.bin_path:
            return
        if not messagebox.askyesno("Confirmar actualizacion",
                                   "Se va a subir el firmware al robot por BLE.\n"
                                   "No apagues ni desconectes el robot durante el proceso.\n\n"
                                   "Continuar?"):
            return
        use_response = (self.cmb_mode.current() == 0)
        self.t_start = time.time()
        self.lbl_time.config(text="Tiempo:  0:00")
        self._set_busy(True, "Actualizando...")
        self.pbar["value"] = 0
        self.worker.submit(self.worker.do_ota(self.bin_path, use_response))

    def _on_cancel(self):
        if self.busy:
            self.worker.request_abort()
            self._log("Cancelando...")

    def _on_close(self):
        try:
            self.worker.submit(self.worker.disconnect())
        except Exception:
            pass
        self.root.after(200, self.root.destroy)

    # ── Utilidades GUI ─────────────────────────────────────────────────────
    def _selected_address(self):
        idx = self.cmb_dev.current()
        if 0 <= idx < len(self.devices):
            return self.devices[idx][1]
        return None

    def _set_busy(self, busy, status=None):
        self.busy = busy
        if status:
            self._log(status)
        self._refresh_buttons()

    def _refresh_buttons(self):
        can_scan = (not self.busy)
        can_conn = (not self.busy) or self.connected
        can_upd = (self.connected and self.bin_path and not self.busy)
        self.btn_scan.config(state="normal" if can_scan else "disabled")
        self.btn_conn.config(state="normal" if can_conn else "disabled",
                             text="Desconectar" if self.connected else "Conectar")
        self.btn_update.config(state="normal" if can_upd else "disabled")
        self.btn_cancel.config(state="normal" if self.busy else "disabled")

    def _log(self, msg):
        self.txt.configure(state="normal")
        self.txt.insert("end", time.strftime("[%H:%M:%S] ") + str(msg) + "\n")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    @staticmethod
    def _mmss(secs):
        secs = int(secs)
        return f"{secs // 60}:{secs % 60:02d}"

    # ── Cola de mensajes del worker BLE ───────────────────────────────────────
    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "log":
                    self._log(payload)
                elif kind == "devices":
                    self.devices = payload
                    labels = [f"{n}   [{a}]" for (n, a) in payload]
                    self.cmb_dev["values"] = labels
                    if labels:
                        self.cmb_dev.current(0)
                    self._set_busy(False)
                elif kind == "connected":
                    self.connected = bool(payload)
                    self.lbl_state.config(
                        text="● Conectado" if self.connected else "● Desconectado",
                        fg=GREEN if self.connected else RED)
                    self._set_busy(False)
                elif kind == "progress":
                    sent, total, speed = payload
                    pct = (sent / total * 100) if total else 0
                    self.pbar["value"] = pct
                    eta = (total - sent) / speed if speed > 1 else 0
                    self.lbl_prog.config(
                        text=f"{pct:5.1f}%   ·   {speed/1024:5.1f} KB/s   ·   "
                             f"{sent}/{total} B   ·   ETA {eta:4.0f}s")
                elif kind == "ota_done":
                    ok, msg = payload
                    self._set_busy(False)
                    total = (time.time() - self.t_start) if self.t_start else 0
                    self.lbl_time.config(text=f"Tiempo:  {self._mmss(total)}")
                    if ok:
                        self.pbar["value"] = 100
                        self._log(f"RESULTADO: {msg}   (tiempo: {self._mmss(total)})")
                        messagebox.showinfo("OTA",
                                            f"Actualizacion terminada:\n{msg}\n\n"
                                            f"Tiempo: {self._mmss(total)} (min:seg)")
                    else:
                        self._log(f"RESULTADO: fallo ({msg})   (tiempo: {self._mmss(total)})")
                        messagebox.showerror("OTA",
                                             f"No se completo:\n{msg}\n\n"
                                             f"Tiempo: {self._mmss(total)} (min:seg)")
                    self.t_start = None
        except queue.Empty:
            pass
        # Cronometro en vivo mientras sube
        if self.busy and self.t_start:
            self.lbl_time.config(text=f"Tiempo:  {self._mmss(time.time() - self.t_start)}")
        self.root.after(50, self._poll_queue)


def main():
    root = tk.Tk()
    OtaGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
