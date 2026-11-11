"""
Estación terrena LoRa - Interfaz de telemetría
Lee por puerto serie lo que imprime la Heltec V3 (115200 baudios)
y manda los comandos: Sx, DHT, BMP, GY, TALLON, TALLOFF, FLOOR.

Requisitos:  pip install pyserial
Uso:         python interfaz_telemetria.py
"""
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports

BAUDIOS = 115200
PUERTO_POR_DEFECTO = "COM7"
TIMEOUT_COMUNICACION = 6.0

BG = "#0f1419"
PANEL = "#171d25"
BORDE = "#263140"
TEXTO = "#e6edf3"
GRIS = "#9aa7b4"
CIAN = "#4cc9f0"
VERDE = "#7ff0bc"
VERDE_BG = "#17301f"
ROJO = "#ff8a8a"
ROJO_BG = "#3a1a1a"
BTN = "#1f2833"
BTN_HOVER = "#2a3644"

RE_PKT = re.compile(r"^Pkt #(\d+)")
RE_ALT = re.compile(r"Altura: (-?[\d.]+) m")
RE_REL = re.compile(r"Altura relativa: (?:(-?[\d.]+) m|\(sin piso\))")
RE_TEMP = re.compile(r"Temp: (-?[\d.]+) C")
RE_PRES = re.compile(r"Presion: (-?[\d.]+) hPa")
RE_HUM = re.compile(r"Humedad: (-?[\d.]+) %")
RE_ACC = re.compile(r"Acc: (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) g")
RE_SERVO = re.compile(r"Servo: (\d+)")
RE_ESTADO = re.compile(r"Estado: DHT=(ON|OFF) BMP=(ON|OFF) GY=(ON|OFF)")
RE_SENSOR = re.compile(r"\b(BMP|DHT|GY): (OFF|ERROR)")
RE_RSSI = re.compile(r"RSSI: (-?[\d.]+) dBm")
RE_SNR = re.compile(r"SNR: (-?[\d.]+) dB")
RE_PISO = re.compile(r"Piso definido en (-?[\d.]+) m")
RE_CMD = re.compile(r">> Comando '(\w+)' (enviado|confirmado|SIN CONFIRMAR)")


class App:
    def __init__(self, root):
        self.root = root
        root.title("Estación terrena LoRa")
        root.configure(bg=BG)
        root.geometry("1100x800")
        root.minsize(1000, 760)

        self.ser = None
        self.stop = threading.Event()
        self.cola = queue.Queue()
        self.ultimo_rx = None

        self.construir()
        self.tick()

    # ------------------------------------------------------ widgets base
    def panel(self, parent, **kw):
        return tk.Frame(parent, bg=PANEL, highlightbackground=BORDE,
                        highlightthickness=1, **kw)

    def etiqueta(self, parent, texto):
        return tk.Label(parent, text=texto, bg=PANEL, fg=GRIS,
                        font=("Segoe UI", 9, "bold"))

    def boton(self, parent, texto, comando, primario=False):
        if primario:
            return tk.Button(parent, text=texto, command=comando, bg=CIAN, fg="#0b1a22",
                             activebackground="#7edcf6", activeforeground="#0b1a22",
                             relief="flat", font=("Segoe UI", 10, "bold"), pady=7)
        return tk.Button(parent, text=texto, command=comando, bg=BTN, fg=TEXTO,
                         activebackground=BTN_HOVER, activeforeground=TEXTO,
                         relief="flat", font=("Segoe UI", 10, "bold"), pady=7)

    def tarjeta_dato(self, parent, titulo, unidad, fuente):
        p = self.panel(parent)
        self.etiqueta(p, titulo).pack(anchor="w", padx=16, pady=(10, 0))
        fila = tk.Frame(p, bg=PANEL)
        fila.pack(anchor="w", padx=16, pady=(0, 10))
        valor = tk.Label(fila, text="---", bg=PANEL, fg=TEXTO, font=("Segoe UI", 22, "bold"))
        valor.pack(side="left")
        tk.Label(fila, text=" " + unidad, bg=PANEL, fg=GRIS,
                 font=("Segoe UI", 12)).pack(side="left", anchor="s", pady=(0, 4))
        tk.Label(p, text=fuente, bg=PANEL, fg=GRIS,
                 font=("Segoe UI", 9)).place(relx=1.0, y=12, x=-14, anchor="ne")
        return p, valor

    def poner(self, lbl, texto, base=TEXTO):
        if texto == "OFF":
            color = GRIS
        elif texto == "ERR":
            color = ROJO
        else:
            color = base
        lbl.configure(text=texto, fg=color)

    # ------------------------------------------------------ construccion
    def construir(self):
        r = self.root
        r.columnconfigure(0, weight=1)
        r.rowconfigure(3, weight=1)

        # ---- Cabecera
        cab = tk.Frame(r, bg=BG)
        cab.grid(row=0, column=0, sticky="ew", padx=24, pady=(20, 10))
        cab.columnconfigure(0, weight=1)

        izq = tk.Frame(cab, bg=BG)
        izq.grid(row=0, column=0, sticky="w")
        tk.Label(izq, text="Estación terrena LoRa", bg=BG, fg=TEXTO,
                 font=("Segoe UI", 18, "bold")).pack(anchor="w")
        tk.Label(izq, text="Radio Node V1  >  Heltec V3  |  915 MHz  |  SF7  |  BW 125 kHz",
                 bg=BG, fg=GRIS, font=("Segoe UI", 10)).pack(anchor="w")

        der = tk.Frame(cab, bg=BG)
        der.grid(row=0, column=1, sticky="e")
        self.var_puerto = tk.StringVar(value=PUERTO_POR_DEFECTO)
        self.combo = ttk.Combobox(der, textvariable=self.var_puerto, width=9,
                                  values=self.listar_puertos())
        self.combo.pack(side="left", padx=(0, 6))
        self.combo.bind("<Button-1>", lambda e: self.combo.configure(values=self.listar_puertos()))
        self.btn_conectar = self.boton(der, "Conectar", self.alternar_conexion)
        self.btn_conectar.configure(padx=12, pady=5)
        self.btn_conectar.pack(side="left", padx=(0, 14))
        self.badge = tk.Label(der, text="Comunicación: False", bg=ROJO_BG, fg=ROJO,
                              font=("Segoe UI", 11, "bold"), padx=16, pady=6)
        self.badge.pack(side="left")

        # ---- Datos principales
        medio = tk.Frame(r, bg=BG)
        medio.grid(row=1, column=0, sticky="ew", padx=24, pady=6)
        medio.columnconfigure(0, weight=11, uniform="a")
        medio.columnconfigure(1, weight=10, uniform="a")

        alt = self.panel(medio)
        alt.grid(row=0, column=0, sticky="nsew", padx=(0, 16))
        self.etiqueta(alt, "ALTURA RELATIVA").pack(anchor="w", padx=22, pady=(16, 0))
        fila = tk.Frame(alt, bg=PANEL)
        fila.pack(anchor="w", padx=22, pady=(10, 6))
        self.lbl_rel = tk.Label(fila, text="---", bg=PANEL, fg=CIAN, font=("Segoe UI", 54, "bold"))
        self.lbl_rel.pack(side="left")
        tk.Label(fila, text=" m", bg=PANEL, fg=GRIS,
                 font=("Segoe UI", 20)).pack(side="left", anchor="s", pady=(0, 12))
        sub = tk.Frame(alt, bg=PANEL)
        sub.pack(anchor="w", padx=22, pady=(10, 16))
        self.lbl_abs = self.sub_dato(sub, "Altura absoluta", 0)
        self.lbl_piso = self.sub_dato(sub, "Piso definido en", 1)

        grid = tk.Frame(medio, bg=BG)
        grid.grid(row=0, column=1, sticky="nsew")
        grid.columnconfigure(0, weight=1, uniform="g")
        grid.columnconfigure(1, weight=1, uniform="g")
        grid.rowconfigure(0, weight=1, uniform="f")
        grid.rowconfigure(1, weight=1, uniform="f")

        p, self.lbl_temp = self.tarjeta_dato(grid, "TEMPERATURA", "°C", "BMP180")
        p.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        p, self.lbl_pres = self.tarjeta_dato(grid, "PRESIÓN", "hPa", "BMP180")
        p.grid(row=0, column=1, sticky="nsew", pady=(0, 8))
        p, self.lbl_hum = self.tarjeta_dato(grid, "HUMEDAD", "%", "DHT11")
        p.grid(row=1, column=0, sticky="nsew", padx=(0, 8))

        pacc = self.panel(grid)
        pacc.grid(row=1, column=1, sticky="nsew")
        self.etiqueta(pacc, "ACELERACIÓN (g)").pack(anchor="w", padx=16, pady=(10, 0))
        tk.Label(pacc, text="GY-521", bg=PANEL, fg=GRIS,
                 font=("Segoe UI", 9)).place(relx=1.0, y=12, x=-14, anchor="ne")
        filaacc = tk.Frame(pacc, bg=PANEL)
        filaacc.pack(fill="x", padx=16, pady=(2, 10))
        self.lbl_acc = []
        for eje in ("X", "Y", "Z"):
            col = tk.Frame(filaacc, bg=PANEL)
            col.pack(side="left", expand=True, fill="x")
            tk.Label(col, text=eje, bg=PANEL, fg=GRIS, font=("Segoe UI", 9)).pack(anchor="w")
            lbl = tk.Label(col, text="---", bg=PANEL, fg=TEXTO, font=("Segoe UI", 16, "bold"))
            lbl.pack(anchor="w")
            self.lbl_acc.append(lbl)

        # ---- Controles
        ctl = tk.Frame(r, bg=BG)
        ctl.grid(row=2, column=0, sticky="ew", padx=24, pady=6)
        for i in range(3):
            ctl.columnconfigure(i, weight=1, uniform="c")

        # Sensores
        ps = self.panel(ctl)
        ps.grid(row=0, column=0, sticky="nsew", padx=(0, 16))
        self.etiqueta(ps, "SENSORES (ON / OFF)").pack(anchor="w", padx=18, pady=(14, 8))
        fila = tk.Frame(ps, bg=PANEL)
        fila.pack(fill="x", padx=18)
        self.btn_sensor = {}
        for nombre, etq in (("DHT", "DHT11"), ("BMP", "BMP180"), ("GY", "GY-521")):
            b = self.boton(fila, f"{etq}: --", lambda n=nombre: self.enviar(n))
            b.configure(fg=GRIS)
            b.pack(side="left", expand=True, fill="x", padx=(0, 6))
            self.btn_sensor[nombre] = (b, etq)
        fila2 = tk.Frame(ps, bg=PANEL)
        fila2.pack(fill="x", padx=18, pady=(8, 14))
        self.boton(fila2, "Todos ON", lambda: self.enviar("TALLON")).pack(
            side="left", expand=True, fill="x", padx=(0, 6))
        self.boton(fila2, "Todos OFF", lambda: self.enviar("TALLOFF")).pack(
            side="left", expand=True, fill="x", padx=(0, 6))

        # Servo
        pv = self.panel(ctl)
        pv.grid(row=0, column=1, sticky="nsew", padx=(0, 16))
        self.etiqueta(pv, "SERVO (SX)").pack(anchor="w", padx=18, pady=(14, 8))
        fila = tk.Frame(pv, bg=PANEL)
        fila.pack(fill="x", padx=18)
        for ang in (0, 90, 180):
            self.boton(fila, f"{ang}°", lambda a=ang: self.enviar(f"S{a}")).pack(
                side="left", expand=True, fill="x", padx=(0, 6))
        fila2 = tk.Frame(pv, bg=PANEL)
        fila2.pack(fill="x", padx=18, pady=(8, 6))
        self.var_ang = tk.StringVar(value="45")
        tk.Entry(fila2, textvariable=self.var_ang, width=6, justify="center",
                 bg=BTN, fg=TEXTO, insertbackground=TEXTO, relief="flat",
                 font=("Segoe UI", 12)).pack(side="left", ipady=5, padx=(0, 8))
        self.boton(fila2, "Mover", self.mover_servo, primario=True).pack(
            side="left", expand=True, fill="x", padx=(0, 6))
        self.lbl_servo = tk.Label(pv, text="Posición actual: --", bg=PANEL, fg=GRIS,
                                  font=("Segoe UI", 10))
        self.lbl_servo.pack(anchor="w", padx=18, pady=(0, 14))

        # Piso y señal
        pp = self.panel(ctl)
        pp.grid(row=0, column=2, sticky="nsew")
        self.etiqueta(pp, "PISO Y SEÑAL").pack(anchor="w", padx=18, pady=(14, 8))
        self.boton(pp, "FLOOR (definir piso)", lambda: self.enviar("FLOOR")).pack(
            fill="x", padx=18)
        fila = tk.Frame(pp, bg=PANEL)
        fila.pack(fill="x", padx=18, pady=(8, 0))
        self.lbl_rssi = tk.Label(fila, text="RSSI -- dBm", bg=PANEL, fg=TEXTO,
                                 font=("Segoe UI", 11, "bold"))
        self.lbl_rssi.pack(side="left", expand=True, anchor="w")
        self.lbl_snr = tk.Label(fila, text="SNR -- dB", bg=PANEL, fg=TEXTO,
                                font=("Segoe UI", 11, "bold"))
        self.lbl_snr.pack(side="left", expand=True, anchor="w")
        self.lbl_cmd = tk.Label(pp, text="Último comando: ninguno", bg=PANEL, fg=GRIS,
                                font=("Segoe UI", 10))
        self.lbl_cmd.pack(anchor="w", padx=18, pady=(8, 14))

        # ---- Registro
        reg = self.panel(r)
        reg.grid(row=3, column=0, sticky="nsew", padx=24, pady=(6, 20))
        self.etiqueta(reg, "REGISTRO").pack(anchor="w", padx=18, pady=(12, 4))
        self.txt = tk.Text(reg, bg=PANEL, fg="#c5d0db", font=("Consolas", 10), relief="flat",
                           height=5, wrap="none", state="disabled", highlightthickness=0)
        self.txt.pack(fill="both", expand=True, padx=18, pady=(0, 12))

    def sub_dato(self, parent, titulo, col):
        f = tk.Frame(parent, bg=PANEL)
        f.grid(row=0, column=col, padx=(0, 32), sticky="w")
        tk.Label(f, text=titulo, bg=PANEL, fg=GRIS, font=("Segoe UI", 10)).pack(anchor="w")
        lbl = tk.Label(f, text="---", bg=PANEL, fg=TEXTO, font=("Segoe UI", 14, "bold"))
        lbl.pack(anchor="w")
        return lbl

    # ------------------------------------------------------ serie
    def listar_puertos(self):
        return [p.device for p in list_ports.comports()]

    def alternar_conexion(self):
        if self.ser:
            self.desconectar()
        else:
            self.conectar()

    def conectar(self):
        try:
            s = serial.Serial()
            s.port = self.var_puerto.get()
            s.baudrate = BAUDIOS
            s.timeout = 0.5
            s.dtr = False   # evita que la Heltec se reinicie al abrir el puerto
            s.rts = False
            s.open()
        except (serial.SerialException, OSError) as e:
            messagebox.showerror(
                "No se pudo abrir el puerto",
                f"{e}\n\nCerrá el monitor serie del Arduino IDE y revisá el puerto.")
            return
        self.ser = s
        self.stop.clear()
        threading.Thread(target=self.leer, args=(s,), daemon=True).start()
        self.btn_conectar.configure(text="Desconectar")
        self.log(f"Conectado a {s.port}")

    def desconectar(self):
        self.stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.btn_conectar.configure(text="Conectar")
        self.log("Desconectado")

    def leer(self, s):
        while not self.stop.is_set():
            try:
                linea = s.readline().decode("utf-8", errors="ignore").strip()
            except (serial.SerialException, OSError):
                self.cola.put(None)
                return
            if linea:
                self.cola.put(linea)

    def enviar(self, cmd):
        if not self.ser:
            messagebox.showinfo("Sin conexión", "Primero conectate al puerto de la Heltec.")
            return
        try:
            self.ser.write((cmd + "\n").encode())
        except (serial.SerialException, OSError):
            self.desconectar()
            return
        self.lbl_cmd.configure(text=f"Último comando: {cmd} (enviando)")

    def mover_servo(self):
        try:
            a = int(self.var_ang.get())
        except ValueError:
            a = -1
        if not 0 <= a <= 180:
            messagebox.showwarning("Ángulo inválido", "Ingresá un número entre 0 y 180.")
            return
        self.enviar(f"S{a}")

    # ------------------------------------------------------ datos
    def estado_sensor(self, nombre, on):
        b, etq = self.btn_sensor[nombre]
        if on:
            b.configure(text=f"{etq}: ON", bg=VERDE_BG, fg=VERDE, activebackground=VERDE_BG,
                        activeforeground=VERDE)
        else:
            b.configure(text=f"{etq}: OFF", bg=BTN, fg=GRIS, activebackground=BTN_HOVER,
                        activeforeground=GRIS)

    def procesar_paquete(self, linea):
        self.ultimo_rx = time.time()
        n = RE_PKT.match(linea).group(1)
        estados = dict(RE_SENSOR.findall(linea))

        def estado_texto(sensor):
            return "ERR" if estados.get(sensor) == "ERROR" else "OFF"

        # BMP180
        m = RE_ALT.search(linea)
        if m:
            self.poner(self.lbl_abs, f"{float(m.group(1)):,.2f} m")
            mr = RE_REL.search(linea)
            rel = mr.group(1) if mr else None
            self.poner(self.lbl_rel, f"{float(rel):+.2f}" if rel else "---", CIAN)
            mt, mp = RE_TEMP.search(linea), RE_PRES.search(linea)
            if mt:
                self.poner(self.lbl_temp, f"{float(mt.group(1)):.1f}")
            if mp:
                self.poner(self.lbl_pres, f"{float(mp.group(1)):.2f}")
        else:
            t = estado_texto("BMP")
            self.poner(self.lbl_abs, t)
            self.poner(self.lbl_rel, t, CIAN)
            self.poner(self.lbl_temp, t)
            self.poner(self.lbl_pres, t)

        # DHT11
        m = RE_HUM.search(linea)
        if m:
            self.poner(self.lbl_hum, f"{float(m.group(1)):.1f}")
        else:
            self.poner(self.lbl_hum, estado_texto("DHT"))

        # GY-521
        m = RE_ACC.search(linea)
        if m:
            for lbl, v in zip(self.lbl_acc, m.groups()):
                self.poner(lbl, f"{float(v):+.3f}")
        else:
            for lbl in self.lbl_acc:
                self.poner(lbl, estado_texto("GY"))

        # Servo, estados y señal
        m = RE_SERVO.search(linea)
        if m:
            self.lbl_servo.configure(text=f"Posición actual: {m.group(1)}°")
        m = RE_ESTADO.search(linea)
        if m:
            for nombre, v in zip(("DHT", "BMP", "GY"), m.groups()):
                self.estado_sensor(nombre, v == "ON")
        mr, ms = RE_RSSI.search(linea), RE_SNR.search(linea)
        if mr:
            self.lbl_rssi.configure(text=f"RSSI {float(mr.group(1)):.0f} dBm")
        if ms:
            self.lbl_snr.configure(text=f"SNR {float(ms.group(1)):.1f} dB")
        self.log(f"Pkt #{n}  RSSI {mr.group(1) if mr else '--'} dBm")

    def procesar(self, linea):
        if RE_PKT.match(linea):
            self.procesar_paquete(linea)
            return
        m = RE_PISO.search(linea)
        if m:
            self.lbl_piso.configure(text=f"{float(m.group(1)):,.2f} m")
            self.lbl_cmd.configure(text="Último comando: FLOOR (listo)")
            self.log(linea)
            return
        m = RE_CMD.search(linea)
        if m:
            self.lbl_cmd.configure(text=f"Último comando: {m.group(1)} ({m.group(2).lower()})")
            self.log(linea)
            return
        if linea.startswith(">>"):
            self.log(linea)

    def log(self, texto):
        self.txt.configure(state="normal")
        self.txt.insert("end", time.strftime("%H:%M:%S  ") + texto + "\n")
        if int(self.txt.index("end-1c").split(".")[0]) > 200:
            self.txt.delete("1.0", "2.0")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def tick(self):
        try:
            while True:
                linea = self.cola.get_nowait()
                if linea is None:
                    self.desconectar()
                    self.log("Se perdió la conexión con el puerto")
                else:
                    self.procesar(linea)
        except queue.Empty:
            pass

        ok = self.ultimo_rx is not None and (time.time() - self.ultimo_rx) < TIMEOUT_COMUNICACION
        if ok:
            self.badge.configure(text="Comunicación: True", bg=VERDE_BG, fg=VERDE)
        else:
            self.badge.configure(text="Comunicación: False", bg=ROJO_BG, fg=ROJO)

        self.root.after(100, self.tick)

    def cerrar(self):
        self.desconectar()
        self.root.destroy()


if __name__ == "__main__":
    ventana = tk.Tk()
    app = App(ventana)
    ventana.protocol("WM_DELETE_WINDOW", app.cerrar)
    ventana.mainloop()
