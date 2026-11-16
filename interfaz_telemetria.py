
"""
Estación terrena LoRa - Interfaz de telemetría
Lee por puerto serie lo que imprime la Heltec V3 (115200 baudios)
y manda los comandos remotos: Sx y FLOOR. Los toggles de sensores solo
ocultan o muestran lecturas localmente.

Requisitos:  pip install pyserial
Uso:         python interfaz_telemetria.py
"""
import queue
import math
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
        self.historial_acc = []
        self.historial_temp = []
        self.historial_hum = []
        self.muestras_altura = []
        self.cda_activo = False
        self.gravedad_base = None
        self.muestras_calibracion = []
        self.muestra_mov_anterior = None
        self.velocidad_ejes = [0.0, 0.0, 0.0]
        self.distancia_ejes = [0.0, 0.0, 0.0]
        self.mostrar_componentes = {"BMP": tk.BooleanVar(value=True),
                                    "DHT": tk.BooleanVar(value=True),
                                    "GY": tk.BooleanVar(value=True)}
        self.marcos_sensor = {"BMP": [], "DHT": [], "GY": []}
        self.mostrar_ejes = {"X": tk.BooleanVar(value=True),
                             "Y": tk.BooleanVar(value=True),
                             "Z": tk.BooleanVar(value=True)}

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
        for col in range(3):
            r.columnconfigure(col, weight=1, uniform="inferior")
        r.rowconfigure(3, weight=1)

        # ---- Cabecera
        cab = tk.Frame(r, bg=BG)
        cab.grid(row=0, column=0, columnspan=3, sticky="ew", padx=24, pady=(20, 10))
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
        medio.grid(row=1, column=0, columnspan=3, sticky="ew", padx=24, pady=6)
        medio.columnconfigure(0, weight=4, uniform="a")
        medio.columnconfigure(1, weight=9, uniform="a")
        medio.columnconfigure(2, weight=6, uniform="a")

        alt = self.panel(medio)
        alt.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.marcos_sensor["BMP"].append(alt)
        self.etiqueta(alt, "ALTURA RELATIVA").pack(anchor="w", padx=14, pady=(12, 0))
        fila_alt = tk.Frame(alt, bg=PANEL)
        fila_alt.pack(fill="x", padx=14, pady=(2, 5))
        self.lbl_rel = tk.Label(fila_alt, text="---", bg=PANEL, fg=CIAN,
                                font=("Segoe UI", 32, "bold"))
        self.lbl_rel.pack(side="left")
        tk.Label(fila_alt, text=" m", bg=PANEL, fg=GRIS,
                 font=("Segoe UI", 14)).pack(side="left", anchor="s", pady=(0, 6))
        sub = tk.Frame(alt, bg=PANEL)
        sub.pack(anchor="w", padx=14, pady=(2, 10))
        self.lbl_abs = self.sub_dato(sub, "Altura absoluta", 0)
        self.lbl_piso = self.sub_dato(sub, "Piso definido", 1)

        grid = tk.Frame(medio, bg=BG)
        grid.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        grid.columnconfigure(0, weight=1, uniform="g")
        grid.columnconfigure(1, weight=1, uniform="g")
        grid.rowconfigure(0, weight=1, uniform="f")
        grid.rowconfigure(1, weight=1, uniform="f")

        recorrido = self.panel(medio)
        recorrido.grid(row=0, column=2, sticky="nsew")
        self.etiqueta(recorrido, "DISTANCIA RECORRIDA · CDA").pack(
            anchor="w", padx=14, pady=(12, 6))
        self.lbl_dist = {}
        for eje, color in (("X", "#4c9aff"), ("Y", "#ff5c5c"), ("Z", "#48d17a")):
            fila = tk.Frame(recorrido, bg=PANEL)
            fila.pack(fill="x", padx=14, pady=2)
            tk.Label(fila, text=eje, bg=PANEL, fg=color,
                     font=("Segoe UI", 11, "bold"), width=3, anchor="w").pack(side="left")
            self.lbl_dist[eje] = tk.Label(fila, text="--- m", bg=PANEL, fg=TEXTO,
                                          font=("Segoe UI", 12, "bold"))
            self.lbl_dist[eje].pack(side="left")
        self.lbl_velocidad = tk.Label(recorrido, text="Rapidez: --- m/s",
                                      bg=PANEL, fg=CIAN,
                                      font=("Segoe UI", 13, "bold"))
        self.lbl_velocidad.pack(anchor="w", padx=14, pady=(8, 0))
        self.btn_cda = self.boton(recorrido, "CDA · iniciar medición",
                                  self.activar_cda, primario=True)
        self.btn_cda.pack(anchor="w", padx=14, pady=(8, 10))

        p, self.lbl_temp = self.tarjeta_dato(grid, "TEMPERATURA", "°C", "BMP180")
        p.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        self.marcos_sensor["BMP"].append(p)
        p, self.lbl_pres = self.tarjeta_dato(grid, "PRESIÓN", "hPa", "BMP180")
        p.grid(row=0, column=1, sticky="nsew", pady=(0, 8))
        self.marcos_sensor["BMP"].append(p)
        p, self.lbl_hum = self.tarjeta_dato(grid, "HUMEDAD", "%", "DHT11")
        p.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        self.marcos_sensor["DHT"].append(p)

        pacc = self.panel(grid)
        pacc.grid(row=1, column=1, sticky="nsew")
        self.marcos_sensor["GY"].append(pacc)
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
        ctl.grid(row=2, column=0, columnspan=3, sticky="ew", padx=24, pady=6)
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
            b = tk.Checkbutton(
                fila, text=etq, variable=self.mostrar_componentes[nombre],
                command=lambda n=nombre: self.alternar_vista_sensor(n),
                bg=PANEL, fg=TEXTO, activebackground=PANEL,
                activeforeground=TEXTO, selectcolor=BTN,
                font=("Segoe UI", 9, "bold")
            )
            b.pack(side="left", expand=True, fill="x", padx=(0, 6))
            self.btn_sensor[nombre] = b
        fila2 = tk.Frame(ps, bg=PANEL)
        fila2.pack(fill="x", padx=18, pady=(8, 14))
        self.boton(fila2, "Mostrar todos", lambda: self.set_todos_visibles(True)).pack(
            side="left", expand=True, fill="x", padx=(0, 6))
        self.boton(fila2, "Ocultar todos", lambda: self.set_todos_visibles(False)).pack(
            side="left", expand=True, fill="x", padx=(0, 6))

        # Servo
        pv = self.panel(ctl)
        pv.grid(row=0, column=1, sticky="nsew", padx=(0, 16))
        self.etiqueta(pv, "SG90 360 · SENTIDO/VELOCIDAD").pack(anchor="w", padx=18, pady=(14, 8))
        fila = tk.Frame(pv, bg=PANEL)
        fila.pack(fill="x", padx=18)
        for ang, texto in ((0, "S0"), (90, "S90 · detener"), (180, "S180")):
            self.boton(fila, texto, lambda a=ang: self.enviar(f"S{a}")).pack(
                side="left", expand=True, fill="x", padx=(0, 6))
        fila2 = tk.Frame(pv, bg=PANEL)
        fila2.pack(fill="x", padx=18, pady=(8, 6))
        self.var_ang = tk.StringVar(value="45")
        tk.Entry(fila2, textvariable=self.var_ang, width=6, justify="center",
                 bg=BTN, fg=TEXTO, insertbackground=TEXTO, relief="flat",
                 font=("Segoe UI", 12)).pack(side="left", ipady=5, padx=(0, 8))
        self.boton(fila2, "Mover", self.mover_servo, primario=True).pack(
            side="left", expand=True, fill="x", padx=(0, 6))
        self.lbl_servo = tk.Label(pv, text="Último valor enviado: --", bg=PANEL, fg=GRIS,
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

        # ---- Tres graficos compactos en la franja inferior
        tele = self.panel(r)
        tele.grid(row=3, column=0, sticky="nsew", padx=(24, 6), pady=(6, 8))
        self.etiqueta(tele, "ACELERACIÓN · última muestra (nodo: 2 s)").pack(
            anchor="w", padx=12, pady=(9, 3))
        graf = tk.Frame(tele, bg=PANEL)
        graf.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        controles = tk.Frame(graf, bg=PANEL)
        controles.pack(fill="x", anchor="w")
        colores = {"X": "#4c9aff", "Y": "#ff5c5c", "Z": "#48d17a"}
        for eje in ("X", "Y", "Z"):
            tk.Checkbutton(
                controles, text=eje, variable=self.mostrar_ejes[eje],
                command=self.dibujar_grafico, bg=PANEL, fg=colores[eje],
                activebackground=PANEL, activeforeground=colores[eje],
                selectcolor=BG, font=("Segoe UI", 9, "bold")
            ).pack(side="left", padx=(0, 8))

        self.canvas_acc = tk.Canvas(
            graf, height=125, bg="#111820", highlightthickness=1,
            highlightbackground=BORDE
        )
        self.canvas_acc.pack(fill="both", expand=True, pady=(2, 0))
        self.canvas_acc.bind("<Configure>", lambda _e: self.dibujar_grafico())
        self.marcos_sensor["GY"].append(tele)

        metricas = tk.Frame(tele, bg=PANEL)
        metricas.pack(fill="x", padx=12, pady=(0, 8))
        self.etiqueta(metricas, "ACELERACIÓN TOTAL").pack(side="left")
        self.lbl_acc_total = tk.Label(
            metricas, text="--- g | --- m/s²", bg=PANEL, fg=TEXTO,
            font=("Segoe UI", 11, "bold")
        )
        self.lbl_acc_total.pack(side="left", padx=(6, 14))
        self.etiqueta(metricas, "IMPACTO").pack(side="left")
        self.lbl_impacto = tk.Label(
            metricas, text="", bg=PANEL, fg=GRIS, font=("Segoe UI", 12, "bold")
        )
        self.lbl_impacto.pack_forget()

        self.canvas_temp = self.crear_grafico_simple(r, 3, 1, "TEMPERATURA · 1 s", "°C", "#ffb454")
        self.canvas_hum = self.crear_grafico_simple(r, 3, 2, "HUMEDAD · 1 s", "%", "#4cc9f0")
        self.marcos_sensor["BMP"].append(self.canvas_temp.master)
        self.marcos_sensor["DHT"].append(self.canvas_hum.master)

        # ---- Registro debajo de los graficos
        reg = self.panel(r)
        reg.grid(row=4, column=0, columnspan=3, sticky="nsew", padx=24, pady=(0, 14))
        self.etiqueta(reg, "REGISTRO").pack(anchor="w", padx=18, pady=(6, 2))
        self.txt = tk.Text(reg, bg=PANEL, fg="#c5d0db", font=("Consolas", 9), relief="flat",
                           height=3, wrap="none", state="disabled", highlightthickness=0)
        self.txt.pack(fill="both", expand=True, padx=18, pady=(0, 6))

    def crear_grafico_simple(self, parent, fila, columna, titulo, unidad, color):
        panel = self.panel(parent)
        panel.grid(row=fila, column=columna, sticky="nsew", padx=6, pady=(6, 8))
        self.etiqueta(panel, titulo).pack(anchor="w", padx=12, pady=(9, 3))
        canvas = tk.Canvas(panel, height=125, bg="#111820", highlightthickness=1,
                           highlightbackground=BORDE)
        canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        canvas.bind("<Configure>", lambda _e, c=canvas, k=unidad, col=color:
                    self.dibujar_grafico_simple(c, k, col))
        return canvas

    def sub_dato(self, parent, titulo, col):
        f = tk.Frame(parent, bg=PANEL)
        f.grid(row=0, column=col, padx=(0, 12), sticky="w")
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

    def activar_cda(self):
        # CDA inicia una nueva estimacion local desde el proximo dato del GY.
        self.cda_activo = True
        self.gravedad_base = None
        self.muestras_calibracion = []
        self.muestra_mov_anterior = None
        self.velocidad_ejes = [0.0, 0.0, 0.0]
        self.distancia_ejes = [0.0, 0.0, 0.0]
        self.muestras_altura.clear()
        for eje in ("X", "Y", "Z"):
            self.lbl_dist[eje].configure(text="0.00 m")
        self.lbl_velocidad.configure(text="Velocidad: 0.00 m/s")
        self.lbl_impacto.configure(text="Esperando descenso…", fg=GRIS)
        self.lbl_impacto.pack(side="left", padx=(6, 0))
        self.btn_cda.configure(text="CDA · reiniciar medición")
        self.lbl_cmd.configure(text="Último comando: CDA (medición iniciada)")

    def integrar_recorrido(self, ahora, valores_g):
        if not self.cda_activo:
            return
        gravedad = 9.80665
        vector = [v * gravedad for v in valores_g]
        if self.gravedad_base is None:
            # Promedia varias muestras iniciales para estimar el sesgo/gravedad en reposo.
            self.muestras_calibracion.append(vector)
            if len(self.muestras_calibracion) < 5:
                self.lbl_velocidad.configure(text="Calibrando en reposo…")
                return
            self.gravedad_base = [
                sum(m[i] for m in self.muestras_calibracion) / len(self.muestras_calibracion)
                for i in range(3)
            ]
            self.muestra_mov_anterior = (ahora, [0.0, 0.0, 0.0])
            self.lbl_velocidad.configure(text="Rapidez: 0.00 m/s")
            return
        anterior_t, anterior_a = self.muestra_mov_anterior
        dt = ahora - anterior_t
        if dt <= 0 or dt > 3:
            self.muestra_mov_anterior = (ahora, [v - b for v, b in zip(vector, self.gravedad_base)])
            return
        aceleracion = [v - b for v, b in zip(vector, self.gravedad_base)]
        # Zona muerta para ruido del acelerometro: sin movimiento medible, no integrar.
        if math.sqrt(sum(a * a for a in aceleracion)) < 1.5:  # ~0.15 g: inmovil/ruido
            # Adaptacion lenta del vector de gravedad mientras esta quieto.
            self.gravedad_base = [0.98 * b + 0.02 * v
                                  for b, v in zip(self.gravedad_base, vector)]
            self.velocidad_ejes = [0.0, 0.0, 0.0]
            self.muestra_mov_anterior = (ahora, [0.0, 0.0, 0.0])
            self.lbl_velocidad.configure(text="Rapidez: 0.00 m/s")
            return
        for i in range(3):
            v0 = self.velocidad_ejes[i]
            v1 = v0 + (anterior_a[i] + aceleracion[i]) * 0.5 * dt
            self.distancia_ejes[i] += (v0 + v1) * 0.5 * dt
            self.velocidad_ejes[i] = v1
        self.muestra_mov_anterior = (ahora, aceleracion)
        for eje, distancia in zip(("X", "Y", "Z"), self.distancia_ejes):
            self.lbl_dist[eje].configure(text=f"{distancia:+.2f} m")
        rapidez = math.sqrt(sum(v * v for v in self.velocidad_ejes))
        self.lbl_velocidad.configure(text=f"Rapidez: {rapidez:.2f} m/s")

    # ------------------------------------------------------ datos
    def alternar_vista_sensor(self, nombre):
        visible = self.mostrar_componentes[nombre].get()
        for marco in self.marcos_sensor[nombre]:
            if visible:
                marco.grid()
            else:
                marco.grid_remove()

    def set_todos_visibles(self, visible):
        for nombre, variable in self.mostrar_componentes.items():
            variable.set(visible)
            self.alternar_vista_sensor(nombre)

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
            if rel:
                self.poner(self.lbl_rel, f"{float(rel):+.2f}", CIAN)
            else:
                self.poner(self.lbl_rel, "---", CIAN)
            mt, mp = RE_TEMP.search(linea), RE_PRES.search(linea)
            if mt:
                temp = float(mt.group(1))
                self.poner(self.lbl_temp, f"{temp:.1f}")
                self.registrar_muestra(self.historial_temp, temp)
                self.dibujar_grafico_simple(self.canvas_temp, "°C", "#ffb454")
            if mp:
                self.poner(self.lbl_pres, f"{float(mp.group(1)):.2f}")
            mrel = RE_REL.search(linea)
            if mrel and mrel.group(1) is not None:
                self.registrar_altura(float(mrel.group(1)))
        else:
            t = estado_texto("BMP")
            self.poner(self.lbl_abs, t)
            self.poner(self.lbl_rel, t, CIAN)
            self.poner(self.lbl_temp, t)
            self.poner(self.lbl_pres, t)

        # DHT11
        m = RE_HUM.search(linea)
        if m:
            humedad = float(m.group(1))
            self.poner(self.lbl_hum, f"{humedad:.1f}")
            self.registrar_muestra(self.historial_hum, humedad)
            self.dibujar_grafico_simple(self.canvas_hum, "%", "#4cc9f0")
        else:
            self.poner(self.lbl_hum, estado_texto("DHT"))

        # GY-521
        m = RE_ACC.search(linea)
        if m:
            valores = [float(v) for v in m.groups()]
            for lbl, v in zip(self.lbl_acc, valores):
                self.poner(lbl, f"{float(v):+.3f}")
            ahora = time.time()
            self.historial_acc.append((ahora, *valores))
            self.historial_acc = [p for p in self.historial_acc if ahora - p[0] <= 10.0]
            total = math.sqrt(sum(v * v for v in valores))
            self.lbl_acc_total.configure(text=f"{total:.3f} g | {total * 9.80665:.2f} m/s²")
            self.dibujar_grafico()
            self.integrar_recorrido(ahora, valores)
        else:
            for lbl in self.lbl_acc:
                self.poner(lbl, estado_texto("GY"))
            self.lbl_acc_total.configure(
                text="ERR" if estados.get("GY") == "ERROR" else "OFF"
            )

        # Servo, estados y señal
        m = RE_SERVO.search(linea)
        if m:
            self.lbl_servo.configure(text=f"Último valor recibido: S{m.group(1)}")
        mr, ms = RE_RSSI.search(linea), RE_SNR.search(linea)
        if mr:
            self.lbl_rssi.configure(text=f"RSSI {float(mr.group(1)):.0f} dBm")
        if ms:
            self.lbl_snr.configure(text=f"SNR {float(ms.group(1)):.1f} dB")
        self.log(f"Pkt #{n}  RSSI {mr.group(1) if mr else '--'} dBm")

    def registrar_altura(self, altura_relativa):
        if not self.cda_activo:
            return
        ahora = time.time()
        self.muestras_altura.append((ahora, altura_relativa))
        self.muestras_altura = [m for m in self.muestras_altura if ahora - m[0] <= 20]
        if len(self.muestras_altura) < 2:
            self.lbl_impacto.configure(text="Calculando…", fg=GRIS)
            return

        t0, h0 = self.muestras_altura[0]
        t1, h1 = self.muestras_altura[-1]
        dt = t1 - t0
        if dt <= 0:
            return
        velocidad_vertical = (h1 - h0) / dt
        if h1 <= 0:
            self.lbl_impacto.configure(text="Suelo alcanzado", fg=ROJO)
        elif velocidad_vertical < -0.1:
            segundos = h1 / -velocidad_vertical
            self.lbl_impacto.configure(text=f"~{segundos:.1f} s", fg=ROJO)
        else:
            self.lbl_impacto.configure(text="Sin descenso", fg=VERDE)

    def dibujar_grafico(self):
        if not hasattr(self, "canvas_acc"):
            return
        c = self.canvas_acc
        c.delete("all")
        w = max(c.winfo_width(), 200)
        h = max(c.winfo_height(), 100)
        izq, der, arriba, abajo = 48, w - 8, 10, h - 22
        c.create_line(izq, arriba, izq, abajo, fill=BORDE)
        c.create_line(izq, abajo, der, abajo, fill=BORDE)
        c.create_text(4, arriba, text="g", fill=GRIS, anchor="nw")
        c.create_text(izq, h - 3, text="−1 s", fill=GRIS, anchor="sw")
        c.create_text(der, h - 3, text="ahora", fill=GRIS, anchor="se")
        for tick in (2, 1, 0, -1, -2):
            ytick = arriba + (2 - tick) / 4 * (abajo - arriba)
            c.create_text(izq - 4, ytick, text=f"{tick:+d}" if tick else "0",
                          fill=GRIS, anchor="e")
            if tick:
                c.create_line(izq, ytick, der, ytick, fill="#35404d", dash=(2, 3))
            else:
                c.create_line(izq, ytick, der, ytick, fill="#526170")
        ahora = time.time()
        muestras = [m for m in self.historial_acc if ahora - m[0] <= 1.0]
        if not muestras:
            c.create_text((izq + der) / 2, (arriba + abajo) / 2,
                          text="Esperando acelerómetro…", fill=GRIS)
            return
        colores = {"X": "#4c9aff", "Y": "#ff5c5c", "Z": "#48d17a"}
        for idx, eje in enumerate(("X", "Y", "Z"), start=1):
            if not self.mostrar_ejes[eje].get():
                continue
            puntos = []
            for muestra in muestras:
                x = der - min(1.0, max(0.0, ahora - muestra[0])) * (der - izq)
                val = max(-2.0, min(2.0, muestra[idx]))
                y = arriba + (2.0 - val) / 4.0 * (abajo - arriba)
                puntos.extend((x, y))
            if len(puntos) >= 4:
                c.create_line(*puntos, fill=colores[eje], width=2, smooth=True)
            for j in range(0, len(puntos), 2):
                x, y = puntos[j], puntos[j + 1]
                c.create_oval(x - 2, y - 2, x + 2, y + 2,
                              fill=colores[eje], outline=colores[eje])

    def registrar_muestra(self, historial, valor):
        ahora = time.time()
        historial.append((ahora, valor))
        historial[:] = [m for m in historial if ahora - m[0] <= 10.0]

    def dibujar_grafico_simple(self, canvas, unidad, color):
        historial = self.historial_temp if unidad == "°C" else self.historial_hum
        canvas.delete("all")
        w = max(canvas.winfo_width(), 180)
        h = max(canvas.winfo_height(), 100)
        izq, der, arriba, abajo = 34, w - 8, 8, h - 22
        canvas.create_line(izq, arriba, izq, abajo, fill=BORDE)
        canvas.create_line(izq, abajo, der, abajo, fill=BORDE)
        canvas.create_text(3, arriba, text=unidad, fill=GRIS, anchor="nw")
        canvas.create_text(izq, h - 3, text="−1 s", fill=GRIS, anchor="sw")
        canvas.create_text(der, h - 3, text="ahora", fill=GRIS, anchor="se")
        minimo, maximo = (0.0, 50.0) if unidad == "°C" else (0.0, 100.0)
        marcas = (0, 10, 20, 30, 40, 50) if unidad == "°C" else (0, 25, 50, 75, 100)
        for marca in marcas:
            ymarca = abajo - (marca - minimo) / (maximo - minimo) * (abajo - arriba)
            canvas.create_text(izq - 4, ymarca, text=str(marca), fill=GRIS, anchor="e")
            canvas.create_line(izq, ymarca, der, ymarca, fill="#35404d", dash=(2, 3))
        ahora = time.time()
        muestras = [m for m in historial if ahora - m[0] <= 1.0]
        if not muestras:
            canvas.create_text((izq + der) / 2, (arriba + abajo) / 2,
                               text="Esperando datos…", fill=GRIS)
            return
        puntos = []
        for t, valor in muestras:
            x = der - min(1.0, max(0.0, ahora - t)) * (der - izq)
            valor = max(minimo, min(maximo, valor))
            y = abajo - (valor - minimo) / (maximo - minimo) * (abajo - arriba)
            puntos.extend((x, y))
        if len(puntos) >= 4:
            canvas.create_line(*puntos, fill=color, width=2, smooth=True)
        for j in range(0, len(puntos), 2):
            x, y = puntos[j], puntos[j + 1]
            canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color)

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

        if hasattr(self, "canvas_acc"):
            self.dibujar_grafico()
            self.dibujar_grafico_simple(self.canvas_temp, "°C", "#ffb454")
            self.dibujar_grafico_simple(self.canvas_hum, "%", "#4cc9f0")

        self.root.after(100, self.tick)

    def cerrar(self):
        self.desconectar()
        self.root.destroy()


if __name__ == "__main__":
    ventana = tk.Tk()
    app = App(ventana)
    ventana.protocol("WM_DELETE_WINDOW", app.cerrar)
    ventana.mainloop()
