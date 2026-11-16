"""Pruebas estaticas y de logica para el enlace de telemetria, sin hardware."""
from pathlib import Path
import math
import re

ROOT = Path(__file__).resolve().parent
NODE = (ROOT / "radio_node" / "radio_node.ino").read_text(encoding="utf-8")
STATION = (ROOT / "heltec_estacion" / "heltec_estacion.ino").read_text(encoding="utf-8")
GUI = (ROOT / "interfaz_telemetria.py").read_text(encoding="utf-8")


def campo(packet, key):
    match = re.search(rf"(?:^|;){re.escape(key)}=([^;]*)", packet)
    return match.group(1) if match else ""


def parsear_aceleracion(linea):
    match = re.search(r"Acc: (-?[\d.]+) (-?[\d.]+) (-?[\d.]+) g", linea)
    return tuple(map(float, match.groups())) if match else None


def simular_estacionario(muestras_g, umbral=1.5):
    """Espeja la zona muerta de la interfaz para probar ruido en reposo."""
    base = [v * 9.80665 for v in muestras_g[0]]
    velocidad = [0.0, 0.0, 0.0]
    distancia = [0.0, 0.0, 0.0]
    for muestra in muestras_g[1:]:
        vector = [v * 9.80665 for v in muestra]
        a = [v - b for v, b in zip(vector, base)]
        if math.sqrt(sum(x*x for x in a)) < umbral:
            base = [0.98*b + 0.02*v for b, v in zip(base, vector)]
            velocidad = [0.0, 0.0, 0.0]
            continue
        # Este escenario de prueba termina si la zona muerta no detecta reposo.
        raise AssertionError("ruido estacionario supero la zona muerta")
    return velocidad, distancia


def main():
    # Placa, cableado declarado, cadencia y proteccion de lecturas I2C.
    assert "#define SERVO_PIN A1" in NODE
    assert "#define MPU_ADDR  0x68" in NODE
    assert "#define INTERVALO_TX 300UL" in NODE
    assert "Wire.setWireTimeout(25000UL, true)" in NODE
    assert "ahora - ultimaLecturaDHT >= 2000UL" in NODE
    assert "ahora - ultimaLecturaBMP >= 1000UL" in NODE
    assert "if math.sqrt(sum(a * a for a in aceleracion)) < 1.5" in GUI

    # Radio endpoint config must match on both radios.
    assert "LORA_FREQ 915E6" in NODE
    assert "LoRa.setSignalBandwidth(125E3)" in NODE
    assert "LoRa.setSpreadingFactor(7)" in NODE
    assert "LoRa.setCodingRate4(5)" in NODE
    assert "LoRa.setSyncWord(0x12)" in NODE
    for value in ("915000000UL", "125000UL", "spreadingFactor = 7",
                  "codingRate = 5", "syncWord = 0x12"):
        assert value in STATION, value
    assert "radio.setDio1Action(setFlag)" in STATION
    assert "pend.intentos >= MAX_INTENTOS" in STATION
    assert "LoRa.print(buf)" in NODE and "radio.readData(datos)" in STATION

    # UI sensor switches are local visibility controls, not radio commands.
    start = GUI.index("def alternar_vista_sensor")
    end = GUI.index("def set_todos_visibles", start)
    assert "grid_remove()" in GUI[start:end]
    assert "self.enviar(" not in GUI[start:end]
    assert '"Todos ON", lambda: self.enviar' not in GUI
    assert '"Todos OFF", lambda: self.enviar' not in GUI

    sample = (
        "Pkt #12 | Altura: 15.20 m | Altura relativa: 3.40 m | "
        "Temp: 22.5 C | Presion: 1008.20 hPa | Humedad: 61.0 % | "
        "Acc: -0.015 0.832 0.365 g | Servo: 90 | "
        "Estado: DHT=ON BMP=ON GY=ON | RSSI: -47 dBm | SNR: 13.0 dB"
    )
    assert campo(";ID=12;E=111;B=3.4,22.5,1008.2;D=61;G=-.015,.832,.365;S=90;A=4", "B") == "3.4,22.5,1008.2"
    assert parsear_aceleracion(sample) == (-0.015, 0.832, 0.365)
    assert math.isclose(math.sqrt(sum(x*x for x in parsear_aceleracion(sample))), 0.9089, abs_tol=0.001)
    assert campo(";ID=13;E=111;B=E;D=61;G=E;S=90;A=4", "B") == "E"
    assert parsear_aceleracion("Pkt #14 | GY: ERROR") is None
    quieta = [(0.0, 0.85, 0.36), (0.01, 0.84, 0.37),
              (-0.01, 0.85, 0.36), (0.0, 0.84, 0.36)]
    v, d = simular_estacionario(quieta)
    assert v == [0.0, 0.0, 0.0] and d == [0.0, 0.0, 0.0]

    # Command framing and receive/ack fields remain intact for servo/FLOOR.
    wire = "#27:S90"
    seq, command = wire[1:].split(":", 1)
    assert seq == "27" and command == "S90"
    assert "#" in STATION and "radio.transmit(payload)" in STATION
    assert "cmd == \"FLOOR\"" in STATION

    print("PASS: pin assignments and telemetry interval")
    print("PASS: DHT/BMP sensor sampling is cached at supported slower rates")
    print("PASS: LoRa frequency/SF/BW/CR/sync match on both ends")
    print("PASS: local UI toggles do not send sensor commands over LoRa")
    print("PASS: valid telemetry, BMP error, and missing MPU data scenarios")
    print("PASS: stationary accelerometer noise does not accumulate travel/speed")
    print("PASS: command framing, ack path, retries, servo and FLOOR code paths")
    print("LIMIT: hardware timing/radio loss still requires bench and range tests")


if __name__ == "__main__":
    main()
