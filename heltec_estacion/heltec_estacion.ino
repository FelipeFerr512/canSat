// ============================================================
//  HELTEC WiFi LoRa 32 V3 (estacion terrena)
//  Placa: Heltec WiFi LoRa 32(V3)  o  ESP32S3 Dev Module
//         (con ESP32S3 Dev Module: USB CDC On Boot = Enabled)
//  Monitor serie: 115200
//
//  Comandos por serie (monitor o interfaz Python):
//    Sx (x = 0..180 grados) | DHT | BMP | GY
//    TALLON | TALLOFF | BZ -> se envian por LoRa
//    FLOOR -> define el piso (altura relativa = 0), se resuelve en la Heltec
//  La telemetria abre una ventana de subida despues de cada quinto paquete.
//  Los comandos se envian alli, con secuencia y reintentos; la confirmacion
//  vuelve en el siguiente paquete normal del nodo.
// ============================================================
#include <RadioLib.h>
#include <U8g2lib.h>

#define LORA_NSS  8
#define LORA_DIO1 14
#define LORA_RST  12
#define LORA_BUSY 13

#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define VEXT_PIN 36

#define TIMEOUT_COMUNICACION 1500UL
#define TIEMPO_MOSTRAR_CMD   1000UL
#define REINTENTO_MS         850UL
#define MAX_INTENTOS         8
#define COPIAS_POR_VENTANA   3
#define TAM_COLA             8

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);

volatile bool paqueteRecibido = false;
IRAM_ATTR void setFlag() { paqueteRecibido = true; }

float ultimaAltura = 0;
bool  hayAltura    = false;
float pisoAltura   = 0;
bool  pisoDefinido = false;

bool hayDato = false;
unsigned long ultimoRx = 0;

String comandoMostrado = "";
unsigned long inicioComando = 0;

bool ultimoEstadoComm = false;
bool ultimoEstadoCmd  = false;
bool displayInicializado = false;

struct Pendiente {
  bool activo = false;
  uint8_t seq = 0;
  String cmd = "";
  uint8_t intentos = 0;
  unsigned long ultimo = 0;
};
Pendiente pend;
uint8_t seqCmd = 1;

String colaCmd[TAM_COLA];
uint8_t colaIni = 0;
uint8_t colaCnt = 0;

// ---------- Display ----------
bool hayComunicacion() {
  return hayDato && (millis() - ultimoRx < TIMEOUT_COMUNICACION);
}

bool comandoVisible() {
  return comandoMostrado.length() > 0 && (millis() - inicioComando < TIEMPO_MOSTRAR_CMD);
}

void dibujarPantalla(bool comm, bool cmdVisible) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x13_tr);
  String linea1 = String("Comunicacion: ") + (comm ? "True" : "False");
  display.drawStr(0, 24, linea1.c_str());
  if (cmdVisible) {
    String linea2 = String("Comando: ") + comandoMostrado;
    display.drawStr(0, 52, linea2.c_str());
  }
  display.sendBuffer();
}

void actualizarDisplay() {
  bool comm = hayComunicacion();
  bool cmd  = comandoVisible();
  if (!displayInicializado || comm != ultimoEstadoComm || cmd != ultimoEstadoCmd) {
    dibujarPantalla(comm, cmd);
    ultimoEstadoComm = comm;
    ultimoEstadoCmd  = cmd;
    displayInicializado = true;
  }
}

// ---------- Envio de comandos (cola + confirmacion + reintentos) ----------
void transmitirPendiente() {
  String payload = "#" + String(pend.seq) + ":" + pend.cmd;
  uint8_t copiasOk = 0;
  for (uint8_t i = 0; i < COPIAS_POR_VENTANA; i++) {
    int estado = radio.transmit(payload);
    if (estado == RADIOLIB_ERR_NONE) {
      copiasOk++;
    } else {
      Serial.printf(">> Error al enviar copia %u de '%s': %d\n",
                    i + 1, pend.cmd.c_str(), estado);
    }
    paqueteRecibido = false;
    radio.startReceive();
    if (i + 1 < COPIAS_POR_VENTANA) delay(40);
  }
  pend.ultimo = millis();
  pend.intentos++;
  Serial.printf(">> Comando '%s' enviado: %u/%u copias (ventana %u)\n",
                pend.cmd.c_str(), copiasOk, COPIAS_POR_VENTANA, pend.intentos);
}

void iniciarEnvio(const String &cmd) {
  seqCmd = seqCmd % 250 + 1;
  pend.seq = seqCmd;
  pend.cmd = cmd;
  pend.intentos = 0;
  pend.activo = true;
  comandoMostrado = cmd;
  inicioComando = millis();
}

void encolar(const String &cmd) {
  if (colaCnt >= TAM_COLA) {
    Serial.println(">> Cola llena, comando descartado");
    return;
  }
  colaCmd[(colaIni + colaCnt) % TAM_COLA] = cmd;
  colaCnt++;
}

void siguienteComando() {
  if (pend.activo || colaCnt == 0) return;
  String c = colaCmd[colaIni];
  colaIni = (colaIni + 1) % TAM_COLA;
  colaCnt--;
  iniciarEnvio(c);
}

void revisarReintentos(bool ventanaSubida) {
  if (!ventanaSubida) return;
  if (!pend.activo || millis() - pend.ultimo < REINTENTO_MS) return;
  if (pend.intentos >= MAX_INTENTOS) {
    Serial.printf(">> Comando '%s' SIN CONFIRMAR\n", pend.cmd.c_str());
    pend.activo = false;
    return;
  }
  transmitirPendiente();
}

void atenderVentanaSubida() {
  if (pend.activo) {
    revisarReintentos(true);
  } else if (colaCnt > 0) {
    siguienteComando();
    transmitirPendiente();
  }
}

// ---------- Comandos por serie ----------
bool comandoValido(const String &c) {
  if (c == "DHT" || c == "BMP" || c == "GY" || c == "TALLON" || c == "TALLOFF" ||
      c == "BZ" || c == "ARM" || c == "DESARME" || c == "REARME") return true;
  if (c.length() >= 2 && c.length() <= 4 && c[0] == 'S') {
    for (unsigned int i = 1; i < c.length(); i++) {
      if (!isDigit(c[i])) return false;
    }
    return c.substring(1).toInt() <= 180;
  }
  return false;
}

void leerComandoSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  cmd.replace(" ", "");
  if (cmd.length() == 0) return;

  if (cmd == "FLOOR") {
    if (!hayAltura) {
      Serial.println(">> Todavia no se recibio ninguna altura, no se puede definir el piso");
      return;
    }
    pisoAltura   = ultimaAltura;
    pisoDefinido = true;
    comandoMostrado = "FLOOR";
    inicioComando = millis();
    Serial.printf(">> Piso definido en %.2f m (altura relativa = 0)\n", pisoAltura);
  } else if (comandoValido(cmd)) {
    encolar(cmd);
  } else {
    Serial.printf(">> Comando desconocido: %s\n", cmd.c_str());
  }
}

// ---------- Recepcion de telemetria ----------
// Busca ";clave=valor" dentro del paquete
String campo(const String &datos, const char *clave) {
  String k = String(";") + clave + "=";
  int i = datos.indexOf(k);
  if (i < 0) return "";
  i += k.length();
  int f = datos.indexOf(';', i);
  return (f < 0) ? datos.substring(i) : datos.substring(i, f);
}

// Devuelve el valor numero n de una lista separada por comas
float parte(const String &s, int n) {
  int ini = 0;
  for (int i = 0; i < n; i++) {
    ini = s.indexOf(',', ini);
    if (ini < 0) return NAN;
    ini++;
  }
  int fin = s.indexOf(',', ini);
  String t = (fin < 0) ? s.substring(ini) : s.substring(ini, fin);
  return t.toFloat();
}

void procesarPaquete() {
  String datos;
  int estado = radio.readData(datos);

  if (estado == RADIOLIB_ERR_NONE) {
    float rssi = radio.getRSSI();
    float snr  = radio.getSNR();

    datos = ";" + datos;
    String id = campo(datos, "ID");
    String e  = campo(datos, "E");
    String cam = campo(datos, "C");

    if (id.length() == 0 || e.length() < 3) {
      radio.startReceive();
      return;
    }

    // Cada quinto paquete abre una ventana de subida, justo despues de RX.
    // El nodo sigue enviando telemetria a 5 Hz y confirma comandos por A=seq.
    if ((id.toInt() % 5) == 4) atenderVentanaSubida();

    hayDato  = true;
    ultimoRx = millis();

    bool dOn = e[0] == '1';
    bool bOn = e[1] == '1';
    bool gOn = e[2] == '1';

    Serial.printf("Pkt #%s", id.c_str());
    if (cam == "1") Serial.print(" | CAMARA: INICIADA");

    String b = campo(datos, "B");
    if (!bOn) {
      Serial.print(" | BMP: OFF");
    } else if (b.length() == 0 || b == "E") {
      Serial.print(" | BMP: ERROR");
    } else {
      float alt  = parte(b, 0);
      float temp = parte(b, 1);
      float pres = parte(b, 2);
      ultimaAltura = alt;
      hayAltura = true;
      Serial.printf(" | Altura: %.2f m", alt);
      if (pisoDefinido) {
        Serial.printf(" | Altura relativa: %.2f m", alt - pisoAltura);
      } else {
        Serial.print(" | Altura relativa: (sin piso)");
      }
      Serial.printf(" | Temp: %.1f C | Presion: %.2f hPa", temp, pres);
    }

    String d = campo(datos, "D");
    if (!dOn) {
      Serial.print(" | DHT: OFF");
    } else if (d.length() == 0 || d == "E") {
      Serial.print(" | DHT: ERROR");
    } else {
      Serial.printf(" | Humedad: %.1f %%", d.toFloat());
    }

    String g = campo(datos, "G");
    if (!gOn) {
      Serial.print(" | GY: OFF");
    } else if (g.length() == 0 || g == "E") {
      Serial.print(" | GY: ERROR");
    } else {
      Serial.printf(" | Acc: %.3f %.3f %.3f g", parte(g, 0), parte(g, 1), parte(g, 2));
    }

    Serial.printf(" | Servo: %s", campo(datos, "S").c_str());
    Serial.printf(" | Estado: DHT=%s BMP=%s GY=%s", dOn ? "ON" : "OFF", bOn ? "ON" : "OFF", gOn ? "ON" : "OFF");
    Serial.printf(" | RSSI: %.0f dBm | SNR: %.1f dB\n", rssi, snr);

    String a = campo(datos, "A");
    if (pend.activo && a.length() > 0 && (uint8_t)a.toInt() == pend.seq) {
      Serial.printf(">> Comando '%s' confirmado\n", pend.cmd.c_str());
      pend.activo = false;
    }
  } else if (estado == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("Paquete con error de CRC");
  }

  radio.startReceive();
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);


  seqCmd = (esp_random() % 200) + 1;

  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  delay(100);
  display.begin();

  ConfigLoRa_t config;
  config.frequency = 915000000UL;
  config.bandwidth = 125000UL;
  config.spreadingFactor = 7;
  config.codingRate = 5;
  config.syncWord = 0x12;
  config.power = 10;
  config.preambleLength = 8;

  int estado = radio.begin(config);
  if (estado != RADIOLIB_ERR_NONE) {
    Serial.printf("Error LoRa: %d\n", estado);
    display.clearBuffer();
    display.setFont(u8g2_font_6x13_tr);
    display.drawStr(0, 24, "Error LoRa");
    display.sendBuffer();
    while (true);
  }
  radio.setDio2AsRfSwitch(true);
  radio.setCRC(true);
  radio.setDio1Action(setFlag);
  radio.startReceive();

  Serial.println("Receptor listo. Comandos: Sx (0-180) | DHT | BMP | GY | TALLON | TALLOFF | BZ | ARM | DESARME | REARME | FLOOR");
  actualizarDisplay();
}

void loop() {
  leerComandoSerial();

  if (paqueteRecibido) {
    paqueteRecibido = false;
    procesarPaquete();
  }

  actualizarDisplay();
}
