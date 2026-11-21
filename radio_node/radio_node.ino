// ============================================================6
//  RADIO NODE (ATmega328P + RFM95)   -> placa: Arduino Pro or Pro Mini
//  Procesador: ATmega328P (3.3V, 8 MHz)   |   Monitor serie: 9600
//
//  Sensores: BMP180 (I2C), GY-521 / MPU6050 (I2C), DHT11 (A0)
//  Servo SG90 posicional: señal en A1; switch a D5 y GND; buzzer a D3.
//
//  Paquete que envia (texto):
//    ID=42;E=101;B=alt,temp,pres;D=hum;G=ax,ay,az;S=90;A=7
//    E = estado de DHT, BMP, GY (1 = activo)
//    B/D/G solo aparecen si el sensor esta activo ("E" = error de lectura)
//    S = angulo actual del servo
//    A = ultimo comando recibido (confirmacion para la Heltec)
//
//  Comandos que recibe:  #<seq>:<cmd>
//    SX (X = 0..180 grados)
//    DHT | BMP | GY | TALLON | TALLOFF | BZ
// ============================================================
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <Servo.h>

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 2
#define LORA_FREQ 915E6

#define SEA_LEVEL_PA 101325

#define DHT_PIN   A0
#define DHT_TYPE  DHT11

#define SERVO_PIN A1
#define BUZZER_PIN 3
#define SWITCH_PIN 5
#define CAM_TRIGGER_PIN 4
#define MPU_ADDR  0x68

#define INTERVALO_TX 200UL

Adafruit_BMP085 bmp;
DHT dht(DHT_PIN, DHT_TYPE);
Servo servo;
bool servoAdjunto = false;

bool bmpOk = false;
bool mpuOk = false;
float humedadCacheada = NAN;
unsigned long ultimaLecturaDHT = 0;
float temperaturaCacheada = NAN;
float presionCacheada = NAN;
float alturaCacheada = NAN;
unsigned long ultimaLecturaBMP = 0;

bool dhtOn = true;
bool bmpOn = true;
bool gyOn  = true;
bool buzzerOn = false;
bool servoAutomaticoArmado = false;
bool desarmeBloqueado = false;

int servoAngulo = 90;
bool switchTemporizando = false;
bool switchYaAcciono = false;
bool eventoCamara = false;
bool camaraTriggerActivo = false;
unsigned long camaraTriggerHasta = 0;
bool switchCerradoVisto = false;
bool switchLecturaRaw = HIGH;
unsigned long switchAbiertoDesde = 0;
unsigned long switchCambioDesde = 0;
uint8_t ultimaSeq = 0;
uint16_t contador = 0;
unsigned long ultimoTx = 0;
unsigned long finVentanaSubida = 0;

// ---------- GY-521 (MPU6050) por I2C, sin libreria ----------
bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  return Wire.endTransmission() == 0;
}

int16_t leer16() {
  uint8_t h = Wire.read();
  uint8_t l = Wire.read();
  return (int16_t)((h << 8) | l);
}

bool mpuLeer(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) != 6) return false;
  int16_t x = leer16();
  int16_t y = leer16();
  int16_t z = leer16();
  ax = x / 16384.0;
  ay = y / 16384.0;
  az = z / 16384.0;
  return true;
}

// ---------- Envio de telemetria ----------
void enviarDatos() {
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "ID=%u;E=%d%d%d", contador++, dhtOn, bmpOn, gyOn);

  if (bmpOn) {
    if (bmpOk) {
      unsigned long ahora = millis();
      if (isnan(temperaturaCacheada) || ahora - ultimaLecturaBMP >= 1000UL) {
        temperaturaCacheada = bmp.readTemperature();
        presionCacheada = bmp.readPressure() / 100.0;
        alturaCacheada = bmp.readAltitude(SEA_LEVEL_PA);
        ultimaLecturaBMP = ahora;
      }
      char a[12], t[10], p[12];
      dtostrf(alturaCacheada, 1, 2, a);
      dtostrf(temperaturaCacheada, 1, 1, t);
      dtostrf(presionCacheada, 1, 2, p);
      n += snprintf(buf + n, sizeof(buf) - n, ";B=%s,%s,%s", a, t, p);
    } else {
      n += snprintf(buf + n, sizeof(buf) - n, ";B=E");
    }
  }

  if (dhtOn) {
    unsigned long ahora = millis();
    if (isnan(humedadCacheada) || ahora - ultimaLecturaDHT >= 2000UL) {
      float lectura = dht.readHumidity();
      ultimaLecturaDHT = ahora;
      if (!isnan(lectura)) humedadCacheada = lectura;
    }
    if (isnan(humedadCacheada)) {
      n += snprintf(buf + n, sizeof(buf) - n, ";D=E");
    } else {
      char hs[10];
      dtostrf(humedadCacheada, 1, 1, hs);
      n += snprintf(buf + n, sizeof(buf) - n, ";D=%s", hs);
    }
  }

  if (gyOn) {
    float ax, ay, az;
    if (mpuOk && mpuLeer(ax, ay, az)) {
      char x[10], y[10], z[10];
      dtostrf(ax, 1, 3, x);
      dtostrf(ay, 1, 3, y);
      dtostrf(az, 1, 3, z);
      n += snprintf(buf + n, sizeof(buf) - n, ";G=%s,%s,%s", x, y, z);
    } else {
      n += snprintf(buf + n, sizeof(buf) - n, ";G=E");
    }
  }

  snprintf(buf + n, sizeof(buf) - n, ";S=%d;A=%u;C=%u", servoAngulo, ultimaSeq, eventoCamara ? 1 : 0);

  LoRa.beginPacket();
  LoRa.print(buf);
  LoRa.endPacket();
  LoRa.receive();

  Serial.print(F("Enviado: "));
  Serial.println(buf);
}

// ---------- Comandos ----------
void moverServo(int angulo) {
  if (!servoAdjunto) {
    servo.attach(SERVO_PIN);
    servoAdjunto = true;
  }
  servo.write(angulo);
}

void ejecutarComando(const char *cmd) {
  Serial.print(F("Comando: "));
  Serial.println(cmd);

  if (strcmp(cmd, "DHT") == 0) {
    dhtOn = !dhtOn;
  } else if (strcmp(cmd, "BMP") == 0) {
    bmpOn = !bmpOn;
  } else if (strcmp(cmd, "GY") == 0) {
    gyOn = !gyOn;
  } else if (strcmp(cmd, "TALLON") == 0) {
    dhtOn = bmpOn = gyOn = true;
  } else if (strcmp(cmd, "TALLOFF") == 0) {
    dhtOn = bmpOn = gyOn = false;
  } else if (strcmp(cmd, "BZ") == 0) {
    buzzerOn = !buzzerOn;
    digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
  } else if (strcmp(cmd, "ARM") == 0) {
    if (desarmeBloqueado) {
      Serial.println(F("ARM rechazado: DESARME enclavado; use REARME"));
    } else {
      servoAutomaticoArmado = true;
      switchTemporizando = false;
      switchYaAcciono = false;
      switchCerradoVisto = false;
      switchLecturaRaw = digitalRead(SWITCH_PIN);
      switchCambioDesde = millis();
      Serial.println(F("Servo automatico armado"));
    }
  } else if (strcmp(cmd, "DESARME") == 0) {
    servoAutomaticoArmado = false;
    desarmeBloqueado = true;
    switchTemporizando = false;
    switchYaAcciono = false;
    switchCerradoVisto = false;
    Serial.println(F("Servo automatico desarmado; comandos manuales habilitados"));
  } else if (strcmp(cmd, "REARME") == 0) {
    desarmeBloqueado = false;
    servoAutomaticoArmado = false;
    switchTemporizando = false;
    switchYaAcciono = false;
    switchCerradoVisto = false;
    Serial.println(F("Bloqueo liberado; pulse CDA para armar el servo"));
  } else if (cmd[0] == 'S') {
    int a = atoi(cmd + 1);
    if (a < 0) a = 0;
    if (a > 180) a = 180;
    servoAngulo = a;
    moverServo(a);
  }
}

void revisarComandos() {
  int tam = LoRa.parsePacket();
  if (tam == 0) return;

  char msg[24];
  int i = 0;
  while (LoRa.available()) {
    char c = (char)LoRa.read();
    if (i < (int)sizeof(msg) - 1) msg[i++] = c;
  }
  msg[i] = 0;

  if (msg[0] != '#') return;
  char *sep = strchr(msg, ':');
  if (sep == NULL) return;
  *sep = 0;

  uint8_t seq = (uint8_t)atoi(msg + 1);
  const char *cmd = sep + 1;

  if (seq != ultimaSeq) {
    ultimaSeq = seq;
    ejecutarComando(cmd);
  } else {
    Serial.println(F("Comando repetido, se ignora"));
  }
  // La confirmacion viaja en el siguiente paquete normal; no transmitir
  // inmediatamente evita que la respuesta choque con la telemetria.
}

void revisarSwitchServo() {
  bool lectura = digitalRead(SWITCH_PIN);
  if (lectura != switchLecturaRaw) {
    switchLecturaRaw = lectura;
    switchCambioDesde = millis();
    switchTemporizando = false;
    return;
  }
  if (millis() - switchCambioDesde < 40UL) return;
  bool switchAbierto = switchLecturaRaw == HIGH;
  if (!switchAbierto) {
    switchCerradoVisto = true;
    switchTemporizando = false;
    return;
  }
  // Exige ver primero el switch cerrado tras CDA. Un D5 desconectado queda
  // en HIGH por INPUT_PULLUP y no puede disparar el servo por error.
  if (!switchCerradoVisto) return;
  if (switchYaAcciono) return;
  if (!switchTemporizando) {
    switchTemporizando = true;
    switchAbiertoDesde = millis();
    return;
  }
  if (millis() - switchAbiertoDesde >= 2000UL) {
    switchYaAcciono = true;
    eventoCamara = true;
    digitalWrite(CAM_TRIGGER_PIN, HIGH);
    camaraTriggerActivo = true;
    camaraTriggerHasta = millis() + 500UL;
    if (servoAutomaticoArmado && !desarmeBloqueado) {
      servoAngulo = 180;
      moverServo(servoAngulo);
      Serial.println(F("Switch abierto 2 s: servo a 180 grados"));
    }
    Serial.println(F("Evento switch: grabacion y CDA iniciados"));
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(9600);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(CAM_TRIGGER_PIN, OUTPUT);
  digitalWrite(CAM_TRIGGER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();
  Wire.setWireTimeout(25000UL, true);
  dht.begin();

  bmpOk = bmp.begin();
  if (!bmpOk) Serial.println(F("Aviso: no se encuentra el BMP180"));

  mpuOk = mpuInit();
  if (!mpuOk) Serial.println(F("Aviso: no se encuentra el GY-521"));

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("Error: falla LoRa"));
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  LoRa.setTxPower(17);
  LoRa.receive();

  Serial.println(F("Nodo listo"));
}

void loop() {
  if (camaraTriggerActivo && (long)(millis() - camaraTriggerHasta) >= 0) {
    digitalWrite(CAM_TRIGGER_PIN, LOW);
    camaraTriggerActivo = false;
  }
  revisarComandos();
  revisarSwitchServo();

  if (millis() - ultimoTx >= INTERVALO_TX &&
      (long)(millis() - finVentanaSubida) >= 0) {
    ultimoTx = millis();
    enviarDatos();
    // Cada quinto paquete deja 400 ms para recibir comandos de la Heltec.
    if (contador % 5 == 0) finVentanaSubida = millis() + 400UL;
  }
}

