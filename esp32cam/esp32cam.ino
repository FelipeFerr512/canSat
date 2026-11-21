// ESP32-CAM AI-Thinker: JPEG frames with length prefixes, saved as an MJPEG stream.
// Trigger HIGH (500 ms) on GPIO13 starts recording; a new trigger cannot interrupt recording.
// SD_MMC uses 1-bit mode so GPIO13 is free. Recording stops on low trigger command is not used;
// this sketch records a fixed 20-second flight clip after trigger, then closes the AVI file.
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

#define TRIGGER_PIN 13
#define RECORD_SECONDS 20

// AI Thinker ESP32-CAM OV2640 pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

bool recording = false;
uint32_t startMs = 0;
uint32_t frameNo = 0;
File video;

void stopRecording() {
  if (!recording) return;
  video.close();
  recording = false;
  Serial.printf("Grabacion finalizada: /vuelo_%lu.avi, %lu cuadros\n", (unsigned long)startMs, (unsigned long)frameNo);
}

void startRecording() {
  if (recording) return;
  startMs = millis();
  frameNo = 0;
  char path[40];
  snprintf(path, sizeof(path), "/vuelo_%lu.mjpeg", (unsigned long)startMs);
  video = SD_MMC.open(path, FILE_WRITE);
  if (!video) {
    Serial.println("ERROR: no se pudo crear archivo en microSD");
    return;
  }
  recording = true;
  Serial.printf("Grabacion iniciada: %s\n", path);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, INPUT_PULLDOWN);
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sscb_sda = SIOD_GPIO_NUM;
  c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  // Sensor auto-detection comes from esp_camera_init; VGA is a safe starting
  // point for continuous capture on ESP32-CAM, including 3MP OV3660 modules.
  // Increase to FRAMESIZE_XGA/UXGA only after confirming stable SD writes.
  c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.grab_mode = CAMERA_GRAB_LATEST;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("ERROR: inicializacion de camara/sensor");
    while (true) delay(1000);
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) Serial.printf("Sensor detectado PID: 0x%04X\n", sensor->id.PID);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("ERROR: microSD (1-bit)");
    while (true) delay(1000);
  }
  pinMode(TRIGGER_PIN, INPUT_PULLDOWN);
  Serial.println("ESP32-CAM lista; esperando D4 del nodo en GPIO13");
}

void loop() {
  static bool previous = false;
  bool trigger = digitalRead(TRIGGER_PIN);
  if (trigger && !previous) startRecording();
  previous = trigger;
  if (!recording) {
    delay(5);
    return;
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    uint32_t n = fb->len;
    video.write((uint8_t *)&n, sizeof(n));
    video.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
    frameNo++;
  }
  if (millis() - startMs >= RECORD_SECONDS * 1000UL) stopRecording();
}
