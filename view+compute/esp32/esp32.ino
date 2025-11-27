#include <WiFi.h>
#include <WebServer.h>   // Built-in lightweight HTTP server
#include <math.h>

#include "esp_camera.h"
#include "filters.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// ----------------- CONFIG -----------------
#define ROI_X       100   // top-left x of forehead ROI
#define ROI_Y       50    // top-left y of forehead ROI
#define ROI_WIDTH   65    // ROI width in pixels
#define ROI_HEIGHT  65    // ROI height in pixels
#define BUFFER_LEN  100   // Number of frames to keep in buffer (~8s at 10 FPS)
#define SERIAL_BAUD 115200
#define WAIT_TIME   100

// Wi-Fi credentials
const char* ssid = "iPhone";
const char* password = "GBAJ-dQrD-DaUh-wBT0";

// ----------------- GLOBALS -----------------
WebServer server(80);

// filters configs
float lpf_prev_val_g = 0.0;
float lpf_prev_val_r = 0.0;
float lpf_cut_freq_hz = 0.05;   //TODO: change

BPFState bpf_prev_val_g = {0.0, 0.0, 0.0, 0.0};
BPFState bpf_prev_val_r = {0.0, 0.0, 0.0, 0.0};
float bpf_center_freq_hz = 2.0; //TODO: change
float bpf_bandwidth_hz = 3.0;   //TODO: change

// Rolling buffers for R/G/B
int bufferIndex = 0;
int bufferFull = 0;

float lpf_buffer_g[BUFFER_LEN];
float lpf_buffer_r[BUFFER_LEN];

float bpf_buffer_g[BUFFER_LEN];
float bpf_buffer_r[BUFFER_LEN];


// ROI queue (only stores pointers to frames)
TaskHandle_t streamTaskHandle;
TaskHandle_t roiTaskHandle;

/*****************************
 * Forward fn declarations
 *****************************/

void handleStream();
void streamLoop(void *pvParameters);
void roiLoop(void *pvParameters);

/*****************************
 * ROI DATA EXTRACTION
 *****************************/

void extract_and_send_data(camera_fb_t *fb, float fps) {
  
  // --- Compute mean RGB in ROI every frame ---

  // Compute mean RGB in ROI
  long sumR=0, sumG=0, sumB=0;
  float lpf_r=0, lpf_g=0, lpf_b=0;
  float bpf_r=0, bpf_g=0, bpf_b=0;

  int pixelCount = 0;
  
  for (int y = ROI_Y; y < ROI_Y + ROI_HEIGHT; y++) {
    for (int x = ROI_X; x < ROI_X + ROI_WIDTH; x++) {
      
      int i = (y * fb->width + x) * 2; // RGB565: 2 bytes per pixel
      
      uint16_t pixel = fb->buf[i] | (fb->buf[i+1] << 8);
      uint8_t r = ((pixel >> 11) & 0x1F) << 3;
      uint8_t g = ((pixel >> 5) & 0x3F) << 2;
      // uint8_t b = (pixel & 0x1F) << 3;
      
      sumR += r;
      sumG += g;
      // sumB += b;
      
      pixelCount++;
    }
  }

  float meanR = sumR / (float)pixelCount;
  float meanG = sumG / (float)pixelCount;
  // float meanB = sumB / (float)pixelCount;

  // LPF
  lpf_g = LPF(meanG, fps, lpf_cut_freq_hz, &lpf_prev_val_g);
  lpf_r = LPF(meanR, fps, lpf_cut_freq_hz, &lpf_prev_val_r);

  // BPF
  bpf_g = BPF(meanG, fps, bpf_center_freq_hz, bpf_bandwidth_hz, &bpf_prev_val_g);
  bpf_r = BPF(meanR, fps, bpf_center_freq_hz, bpf_bandwidth_hz, &bpf_prev_val_r);

  // Update moving buffer
  lpf_buffer_g[bufferIndex] = lpf_g;
  lpf_buffer_r[bufferIndex] = lpf_r;

  bpf_buffer_g[bufferIndex] = bpf_g;
  bpf_buffer_r[bufferIndex] = bpf_r;

  if (!(bufferFull > 0)) {
    if ((bufferIndex + 1) >= BUFFER_LEN) {
      bufferFull = 1;
    }
  }
  bufferIndex = (bufferIndex + 1) % BUFFER_LEN;

  float redDC = 0, greenDC = 0, blueDC = 0;
  float redAC = 0, greenAC = 0, blueAC = 0;

  // Compute DC (average) using output from LPF
  for (int i = 0; i < BUFFER_LEN; i++) {
    greenDC += lpf_buffer_g[i];
    redDC += lpf_buffer_r[i];
  }
  redDC /= BUFFER_LEN;
  greenDC /= BUFFER_LEN;

  // Compute AC (std deviation) using output from bandpass
  for (int i = 0; i < BUFFER_LEN; i++) {
    greenAC += pow(bpf_buffer_g[i] - greenDC, 2);
    redAC += pow(bpf_buffer_r[i] - redDC, 2);
  }
  redAC = sqrt(redAC / BUFFER_LEN);
  greenAC = sqrt(greenAC / BUFFER_LEN);

  // send over Serial if buffer is full
  if (bufferFull == 0) {

    Serial.print("Waiting for buffer to fill... (");
    Serial.print(bufferIndex - 1);
    Serial.print("/");
    Serial.print(BUFFER_LEN);
    Serial.println(")");
  } else {

    Serial.print("{\"r_ac\":");
    Serial.print(redAC,2);
    Serial.print(", \"r_dc\":");
    Serial.print(redDC,2);
    Serial.print(", \"g_ac\":");
    Serial.print(greenAC,2);
    Serial.print(", \"g_dc\":");
    Serial.print(greenDC,2);
    // Serial.print(", \"b_ac\":");
    // Serial.print(blueAC,2);
    // Serial.print(", \"b_dc\":");
    // Serial.print(blueDC,2);
    Serial.println("}");
  }
}

/*****************************
 * Main
 *****************************/

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  // ---- Init camera ----
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    while(true) delay(1000);
  }

  // ---- Configure sensor for raw-like linear RGB ----
  sensor_t *s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, 0); // disable auto exposure
  s->set_aec2(s, 0);
  s->set_gain_ctrl(s, 0);     // disable auto gain
  s->set_awb_gain(s, 0);      // disable AWB gain
  s->set_whitebal(s, 0);      // disable white balance
  s->set_saturation(s, -2);   // reduce saturation (closer to raw)
  s->set_contrast(s, 0);
  s->set_brightness(s, 0);
  s->set_special_effect(s, 0); // no effect
  s->set_colorbar(s, 0);
  s->set_lenc(s, 0);           // disable lens correction
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);

  // Initialize buffers
  for (int i = 0; i < BUFFER_LEN; i++) {
    lpf_buffer_g[i] = 0;
    lpf_buffer_r[i] = 0;

    bpf_buffer_g[i] = 0;
    bpf_buffer_r[i] = 0;
  }

  // start ROI loop
  xTaskCreatePinnedToCore(roiLoop, "roiLoop", 4096, NULL, 1, &roiTaskHandle, 1);

  // ---- Connect Wi-Fi ----
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println(WiFi.localIP());

  // ---- HTTP server ----
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html",
      "<html><body><h1>ESP32-CAM Live Feed</h1>"
      "<img src='/stream' style='width:100%;'></body></html>");
  });
  server.begin();
  Serial.println("HTTP server started");

  // Start Stream Loop
  xTaskCreatePinnedToCore(streamLoop, "streamLoop", 4096, NULL, 1, &streamTaskHandle, 0); // core 0

  Serial.println("Camera ready");
}

void loop() {}

/*****************************
 * Concurrent loops declarations
 *****************************/

void streamLoop(void *pvParameters) {
  while (true) {
    // Capture and stream
    server.handleClient();
  }
}

void roiLoop(void *pvParameters) {
  
  static uint32_t last_time = 0;
  static float fps = 0.0;

  while (true) {
    uint32_t now = millis();
    if (last_time != 0) {
      float dt = (now - last_time) / 1000.0f; // seconds per frame
      if (dt > 0) fps = 1.0f / dt;
    }
    last_time = now;

    // Process ROI or run data extraction
    camera_fb_t *fb = esp_camera_fb_get();
    extract_and_send_data(fb, fps);
    esp_camera_fb_return(fb);
    
    vTaskDelay(pdMS_TO_TICKS(10)); // slower sampling
  }
}

/*****************************
 * HTTP Stream Handlers
 *****************************/

void drawRectRGB565(uint8_t *buf, int w, int h, int x, int y, int rectW, int rectH, uint16_t color) {
  for (int i = 0; i < rectW; i++) {
    int top = (y * w + (x + i)) * 2;
    int bottom = ((y + rectH - 1) * w + (x + i)) * 2;
    buf[top] = color & 0xFF;
    buf[top + 1] = color >> 8;
    buf[bottom] = color & 0xFF;
    buf[bottom + 1] = color >> 8;
  }
  for (int j = 0; j < rectH; j++) {
    int left = ((y + j) * w + x) * 2;
    int right = ((y + j) * w + (x + rectW - 1)) * 2;
    buf[left] = color & 0xFF;
    buf[left + 1] = color >> 8;
    buf[right] = color & 0xFF;
    buf[right + 1] = color >> 8;
  }
}

void handleStream() {
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Cache-Control: no-cache");
  client.println("Connection: close");
  client.println();

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) continue;

    // Draw ROI on buffer
    drawRectRGB565(fb->buf, fb->width, fb->height, ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT, 0xF800);

    // Convert and send JPEG
    uint8_t *jpgBuf = NULL;
    size_t jpgLen = 0;
    if (frame2jpg(fb, 80, &jpgBuf, &jpgLen)) {
      client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", jpgLen);
      client.write(jpgBuf, jpgLen);
      client.print("\r\n");
      free(jpgBuf);
    }

    esp_camera_fb_return(fb);
    delay(WAIT_TIME);
  }
}

