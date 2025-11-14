#include "esp_camera.h"
#include "img_converters.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

#include "filters.h"

#include <WiFi.h>
#include <WebServer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ----------------- CONFIG -----------------
#define ROI_X       100   // top-left x of forehead ROI
#define ROI_Y       50    // top-left y of forehead ROI
#define ROI_WIDTH   65    // ROI width in pixels
#define ROI_HEIGHT  65    // ROI height in pixels

#define BUFFER_LEN  100
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

BPFState bpf_prev_val_g = {0.0, 0.0};
BPFState bpf_prev_val_r = {0.0, 0.0};
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
TaskHandle_t roiTaskHandle;

// Forward declarations
void handleStream();
void roiLoop(void *pvParameters);

/*****************************
 * ROI DATA EXTRACTION
 *****************************/

void extract_and_send_data(camera_fb_t *fb, float fps) {

  // --- Decode JPEG to RGB888 ---
  size_t out_len = fb->width * fb->height * 3; // RGB888 = 3 bytes per pixel
  uint8_t *rgb_buf = (uint8_t *)malloc(out_len);
  if (!rgb_buf) {
    Serial.println("Failed to allocate RGB buffer!");
    return;
  }

  if (!fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_buf)) {
    Serial.println("JPEG decode failed!");
    free(rgb_buf);
    return;
  }

  // --- Compute mean RGB in ROI ---
  long sumR = 0, sumG = 0, sumB = 0;
  float lpf_r = 0, lpf_g = 0, lpf_b = 0;
  float bpf_r = 0, bpf_g = 0, bpf_b = 0;

  int pixelCount = 0;

  for (int y = ROI_Y; y < ROI_Y + ROI_HEIGHT; y++) {
    for (int x = ROI_X; x < ROI_X + ROI_WIDTH; x++) {
      int i = (y * fb->width + x) * 3; // RGB888: 3 bytes per pixel

      uint8_t r = rgb_buf[i];
      uint8_t g = rgb_buf[i + 1];
      // uint8_t b = rgb_buf[i + 2];

      sumR += r;
      sumG += g;
      // sumB += b;

      pixelCount++;
    }
  }

  free(rgb_buf); // Done with temporary buffer

  float meanR = sumR / (float)pixelCount;
  float meanG = sumG / (float)pixelCount;
  // float meanB = sumB / (float)pixelCount;

  // LPF
  lpf_g = LPF(meanG, fps, lpf_cut_freq_hz, &lpf_prev_val_g);
  lpf_r = LPF(meanR, fps, lpf_cut_freq_hz, &lpf_prev_val_r);

  // BPF
  bpf_g = BPF(meanG, fps, bpf_center_freq_hz, bpf_bandwidth_hz, &bpf_prev_val_g);
  bpf_r = BPF(meanR, fps, bpf_center_freq_hz, bpf_bandwidth_hz, &bpf_prev_val_r);

  
  // Serial.print("fps: ");
  // Serial.print(fps);
  // Serial.print(" meanG: ");
  // Serial.print(meanG);
  // Serial.print(" bpf_g: ");
  // Serial.print(bpf_g);
  // Serial.print(" meanR: ");
  // Serial.print(meanR);
  // Serial.print(" bpf_r:");
  // Serial.print(bpf_r);


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

  float redDC = 0, greenDC = 0;
  float redAC = 0, greenAC = 0;

  // Compute DC (average)
  for (int i = 0; i < BUFFER_LEN; i++) {
    greenDC += lpf_buffer_g[i];
    redDC += lpf_buffer_r[i];
  }
  redDC /= BUFFER_LEN;
  greenDC /= BUFFER_LEN;

  // Compute AC (std deviation)
  for (int i = 0; i < BUFFER_LEN; i++) {

    greenAC += pow(bpf_buffer_g[i], 2);
    redAC += pow(bpf_buffer_r[i], 2);
  }
  redAC = sqrt(redAC / BUFFER_LEN);
  greenAC = sqrt(greenAC / BUFFER_LEN);

  // Send over Serial if buffer full
  if (bufferFull == 0) {
    Serial.print("Waiting for buffer to fill... (");
    Serial.print(bufferIndex - 1);
    Serial.print("/");
    Serial.print(BUFFER_LEN);
    Serial.println(")");
  } else {
    Serial.print("{\"r_ac\":");
    Serial.print(redAC, 2);
    Serial.print(", \"r_dc\":");
    Serial.print(redDC, 2);
    Serial.print(", \"g_ac\":");
    Serial.print(greenAC, 2);
    Serial.print(", \"g_dc\":");
    Serial.print(greenDC, 2);
    Serial.println("}");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  // ---- Camera init ----
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA; // HD can be too heavy for streaming
  config.jpeg_quality = 12;
  config.fb_count     = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    while(true) delay(1000);
  }
  Serial.println("Camera ready");

  // Initialize buffers
  for (int i = 0; i < BUFFER_LEN; i++) {
    lpf_buffer_g[i] = 0;
    lpf_buffer_r[i] = 0;

    bpf_buffer_g[i] = 0;
    bpf_buffer_r[i] = 0;
  }

  // start ROI loop
  xTaskCreatePinnedToCore(roiLoop, "roiLoop", 4096, NULL, 1, &roiTaskHandle, 1);

  // ---- Wi-Fi ----
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println(WiFi.localIP());

  // ---- Web server ----
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html",
      "<html><body><h1>ESP32-CAM Live Feed</h1>"
      "<img src='/stream' style='width:100%;'></body></html>");
  });

  server.on("/stream", HTTP_GET, handleStream);

  server.begin();
  Serial.println("HTTP server started");
}

/*****************************
 * Concurrent loops declarations
 *****************************/

void loop() {
  server.handleClient();
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

void handleStream() {
  WiFiClient client = server.client();

  // Send HTTP headers for multipart MJPEG stream
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Cache-Control: no-cache");
  client.println("Connection: close");
  client.println();

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame capture failed");
      continue;
    }

    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);

    delay(50); // regulate frame rate
  }

  client.stop();
}

/*****************************
 * Filters
 *****************************/

float LPF(float input, float fps, float cut_freq, float *prev_value) {
  // fps: samples per second (sampling frequency)
  // cut_freq: desired cutoff frequency in Hz
  // prev_value: pointer to the previous filtered output

  // Protect against invalid parameters
  if (fps <= 0 || cut_freq <= 0) return input;

  // Calculate RC (time constant)
  float RC = 1.0f / (2.0f * M_PI * cut_freq);

  // Calculate sampling period
  float dt = 1.0f / fps;

  // Compute alpha (filter coefficient)
  float alpha = dt / (RC + dt);

  // Apply low-pass filter formula
  float output = *prev_value + alpha * (input - *prev_value);

  // Update previous value
  *prev_value = output;

  return output;
}

float BPF(float input, float fps, float center_freq, float bandwidth, BPFState *s) {
  
  if (fps <= 0 || center_freq <= 0 || bandwidth <= 0) return input;

  // Compute cutoff frequencies
  float f_low = center_freq - bandwidth / 2.0f;
  float f_high = center_freq + bandwidth / 2.0f;
  if (f_low < 0.0f) f_low = 0.0f;
  if (f_high > fps / 2.0f) f_high = fps / 2.0f;

  // Convert to smoothing coefficients (EMA alpha)
  float alpha_low  = 1.0f - expf(-2.0f * M_PI * f_low / fps);
  float alpha_high = 1.0f - expf(-2.0f * M_PI * f_high / fps);

  // --- Low-pass EMA ---
  s->lp += alpha_low * (input - s->lp);

  // --- High-pass EMA ---
  s->hp += alpha_high * (input - s->hp);

  // Band-pass = difference between low-pass and high-pass
  return s->lp - s->hp;
}

filto butterworth ord
FIR
FFT + cut f0

differenza pixel - pixel


