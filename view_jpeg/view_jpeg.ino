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

typedef struct {
  float r;
  float g;
  float b;
} RGBdata;


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

RGBdata extract_data(camera_fb_t *fb) {

  // Decode JPEG to RGB888
  uint8_t *rgb_buf = NULL;
  size_t rgb_buf_len = 0;

  long sumR=0, sumG=0, sumB=0;
  int pixelCount = 0;

  RGBdata mean = {
    .r = 1,
    .g = 1,
    .b = 1
  };

  // Decode JPEG to RGB888 using esp32-camera's helper
  bool decoded = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_buf);

  if (!decoded || rgb_buf == NULL) {
    Serial.println("JPEG decode failed");
    return mean;
  }

  // Note: rgb_buf is an array of [r,g,b,r,g,b,...] bytes, 3 bytes per pixel
  for (int y = ROI_Y; y < ROI_Y + ROI_HEIGHT; y++) {
    for (int x = ROI_X; x < ROI_X + ROI_WIDTH; x++) {
      int i = (y * fb->width + x) * 3;
      uint8_t r = rgb_buf[i];
      uint8_t g = rgb_buf[i + 1];
      uint8_t b = rgb_buf[i + 2];

      sumR += r;
      sumG += g;
      // sumB += b;
      pixelCount++;
    }
  }

  free(rgb_buf); // Important: free the temporary buffer

  mean = {
    .r = sumR / (float)pixelCount,
    .g = sumG / (float)pixelCount,
    .b = sumB / (float)pixelCount
  };

  return mean;
}

void extract_and_send_data(camera_fb_t *fb, float fps) {
  
  // --- Compute mean RGB in ROI every frame ---

  // Compute mean RGB in ROI
  float lpf_r=0, lpf_g=0, lpf_b=0;
  float bpf_r=0, bpf_g=0, bpf_b=0;
  
  RGBdata mean = extract_data(fb);

  float meanR = mean.r;
  float meanG = mean.g;

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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_HD;
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

// void draw_rect(uint8_t *buf, int w, int h, int x, int y, int rectW, int rectH, uint16_t color) {
  
//   // Step 1: Decode JPEG to RGB888
//   uint8_t *rgb_buf = NULL;
//   size_t rgb_len = 0;
//   int w = fb->width;
//   int h = fb->height;

//   bool decoded = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, &rgb_buf);
//   if (!decoded || rgb_buf == NULL) {
//     Serial.println("JPEG decode failed");
//     return;
//   }

//   // Step 2: Draw rectangle on the RGB888 buffer
//   // Color conversion from RGB565 -> RGB888
//   uint8_t r = ((color >> 11) & 0x1F) << 3;
//   uint8_t g = ((color >> 5) & 0x3F) << 2;
//   uint8_t b = (color & 0x1F) << 3;

//   for (int i = 0; i < rectW; i++) {
//     int top = ((y * w) + (x + i)) * 3;
//     int bottom = (((y + rectH - 1) * w) + (x + i)) * 3;

//     rgb_buf[top] = r;     rgb_buf[top + 1] = g;     rgb_buf[top + 2] = b;
//     rgb_buf[bottom] = r;  rgb_buf[bottom + 1] = g;  rgb_buf[bottom + 2] = b;
//   }

//   for (int j = 0; j < rectH; j++) {
//     int left = (((y + j) * w) + x) * 3;
//     int right = (((y + j) * w) + (x + rectW - 1)) * 3;

//     rgb_buf[left] = r;    rgb_buf[left + 1] = g;    rgb_buf[left + 2] = b;
//     rgb_buf[right] = r;   rgb_buf[right + 1] = g;   rgb_buf[right + 2] = b;
//   }

//   // Step 3: Re-encode the modified image back to JPEG
//   uint8_t *jpeg_buf = NULL;
//   size_t jpeg_len = 0;

//   if (!fmt2jpg(rgb_buf, w * h * 3, w, h, PIXFORMAT_RGB888, 80, &jpeg_buf, &jpeg_len)) {
//     Serial.println("JPEG encode failed");
//     free(rgb_buf);
//     return;
//   }

//   // Step 4: Replace old frame buffer contents
//   free(fb->buf);
//   fb->buf = jpeg_buf;
//   fb->len = jpeg_len;

//   free(rgb_buf);
// }

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
    // draw_rect(fb->buf, fb->width, fb->height, ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT, 0xF800);

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
  // fps         = sampling rate (Hz)
  // center_freq = center frequency (Hz)
  // bandwidth   = bandwidth (Hz)
  // s           = filter state (must persist between calls)

  if (fps <= 0 || center_freq <= 0 || bandwidth <= 0) return input;

  // Pre-warped values
  float w0 = 2.0f * M_PI * center_freq / fps;
  float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bandwidth * w0 / sinf(w0));

  // Coefficients for a constant skirt gain, peak gain = Q
  float b0 = alpha;
  float b1 = 0.0f;
  float b2 = -alpha;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cosf(w0);
  float a2 = 1.0f - alpha;

  // Normalize coefficients
  b0 /= a0;
  b1 /= a0;
  b2 /= a0;
  a1 /= a0;
  a2 /= a0;

  // Biquad difference equation
  float output = b0 * input + b1 * s->x1 + b2 * s->x2
                               - a1 * s->y1 - a2 * s->y2;

  // Shift history
  s->x2 = s->x1;
  s->x1 = input;
  s->y2 = s->y1;
  s->y1 = output;

  return output;
}
