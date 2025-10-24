#include "esp_camera.h"
#include <Arduino.h>

// Camera model AI Thinker
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// ----------------- CONFIG -----------------
#define ROI_X       50    // top-left x of forehead ROI
#define ROI_Y       10    // top-left y of forehead ROI
#define ROI_WIDTH   40    // ROI width in pixels
#define ROI_HEIGHT  40    // ROI height in pixels
#define BUFFER_LEN  100   // Number of frames to keep in buffer (~8s at 10 FPS)
#define SERIAL_BAUD 115200

// Rolling buffers for R/G/B
float redBuffer[BUFFER_LEN];
float greenBuffer[BUFFER_LEN];
float blueBuffer[BUFFER_LEN];
int bufferIndex = 0;

// ----------------- UTILS -----------------
float mean(const float* arr, int len) {
  float sum = 0;
  for(int i=0;i<len;i++) sum += arr[i];
  return sum / len;
}

float stddev(const float* arr, int len, float avg) {
  float sum = 0;
  for(int i=0;i<len;i++) sum += (arr[i]-avg)*(arr[i]-avg);
  return sqrt(sum / len);
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

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
  config.frame_size   = FRAMESIZE_QQVGA; // 160x120 for speed
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
    redBuffer[i] = 0;
    greenBuffer[i] = 0;
    blueBuffer[i] = 0;
  }

  Serial.println("Camera ready");
}

// ----------------- LOOP -----------------
void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Frame capture failed");
    return;
  }

  // Compute mean RGB in ROI
  long sumR = 0, sumG = 0, sumB = 0;
  int pixelCount = 0;
  
  for (int y = ROI_Y; y < ROI_Y + ROI_HEIGHT; y++) {
    for (int x = ROI_X; x < ROI_X + ROI_WIDTH; x++) {
      
      int i = (y * fb->width + x) * 2; // RGB565: 2 bytes per pixel
      
      uint16_t pixel = fb->buf[i] | (fb->buf[i+1] << 8);
      uint8_t r = ((pixel >> 11) & 0x1F) << 3;
      uint8_t g = ((pixel >> 5) & 0x3F) << 2;
      uint8_t b = (pixel & 0x1F) << 3;
      
      sumR += r;
      sumG += g;
      sumB += b;
      
      pixelCount++;
    }
  }

  float meanR = sumR / (float)pixelCount;
  float meanG = sumG / (float)pixelCount;
  float meanB = sumB / (float)pixelCount;

  // Update moving buffer
  redBuffer[bufferIndex] = meanR;
  greenBuffer[bufferIndex] = meanG;
  blueBuffer[bufferIndex] = meanB;
  bufferIndex = (bufferIndex + 1) % BUFFER_LEN;

  // Compute DC (average) and AC (std deviation)
  float redDC = 0, greenDC = 0, blueDC = 0;
  float redAC = 0, greenAC = 0, blueAC = 0;

  for (int i = 0; i < BUFFER_LEN; i++) {
    redDC += redBuffer[i];
    greenDC += greenBuffer[i];
    blueDC += blueBuffer[i];
  }
  redDC /= BUFFER_LEN;
  greenDC /= BUFFER_LEN;
  blueDC /= BUFFER_LEN;

  for (int i = 0; i < BUFFER_LEN; i++) {
    redAC += pow(redBuffer[i] - redDC, 2);
    greenAC += pow(greenBuffer[i] - greenDC, 2);
    blueAC += pow(blueBuffer[i] - blueDC, 2);
  }
  redAC = sqrt(redAC / BUFFER_LEN);
  greenAC = sqrt(greenAC / BUFFER_LEN);
  blueAC = sqrt(blueAC / BUFFER_LEN);

  // send over Serial []
  Serial.print("r_ac:");
  Serial.print(redAC,2);
  Serial.print(", r_dc:");
  Serial.print(redDC,2);
  Serial.print(", g_ac:");
  Serial.print(greenAC,2);
  Serial.print(", g_dc:");
  Serial.print(greenDC,2);
  Serial.print(", b_ac:");
  Serial.print(blueAC,2);
  Serial.print(", b_dc:");
  Serial.print(blueDC,2);
  Serial.println("");

  esp_camera_fb_return(fb);
  delay(100); // ~10 FPS
}
