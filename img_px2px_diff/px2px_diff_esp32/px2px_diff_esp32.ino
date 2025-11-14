#include "esp_camera.h"
#include "img_converters.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ----------------- CONFIG -----------------
#define ROI_X       100   // top-left x of forehead ROI
#define ROI_Y       50    // top-left y of forehead ROI
#define ROI_WIDTH   65    // ROI width in pixels
#define ROI_HEIGHT  65    // ROI height in pixels

#define BUFFER_LEN  10
#define SERIAL_BAUD 115200 // (115200/10)/1024 ~ 11 KB/s
#define WAIT_TIME   100

// ----------------- GLOBAL -----------------

typedef struct {
  uint8_t red;
  uint8_t green;
} Point;

Point frames[BUFFER_LEN][ROI_HEIGHT][ROI_WIDTH];

int buffer_index = 0;



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

  Serial.println("Completed init");
}

uint8_t * get_rgb888_buf(camera_fb_t *fb) {

  // --- Decode JPEG to RGB888 ---
  size_t out_len = fb->width * fb->height * 3; // RGB888 = 3 bytes per pixel
  uint8_t *rgb_buf = (uint8_t *)malloc(out_len);
  if (!rgb_buf) {
    Serial.println("Failed to allocate RGB buffer!");
    return NULL;
  }

  if (!fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_buf)) {
    Serial.println("JPEG decode failed!");
    free(rgb_buf);
    return NULL;
  }

  return rgb_buf;
}

void extract_data_and_save_to_buffer(camera_fb_t *fb) {

  // --- Get frame in RGB888 format ---
  uint8_t *rgb_buf = get_rgb888_buf(fb);

  // (65x65x2)/1024 ~ 8,5 KB
  for (int y = ROI_Y; y < ROI_Y + ROI_HEIGHT; y++) {
    for (int x = ROI_X; x < ROI_X + ROI_WIDTH; x++) {
      int i = (y * fb->width + x) * 3; // RGB888: 3 bytes per pixel

      uint8_t r = rgb_buf[i];
      uint8_t g = rgb_buf[i + 1];
      // uint8_t b = rgb_buf[i + 2];


      frames[buffer_index][y - ROI_Y][x - ROI_X].green = g;
      frames[buffer_index][y - ROI_Y][x - ROI_X].red = r;
    }
  }

  free(rgb_buf);
}

void send_full_buffer() {
  Serial.println("\n--- SENDING BLOCK ---");

  // Optional: send a header so PC can detect block
  Serial.write("FRAMEBLOCK", 10);

  // Send raw binary data
  for (int b = 0; b < BUFFER_LEN; b++) {
    for (int ry = 0; ry < ROI_HEIGHT; ry++) {
      for (int rx = 0; rx < ROI_WIDTH; rx++) {

        // 2 bytes per point → (R,G)
        Serial.write(frames[b][ry][rx].red);
        Serial.write(frames[b][ry][rx].green);
      }
    }
  }

  Serial.println("\n--- DONE ---");
}

void loop() {
  
  for (buffer_index = 0; buffer_index < BUFFER_LEN; buffer_index++) {

    // extract BUFFER_LEN number of samples and saves to RAM
    camera_fb_t *fb = esp_camera_fb_get();
    extract_data_and_save_to_buffer(fb);
    esp_camera_fb_return(fb);
  }

  // When buffer full, send it
  send_full_buffer();

  // Optional: small pause
  delay(WAIT_TIME);
}

