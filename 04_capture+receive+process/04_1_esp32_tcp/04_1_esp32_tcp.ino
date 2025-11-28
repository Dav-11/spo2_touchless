#include "esp_camera.h"
#include "img_converters.h"

#include <WiFi.h>
#include "wifi_credentials.h" // contains WIFI_SSID and WIFI_PASSWORD
#include "lwip/sockets.h"
#include "lwip/opt.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ----------------- CONFIG -----------------
#define SERIAL_BAUD 250000  // (250000/10)/1024 ~ 24 KB/s

#define ROI_X       0      // top-left x of forehead ROI
#define ROI_Y       0      // top-left y of forehead ROI
#define ROI_WIDTH   160     // ROI width in pixels
#define ROI_HEIGHT  120     // ROI height in pixels

#define BUFFER_LEN  100

#define WAIT_TIME_SAMPLE      50
#define WAIT_TIME_SEND_STALL  10

// ----------------- GLOBAL -----------------

typedef struct {
  uint8_t red;
  uint8_t green;
} Point;

volatile uint16_t (*frames)[ROI_HEIGHT][ROI_WIDTH];
volatile float sample_time[BUFFER_LEN];

volatile int head = 0; // write index
volatile int tail = 0; // read index

volatile unsigned long prev_time = 0;

WiFiClient client;

/*****************************
 * Forward fn declarations
 *****************************/
void streamLoop(void *pvParameters);
void samplingTask(void *pvParameters);

void extract_data_and_save_to_buffer(camera_fb_t *fb, int current_index);
void send_frame(int index);

/*****************************
 * Task declarations
 *****************************/
TaskHandle_t streamTaskHandle;


/*****************************
 * Setup
 *****************************/

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  if (psramFound()) {
    Serial.println("PSRAM is present and enabled!");
    Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
  } else {
    Serial.println("No PSRAM detected.");
  }

  // Malloc 
  size_t total_size = sizeof(uint16_t) * BUFFER_LEN * ROI_HEIGHT * ROI_WIDTH;
  frames = (uint16_t (*)[ROI_HEIGHT][ROI_WIDTH]) heap_caps_malloc(
    total_size,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  if (!frames) {
    Serial.println("Failed to allocate frames in PSRAM");
    while (1);
  }

  Serial.println("Allocated frames in PSRAM!");
  Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());

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
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA; // HD can be too heavy for streaming
  config.jpeg_quality = 12;
  config.fb_count     = 2;

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
  
  Serial.println("Camera ready");

  Serial.print("Connecting to WiFi (SSID: ");
  Serial.print(WIFI_SSID);
  Serial.print(", PASS: ");
  Serial.print(WIFI_PASSWORD);
  Serial.println(")...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected! IP:");
  Serial.println(WiFi.localIP());

  // Connect to PC
  Serial.print("Connecting to host...");
  while (!client.connect(HOST_IP, HOST_PORT)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nTCP connection established!");

  // Start Stream Loop
  xTaskCreatePinnedToCore(streamLoop, "streamLoop", 4096, NULL, 1, &streamTaskHandle, 0); // pin to C0
  Serial.println("\nStarted stream loop");

  // Start sampling task (Core 0)
  xTaskCreatePinnedToCore(
    samplingTask,
    "samplingTask",
    4096,
    NULL,
    2,          // higher priority than streamLoop
    NULL,
    1
  );

  Serial.println("Sampling task started");

  Serial.println("Completed init");
}


/*****************************
 * Loops
 *****************************/

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));  // nothing here, sampling is handled in its own task
}

void samplingTask(void *pvParameters) {
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t sample_period = pdMS_TO_TICKS(WAIT_TIME_SAMPLE);

  prev_time = millis();  // initialize timing

  while (true) {

    uint8_t next = (head + 1) % BUFFER_LEN;

    if (next == tail) {
      // buffer full → skip this sample
      Serial.println("Sampling: buffer full, skipping frame");
    } else {
      
      // ---- compute sampling frequency ----
      unsigned long now = millis();
      float hz = 1000.0f / (now - prev_time);
      prev_time = now;

      sample_time[head] = hz;

      // ---- capture frame ----
      camera_fb_t *fb = esp_camera_fb_get();
      extract_data_and_save_to_buffer(fb, head);
      esp_camera_fb_return(fb);

      // advance head ptr
      head = next;
    }

    // Wait until next fixed period
    vTaskDelayUntil(&last_wake_time, sample_period);
  }
}

void streamLoop(void *pvParameters) {
  while (true) {

    if (head == tail) {
      
      // buffer empty => stall      
      Serial.println("Empty buffer, skip");
      vTaskDelay(WAIT_TIME_SEND_STALL);
      continue;
    }

    // send frame
    send_frame(tail, 1);

    // increment tail ptr
    tail = (tail + 1) % BUFFER_LEN;
  }
}


/*****************************
 * Data Sample
 *****************************/

void extract_data_and_save_to_buffer(camera_fb_t *fb, int current_index) {

  const uint8_t *src = fb->buf;
  volatile uint16_t (*dst)[ROI_WIDTH] = (volatile uint16_t (*)[ROI_WIDTH]) frames[current_index];

  size_t bytes_per_row = ROI_WIDTH * 2;      // RGB565 = 2 bytes/pixel
  size_t fb_row_stride = fb->width * 2;      // bytes per camera row

  for (int y = 0; y < ROI_HEIGHT; y++) {

    const uint8_t *src_row = src + ((ROI_Y + y) * fb_row_stride) + ROI_X * 2;
    memcpy((void*)dst[y], src_row, bytes_per_row);
  }
}


/*****************************
 * Data transmit
 *****************************/

void send_frame(int index, int frame_number) {

  if (!client.connected()) {
    client.connect(HOST_IP, HOST_PORT);
    if (!client.connected()) return;
  }

  size_t chunk_size = frame_number * ROI_WIDTH * ROI_HEIGHT * sizeof(uint16_t);

  // send frame
  client.write((uint8_t*)frames[index], chunk_size);

  // Send sample frequency
  client.write((uint8_t*)&sample_time[index], sizeof(float));
}
