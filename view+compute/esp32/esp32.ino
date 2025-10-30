#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>   // Built-in lightweight HTTP server

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

// Wi-Fi credentials
const char* ssid = "iPhone";
const char* password = "GBAJ-dQrD-DaUh-wBT0";

// ----------------- GLOBALS -----------------
WebServer server(80);

// Rolling buffers for R/G/B
float redBuffer[BUFFER_LEN];
float greenBuffer[BUFFER_LEN];
float blueBuffer[BUFFER_LEN];
int bufferIndex = 0;

// ----------------- UTILS -----------------
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

// ----------------- DATA EXTRACTION -----------------

void extract_and_send_data(camera_fb_t *fb) {
  
  // --- Compute mean RGB in ROI every frame ---

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
  Serial.print("{\"r_ac\":");
  Serial.print(redAC,2);
  Serial.print(", \"r_dc\":");
  Serial.print(redDC,2);
  Serial.print(", \"g_ac\":");
  Serial.print(greenAC,2);
  Serial.print(", \"g_dc\":");
  Serial.print(greenDC,2);
  Serial.print(", \"b_ac\":");
  Serial.print(blueAC,2);
  Serial.print(", \"b_dc\":");
  Serial.print(blueDC,2);
  Serial.println("}");
}

// ----------------- STREAM HANDLER -----------------
void handleStream() {
  WiFiClient client = server.client();
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

    // Draw ROI box (in red)
    drawRectRGB565(fb->buf, fb->width, fb->height, ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT, 0xF800);

    // Convert RGB565 → JPEG
    uint8_t *jpgBuf = NULL;
    size_t jpgLen = 0;
    bool ok = frame2jpg(fb, 80, &jpgBuf, &jpgLen);
    if (!ok) continue;

    extract_and_send_data(fb);
    esp_camera_fb_return(fb);


    // Send MJPEG frame
    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", jpgLen);
    client.write(jpgBuf, jpgLen);
    client.print("\r\n");

    free(jpgBuf);
    delay(100); // ~10 FPS
  }
}

// ----------------- SETUP -----------------
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
  server.handleClient();
}
