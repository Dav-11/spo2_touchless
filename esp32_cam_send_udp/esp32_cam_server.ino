#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

//
// Select camera model (AI Thinker most common)
//
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "iPhone";
const char *password = "GBAJ-dQrD-DaUh-wBT0";

// ---- UDP target ----
const char* udpAddress = "172.20.10.3";   // PC IP address (run `ipconfig` / `ifconfig`)
const int udpPort = 5005;

void startCameraServer();

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // connect Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected, IP: " + WiFi.localIP().toString());

  // init camera
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

  // try QVGA for higher FPS
  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 10;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    while (true) delay(1000);
  }
  Serial.println("Camera ready.");

  udp.begin(WiFi.localIP(), udpPort);
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Frame capture failed");
    return;
  }

  // Send frame over UDP in chunks
  const size_t packetSize = 1024;
  size_t total = fb->len;
  size_t offset = 0;

  while (offset < total) {
    size_t len = min(packetSize, total - offset);
    udp.beginPacket(udpAddress, udpPort);
    // small 4-byte header: frame ID + offset flag
    udp.write((uint8_t*)&offset, 4);
    udp.write(fb->buf + offset, len);
    udp.endPacket();
    offset += len;
    delayMicroseconds(200); // throttle slightly
  }

  esp_camera_fb_return(fb);
}

// #include <WebServer.h>
// #include "esp_timer.h"
// #include "img_converters.h"
// #include "fb_gfx.h"
// #include "esp_http_server.h"

// // simple HTTP stream handler
// static esp_err_t stream_handler(httpd_req_t *req){
//   camera_fb_t * fb = NULL;
//   esp_err_t res = ESP_OK;
//   char part_buf[64];

//   res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
//   if(res != ESP_OK){ return res; }

//   while(true){
//     fb = esp_camera_fb_get();
//     if (!fb) {
//       Serial.println("Camera capture failed");
//       res = ESP_FAIL;
//     } else {
//       size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);
//       res = httpd_resp_send_chunk(req, part_buf, hlen);
//       if(res == ESP_OK){
//         res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
//       }
//       if(res == ESP_OK){
//         res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
//       }
//       esp_camera_fb_return(fb);
//     }
//     if(res != ESP_OK){ break; }
//   }
//   return res;
// }

// void startCameraServer(){
//   httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//   config.server_port = 81;
//   httpd_handle_t stream_httpd = NULL;
//   httpd_uri_t stream_uri = {
//     .uri       = "/stream",
//     .method    = HTTP_GET,
//     .handler   = stream_handler,
//     .user_ctx  = NULL
//   };
//   if (httpd_start(&stream_httpd, &config) == ESP_OK) {
//     httpd_register_uri_handler(stream_httpd, &stream_uri);
//   }
// }
