#include <WiFi.h>
#include <esp_now.h>
#include "esp_camera.h"
#include "esp_http_server.h"

#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "ESP32-Camera-AP";
const char* password = "123456789";

// Alamat MAC ESP B (penerima)
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0xE5, 0x4C};

// Struktur data yang akan dikirim melalui ESP-NOW
typedef struct struct_message {
  char data[8];  // Data yang akan dikirimkan, bisa diubah sesuai kebutuhan
} struct_message;

struct_message msg;

// Inisialisasi server Wi-Fi di port 80
WiFiServer server(80);

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // PSRAM handling
  if (psramFound()) {
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Inisialisasi kamera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera gagal diinisialisasi dengan kode error 0x%x", err);
    return;
  }
  Serial.println("Kamera berhasil diinisialisasi");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nPengiriman data ke MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print("\nStatus pengiriman: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Terkirim" : "Gagal");
}

void sendData() {
  WiFiClient client = server.available();   
  if (client) {                            
    Serial.println("Client terhubung");
    while (client.connected()) {            
      if (client.available()) {             
        // Membaca data yang diterima dari komputer
        String request = client.readStringUntil('\n');
        client.flush(); // Membersihkan buffer

        // Menampilkan data yang diterima di serial monitor
        Serial.print("Data dari port 80: ");
        Serial.println(request);

        // Mengirim data yang diterima ke ESP B melalui ESP-NOW
        request.toCharArray(msg.data, sizeof(msg.data));
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &msg, sizeof(msg));
        if (result == ESP_OK) {
          Serial.println("Data dikirim ke ESP B melalui ESP-NOW");
        } else {
          Serial.print("Gagal mengirim data ke ESP B. Error code: ");
          Serial.println(result);  // Tampilkan error code untuk debugging
        }
      }
    }
    client.stop();
    Serial.println("Client terputus");
  }
}

// Server handler for streaming MJPEG
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t * _jpg_buf;
  char * part_buf[64];
  
  res = httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

  if(res != ESP_OK){
      return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
      res = httpd_resp_send_chunk(req, "--frame\r\n", strlen("--frame\r\n"));
      if (res == ESP_OK){
          res = httpd_resp_send_chunk(req, "Content-Type: image/jpeg\r\n\r\n", 
                                    strlen("Content-Type: image/jpeg\r\n\r\n"));
      }
      if (res == ESP_OK){
          res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
      if (res == ESP_OK){
          res = httpd_resp_send_chunk(req, "\r\n", strlen("\r\n"));
      }
      esp_camera_fb_return(fb);
    }
    if(res != ESP_OK){
        break;
    }
  }
  return res;
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);
    }
}


void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  initCamera();

  // Set up WiFi Access Point
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Start streaming
  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(IP);
  Serial.println(":81/stream' to connect");

  // Dapatkan channel Wi-Fi yang digunakan oleh AP
  int channel = WiFi.channel();
  Serial.print("Channel AP (ESP A): ");
  Serial.println(channel);

  // Mulai server Wi-Fi di port 80
  server.begin();
  Serial.println("Server berjalan di port 80");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inisialisasi ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW berhasil diinisialisasi");

  // Register callback untuk status pengiriman ESP-NOW
  esp_now_register_send_cb(OnDataSent);

  // Tambahkan peer ESP-NOW (ESP B)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = channel;  // Menggunakan channel yang sama dengan AP
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error menambahkan peer ESP-NOW");
    return;
  }
  Serial.println("Peer ESP B berhasil ditambahkan");
}

void loop() {
  sendData();
}