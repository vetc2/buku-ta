#include <WiFi.h>
#include <Arduino.h>
#include <esp_now.h>

// Motor Kiri
#define pwmpin1 5
#define dir1 18
#define dir2 19

// Motor kanan
#define pwmpin2 25
#define dir3 32
#define dir4 33

#define pwmChannel1 0
#define pwmChannel2 1
#define freq 15000
#define res 8

int PWM1_DutyCycle = 0;
int maxspeed = 70;
int turnspeed = 35;
int kecepatan = 0;

const char* ssid = "ESP32-Camera-AP";
const char* password = "123456789";

char arah[200];
bool _mac = true;

// Fungsi untuk membagi arah berdasarkan 5 level
void setMaxSpeed(char arah[]) {
  int level = atoi(arah);
  maxspeed = 70 + 37 * (level - 1);
  Serial.print("Max Speed: ");
  Serial.println(maxspeed);
}

// Fungsi untuk mengontrol motor dengan 4 parameter
void kontrolMotor(int d1, int d2, int d3, int d4) {
  // Tentukan target speed
  int targetSpeed;
  d1 == HIGH && d3 == HIGH ?
    targetSpeed = maxspeed :  // Jika d1 dan d3 HIGH, gunakan maxspeed
  d1 == HIGH || d2 == HIGH || d3 == HIGH || d4 == HIGH ?
    targetSpeed = turnspeed :  // Jika ada salah satu HIGH, gunakan turnspeed
    targetSpeed = 0;  // Jika semuanya LOW, kecepatan 0

  // Write pin direction
  digitalWrite(dir1, d1);
  digitalWrite(dir2, d2);
  digitalWrite(dir3, d3);
  digitalWrite(dir4, d4);

  // Tentukan apakah kecepatan naik atau turun
  if (PWM1_DutyCycle < targetSpeed) {
    ledcWrite(pwmChannel1, PWM1_DutyCycle++);
    ledcWrite(pwmChannel2, PWM1_DutyCycle++);
  } 
  else {
    ledcWrite(pwmChannel1, PWM1_DutyCycle--);
    ledcWrite(pwmChannel2, PWM1_DutyCycle--);
  }
  delay(10);
}

void OnDataRecv(const esp_now_recv_info* recvInfo, const uint8_t *data, int data_len) {
  char macStr[18];  // Buffer untuk menyimpan string MAC address
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
    recvInfo->src_addr[0], recvInfo->src_addr[1], recvInfo->src_addr[2],
    recvInfo->src_addr[3], recvInfo->src_addr[4], recvInfo->src_addr[5]);
  
  Serial.print("Received data from: ");
  Serial.println(macStr);
  _mac = false;

  if (data_len < sizeof(arah)) {
    memcpy(arah, data, data_len);
    arah[data_len] = '\0';
  } else {
    Serial.println("Received data is too large to fit in 'arah' buffer.");
  }

  Serial.print("Arah: ");
  Serial.println(arah);

  strcmp(arah, "1") == 0 || strcmp(arah, "2") == 0 || strcmp(arah, "3") == 0 || 
  strcmp(arah, "4") == 0 || strcmp(arah, "5") == 0 ?
    setMaxSpeed(arah):
  
  strcmp(arah, "A") == 0 ?
    kontrolMotor(LOW, LOW, HIGH, LOW):
  strcmp(arah, "B") == 0 ?
    kontrolMotor(HIGH, LOW, HIGH, LOW):
  strcmp(arah, "C") == 0 ?
    kontrolMotor(LOW, LOW, LOW, LOW):
  strcmp(arah, "D") == 0 ?
    kontrolMotor(LOW, HIGH, LOW, HIGH):
  strcmp(arah, "E") == 0 ?
    kontrolMotor(HIGH, LOW, LOW, LOW):
  delay(10);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("WiFi connected.");

  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(dir4, OUTPUT);

  // Penggunaan ledcAttach untuk mengatur PWM pada pin
  ledcAttachChannel(pwmpin1, freq, res, pwmChannel1);
  ledcAttachChannel(pwmpin2, freq, res, pwmChannel2);

  // Inisialisasi ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.print("Alamat MAC ESP32 B: ");
  Serial.println(WiFi.macAddress());

  // Register callback untuk menerima pesan ESP-NOW
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {}