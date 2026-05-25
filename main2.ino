#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "Protocentral_MAX30205.h"
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <numeric>
#include <WiFiManager.h>
#include <algorithm>

#define DATABASE_URL "https://healthmonitor-a6b4e-default-rtdb.firebaseio.com/"
#define DATABASE_SECRET "7HKiz6i3KRUnQ9b69ldM83uu4cHVvqCQwLF1OrDP"

FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;
String path = "/Health";

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;
MAX30205 tempSensor;

// --- TỐI ƯU BỘ ĐỆM VÒNG (Cố định RAM) ---
#define BUFFER_SIZE 10
long irBuffer[BUFFER_SIZE];
int irIndex = 0;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
float spO2 = 0.0;
float temperatureC = 0.0;

const long MIN_IR_VALUE = 4000;
bool fingerDetected = false;
long lastBeatDetectedTime = 0;
const int beatThreshold = 5000;
const int debounceDelay = 330;

// Thay thế std::vector bằng mảng cố định làm bộ đệm vòng
#define MAX_HISTORY 50 // Giảm xuống 50 vẫn đủ mịn và tiết kiệm CPU khi sort
float hrHistory[MAX_HISTORY];
int hrHistoryIndex = 0;
int hrHistoryCount = 0;

long irMovingAverage = 0;
const int smoothingFactor = 20;
unsigned long previousMillis = 0;
unsigned long lastNoFingerUpdate = 0;

uint8_t graph[SCREEN_WIDTH];
int graphHead = 0; // Con trỏ vòng cho đồ thị (Không cần dịch mảng)

const float spo2Waveform[32] = {
  0.0, 0.2, 0.5, 0.8, 1.0, 1.2, 1.3, 1.5, 1.55, 1.5, 1.3, 1.0, 0.8, 0.82, 
  0.84, 0.86, 0.88, 0.92, 0.8, 0.7, 0.6, 0.4, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

// --- BIẾN ĐỒNG BỘ GIỮA 2 CORE (FreeRTOS) ---
TaskHandle_t FirebaseTaskHandle;
portMUX_TYPE dataMutex = vPORT_MUX_INITIALIZER_UNLOCKED;
float shared_HR = 0, shared_SpO2 = 0, shared_Temp = 0;
bool dataReadyToSend = false;

// --- TỐI ƯU CÁC HÀM LỌC ---
float medianFilter() {
  if (hrHistoryCount == 0) return 0;
  float temp[MAX_HISTORY];
  memcpy(temp, hrHistory, hrHistoryCount * sizeof(float));
  std::sort(temp, temp + hrHistoryCount);
  return temp[hrHistoryCount / 2];
}

float movingAverage() {
  if (hrHistoryCount == 0) return 0;
  float sum = 0;
  for (int i = 0; i < hrHistoryCount; i++) sum += hrHistory[i];
  return sum / hrHistoryCount;
}

inline bool isOutlier(float current, float previous, float threshold = 11.0) {
  return fabs(current - previous) > threshold;
}

inline float applyIIRFilter(float current, float previous, float alpha = 0.4) {
  return alpha * current + (1.0f - alpha) * previous;
}

float lastFilteredHR = 0;

inline float calculateSpO2Simple(long redValue, long irValue) {
  if (irValue == 0) return 0;
  float ratio = (float)redValue / irValue;
  return constrain(110.0f - (25.0f * ratio), 0.0f, 100.0f);
}

bool improvedCheckForBeat(long irValue) {
  irBuffer[irIndex] = irValue;
  irIndex = (irIndex + 1) % BUFFER_SIZE;

  int prev = (irIndex + BUFFER_SIZE - 2) % BUFFER_SIZE;
  int curr = (irIndex + BUFFER_SIZE - 1) % BUFFER_SIZE;
  int next = irIndex;

  if (irBuffer[prev] < irBuffer[curr] && irBuffer[curr] > irBuffer[next] && irBuffer[curr] > beatThreshold) {
    if (millis() - lastBeatDetectedTime > debounceDelay) {
      lastBeatDetectedTime = millis();
      return true;
    }
  }
  return false;
}

// Vẽ sóng sử dụng kỹ thuật vòng (Circular Graph Buffer) giúp giảm thời gian dịch mảng từ O(N) về O(1)
void drawPlethWave(float amplitude, float bpm, float spo2, float temp) {
  static int waveIndex = 0;
  float yOffset = spo2Waveform[waveIndex] * amplitude;
  int baseline = SCREEN_HEIGHT / 2;
  int newY = constrain(baseline - (int)yOffset, 0, SCREEN_HEIGHT - 17);
  
  graph[graphHead] = newY;
  graphHead = (graphHead + 1) % SCREEN_WIDTH;
  waveIndex = (waveIndex + 1) % 32;

  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT - 17, SSD1306_BLACK);

  // Vẽ lại từ điểm đầu đến điểm cuối dựa vào con trỏ head
  for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
    int idx1 = (graphHead + i) % SCREEN_WIDTH;
    int idx2 = (graphHead + i + 1) % SCREEN_WIDTH;
    display.drawLine(i, graph[idx1], i + 1, graph[idx2], SSD1306_WHITE);
  }

  // Khối thông tin hiển thị bên dưới
  display.fillRect(0, SCREEN_HEIGHT - 17, SCREEN_WIDTH, 17, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, SCREEN_HEIGHT - 16);
  display.printf("HR:%.0f SpO2:%.0f", bpm, spo2); // Dùng printf ngắn gọn hơn
  display.setCursor(0, SCREEN_HEIGHT - 8);
  display.printf("Temp:%.1fC", temp);

  display.display();
}

void setupWiFi() {
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180); // Tránh treo vô hạn nếu không cấu hình được
  if (!wifiManager.autoConnect("ESP32_ConfigAP")) {
    Serial.println("Không kết nối được WiFi, khởi động lại...");
    delay(3000);
    ESP.restart();
  }
  Serial.println("Đã kết nối WiFi!");
}

void initFirebase() {
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// --- TASK LIÊN QUAN ĐẾN ĐẨY DATA LÊN NETWORK (CHẠY RIÊNG Ở CORE 0) ---
void FirebaseTask(void * pvParameters) {
  FirebaseJson json;
  float local_HR, local_SpO2, local_Temp;
  
  for(;;) {
    bool pushData = false;
    
    // Đọc an toàn dữ liệu từ Core 1 sang Core 0 thông qua Mutex (Critical Section)
    portENTER_CRITICAL(&dataMutex);
    if (dataReadyToSend) {
      local_HR = shared_HR;
      local_SpO2 = shared_SpO2;
      local_Temp = shared_Temp;
      dataReadyToSend = false;
      pushData = true;
    }
    portEXIT_CRITICAL(&dataMutex);

    if (pushData && !isnan(local_HR) && !isnan(local_SpO2) && !isnan(local_Temp)) {
      json.set("HR", local_HR);
      json.set("SpO2", local_SpO2);
      json.set("Temp", local_Temp);

      if (Firebase.setJSON(firebaseData, path, json)) {
        Serial.println(">> Core 0: Sent to Firebase successfully.");
      } else {
        Serial.print(">> Core 0: Firebase Error: ");
        Serial.println(firebaseData.errorReason());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Nhường nhịn CPU cho hệ thống handle mạng
  }
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
  initFirebase();
  Wire.begin();
  Wire.setClock(400000); // Ép xung I2C lên 400kHz để tăng tốc OLED và MAX30105

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Initializing...");
  display.display();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) while (1);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x12);
  particleSensor.setPulseAmplitudeGreen(0);

  while (!tempSensor.scanAvailableSensors()) delay(1000);
  tempSensor.begin();

  // Tạo Task chạy Firebase trên Core 0 (độ ưu tiên thấp hơn tác vụ đọc cảm biến ở loop)
  xTaskCreatePinnedToCore(FirebaseTask, "FirebaseTask", 8192, NULL, 1, &FirebaseTaskHandle, 0);
}

void loop() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  fingerDetected = (irValue > MIN_IR_VALUE);

  if (fingerDetected) {
    if (improvedCheckForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      float rawHR = 60.0f / (delta / 1000.0f);

      if (rawHR < 130.0f && rawHR > 30.0f) {
        float filteredHR = rawHR;
        
        // Lấy phần tử cuối cùng được nạp vào bộ đệm vòng
        int prevIdx = (hrHistoryIndex + MAX_HISTORY - 1) % MAX_HISTORY;
        if (hrHistoryCount > 0 && isOutlier(rawHR, hrHistory[prevIdx])) {
          filteredHR = hrHistory[prevIdx];
        }

        // Đẩy vào bộ đệm vòng
        hrHistory[hrHistoryIndex] = filteredHR;
        hrHistoryIndex = (hrHistoryIndex + 1) % MAX_HISTORY;
        if (hrHistoryCount < MAX_HISTORY) hrHistoryCount++;

        float medianHR = medianFilter();
        float averageHR = movingAverage();
        float iirFilteredHR = applyIIRFilter(filteredHR, lastFilteredHR);

        lastFilteredHR = iirFilteredHR;
        beatsPerMinute = iirFilteredHR;

        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
    spO2 = calculateSpO2Simple(redValue, irValue);
    irMovingAverage = (irMovingAverage * (smoothingFactor - 1) + irValue) / smoothingFactor;

    // Chu kỳ cập nhật hiển thị và gửi lệnh đẩy dữ liệu (Mỗi 500ms)
    if (millis() - previousMillis > 500) {
        previousMillis = millis();
        temperatureC = tempSensor.getTemperature() + 0.3f;

        Serial.printf("SpO2: %.1f | Raw HR: %.1f | Avg HR: %d | Temp: %.1f\n", spO2, beatsPerMinute, beatAvg, temperatureC);

        float amplitude = map(spO2, 90, 100, 5, 20);
        amplitude = constrain(amplitude, 5, 20);

        drawPlethWave(amplitude, beatAvg, spO2, temperatureC);

        // Gửi dữ liệu an toàn sang Core 0 (Không gây block Core 1)
        portENTER_CRITICAL(&dataMutex);
        shared_HR = beatsPerMinute;
        shared_SpO2 = spO2;
        shared_Temp = temperatureC;
        dataReadyToSend = true;
        portEXIT_CRITICAL(&dataMutex);
    }
  } else {
    // Không có ngón tay: Reset biến tĩnh
    beatsPerMinute = 0;
    beatAvg = 0;
    spO2 = 0.0;
    temperatureC = 0.0;
    hrHistoryIndex = 0;
    hrHistoryCount = 0;
    lastFilteredHR = 0;

    // Giới hạn tần suất in Serial và vẽ OLED ở nhánh ELSE để tránh nghẽn bus I2C (Mỗi 2 giây)
    if (millis() - lastNoFingerUpdate > 2000) {
      lastNoFingerUpdate = millis();
      
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(10, 20);
      display.print("Place your finger");
      display.setCursor(35, 32);
      display.print("on sensor");
      display.display();

      Serial.println("Waiting for finger...");
    }
  }
}
