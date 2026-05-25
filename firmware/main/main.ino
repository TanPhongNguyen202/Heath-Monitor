/**
 * ============================================================
 *  Health Monitor v4.0 – FreeRTOS Dual-Core
 *  ESP32 + MAX30105 + MAX30205 + SSD1306 + Firebase
 * ============================================================
 *
 *  [1] FreeRTOS dual-core
 *      Core 0: firebaseTask (P1) – WiFi/Firebase nằm đây vì WiFi stack cũng ở Core 0
 *      Core 1: sensorTask (P3) + displayTask (P2) – real-time, không bị WiFi tranh CPU
 *
 *  [2] Circular waveform buffer
 *      push() = O(1), get(i) = O(1)
 *      Không còn memmove() (O(N) mỗi frame = ~12 KB copy/frame @ 10 FPS)
 *
 *  [3] FIFO reading MAX30105
 *      particleSensor.check() → available() → getFIFOIR/Red() → nextSample()
 *      Thay vì gọi getIR()/getRed() mỗi loop (polling bỏ lỡ mẫu giữa các call)
 *
 *  [4] Wire mutex
 *      MAX30105, MAX30205, SSD1306 đều dùng I2C.
 *      wireMutex bảo đảm chỉ 1 task dùng bus tại một thời điểm.
 *
 *  [5] HealthData chia sẻ qua healthMutex (SemaphoreHandle_t)
 *  [6] IR samples truyền từ sensorTask → displayTask qua irQueue
 * ============================================================
 */

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"               // checkForBeat() chính thức
#include "Protocentral_MAX30205.h"
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiManager.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <algorithm>
#include <cmath>

// ============================================================
//  CẤU HÌNH
// ============================================================

#define DATABASE_URL    "https://healthmonitor-a6b4e-default-rtdb.firebaseio.com/"
#define DATABASE_SECRET "7HKiz6i3KRUnQ9b69ldM83uu4cHVvqCQwLF1OrDP"
#define FIREBASE_PATH   "/Health"

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1

// Hysteresis ngón tay (MAX30105 trả ~50k-150k khi có ngón)
#define FINGER_ON_THRESHOLD  50000L
#define FINGER_OFF_THRESHOLD 30000L

// HR
#define HR_RATES_SIZE    8
#define HR_HISTORY_SIZE  30
#define HR_MIN           30.0f
#define HR_MAX           200.0f
#define HR_OUTLIER_THR   20.0f
#define IIR_ALPHA_HR     0.25f
#define IIR_ALPHA_SPO2   0.10f

// Nhiệt độ
#define TEMP_OFFSET   0.3f
#define TEMP_MIN      34.0f
#define TEMP_MAX      42.0f
#define TEMP_INTERVAL_MS 2000

// SpO2
#define SPO2_BUF_SIZE    100
#define SPO2_MIN_SAMPLES  40

// Task
#define SENSOR_STACK    6144
#define DISPLAY_STACK   4096
#define FIREBASE_STACK  8192    // Firebase + WiFi cần stack lớn

// Queue IR: đủ chứa 1 giây mẫu (100 SPS / 4 avg = 25 entries/s)
// displayTask chạy 10Hz nên drain 2-3 entry/lần là bình thường
#define IR_QUEUE_LEN  64

// Firebase interval
#define FIREBASE_INTERVAL_MS 2000

// ============================================================
//  SHARED DATA – bảo vệ bởi healthMutex
// ============================================================

struct HealthData {
  float hr       = 0.0f;
  float beatAvg  = 0.0f;
  float spO2     = 0.0f;
  float tempC    = 0.0f;
  bool  fingerOn = false;
  bool  ready    = false;   // đủ dữ liệu HR + SpO2
};

SemaphoreHandle_t healthMutex;   // Bảo vệ sharedHealth
HealthData        sharedHealth;

// Queue IR samples từ sensorTask → displayTask
QueueHandle_t irQueue;

// Mutex I2C Wire (dùng chung bởi sensorTask + displayTask)
SemaphoreHandle_t wireMutex;

// ============================================================
//  HARDWARE OBJECTS
// ============================================================

FirebaseData   fbData;
FirebaseAuth   fbAuth;
FirebaseConfig fbConfig;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105  particleSensor;
MAX30205  tempSensor;
bool      tempSensorOK = false;

// ============================================================
//  [2] CIRCULAR WAVEFORM BUFFER – O(1) push & get, không memmove
// ============================================================

struct CircularWaveBuffer {
  int16_t buf[SCREEN_WIDTH];
  int     head = 0;           // Điểm sẽ ghi tiếp theo
  long    irMin = FINGER_ON_THRESHOLD;
  long    irMax = FINGER_ON_THRESHOLD + 10000L;

  // Gọi sau khi finger removed
  void reset() {
    int mid = (SCREEN_HEIGHT - 18) / 2;
    for (int i = 0; i < SCREEN_WIDTH; i++) buf[i] = mid;
    head  = 0;
    irMin = FINGER_ON_THRESHOLD;
    irMax = FINGER_ON_THRESHOLD + 10000L;
  }

  // O(1): ghi điểm mới, tự scale
  void push(long irValue) {
    // Auto-scale với decay chậm (tránh drift khi tín hiệu thay đổi dần)
    if (irValue > irMax)     irMax = irValue;
    else                     irMax = (long)(irMax * 0.9995f + irValue * 0.0005f);
    if (irValue < irMin)     irMin = irValue;
    else                     irMin = (long)(irMin * 0.9995f + irValue * 0.0005f);
    if (irMax - irMin < 500) irMax = irMin + 500;   // tránh chia 0

    const int waveH = SCREEN_HEIGHT - 18;
    int pt = (int)map(irValue, irMin, irMax, waveH - 2, 2);
    pt = constrain(pt, 0, waveH - 1);

    buf[head] = (int16_t)pt;
    head = (head + 1) % SCREEN_WIDTH;  // wrap around
  }

  // O(1): lấy điểm thứ i (i=0 cũ nhất, i=W-1 mới nhất)
  int16_t get(int i) const {
    return buf[(head + i) % SCREEN_WIDTH];
  }
};

CircularWaveBuffer waveBuffer;

// ============================================================
//  SPO2 – tỷ số AC/DC (RMS)
// ============================================================

float calculateSpO2(long* red, long* ir, int len) {
  if (len < SPO2_MIN_SAMPLES) return 0.0f;

  double dcRed = 0, dcIR = 0;
  for (int i = 0; i < len; i++) { dcRed += red[i]; dcIR += ir[i]; }
  dcRed /= len;
  dcIR  /= len;

  if (dcRed < 5000 || dcIR < 5000) return 0.0f;

  double acRed = 0, acIR = 0;
  for (int i = 0; i < len; i++) {
    double dr = red[i] - dcRed;
    double di = ir[i]  - dcIR;
    acRed += dr * dr;
    acIR  += di * di;
  }
  acRed = sqrt(acRed / len);
  acIR  = sqrt(acIR  / len);

  if (acIR < 1.0) return 0.0f;

  double R    = (acRed / dcRed) / (acIR / dcIR);
  double spo2 = 104.0 - 17.0 * R;
  return (float)constrain(spo2, 80.0, 100.0);
}

inline float iirFilter(float nv, float pv, float a) {
  return a * nv + (1.0f - a) * pv;
}

// ============================================================
//  WIFI & FIREBASE
// ============================================================

void setupWiFi() {
  WiFiManager wm;
  wm.setConnectTimeout(30);
  wm.setConfigPortalTimeout(120);
  if (!wm.autoConnect("ESP32_HealthMonitor")) {
    Serial.println("WiFi failed, restarting...");
    delay(2000);
    ESP.restart();
  }
  Serial.printf("[WiFi] %s\n", WiFi.localIP().toString().c_str());
}

void initFirebase() {
  fbConfig.database_url               = DATABASE_URL;
  fbConfig.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(1);
}

// ============================================================
//  DISPLAY HELPERS (gọi trong wireMutex)
// ============================================================

void showMessage(const char* l1, const char* l2 = nullptr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(8, 18); display.print(l1);
  if (l2) { display.setCursor(8, 32); display.print(l2); }
  display.display();
}

// ============================================================
//  [1] SENSOR TASK – Core 1, Priority 3
//  [3] Dùng FIFO reading, không polling
// ============================================================

void sensorTask(void* pv) {
  // --- State cục bộ của task ---
  float  hrRates[HR_RATES_SIZE]     = {0};
  float  hrHistory[HR_HISTORY_SIZE] = {0};
  int    hrRateSpot = 0, hrHistIdx = 0, hrHistCount = 0;
  float  hrIIR = 0.0f, spo2IIR = 0.0f;
  long   lastBeatTime = 0;

  long   redBuf[SPO2_BUF_SIZE];
  long   irBuf[SPO2_BUF_SIZE];
  int    spo2Idx = 0;
  bool   spo2Full = false;

  bool   fingerOn = false;
  unsigned long lastTempMs = 0;
  HealthData local;

  while (true) {
    // ============================================================
    //  [3] FIFO READ – đọc tất cả samples tích lũy kể từ lần trước
    //  Thay vì getIR()/getRed() chỉ lấy 1 giá trị và bỏ các mẫu giữa
    // ============================================================
    byte newSamples = particleSensor.check();   // đọc FIFO vào buffer nội

    // Xử lý từng sample trong internal buffer
    while (particleSensor.available()) {
      long ir  = particleSensor.getFIFOIR();
      long red = particleSensor.getFIFORed();
      particleSensor.nextSample();    // advance FIFO pointer

      unsigned long now = millis();

      // ---- Hysteresis ngón tay ----
      bool prevFinger = fingerOn;
      if (!fingerOn && ir > FINGER_ON_THRESHOLD)  fingerOn = true;
      if ( fingerOn && ir < FINGER_OFF_THRESHOLD) fingerOn = false;

      if (prevFinger && !fingerOn) {
        // Vừa rút ngón – reset toàn bộ
        hrRateSpot = hrHistIdx = hrHistCount = 0;
        hrIIR = spo2IIR = 0.0f;
        spo2Idx = 0; spo2Full = false;
        lastBeatTime = 0;
        memset(hrRates,   0, sizeof(hrRates));
        memset(hrHistory, 0, sizeof(hrHistory));
        local = HealthData();
        // Cập nhật shared
        if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          sharedHealth = local;
          xSemaphoreGive(healthMutex);
        }
        // Báo displayTask biết không có ngón
        long dummy = 0;
        xQueueSend(irQueue, &dummy, 0);
        continue;
      }

      if (!fingerOn) {
        long dummy = 0;
        xQueueSend(irQueue, &dummy, 0);
        continue;
      }

      // ---- Phát hiện nhịp tim ----
      if (checkForBeat(ir)) {
        long delta = now - lastBeatTime;
        if (lastBeatTime > 0 && delta > 200 && delta < 2000) {  // 30-300 BPM
          float rawHR = 60000.0f / (float)delta;

          if (rawHR >= HR_MIN && rawHR <= HR_MAX) {
            // Bỏ outlier
            float prevHR = (hrHistCount > 0)
              ? hrHistory[(hrHistIdx - 1 + HR_HISTORY_SIZE) % HR_HISTORY_SIZE]
              : rawHR;
            float useHR = (fabsf(rawHR - prevHR) > HR_OUTLIER_THR) ? prevHR : rawHR;

            // IIR
            hrIIR = (hrIIR < 1.0f) ? useHR : iirFilter(useHR, hrIIR, IIR_ALPHA_HR);

            // Lịch sử vòng
            hrHistory[hrHistIdx] = hrIIR;
            hrHistIdx = (hrHistIdx + 1) % HR_HISTORY_SIZE;
            if (hrHistCount < HR_HISTORY_SIZE) hrHistCount++;

            // beatAvg = trung bình HR_RATES_SIZE nhịp gần nhất
            hrRates[hrRateSpot] = (byte)constrain((int)hrIIR, 0, 255);
            hrRateSpot = (hrRateSpot + 1) % HR_RATES_SIZE;
            float avg = 0;
            for (int i = 0; i < HR_RATES_SIZE; i++) avg += hrRates[i];
            avg /= HR_RATES_SIZE;

            local.hr      = hrIIR;
            local.beatAvg = avg;
            Serial.printf("[HR] raw=%.0f iir=%.1f avg=%.1f\n", rawHR, hrIIR, avg);
          }
        }
        lastBeatTime = now;
      }

      // ---- Buffer SpO2 ----
      redBuf[spo2Idx] = red;
      irBuf[spo2Idx]  = ir;
      spo2Idx = (spo2Idx + 1) % SPO2_BUF_SIZE;
      if (spo2Idx == 0) spo2Full = true;

      int sampLen = spo2Full ? SPO2_BUF_SIZE : spo2Idx;
      if (sampLen >= SPO2_MIN_SAMPLES) {
        float raw = calculateSpO2(redBuf, irBuf, sampLen);
        if (raw >= 80.0f) {
          spo2IIR = (spo2IIR < 1.0f) ? raw : iirFilter(raw, spo2IIR, IIR_ALPHA_SPO2);
          local.spO2 = spo2IIR;
        }
      }

      local.fingerOn = true;
      local.ready    = (local.beatAvg > 0 && local.spO2 > 0);

      // Đẩy IR vào queue cho displayTask vẽ sóng
      // xQueueSend với timeout=0: bỏ qua nếu queue đầy (display lag thì cũng không crash)
      xQueueSend(irQueue, &ir, 0);
    }

    // ---- Đọc nhiệt độ (ngoài FIFO loop, mỗi 2s) ----
    unsigned long now2 = millis();
    if (tempSensorOK && now2 - lastTempMs > TEMP_INTERVAL_MS) {
      lastTempMs = now2;

      // Lấy wireMutex trước khi dùng I2C
      if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float t = tempSensor.getTemperature() + TEMP_OFFSET;
        xSemaphoreGive(wireMutex);
        if (t >= TEMP_MIN && t <= TEMP_MAX) local.tempC = t;
      }
    }

    // ---- Cập nhật shared health ----
    if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      sharedHealth = local;
      xSemaphoreGive(healthMutex);
    }

    // Yield 5ms – FIFO 100SPS / avg4 = 25 entries/s, task chạy 200Hz là dư
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ============================================================
//  [1] DISPLAY TASK – Core 1, Priority 2
// ============================================================

void displayTask(void* pv) {
  const TickType_t period  = pdMS_TO_TICKS(100);   // 10 FPS
  TickType_t       lastWake = xTaskGetTickCount();

  while (true) {
    // Đọc shared health
    HealthData h;
    if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      h = sharedHealth;
      xSemaphoreGive(healthMutex);
    }

    if (!h.fingerOn) {
      // Không có ngón: xóa buffer, hiện thông báo
      waveBuffer.reset();
      if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        showMessage("Place your finger", "   on sensor");
        xSemaphoreGive(wireMutex);
      }
      // Drain queue để không tích lũy
      long dummy;
      while (xQueueReceive(irQueue, &dummy, 0) == pdTRUE) {}

    } else {
      // Drain toàn bộ IR queue → push vào circular buffer
      long irVal;
      while (xQueueReceive(irQueue, &irVal, 0) == pdTRUE) {
        if (irVal > 0) waveBuffer.push(irVal);
      }

      // Vẽ lên OLED (cần wireMutex vì dùng I2C)
      if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        const int waveH = SCREEN_HEIGHT - 18;

        // Xóa vùng sóng
        display.fillRect(0, 0, SCREEN_WIDTH, waveH, SSD1306_BLACK);

        // [2] Vẽ từ circular buffer – không memmove
        for (int i = 1; i < SCREEN_WIDTH; i++) {
          display.drawLine(
            i - 1, waveBuffer.get(i - 1),
            i,     waveBuffer.get(i),
            SSD1306_WHITE
          );
        }

        // Thanh thông tin
        display.fillRect(0, waveH, SCREEN_WIDTH, SCREEN_HEIGHT - waveH, SSD1306_BLACK);
        display.drawFastHLine(0, waveH, SCREEN_WIDTH, SSD1306_WHITE);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        char buf[24];
        if (h.beatAvg > 0 && h.spO2 > 0) {
          snprintf(buf, sizeof(buf), "HR:%3d SpO2:%2d%%", (int)h.beatAvg, (int)h.spO2);
        } else {
          snprintf(buf, sizeof(buf), "HR:--- SpO2:--%% ");
        }
        display.setCursor(0, waveH + 2);  display.print(buf);

        if (h.tempC > 0) {
          snprintf(buf, sizeof(buf), "Temp: %.1fC", h.tempC);
        } else {
          snprintf(buf, sizeof(buf), "Temp: --.--C");
        }
        display.setCursor(0, waveH + 11); display.print(buf);

        display.display();
        xSemaphoreGive(wireMutex);
      }
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================
//  [1] FIREBASE TASK – Core 0, Priority 1
//  Ở Core 0 vì WiFi stack của ESP32 chạy trên Core 0
// ============================================================

void firebaseTask(void* pv) {
  const TickType_t period  = pdMS_TO_TICKS(FIREBASE_INTERVAL_MS);
  TickType_t       lastWake = xTaskGetTickCount();

  while (true) {
    HealthData h;
    if (xSemaphoreTake(healthMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      h = sharedHealth;
      xSemaphoreGive(healthMutex);
    }

    if (h.ready && h.fingerOn) {
      FirebaseJson json;
      json.set("HR",   (double)h.beatAvg);
      json.set("SpO2", (double)h.spO2);
      json.set("Temp", (double)h.tempC);
      json.set("ts",   (long)(millis() / 1000));

      if (Firebase.setJSON(fbData, FIREBASE_PATH, json)) {
        Serial.printf("[FB] HR:%.0f SpO2:%.1f Temp:%.1f\n",
                      h.beatAvg, h.spO2, h.tempC);
      } else {
        Serial.println("[FB] Err: " + fbData.errorReason());
      }
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Health Monitor v4.0 (FreeRTOS) ===");

  Wire.begin();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED FAILED"); while (1);
  }
  waveBuffer.reset();
  showMessage("Connecting...");

  setupWiFi();
  initFirebase();

  showMessage("Init sensors...");

  // MAX30105 – cấu hình 18-bit, 100 SPS, avg=4
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    showMessage("MAX30105 error!"); while (1);
  }
  particleSensor.setup(
    0x1F,   // LED brightness ~6.4 mA
    4,      // sample average (giảm nhiễu)
    2,      // mode: Red + IR (bắt buộc cho SpO2)
    100,    // sample rate: 100 SPS
    411,    // pulse width: 411µs = 18-bit ADC
    4096    // ADC range: 4096 nA full scale
  );
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0);

  // MAX30205
  for (int i = 5; i > 0; i--) {
    if (tempSensor.scanAvailableSensors()) {
      tempSensor.begin();
      tempSensorOK = true;
      Serial.println("[TEMP] MAX30205 OK");
      break;
    }
    Serial.printf("[TEMP] Retry %d...\n", 6 - i);
    delay(2000);
  }
  if (!tempSensorOK) Serial.println("[TEMP] Not found – disabled");

  // FreeRTOS primitives
  healthMutex = xSemaphoreCreateMutex();
  wireMutex   = xSemaphoreCreateMutex();
  irQueue     = xQueueCreate(IR_QUEUE_LEN, sizeof(long));

  if (!healthMutex || !wireMutex || !irQueue) {
    Serial.println("RTOS init FAILED"); while (1);
  }

  // Tạo tasks – pinned to core
  //                         name        stack           param prio handle core
  xTaskCreatePinnedToCore(sensorTask,  "Sensor",  SENSOR_STACK,  NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(displayTask, "Display", DISPLAY_STACK, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(firebaseTask,"Firebase",FIREBASE_STACK,NULL, 1, NULL, 0);

  Serial.println("[RTOS] Tasks started");

  // Xóa Arduino loop task – không dùng nữa
  vTaskDelete(NULL);
}

// ============================================================
//  LOOP – không dùng, đã xóa ở setup()
// ============================================================
void loop() {}
