#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"               // checkForBeat() chính thức
#include "Protocentral_MAX30205.h"
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiManager.h>
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

// --- Ngưỡng phát hiện ngón tay (hysteresis) ---
// MAX30105 trả IR ~50k-150k khi có ngón tay, ~< 5k khi không có
#define FINGER_ON_THRESHOLD  50000L   // Vượt ngưỡng này → coi là có ngón tay
#define FINGER_OFF_THRESHOLD 30000L   // Dưới ngưỡng này → coi là đã nhấc ngón

// --- Tham số lọc tín hiệu ---
#define HR_RATES_SIZE   8             // Số nhịp cuối để tính beatAvg
#define HR_HISTORY_SIZE 30            // Lịch sử dài hơn để median
#define HR_MIN          30.0f
#define HR_MAX          200.0f
#define HR_OUTLIER_THR  20.0f         // Chênh lệch tối đa giữa 2 nhịp liên tiếp (BPM)
#define IIR_ALPHA_HR    0.25f         // Nhỏ = mượt hơn, chậm hơn
#define IIR_ALPHA_SPO2  0.10f         // SpO2 cần mượt hơn HR

// --- Buffer SpO2 ---
#define SPO2_BUF_SIZE 100             // Số mẫu tính AC/DC (1 giây ở 100SPS)
#define SPO2_MIN_SAMPLES 40           // Cần ít nhất N mẫu mới tính

// --- Offset nhiệt độ (hiệu chỉnh sensor) ---
#define TEMP_OFFSET   0.3f
#define TEMP_MIN      34.0f           // Dải nhiệt độ hợp lệ (°C)
#define TEMP_MAX      42.0f

// --- Khoảng thời gian (ms) ---
#define DISPLAY_INTERVAL_MS  100      // Cập nhật OLED
#define TEMP_INTERVAL_MS    2000      // Đọc nhiệt độ
#define FIREBASE_INTERVAL_MS 2000     // Gửi Firebase

// ============================================================
//  ĐỐI TƯỢNG TOÀN CỤC
// ============================================================

FirebaseData   fbData;
FirebaseAuth   fbAuth;
FirebaseConfig fbConfig;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105  particleSensor;
MAX30205  tempSensor;
bool tempSensorOK = false;

// ============================================================
//  DỮ LIỆU SỨC KHỎE
// ============================================================

struct HealthData {
  float hr      = 0.0f;    // HR IIR-filtered (BPM)
  float beatAvg = 0.0f;    // Trung bình N nhịp gần nhất
  float spO2    = 0.0f;    // SpO2 (%)
  float tempC   = 0.0f;    // Nhiệt độ (°C)
  bool  ready   = false;   // Đủ dữ liệu để gửi/hiển thị
};
HealthData health;

// --- Bộ đệm HR ---
float  hrRates[HR_RATES_SIZE] = {0};
int    hrRateSpot = 0;
float  hrHistory[HR_HISTORY_SIZE] = {0};
int    hrHistIdx = 0;
int    hrHistCount = 0;
float  hrIIR = 0.0f;
long   lastBeatTime = 0;

// --- Bộ đệm SpO2 ---
long redBuf[SPO2_BUF_SIZE];
long irBuf[SPO2_BUF_SIZE];
int  spo2BufIdx = 0;
bool spo2BufFull = false;

// --- Trạng thái ngón tay ---
enum FingerState { FINGER_OFF, FINGER_ON };
FingerState fingerState = FINGER_OFF;

// --- Bộ đệm waveform OLED ---
int16_t graphBuf[SCREEN_WIDTH];
long    irMin = FINGER_ON_THRESHOLD;
long    irMax = FINGER_ON_THRESHOLD + 10000;

// --- Timers ---
unsigned long lastDisplay  = 0;
unsigned long lastTemp     = 0;
unsigned long lastFirebase = 0;

// ============================================================
//  HÀM: TÍNH SPO2 (AC/DC RMS – chính xác hơn raw ratio)
// ============================================================
/**
 * Nguyên lý:
 *   DC = giá trị trung bình (thành phần 1 chiều, ánh sáng môi trường)
 *   AC = biến thiên do mạch đập (RMS của phần xoay chiều)
 *   R  = (AC_red / DC_red) / (AC_ir / DC_ir)
 *   SpO2 ≈ 104 – 17*R   (công thức thực nghiệm phổ biến)
 */
float calculateSpO2(long* red, long* ir, int len) {
  if (len < SPO2_MIN_SAMPLES) return 0.0f;

  double dcRed = 0, dcIR = 0;
  for (int i = 0; i < len; i++) {
    dcRed += red[i];
    dcIR  += ir[i];
  }
  dcRed /= len;
  dcIR  /= len;

  // DC phải đủ lớn (ngón tay đang đặt)
  if (dcRed < 5000 || dcIR < 5000) return 0.0f;

  double acRed = 0, acIR = 0;
  for (int i = 0; i < len; i++) {
    double dr = red[i] - dcRed;
    double di = ir[i]  - dcIR;
    acRed += dr * dr;
    acIR  += di * di;
  }
  acRed = sqrt(acRed / len);  // RMS của AC
  acIR  = sqrt(acIR  / len);

  if (acIR < 1.0) return 0.0f;   // Tránh chia 0

  double R    = (acRed / dcRed) / (acIR / dcIR);
  double spo2 = 104.0 - 17.0 * R;
  return (float)constrain(spo2, 80.0, 100.0);
}

// ============================================================
//  HÀM: LỌC IIR
// ============================================================
float iirFilter(float newVal, float prevVal, float alpha) {
  return alpha * newVal + (1.0f - alpha) * prevVal;
}

// ============================================================
//  HÀM: MEDIAN (bộ đệm vòng HR history)
// ============================================================
float medianHRHistory() {
  int cnt = min(hrHistCount, HR_HISTORY_SIZE);
  if (cnt == 0) return 0.0f;
  float tmp[HR_HISTORY_SIZE];
  memcpy(tmp, hrHistory, cnt * sizeof(float));
  std::sort(tmp, tmp + cnt);
  return tmp[cnt / 2];
}

// ============================================================
//  HÀM: RESET KHI RÚT NGÓN TAY
// ============================================================
void resetMeasurement() {
  health   = HealthData();
  hrRateSpot = 0;
  hrHistIdx  = 0;
  hrHistCount = 0;
  hrIIR      = 0.0f;
  spo2BufIdx = 0;
  spo2BufFull = false;
  lastBeatTime = 0;
  memset(hrRates,   0, sizeof(hrRates));
  memset(hrHistory, 0, sizeof(hrHistory));
  // Reset auto-scale waveform
  irMin = FINGER_ON_THRESHOLD;
  irMax = FINGER_ON_THRESHOLD + 10000;
  memset(graphBuf, 0, sizeof(graphBuf));
}

// ============================================================
//  HÀM: WIFI & FIREBASE
// ============================================================
void setupWiFi() {
  WiFiManager wm;
  wm.setConnectTimeout(30);
  wm.setConfigPortalTimeout(120);
  wm.setAPCallback([](WiFiManager* wm) {
    Serial.printf("Config Portal: %s\n", wm->getConfigPortalSSID().c_str());
  });
  if (!wm.autoConnect("ESP32_HealthMonitor")) {
    Serial.println("WiFi failed, restarting...");
    delay(2000);
    ESP.restart();
  }
  Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
}

void initFirebase() {
  fbConfig.database_url               = DATABASE_URL;
  fbConfig.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(1);
  Serial.println("Firebase initialized");
}

// ============================================================
//  HÀM: HIỂN THỊ OLED
// ============================================================
void showMessage(const char* line1, const char* line2 = nullptr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(8, 18);
  display.print(line1);
  if (line2) {
    display.setCursor(8, 32);
    display.print(line2);
  }
  display.display();
}

/**
 * Vẽ sóng Pleth từ tín hiệu IR thực (auto-scale)
 * + thanh thông tin phía dưới
 */
void drawWaveform(long irValue) {
  // Cuộn bộ đệm sang trái 1 pixel
  memmove(graphBuf, graphBuf + 1, (SCREEN_WIDTH - 1) * sizeof(int16_t));

  // Auto-scale (theo dõi min/max, decay chậm để tránh drift)
  if (irValue > irMax) irMax = irValue;
  else irMax = irMax * 0.9995 + irValue * 0.0005;   // decay rất chậm

  if (irValue < irMin) irMin = irValue;
  else irMin = irMin * 0.9995 + irValue * 0.0005;

  // Tránh dải quá hẹp
  if (irMax - irMin < 500) irMax = irMin + 500;

  int waveH = SCREEN_HEIGHT - 18;
  int pt = map(irValue, irMin, irMax, waveH - 2, 2);
  pt = constrain(pt, 0, waveH - 1);
  graphBuf[SCREEN_WIDTH - 1] = pt;

  // --- Vẽ vùng sóng ---
  display.fillRect(0, 0, SCREEN_WIDTH, waveH, SSD1306_BLACK);
  for (int i = 1; i < SCREEN_WIDTH; i++) {
    display.drawLine(i - 1, graphBuf[i - 1], i, graphBuf[i], SSD1306_WHITE);
  }

  // --- Thanh thông tin ---
  display.fillRect(0, waveH, SCREEN_WIDTH, SCREEN_HEIGHT - waveH, SSD1306_BLACK);
  display.drawFastHLine(0, waveH, SCREEN_WIDTH, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  char buf[32];
  // Dòng 1: HR & SpO2
  if (health.beatAvg > 0 && health.spO2 > 0) {
    snprintf(buf, sizeof(buf), "HR:%3d SpO2:%2d%%",
             (int)health.beatAvg, (int)health.spO2);
  } else {
    snprintf(buf, sizeof(buf), "HR:--- SpO2:--%%");
  }
  display.setCursor(0, waveH + 2);
  display.print(buf);

  // Dòng 2: Nhiệt độ
  if (health.tempC > 0) {
    snprintf(buf, sizeof(buf), "Temp: %.1f C", health.tempC);
  } else {
    snprintf(buf, sizeof(buf), "Temp: --.- C");
  }
  display.setCursor(0, waveH + 11);
  display.print(buf);

  display.display();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Health Monitor v2.0 ===");

  // OLED
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED FAILED – check wiring");
    while (1) delay(500);
  }
  memset(graphBuf, SCREEN_HEIGHT / 2, sizeof(graphBuf));
  showMessage("Connecting WiFi...");

  setupWiFi();
  initFirebase();

  showMessage("Init sensors...");

  // --- MAX30105 ---
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    showMessage("MAX30105 error!", "Check wiring");
    Serial.println("MAX30105 FAILED");
    while (1) delay(500);
  }
  /**
   * Cấu hình tối ưu cho độ chính xác:
   *   ledBrightness = 0x1F (~6.4 mA) – đủ sáng, không quá nóng
   *   sampleAverage = 4  – trung bình 4 mẫu/đọc → giảm nhiễu
   *   ledMode       = 2  – Red + IR (bắt buộc cho SpO2)
   *   sampleRate    = 100 – 100 SPS
   *   pulseWidth    = 411 µs – 18-bit ADC resolution (cao nhất)
   *   adcRange      = 4096 nA – full scale
   */
  particleSensor.setup(0x1F, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0);

  // --- MAX30205 ---
  int retries = 5;
  while (retries-- > 0) {
    if (tempSensor.scanAvailableSensors()) {
      tempSensor.begin();
      tempSensorOK = true;
      Serial.println("MAX30205 OK");
      break;
    }
    Serial.printf("MAX30205 not found, retry %d...\n", 5 - retries);
    delay(2000);
  }
  if (!tempSensorOK) {
    Serial.println("MAX30205 FAILED – temperature disabled");
  }

  showMessage("Ready!", "Place finger...");
  delay(1000);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  unsigned long now = millis();
  long irValue  = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // ============================================================
  //  1. PHÁT HIỆN NGÓN TAY (hysteresis)
  // ============================================================
  if (fingerState == FINGER_OFF) {
    if (irValue > FINGER_ON_THRESHOLD) {
      fingerState = FINGER_ON;
      Serial.println("[INFO] Finger detected");
      resetMeasurement();
    } else {
      // Không có ngón tay
      if (now - lastDisplay > 500) {
        lastDisplay = now;
        showMessage("Place your finger", "   on sensor");
      }
      return;  // Không xử lý gì thêm
    }
  } else {
    if (irValue < FINGER_OFF_THRESHOLD) {
      fingerState = FINGER_OFF;
      Serial.println("[INFO] Finger removed");
      return;
    }
  }

  // ============================================================
  //  2. ĐO NHỊP TIM
  // ============================================================
  if (checkForBeat(irValue)) {
    long delta = now - lastBeatTime;
    lastBeatTime = now;

    if (delta > 0 && lastBeatTime > 0) {
      float rawHR = 60000.0f / (float)delta;   // ms → BPM

      if (rawHR >= HR_MIN && rawHR <= HR_MAX) {
        // Bỏ outlier so với nhịp trước
        float prevHR = (hrHistCount > 0)
          ? hrHistory[(hrHistIdx - 1 + HR_HISTORY_SIZE) % HR_HISTORY_SIZE]
          : rawHR;
        float useHR = (fabsf(rawHR - prevHR) > HR_OUTLIER_THR) ? prevHR : rawHR;

        // IIR filter
        hrIIR = (hrIIR < 1.0f) ? useHR : iirFilter(useHR, hrIIR, IIR_ALPHA_HR);

        // Lịch sử (bộ đệm vòng)
        hrHistory[hrHistIdx] = hrIIR;
        hrHistIdx = (hrHistIdx + 1) % HR_HISTORY_SIZE;
        if (hrHistCount < HR_HISTORY_SIZE) hrHistCount++;

        // Trung bình N nhịp gần nhất (beatAvg)
        hrRates[hrRateSpot] = (byte)constrain((int)hrIIR, 0, 255);
        hrRateSpot = (hrRateSpot + 1) % HR_RATES_SIZE;

        float avg = 0;
        for (int i = 0; i < HR_RATES_SIZE; i++) avg += hrRates[i];
        avg /= HR_RATES_SIZE;

        health.hr      = hrIIR;
        health.beatAvg = avg;

        Serial.printf("[HR] raw=%.0f filtered=%.1f avg=%.1f\n",
                      rawHR, hrIIR, avg);
      }
    }
  }

  // ============================================================
  //  3. TÍNH SPO2 (từ buffer Red/IR tích lũy)
  // ============================================================
  redBuf[spo2BufIdx] = redValue;
  irBuf[spo2BufIdx]  = irValue;
  spo2BufIdx = (spo2BufIdx + 1) % SPO2_BUF_SIZE;
  if (spo2BufIdx == 0) spo2BufFull = true;

  int sampLen = spo2BufFull ? SPO2_BUF_SIZE : spo2BufIdx;
  if (sampLen >= SPO2_MIN_SAMPLES) {
    float rawSpo2 = calculateSpO2(redBuf, irBuf, sampLen);
    if (rawSpo2 >= 80.0f) {
      health.spO2 = (health.spO2 < 1.0f)
        ? rawSpo2
        : iirFilter(rawSpo2, health.spO2, IIR_ALPHA_SPO2);
    }
  }

  // ============================================================
  //  4. ĐỌC NHIỆT ĐỘ (mỗi 2 giây)
  // ============================================================
  if (now - lastTemp > TEMP_INTERVAL_MS) {
    lastTemp = now;
    if (tempSensorOK) {
      float t = tempSensor.getTemperature() + TEMP_OFFSET;
      if (t >= TEMP_MIN && t <= TEMP_MAX) {
        health.tempC = t;
      }
    }
    // Đánh dấu dữ liệu sẵn sàng khi có HR & SpO2
    health.ready = (health.beatAvg > 0 && health.spO2 > 0);
  }

  // ============================================================
  //  5. HIỂN THỊ OLED
  // ============================================================
  if (now - lastDisplay > DISPLAY_INTERVAL_MS) {
    lastDisplay = now;
    drawWaveform(irValue);
  }

  // ============================================================
  //  6. GỬI FIREBASE
  // ============================================================
  if (now - lastFirebase > FIREBASE_INTERVAL_MS && health.ready) {
    lastFirebase = now;

    FirebaseJson json;
    json.set("HR",    (double)health.beatAvg);  // Trung bình N nhịp (ổn định)
    json.set("SpO2",  (double)health.spO2);
    json.set("Temp",  (double)health.tempC);
    json.set("ts",    (long)(now / 1000));       // Timestamp (giây từ lúc boot)

    if (Firebase.setJSON(fbData, FIREBASE_PATH, json)) {
      Serial.printf("[Firebase] OK | HR:%.0f SpO2:%.1f Temp:%.1f\n",
                    health.beatAvg, health.spO2, health.tempC);
    } else {
      Serial.println("[Firebase] Error: " + fbData.errorReason());
    }
  }
}
