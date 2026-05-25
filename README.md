# Health Monitor v4.0

**ESP32 · FreeRTOS · MAX30105 · MAX30205 · SSD1306 · Firebase**

Hệ thống theo dõi nhịp tim, SpO2 và nhiệt độ cơ thể theo thời gian thực, chạy trên ESP32 với kiến trúc FreeRTOS dual-core. Dữ liệu được lọc nhiễu nhiều tầng, hiển thị sóng Pleth trên OLED 128×64, và đồng bộ lên Firebase Realtime Database mỗi 2 giây.

> ⚠️ **Tuyên bố miễn trách nhiệm:** Thiết bị không phải dụng cụ y tế. Các chỉ số chỉ mang tính tham khảo, không dùng để chẩn đoán bệnh.

---

## Mục lục

- [Tính năng](#tính-năng)
- [Kiến trúc FreeRTOS](#kiến-trúc-freertos)
- [Phần cứng](#phần-cứng)
- [Sơ đồ kết nối](#sơ-đồ-kết-nối)
- [Thư viện](#thư-viện)
- [Cài đặt](#cài-đặt)
- [Cấu trúc dữ liệu Firebase](#cấu-trúc-dữ-liệu-firebase)
- [Tối ưu kỹ thuật](#tối-ưu-kỹ-thuật)
- [Xử lý sự cố](#xử-lý-sự-cố)

---

## Tính năng

- **Dual-core thực sự:** Core 0 xử lý WiFi/Firebase, Core 1 xử lý cảm biến và OLED — không còn giật lag khi gửi dữ liệu.
- **FIFO reading MAX30105:** Drain toàn bộ FIFO mỗi chu kỳ, không bỏ lỡ mẫu giữa các lần gọi như polling thông thường.
- **Circular waveform buffer:** O(1) push và get, không `memmove()`.
- **Lọc tín hiệu nhiều tầng:** Bỏ outlier → IIR → trung bình vòng 8 nhịp.
- **SpO2 chính xác:** Tính tỷ số AC/DC (RMS) của Red và IR, không dùng raw ratio.
- **Phát hiện ngón tay hysteresis:** Ngưỡng ON/OFF tách biệt, tránh trạng thái lật liên tục.
- **Wire mutex:** MAX30105, MAX30205 và SSD1306 dùng chung I2C bus một cách an toàn.

---

## Kiến trúc FreeRTOS

```
Core 0                          Core 1
──────────────────────          ──────────────────────────────────
firebaseTask  (Priority 1)      sensorTask   (Priority 3)
  Đọc healthMutex mỗi 2s          FIFO drain MAX30105
  Firebase.setJSON()               checkForBeat() → HR
  vTaskDelayUntil()                SpO2 AC/DC RMS
                                   MAX30205 mỗi 2s
                                   → write healthMutex
                                   → push irQueue

                                displayTask  (Priority 2)
                                   Drain irQueue → CircBuf
                                   Vẽ sóng IR (không memmove)
                                   Hiển thị HR, SpO2, Temp
                                   vTaskDelayUntil(100ms)

        ┌─────────────────────────────────┐
        │  healthMutex  (HealthData)      │  ← chia sẻ giữa 3 tasks
        │  irQueue      (long, 64 slots)  │  ← sensor → display
        │  wireMutex    (I2C bus)         │  ← sensor + display
        └─────────────────────────────────┘
```

`firebaseTask` bắt buộc ở Core 0 vì WiFi stack của ESP32 chạy trên Core 0. Đặt Firebase ở Core 1 có thể gây deadlock khi chờ WiFi.

---

## Phần cứng

| Linh kiện | Số lượng | Ghi chú |
|---|---|---|
| ESP32 Dev Board | 1 | WROOM, WROVER hoặc tương đương |
| MAX30105 | 1 | Đo nhịp tim + SpO2 |
| MAX30205 | 1 | Đo nhiệt độ cơ thể (I2C) |
| SSD1306 OLED 128×64 | 1 | I2C |
| Dây nối, breadboard | — | |

---

## Sơ đồ kết nối

Tất cả thiết bị dùng chung bus I2C:

| Chân thiết bị | ESP32 |
|---|---|
| VCC (tất cả) | 3.3V |
| GND (tất cả) | GND |
| SDA (tất cả) | GPIO 21 |
| SCL (tất cả) | GPIO 22 |

Địa chỉ I2C mặc định:

| Thiết bị | Địa chỉ |
|---|---|
| SSD1306 OLED | `0x3C` |
| MAX30205 | `0x48` |
| MAX30105 | `0x57` |

> Nếu OLED không hiển thị, dùng sketch quét I2C để xác nhận địa chỉ thực tế.

---

## Thư viện

Cài qua **Library Manager** (Arduino IDE) hoặc khai báo trong `platformio.ini`:

| Thư viện | Nguồn |
|---|---|
| SparkFun MAX30105 | `sparkfun/SparkFun MAX3010x Pulse and Proximity Sensor Library` |
| Protocentral MAX30205 | `protocentral/Protocentral MAX30205` |
| Adafruit SSD1306 | `adafruit/Adafruit SSD1306` |
| Adafruit GFX | `adafruit/Adafruit GFX Library` |
| Firebase ESP32 Client | `mobizt/Firebase ESP32 Client` |
| WiFiManager | `tzapu/WiFiManager` |

**`platformio.ini` mẫu:**

```ini
[env:esp32dev]
platform  = espressif32
board     = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    sparkfun/SparkFun MAX30105 Pulse Oximeter and Heart Rate Sensor@^1.1.3
    protocentral/Protocentral MAX30205@^1.0.0
    adafruit/Adafruit SSD1306@^2.5.7
    adafruit/Adafruit GFX Library@^1.11.5
    mobizt/Firebase ESP32 Client@^4.3.9
    tzapu/WiFiManager@^2.0.16-rc2
```

---

## Cài đặt

### 1. Cấu hình Firebase

Sửa 2 hằng số trong file `.ino`:

```cpp
#define DATABASE_URL    "https://your-project-default-rtdb.firebaseio.com/"
#define DATABASE_SECRET "your_database_secret"
```

### 2. Nạp code

Nạp code lên ESP32 qua Arduino IDE hoặc PlatformIO.

### 3. Kết nối WiFi lần đầu

ESP32 tạo Access Point tên `ESP32_HealthMonitor`. Kết nối vào AP đó bằng điện thoại hoặc laptop, chọn mạng WiFi và nhập mật khẩu. Thông tin WiFi được lưu lại, các lần sau tự kết nối.

### 4. Kiểm tra

Đặt ngón tay lên MAX30105. OLED hiển thị sóng Pleth và các chỉ số. Mở Serial Monitor (115200 baud) để theo dõi log từng task.

---

## Cấu trúc dữ liệu Firebase

Dữ liệu ghi vào đường dẫn `/Health`, cập nhật mỗi 2 giây khi có ngón tay và đủ dữ liệu:

```json
{
  "HR":   78.0,
  "SpO2": 97.2,
  "Temp": 36.8,
  "ts":   1698765432
}
```

| Trường | Kiểu | Mô tả |
|---|---|---|
| `HR` | float | Trung bình 8 nhịp gần nhất (BPM) |
| `SpO2` | float | Độ bão hòa oxy máu, 80–100% |
| `Temp` | float | Nhiệt độ cơ thể (°C) |
| `ts` | int | Giây tính từ khi ESP32 khởi động (`millis()/1000`) |

Nếu ngón tay chưa đặt hoặc HR/SpO2 chưa ổn định (= 0), dữ liệu không được gửi.

---

## Tối ưu kỹ thuật

### FIFO reading MAX30105

Thay vì `getIR()` / `getRed()` (polling, bỏ mẫu), code drain toàn bộ FIFO mỗi chu kỳ:

```cpp
particleSensor.check();                  // nạp FIFO vào internal buffer
while (particleSensor.available()) {
    long ir  = particleSensor.getFIFOIR();
    long red = particleSensor.getFIFORed();
    particleSensor.nextSample();         // advance pointer
    // xử lý ir, red...
}
```

MAX30105 cấu hình 100 SPS, sample average 4 → ~25 entries/giây vào FIFO 32 slots. `sensorTask` chạy 200Hz (`vTaskDelay(5ms)`) — không bao giờ overflow.

### Circular waveform buffer

```cpp
// Push O(1) — không copy
buf[head] = pt;
head = (head + 1) % SCREEN_WIDTH;

// Get O(1) — index vòng
int16_t get(int i) { return buf[(head + i) % SCREEN_WIDTH]; }
```

So sánh với `memmove()` cũ: 10 FPS × 127 × 2 bytes = ~2.5 KB/s copy không cần thiết trên Core 1.

### Lọc nhịp tim

```
rawHR → bỏ outlier (|Δ| > 20 BPM) → IIR α=0.25 → lưu history
beatAvg = trung bình 8 nhịp gần nhất → gửi Firebase
```

### Tính SpO2

```
DC  = mean(buffer 100 mẫu)
AC  = RMS(buffer - DC)
R   = (AC_red / DC_red) / (AC_ir / DC_ir)
SpO2 = 104 - 17×R  →  IIR α=0.10
```

---

## Xử lý sự cố

| Hiện tượng | Nguyên nhân / Cách xử lý |
|---|---|
| OLED trống | Sai địa chỉ I2C — dùng sketch quét I2C kiểm tra |
| Không vào WiFi | Kết nối vào AP `ESP32_HealthMonitor`, cấu hình lại |
| HR/SpO2 = 0 mãi | Chưa đặt ngón tay đúng cách hoặc LED brightness quá thấp |
| MAX30205 không đọc được | Code tự retry 5 lần — kiểm tra hàn SDA/SCL |
| Firebase lỗi auth | Sai `DATABASE_SECRET` hoặc `DATABASE_URL` |
| ESP32 reset liên tục | Stack task quá nhỏ — tăng `*_STACK` trong code |
| Sóng OLED không mượt | Tăng `IR_QUEUE_LEN` hoặc giảm tần suất ghi Serial |
| Nhiệt độ lệch | Điều chỉnh `TEMP_OFFSET` (mặc định `0.3°C`) |

---

## Giấy phép

Mã nguồn mở, miễn phí cho mục đích học tập và nghiên cứu cá nhân.
