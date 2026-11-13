#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========== KHAI BÁO CHÂN ==========
#define LED_PIN 25
#define BUZZER_PIN 26

// ========== WIFI & MQTT ==========
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqttServer = "broker.emqx.io"; // Dùng EMQX broker công cộng cho ổn định
const int mqttPort = 1883;

const char* topicData = "fall/nhom01/data";
const char* topicControl = "fall/nhom01/control";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Adafruit_MPU6050 mpu;

// ========== CẤU TRÚC DỮ LIỆU & QUEUE ==========
struct SensorData {
  float ax, ay, az, total;
  float roll, pitch;
  unsigned long timestamp; // Lưu thời gian đọc để tính toán ngã
};

QueueHandle_t dataQueue;
QueueHandle_t alertQueue;

// ========== BIẾN TOÀN CỤC (TRẠNG THÁI) ==========
volatile bool manualMode = false;
volatile bool manualLed = false;
volatile bool manualBuzzer = false;
volatile bool fallDetected = false;

// Trạng thái thuật toán té ngã
enum FallState { NORMAL, FREE_FALL, IMPACT, ALERT_STATE };
FallState state = NORMAL;
float oldRoll = 0, oldPitch = 0;
unsigned long stateTime = 0;

// ========== HÀM CALLBACK NHẬN LỆNH TỪ WEB ==========
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();
  Serial.println("MQTT nhận lệnh: " + message);

  if (message == "MODE_MANUAL") manualMode = true;
  else if (message == "MODE_AUTO") {
    manualMode = false;
    manualLed = false;
    manualBuzzer = false;
  }
  else if (message == "RESET") {
    fallDetected = false;
    state = NORMAL;
    manualLed = false;
    manualBuzzer = false;
    Serial.println("ĐÃ XÓA CẢNH BÁO!");
  }
  else if (manualMode) {
    if (message == "LED_ON") manualLed = true;
    else if (message == "LED_OFF") manualLed = false;
    else if (message == "BUZ_ON") manualBuzzer = true;
    else if (message == "BUZ_OFF") manualBuzzer = false;
  }
}

// ---------------------------------------------------------
// TASK 1: TRUYỀN DATA (CORE 0) - WiFi & MQTT
// ---------------------------------------------------------
void TaskTransmit(void *pvParameters) {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  
  mqtt.setServer(mqttServer, mqttPort);
  mqtt.setCallback(mqttCallback);
  
  SensorData dataToSend;
  char payload[300];

  for (;;) {
    if (!mqtt.connected()) {
      String clientId = "ESP32-FALL-" + String(random(0xffff), HEX);
      if (mqtt.connect(clientId.c_str())) {
        mqtt.subscribe(topicControl);
      }
    }
    mqtt.loop();

    // Lấy dữ liệu từ dataQueue và gửi lên Web (Giới hạn 1 giây gửi 1 lần để không nghẽn)
    if (xQueueReceive(dataQueue, &dataToSend, 0) == pdPASS) {
      static unsigned long lastSend = 0;
      if (millis() - lastSend >= 1000) {
        lastSend = millis();
        snprintf(payload, sizeof(payload),
          "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"fall\":%s,\"mode\":\"%s\"}",
          dataToSend.ax, dataToSend.ay, dataToSend.az, dataToSend.pitch, dataToSend.roll,
          fallDetected ? "true" : "false", manualMode ? "MANUAL" : "AUTO"
        );
        if (mqtt.connected()) {
          mqtt.publish(topicData, payload);
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Nhường CPU
  }
}

// ---------------------------------------------------------
// TASK 2: ĐỌC DATA CẢM BIẾN (CORE 1)
// ---------------------------------------------------------
void TaskReadData(void *pvParameters) {
  if (!mpu.begin()) {
    Serial.println("LỖI MPU6050!");
    while (1) vTaskDelay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  sensors_event_t a, g, temp;
  SensorData sData;

  for (;;) {
    mpu.getEvent(&a, &g, &temp);
    
    sData.timestamp = millis();
    sData.ax = a.acceleration.x / 9.80665;
    sData.ay = a.acceleration.y / 9.80665;
    sData.az = a.acceleration.z / 9.80665;
    sData.total = sqrt(sData.ax * sData.ax + sData.ay * sData.ay + sData.az * sData.az);
    sData.roll = atan2(sData.ay, sData.az) * 180.0 / PI;
    sData.pitch = atan2(-sData.ax, sqrt(sData.ay * sData.ay + sData.az * sData.az)) * 180.0 / PI;

    // Gửi bản sao dữ liệu vào cả 2 Queue
    xQueueOverwrite(dataQueue, &sData); // Dùng cho Task gửi WiFi
    xQueueOverwrite(alertQueue, &sData); // Dùng cho Task phân tích ngã (cần độ trễ bằng 0)

    vTaskDelay(50 / portTICK_PERIOD_MS); // Đọc 20 lần/giây
  }
}

// ---------------------------------------------------------
// HÀM TOÁN HỌC TRỢ GIÚP CHO TÍNH GÓC
// ---------------------------------------------------------
float angleDifference(float a, float b) {
  float d = fabs(a - b);
  return d > 180 ? 360 - d : d;
}

// ---------------------------------------------------------
// TASK 3: CẢNH BÁO & PHÂN TÍCH TÉ NGÃ (CORE 1)
// ---------------------------------------------------------
void TaskAlert(void *pvParameters) {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  SensorData alertData;

  for (;;) {
    if (xQueueReceive(alertQueue, &alertData, portMAX_DELAY) == pdPASS) {
      
      // THUẬT TOÁN MÁY TRẠNG THÁI (Lấy từ code của bạn)
      if (!fallDetected && !manualMode) {
        switch (state) {
          case NORMAL:
            if (alertData.total >= 0.8 && alertData.total <= 1.2) {
              oldRoll = alertData.roll; oldPitch = alertData.pitch;
            }
            if (alertData.total < 0.5) { // Rơi tự do
              state = FREE_FALL; stateTime = alertData.timestamp;
              Serial.println("NGHI NGỜ RƠI TỰ DO!");
            }
            break;

          case FREE_FALL:
            if (alertData.total > 2.5 && (alertData.timestamp - stateTime <= 1500)) {
              state = IMPACT; stateTime = alertData.timestamp;
              Serial.println("PHÁT HIỆN VA CHẠM!");
            } else if (alertData.timestamp - stateTime > 1500) {
              state = NORMAL;
            }
            break;

          case IMPACT:
            if (angleDifference(alertData.roll, oldRoll) >= 45 || angleDifference(alertData.pitch, oldPitch) >= 45) {
              state = ALERT_STATE;
              fallDetected = true;
              Serial.println("CẢNH BÁO: TÉ NGÃ!");
            } else if (alertData.timestamp - stateTime > 2000) {
              state = NORMAL;
            }
            break;
            
          case ALERT_STATE:
            break;
        }
      }

      // ĐIỀU KHIỂN THIẾT BỊ CHẤP HÀNH (Đèn/Còi)
      if (manualMode) {
        digitalWrite(LED_PIN, manualLed ? HIGH : LOW);
        digitalWrite(BUZZER_PIN, manualBuzzer ? HIGH : LOW);
      } else {
        digitalWrite(LED_PIN, fallDetected ? HIGH : LOW);
        digitalWrite(BUZZER_PIN, fallDetected ? HIGH : LOW);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Tạo hàng đợi (Kích thước 1 phần tử vì ta dùng xQueueOverwrite để lấy data mới nhất)
  dataQueue = xQueueCreate(1, sizeof(SensorData));
  alertQueue = xQueueCreate(1, sizeof(SensorData));

  // Gán Task vào các Core theo đúng đề bài
  xTaskCreatePinnedToCore(TaskTransmit, "TruyenData", 4096, NULL, 1, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(TaskReadData, "DocData", 4096, NULL, 2, NULL, 1);    // Core 1
  xTaskCreatePinnedToCore(TaskAlert, "CanhBao", 4096, NULL, 3, NULL, 1);       // Core 1
}

void loop() {
}