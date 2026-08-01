// ESP32 Smart Cane - DFPlayer voice warnings, haptics, and fall detection
#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Hardware Nodes
Adafruit_MPU6050 mpu;
HardwareSerial dfPlayerSerial(2);
DFRobotDFPlayerMini dfPlayer;

// Hardwired Pin Infrastructure (Matches Your Setup Exactly)
const int PIN_TRIG_BOTTOM = 12;
const int PIN_ECHO_BOTTOM = 13;
const int PIN_TRIG_TOP = 14;
const int PIN_ECHO_TOP = 27;
const int PIN_MOTOR = 18; // Haptics: Ground obstacles ONLY

// DFPlayer Mini UART. Connect ESP32 TX through a 1 kOhm resistor to DFPlayer RX.
const int PIN_DFPLAYER_RX = 16; // ESP32 RX  <- DFPlayer TX
const int PIN_DFPLAYER_TX = 17; // ESP32 TX  -> DFPlayer RX
const int PIN_DFPLAYER_BUSY = 19; // ESP32 input <- DFPlayer BUSY (LOW while audio plays)

// Enter your Wi-Fi details and an HTTPS webhook URL before uploading.
// Example IFTTT URL: https://maker.ifttt.com/trigger/fall_detected/with/key/YOUR_WEBHOOK_KEY
const char *WIFI_SSID = "Maurice64bit";
const char *WIFI_PASSWORD = "pineapple";
const char *FALL_WEBHOOK_URL = "https://maker.ifttt.com/trigger/cane_fall/with/key/nnjsrGoUdBTAdDhmFUjwhxBz7gDHPO1obV-Ep11IAcr";

// PWM Channels for Haptics Motor
const int pwmChannel = 0;
const int pwmFrequency = 5000;  // 5 kHz works great for DC haptic discs
const int pwmResolution = 8;

// Navigation Matrix Constants (in centimeters)
const int MAX_ALERT_DISTANCE = 200; // First voice warning at 2 metres
const int MID_ZONE_DISTANCE = 100;  // Second voice warning at 1 metre
const int CRITICAL_DISTANCE = 40;   // Final warning when very close
const int HEIGHT_TOLERANCE = 20;  // Allowed variance between sensors to verify size

// Copy these files to the root of the DFPlayer microSD card:
// 0001.mp3 = obstacle/person at 2 metres
// 0002.mp3 = obstacle/person at 1 metre
// 0003.mp3 = obstacle/person very close
// 0004.mp3 = emergency fall alert
const uint8_t TRACK_TWO_METRES = 1;
const uint8_t TRACK_ONE_METRE = 2;
const uint8_t TRACK_VERY_CLOSE = 3;
const uint8_t TRACK_FALL_EMERGENCY = 4;

// MPU6050 Fall Detection Settings
const float FALL_IMPACT_THRESHOLD = 12.0;   // Hard impact drop spike (~2.8G)
const float TILT_CHANGE_THRESHOLD = 0.70;   // ~45 degree tilt shift away from baseline posture
const unsigned long FALL_CONFIRM_TIME = 1500; // Time cane must remain flat to trigger emergency

// System State Variables
bool impactDetected = false;
bool isFallen = false;
bool hasPlayedFallAlert = false;
bool fallNotificationSent = false;
bool fallNotificationAttempted = false;
unsigned long impactTimestamp = 0;
unsigned long lastFallNotificationAttempt = 0;
const unsigned long FALL_NOTIFICATION_RETRY_DELAY = 30000;

// Bit flags remember announcements during the current obstacle encounter.
// They reset only after the obstacle is gone, preventing MP3s from looping.
uint8_t announcedWarnings = 0;
const uint8_t WARNING_TWO_METRES = 1 << 0;
const uint8_t WARNING_ONE_METRE = 1 << 1;
const uint8_t WARNING_VERY_CLOSE = 1 << 2;
const unsigned long WARNING_RESET_DELAY = 1200;
unsigned long lastTallObstacleTime = 0;
uint8_t queuedAudioTrack = 0;

// Upright Calibration Vectors
float baseNormalX = 0.0, baseNormalY = 0.0, baseNormalZ = 1.0;

bool isAudioPlaying() {
  return digitalRead(PIN_DFPLAYER_BUSY) == LOW;
}

void processQueuedAudio() {
  if (queuedAudioTrack == 0 || isAudioPlaying()) return;

  dfPlayer.play(queuedAudioTrack);
  queuedAudioTrack = 0;
}

void queueWarningOnce(uint8_t warningFlag, uint8_t track) {
  if ((announcedWarnings & warningFlag) != 0) return;

  // A closer warning replaces a queued less-urgent warning, but never interrupts speech.
  queuedAudioTrack = track;
  announcedWarnings |= warningFlag;
}

void setTallObstacleVibration(long distance) {
  // 2 m: one short pulse; 1 m: quicker, longer pulses; very close: continuous.
  unsigned long phase = millis();
  if (distance <= CRITICAL_DISTANCE) {
    ledcWriteChannel(pwmChannel, 255);
  } else if (distance <= MID_ZONE_DISTANCE) {
    ledcWriteChannel(pwmChannel, (phase % 450 < 180) ? 210 : 0);
  } else {
    ledcWriteChannel(pwmChannel, (phase % 1000 < 100) ? 170 : 0);
  }
}

bool notificationsConfigured() {
  return strcmp(WIFI_SSID, "YOUR_WIFI_NAME") != 0 &&
         strstr(FALL_WEBHOOK_URL, "YOUR_WEBHOOK_KEY") == nullptr;
}

void connectToWiFi() {
  if (!notificationsConfigured()) {
    Serial.println("Wi-Fi/webhook is not configured; fall notifications are disabled.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  unsigned long startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " connected." : " unavailable; will retry after a fall.");
}

bool sendFallNotification() {
  if (!notificationsConfigured()) return false;

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    return false;
  }

  WiFiClientSecure secureClient;
  secureClient.setInsecure(); // Required for a generic HTTPS webhook without a bundled certificate.
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);
  if (!http.begin(secureClient, FALL_WEBHOOK_URL)) return false;

  http.addHeader("Content-Type", "application/json");
  int responseCode = http.POST("{\"value1\":\"Fall detected\",\"value2\":\"Smart walking stick\",\"value3\":\"Please check on the user immediately\"}");
  http.end();

  Serial.printf("Fall notification response: %d\n", responseCode);
  return responseCode >= 200 && responseCode < 300;
}

void processIMUData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float totalAccel = sqrt(a.acceleration.x * a.acceleration.x +
                          a.acceleration.y * a.acceleration.y +
                          a.acceleration.z * a.acceleration.z);

  if (totalAccel > FALL_IMPACT_THRESHOLD && !impactDetected && !isFallen) {
    impactDetected = true;
    impactTimestamp = millis();
  }

  if (impactDetected || isFallen) {
    if (totalAccel > 0.1) {
      float currentNormalX = a.acceleration.x / totalAccel;
      float currentNormalY = a.acceleration.y / totalAccel;
      float currentNormalZ = a.acceleration.z / totalAccel;

      float dotProduct = (currentNormalX * baseNormalX) + (currentNormalY * baseNormalY) + (currentNormalZ * baseNormalZ);

      if (isFallen) {
        if (dotProduct > 0.85) {
          isFallen = false;
          impactDetected = false;
          ledcWriteChannel(pwmChannel, 0);
          hasPlayedFallAlert = false;
          fallNotificationSent = false;
          fallNotificationAttempted = false;
          announcedWarnings = 0;
        }
      }
      else if (impactDetected) {
        if (dotProduct < TILT_CHANGE_THRESHOLD) {
          if (millis() - impactTimestamp > FALL_CONFIRM_TIME) {
            isFallen = true;
          }
        } else {
          if (millis() - impactTimestamp > FALL_CONFIRM_TIME) {
            impactDetected = false;
          }
        }
      }
    }
  }
}

long measureDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 15000);
  if (duration == 0) return 400;
  return duration * 0.0343 / 2;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_TRIG_BOTTOM, OUTPUT);
  pinMode(PIN_ECHO_BOTTOM, INPUT);
  pinMode(PIN_TRIG_TOP, OUTPUT);
  pinMode(PIN_ECHO_TOP, INPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_DFPLAYER_BUSY, INPUT_PULLUP);

  ledcAttachChannel(PIN_MOTOR, pwmFrequency, pwmResolution, pwmChannel);
  ledcWriteChannel(pwmChannel, 0);
  dfPlayerSerial.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);
  if (!dfPlayer.begin(dfPlayerSerial, false, false)) {
    Serial.println("DFPlayer Mini not found. Check its power, microSD card, and UART wiring.");
  } else {
    dfPlayer.volume(24); // 0-30; adjust to suit the speaker and enclosure.
  }
  connectToWiFi();
  Wire.begin();

  if (!mpu.begin()) {
    while (1) {
      ledcWriteChannel(pwmChannel, 255);
      delay(80);
      ledcWriteChannel(pwmChannel, 0);
      delay(200);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 20; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    delay(20);
  }

  float magnitude = sqrt(sumX * sumX + sumY * sumY + sumZ * sumZ);
  if (magnitude > 0) {
    baseNormalX = sumX / magnitude;
    baseNormalY = sumY / magnitude;
    baseNormalZ = sumZ / magnitude;
  }
}

void loop() {
  unsigned long currentTime = millis();
  processIMUData();

  // ============================================================================
  // OVERRIDE MATRIX: FALL EMERGENCY STATE
  // ============================================================================
  if (isFallen) {
    ledcWriteChannel(pwmChannel, 0);
    if (!hasPlayedFallAlert) {
      queuedAudioTrack = 0;
      if (isAudioPlaying()) dfPlayer.stop(); // Emergency audio has priority.
      dfPlayer.play(TRACK_FALL_EMERGENCY);
      hasPlayedFallAlert = true;
    }

    if (!fallNotificationSent &&
        (!fallNotificationAttempted || currentTime - lastFallNotificationAttempt >= FALL_NOTIFICATION_RETRY_DELAY)) {
      fallNotificationAttempted = true;
      lastFallNotificationAttempt = currentTime;
      fallNotificationSent = sendFallNotification();
    }
    return;
  }

  // ============================================================================
  // NORMAL OPERATION: RADAR MATRIX PROCESSING
  // ============================================================================
  long distanceBottom = measureDistance(PIN_TRIG_BOTTOM, PIN_ECHO_BOTTOM);
  delay(25);
  long distanceTop = measureDistance(PIN_TRIG_TOP, PIN_ECHO_TOP);

  if (distanceBottom < MAX_ALERT_DISTANCE) {
    bool isTallObstacle = (distanceTop < MAX_ALERT_DISTANCE && abs(distanceBottom - distanceTop) <= HEIGHT_TOLERANCE);

    if (isTallObstacle) {
      lastTallObstacleTime = currentTime;
      setTallObstacleVibration(distanceBottom);

      if (distanceBottom <= CRITICAL_DISTANCE) {
        queueWarningOnce(WARNING_VERY_CLOSE, TRACK_VERY_CLOSE);
      }
      else if (distanceBottom <= MID_ZONE_DISTANCE) {
        queueWarningOnce(WARNING_ONE_METRE, TRACK_ONE_METRE);
      }
      else {
        queueWarningOnce(WARNING_TWO_METRES, TRACK_TWO_METRES);
      }
    }
    else {
      // Do not clear on a single missed top-sensor echo; that would replay audio.
      if (currentTime - lastTallObstacleTime > WARNING_RESET_DELAY) {
        announcedWarnings = 0;
      }

      int constrainedDistance = constrain(distanceBottom, CRITICAL_DISTANCE, MAX_ALERT_DISTANCE);
      int motorIntensity = map(constrainedDistance, MAX_ALERT_DISTANCE, CRITICAL_DISTANCE, 60, 255);
      ledcWriteChannel(pwmChannel, motorIntensity);
    }
  }
  else {
    ledcWriteChannel(pwmChannel, 0);
    if (currentTime - lastTallObstacleTime > WARNING_RESET_DELAY) {
      announcedWarnings = 0;
    }
  }

  processQueuedAudio();
  delay(20);
}
