/* 
  This code implements the control algorithm for a quadcopter drone on an ESP32, 
  including a cascaded PID controller and a Kalman filter for angle estimation. 
  It also uses ESP-NOW for wireless communication. 
  The theoretical background is described in the report at: https://github.com/ElektroJonas/Quadcopter-Control
  Created in 2024 by Jonas Gartner and Sigge Axelsson at Uppsala University in Sweden.
*/
#include "MPU6050.h"
#include <EEPROM.h>
#include "Wire.h"
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>

// -------------------- Wifi Config --------------------
#define ESPNOW_WIFI_CHANNEL 6
bool data_available = false;
unsigned long lastTransmission = 0;
uint32_t received_data;

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  ESP_NOW_Peer_Class(const uint8_t* mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t* lmk)
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  ~ESP_NOW_Peer_Class() {}

  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  void onReceive(const uint8_t* data, size_t len, bool broadcast) {
    received_data = *((uint32_t*)data);
    data_available = true;
  }

  // Public wrapper method to call the protected send()
  size_t sendData(const uint8_t* data, int len) {
    return send(data, len);
  }
};
std::vector<ESP_NOW_Peer_Class> masters;

void register_new_master(const esp_now_recv_info_t* info, const uint8_t*, int, void*) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");
    ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);
    masters.push_back(new_master);
    if (!masters.back().add_peer()) Serial.println("Failed to register the new master");
  } else Serial.printf("Received a unicast message from " MACSTR ", ignoring.\n", MAC2STR(info->src_addr));
}

// -------------------- Pins --------------------
const int M1 = 4, M2 = 5, M3 = 6, M4 = 7;
const int SDA_pin = 9, SCL_pin = 10;
const int batMeasPin = 1, calibratePin = 0;

// -------------------- Sensor Variables --------------------
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
float axS, ayS, azS, gxS, gyS, gzS;
float angleAccelX, angleAccelY, denomX, denomY;
float gyroscopeRes, accelerationRes;
int16_t offsets[6];

// -------------------- Timing & PWM --------------------
const int freq = 250, resolution = 14;
unsigned long lastUpdateCool = millis(), lastKalmanUpdate = micros(), lastUpdate = micros(), lastBatMeasurement = millis(), now = micros(), lastCriticalVoltage = millis();
float dt, dt_kalman;
const unsigned long loopTime = 1000000 / freq;
const unsigned long loopTimeKalman = loopTime / 4;

// -------------------- Motor & Control --------------------
float P1 = 0, P2 = 0, P3 = 0, P4 = 0;
int PCOM = 0, PDIFF_x = 0, PDIFF_y = 0, PDIFF_z = 0;
bool motorsOn = false;
bool hold = false;
bool criticalVoltage = false;
float batteryVoltage;
uint8_t throttle;

// PID variables ANGLE (X/Y)
float refXAng = 0, e_x_prev_angle = 0, P_x_angle = 0, I_x_angle = 0, D_x_angle = 0;
float refYAng = 0, e_y_prev_angle = 0, P_y_angle = 0, I_y_angle = 0, D_y_angle = 0;

// PID variables RATE (X/Y/Z)
float refXRate = 0, e_x_prev_rate = 0, P_x_rate = 0, I_x_rate = 0, D_x_rate = 0;
float refYRate = 0, e_y_prev_rate = 0, P_y_rate = 0, I_y_rate = 0, D_y_rate = 0;
float refZRate = 0, e_z_prev_rate = 0, P_z_rate = 0, I_z_rate = 0, D_z_rate = 0;

// PID Coefficients
float Kp_angle = 0.4, Ki_angle = 0, Kd_angle = 0;
float Kp_rate = 0.7, Ki_rate = 2.5, Kd_rate = 0.02;
float Kp_z_rate = 1, Ki_z_rate = 4, Kd_z_rate = 0;

// Kalman Variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 0;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 0;
float Q_angle, R_measure;

// -------------------- Functions --------------------
void rgb(int r, int g, int b) {
  neopixelWrite(RGB_BUILTIN, r, g, b);
}

void setParamsSerial(float& Kp, float& Ki, float& Kd) {
  unsigned long timer = millis();
  while (!Serial.available()) {
    if (millis() - timer >= 10000) {  // Safe timeout handling
      Serial.println("Timeout: No parameters received.");
      Serial.printf("Using standard parameters: Kp,Ki,Kd = %.2f,%.2f,%.2f\n", Kp, Ki, Kd);
      return;
    }
  }

  String input = Serial.readStringUntil('\n');
  input.trim();
  uint8_t vals[3];
  int n = sscanf(input.c_str(), "%hhu,%hhu,%hhu", &vals[0], &vals[1], &vals[2]);
  if (n == 3) {
    Kp = (float)vals[0] / 10;
    Ki = (float)vals[1] / 10;
    Kd = (float)vals[2] / 100;
    Serial.printf("Updated, Kp,Ki,Kd = %.2f,%.2f,%.2f\n", Kp, Ki, Kd);
  } else Serial.println("Error: Send 3 values as 10*Kp, 10*Ki, 100*Kd (0-255)");
}

void setPulseWidth(int pin, int pw, int limit = 2000) {
  pw = min(pw, limit);
  int dutyCycle = (pw * ((1 << resolution) - 1)) / loopTime;
  ledcWrite(pin, dutyCycle);
}

void getTransmission() {
  if (!data_available) return;
  lastTransmission = millis();
  throttle = (received_data >> 24) & 0xFF;
  throttle = (throttle > 55) ? throttle - 55 : 0;
  if(!hold) PCOM = map(throttle, 0, 200, 1300, 1600); //Joystick 0 position equals 138-50 => 1440 throttle
  refXAng = (float)((received_data >> 8) & 0xFF) - 138; //Sensor rotated by 90deg in late stage so refx becomes refy, therefore we shift by 8 instead of 16
  refXAng = (refXAng > -5 && refXAng < 5) ? 0 : refXAng;
  refXAng /= -4;

  refYAng = (float)((received_data >> 16) & 0xFF) - 138;
  refYAng = (refYAng > -5 && refYAng < 5) ? 0 : refYAng;
  refYAng /= 4;

  refZRate = (float)((received_data >> 4) & 0x0F) - 8;
  refZRate = (refZRate > -1 && refZRate < 1) ? 0 : refZRate*8; //Max roration rate will be 4*8 32deg/s

  if ((received_data >> 3) & 0x01) {
    EEPROM.write(0, 1);
    EEPROM.commit();
    delay(10);
    ESP.restart();
  }
  motorsOn = (received_data >> 2) & 0x01;
  hold = (received_data >> 1) & 0x01;
  data_available = false;
}

void calibrateMPU6050() {
  Serial.println("Old offsets:");
  accelgyro.PrintActiveOffsets();
  accelgyro.CalibrateAccel(20);
  accelgyro.CalibrateGyro(20);
  Serial.println("\nCalibration Successful! \nNew offsets:");
  accelgyro.PrintActiveOffsets();
  int16_t* ao = accelgyro.GetActiveOffsets();
  for (int i = 0; i < 6; i++) offsets[i] = ao[i];
  EEPROM.put(1, offsets);
  EEPROM.commit();
}

void setEEpromOffsets() {
  EEPROM.get(1, offsets);
  accelgyro.setXAccelOffset(offsets[0]);
  accelgyro.setYAccelOffset(offsets[1]);
  accelgyro.setZAccelOffset(offsets[2]);
  accelgyro.setXGyroOffset(offsets[3]);
  accelgyro.setYGyroOffset(offsets[4]);
  accelgyro.setZGyroOffset(offsets[5]);
  Serial.println("Offsets set:");
  accelgyro.PrintActiveOffsets();
}

void escSetup() {
  const int motors[] = { M1, M2, M3, M4 };
  for (int m : motors) setPulseWidth(m, 2400, 2400);
  delay(4000);
  for (int m : motors) setPulseWidth(m, 900);
  delay(4000);
}

float calculatePID(float ref, float y, float& e_prev, float& P, float& I, float& D, float Kp, float Ki, float Kd, float dt, int Icap = 400) {
  float e = ref - y;
  P = Kp * e;
  I += Ki * (e + e_prev) * dt;
  I = constrain(I, -Icap, Icap);
  D = Kd * (e - e_prev) / dt;
  e_prev = e;
  return P + I + D;
}

void plotCool(unsigned long currentTime) { //Used for debugging
  if (currentTime - lastUpdateCool >= loopTime * 50) {
    lastUpdateCool = currentTime;
    //Serial.printf("P_x:%.3f,", P_x_rate);
    //Serial.printf("I_x:%.3f,", I_x_rate);
    //Serial.printf("D_x:%.3f,", D_x_rate);

    //Serial.printf("P_y:%.3f,", P_y_rate);
    //Serial.printf("I_y:%.3f,", I_y_rate);
    //Serial.printf("D_y:%.3f,", D_y_rate);

    //Serial.printf("P_z:%.3f,", P_z_rate);
    //Serial.printf("I_z:%.3f,", I_z_rate);
    //Serial.printf("D_z:%.3f,", D_z_rate);

    //Serial.printf("angleAccelX:%.3f,", angleAccelX);
    Serial.printf("Kalman_Angle_X:%.3f,", KalmanAngleRoll);
    Serial.printf("Kalman_Angle_y:%.3f,", KalmanAnglePitch);
    //Serial.printf("axS:%.3f,", axS);
    //Serial.printf("ayS:%.3f,", ayS);
    //Serial.printf("azS:%.3f,", azS);
    //Serial.printf("gxS:%.3f,", gxS);
    //Serial.printf("gyS:%.3f,", gyS);
    //Serial.printf("gzS:%.3f\n", gzS);
    float v = analogRead(batMeasPin);
    Serial.printf("Voltage:%.3f\n", batteryVoltage); //(measurement/2^8)((3300 + 220)/220)*1.18 = measurement/adcres*(voltage division)*measuredADCREF
    Serial.printf("VoltageRaw:%.3f\n", v);
    
  }
}

void kalman_1d(float& x, float& P, float u, float y, float Q, float R, float G, float F, float H) {
  x = F * x + G * u;  // Predictions
  P = F * P * F + Q;
  float y_tilde = y - H * x;  // Updates
  float S = H * P * H + R;
  float K = P * H / S;
  x += K * y_tilde;
  P -= K * H * P;
}

// ============================== SETUP ==============================
void setup() {
  pinMode(calibratePin, INPUT);
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_2_5db);
  Wire.begin(SDA_pin, SCL_pin);
  Wire.setClock(200000);
  EEPROM.begin(16);  // initialize EEPROM (flash memory for esp32) with predefined size
  rgb(10, 10, 0);    // LED indicate config

  // Configure PWM & ESC
  if (ledcAttach(M1, freq, resolution) && ledcAttach(M2, freq, resolution) && ledcAttach(M3, freq, resolution) && ledcAttach(M4, freq, resolution))
    Serial.println("PWM setup successful");
  else {
    Serial.println("PWM setup failed\nRebooting in 5 seconds...");
    rgb(20, 0, 0);
    delay(5000);
    ESP.restart();
  }
  
  // Initialize MPU6050
  accelgyro.initialize();                     // Arguments are the range: A2G= ±2g, G2000DPS = ±2000deg/s
  accelgyro.setFullScaleGyroRange(0x03);      // Full Scale Range: ±2000°/s
  accelgyro.setFullScaleAccelRange(0x01);     // Full Scale Range: ±4g
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_10);  // Low-Pass filter: MPU6050_DLPF_BW_10 = Bandwidth 10Hz
  gyroscopeRes = 2000.0 / 32768.0;            // Adjust based on setFullScaleGyroRange!escset
  accelerationRes = 4.0 / 32768.0;            // Adjust based on setFullScaleAccelRange!

  if (!accelgyro.testConnection()) {
    Serial.println("MPU6050 connection failed\nRebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("MPU6050 connection successful!");

  for (int i = 0; i < 200; i++) {
    delay(10);
    if (!digitalRead(calibratePin)) {
      Serial.print("Calibration Button pressed B)");
      escSetup();
      break;
    }
  }

  // Sensor Calibration
  if (EEPROM.read(0)) {
    Serial.println("Performing calibration...");
    calibrateMPU6050();
    setEEpromOffsets();
    EEPROM.write(0, 0);
    EEPROM.commit();
  } else setEEpromOffsets();

  Serial.println("Initializing Wi-Fi");
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) delay(100);
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW\nRebooting in 5 seconds...");
    rgb(20, 0, 0);
    delay(5000);
    ESP.restart();
  }
  ESP_NOW.onNewPeer(register_new_master, NULL);
  Serial.println("Wifi setup complete");

  // Set Parameters With Serial Monitor
  //Serial.println("\n[RATE] Send rate roll/pitch parameters as K_p,K_i,K_d:");
  setParamsSerial(Kp_rate, Ki_rate, Kd_rate);
  //Serial.println("\n[RATE] Send rate yaw parameters as K_p,K_i,K_d:");
  setParamsSerial(Kp_z_rate, Ki_z_rate, Kd_z_rate);
  Serial.println("\n[ANGLE] Send angle roll/pitch parameters as K_p,K_i,K_d:");
  setParamsSerial(Kp_angle, Ki_angle, Kd_angle);
  Serial.println("\nParameters Set!");

  rgb(0, 0, 20);  // LED indicate config complete
}


// ============================== LOOP ==============================
void loop() {
  //Battery measurement
  now = millis();
  if (now - lastBatMeasurement >= 50 && now - lastCriticalVoltage >= 5000) {
    lastBatMeasurement = now;
    batteryVoltage = analogRead(batMeasPin)*0.00457;                                //(measurement/(2^12-1))((3300 + 220)/220)*1.19 = measurement/adcres*(voltage division)*measuredADCRef
    uint32_t message = batteryVoltage*1000;                                         //Multiply by 1000 to avoid floating point issues
    if (!masters.empty()) masters[0].sendData((uint8_t*)&message, sizeof(message)); //Send drone voltage to remote
    if (batteryVoltage < 9.4){
      criticalVoltage = true;
      lastCriticalVoltage = now;
    }
    else criticalVoltage = false;
  }


  // Kalman loop (fast)
  now = micros();
  if (now - lastKalmanUpdate >= loopTimeKalman) {
    getTransmission();
    dt_kalman = (now - lastKalmanUpdate) * 1e-6f;
    lastKalmanUpdate = now;

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axS = ax * accelerationRes;
    ayS = ay * accelerationRes;
    azS = az * accelerationRes;
    gxS = gx * gyroscopeRes;
    gyS = gy * gyroscopeRes;
    gzS = gz * gyroscopeRes;

    denomX = sqrtf(axS * axS + azS * azS);
    denomX = denomX > 1e-6f ? denomX : 1e-6f;
    denomY = sqrtf(ayS * ayS + azS * azS);
    denomY = denomY > 1e-6f ? denomY : 1e-6f;
    angleAccelX = atan2f(ayS, denomX) * RAD_TO_DEG;
    angleAccelY = -atan2f(axS, denomY) * RAD_TO_DEG;

    Q_angle = 3 * dt_kalman * dt_kalman;
    R_measure = 12;
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, gxS, angleAccelX, Q_angle, R_measure, dt_kalman, 1, 1);
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, gyS, angleAccelY, Q_angle, R_measure, dt_kalman, 1, 1);
    KalmanAngleRoll = constrain(KalmanAngleRoll, -45, 45);
    KalmanAnglePitch = constrain(KalmanAnglePitch, -45, 45);
  }

  // Control loop (limited by ESC frequency)
  now = micros();
  if (now - lastUpdate >= loopTime) {
    dt = (now - lastUpdate) * 1e-6f;
    lastUpdate = now;

    // ANGLE PID
    refXRate = calculatePID(refXAng, KalmanAngleRoll, e_x_prev_angle, P_x_angle, I_x_angle, D_x_angle, Kp_angle, Ki_angle, Kd_angle, dt);
    refYRate = calculatePID(refYAng, KalmanAnglePitch, e_y_prev_angle, P_y_angle, I_y_angle, D_y_angle, Kp_angle, Ki_angle, Kd_angle, dt);

    // RATE PID
    PDIFF_x = calculatePID(refXRate, gxS, e_x_prev_rate, P_x_rate, I_x_rate, D_x_rate, Kp_rate, Ki_rate, Kd_rate, dt);
    PDIFF_y = calculatePID(refYRate, gyS, e_y_prev_rate, P_y_rate, I_y_rate, D_y_rate, Kp_rate, Ki_rate, Kd_rate, dt);
    PDIFF_z = calculatePID(refZRate, gzS, e_z_prev_rate, P_z_rate, I_z_rate, D_z_rate, Kp_z_rate, Ki_z_rate, Kd_z_rate, dt);

    // Compute motor power
    if (!criticalVoltage){ //Battery voltage at stable levels    
      P1 = PCOM + PDIFF_x - PDIFF_y + PDIFF_z;
      P2 = PCOM - PDIFF_x - PDIFF_y - PDIFF_z;
      P3 = PCOM + PDIFF_x + PDIFF_y - PDIFF_z;
      P4 = PCOM - PDIFF_x + PDIFF_y + PDIFF_z;
    }
    else //Battery voltage at critical levels
    {
      P1 = 1400 + PDIFF_x/2 - PDIFF_y/2 + PDIFF_z/2;
      P2 = 1400 - PDIFF_x/2 - PDIFF_y/2 - PDIFF_z/2;
      P3 = 1400 + PDIFF_x/2 + PDIFF_y/2 - PDIFF_z/2;
      P4 = 1400 - PDIFF_x/2 + PDIFF_y/2 + PDIFF_z/2;
    }


    if (!motorsOn) {
      rgb(0, 0, 20);
      P1 = P2 = P3 = P4 = 900;  // Minimum pulse width
      P_x_rate = I_x_rate = D_x_rate = 0;
      P_y_rate = I_y_rate = D_y_rate = 0;
      P_z_rate = I_z_rate = D_z_rate = 0;
      P_x_angle = I_x_angle = D_x_angle = 0;
      P_y_angle = I_y_angle = D_y_angle = 0;
    } else rgb(0, 20, 0);

    setPulseWidth(M1, (int)P1);
    setPulseWidth(M2, (int)P2);
    setPulseWidth(M3, (int)P3);
    setPulseWidth(M4, (int)P4);

    plotCool(now);
  }
}
