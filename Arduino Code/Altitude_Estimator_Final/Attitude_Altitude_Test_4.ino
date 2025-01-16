#include "MPU6050.h"
#include "Wire.h"
#include <Adafruit_BMP280.h>
#include <BasicLinearAlgebra.h>
#include <Kalman.h>

// -------------------- Pins --------------------
const int SDA_pin = 9, SCL_pin = 10;

// -------------------- Sensor Variables --------------------
MPU6050 accelgyro;  // IMU
int16_t ax, ay, az, gx, gy, gz;
float axS, ayS, azS, gxS, gyS, gzS;
float angleAccelX, angleAccelY, denomX, denomY;
float gyroscopeRes, accelerationRes;

Adafruit_BMP280 bmp;  // Barometer
float pressure, refAltitude, altitude, AccZInertial;
float seaLevelhPa = 1016.8;  // Observations from Uppsala 2025-01-16 16:00 SNT

// -------------------- Timing & PWM --------------------
const int freq = 500, resolution = 14;
const float Ts_kalman = (float)1 / (freq * 1);
float dt_kalman;
unsigned long lastUpdateCool = millis(), lastKalmanUpdate = micros(), now = micros();

// -------------------- Kalman --------------------
Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;

// Kalman Variables
float KalmanAngleRoll = 0, KalmanAnglePitch = 0;

float AltitudeKalman, VelocityVerticalKalman;
BLA::Matrix<2, 1, float> X, G, K;  // 2x1 matrix
BLA::Matrix<2, 2, float> F, Q, P;  // 2x2 matrix
BLA::Matrix<1, 2, float> H;        // 1x2 matrix
BLA::Matrix<1, 1, float> Y, Y_tilde, R, S, U;

float AltitudeKalmaN, VelocityVerticalKalmaN;
BLA::Matrix<2, 1, float> X3, G3, K3;  // 2x1 matrix
BLA::Matrix<2, 2, float> F3, Q3, P3;  // 2x2 matrix
BLA::Matrix<1, 2, float> H3;          // 1x2 matrix
BLA::Matrix<1, 1, float> Y3, Y_tilde3, R3, S3, U3;

// Variances Kalman_2D: Height - smaller -> trusted more
float var_A = 100.0;       // Variance for state: Height (IMU)
float var_baro = 10000.0;  // Variance for measurement: Altitude (barometer)

// Variances Kalman_1D: Angle - smaller -> trusted more
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;

// -------------------- ZUPT --------------------

#define N 50  // Define the window size
float threshold, sum = 0.0, movingAverage = 0.0;;
bool zuptDetected3 = false;
float sum3 = 0.0, sumSquared3 = 0.0, samples3[N];
float squaredSamples3[N], variance3 = 0.0;
int currentIndex3 = 0;

// -------------------- Functions --------------------
void rgb(int r, int g, int b) {
  neopixelWrite(RGB_BUILTIN, r, g, b);
}

void calibrateMPU6050() {
  Serial.println("Performing IMU calibration... (~2 seconds)");
  Serial.println("Old offsets:");
  accelgyro.PrintActiveOffsets();
  accelgyro.CalibrateAccel(20);
  accelgyro.CalibrateGyro(20);
  Serial.println("\nCalibration Successful!");
}

void initializeBNP280() {
  unsigned status = bmp.begin();
  if (!status) {
    Serial.println("BNP280 connection failed...");
    rgb(20, 0, 0);
    delay(5000);
    ESP.restart();
  }
  Serial.println("BNP280 connection successful!");

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  Serial.println("Performing altitude calibration... (~5 seconds)");
  for (int i = 0; i < 5000; i++) {
    refAltitude += bmp.readAltitude(seaLevelhPa);
    delay(1);
  }
  Serial.println(refAltitude);
  int measurements = 2000;
  refAltitude = 0.0;
  for (int i = 0; i < measurements; i++) {
    refAltitude += bmp.readAltitude(seaLevelhPa);
    delay(1);
  }
  refAltitude = (refAltitude / measurements) * 100;
  Serial.print("Reference altitude set [cm]: ");
  Serial.println(refAltitude);
}

void plotCool(unsigned long currentTime) {  //Used for debugging
  if (currentTime - lastUpdateCool >= Ts_kalman * 1e6 * 10) {
    lastUpdateCool = currentTime;

    Serial.printf("AltK:%.1f,", AltitudeKalman);
    Serial.printf("velK:%.1f,", VelocityVerticalKalman);

    Serial.printf("AltK3:%.1f,", AltitudeKalmaN);
    Serial.printf("velK3:%.1f,", VelocityVerticalKalmaN);

    Serial.printf("Alt:%.1f,", altitude);
    Serial.printf("AccZ:%.1f,", AccZInertial);

    Serial.printf("dtK:%.4f\n", dt_kalman);
  }
}

// Kalman with Zero velocity update (ZUPT)
void kalman_2d(float input, float observation) {
  U = { input };
  Y = { observation };

  // Prediction
  X = F * X + G * U;
  P = F * P * ~F + Q;  // ~ = transpose

  // Update
  Y_tilde = Y - H * X;
  S = H * P * ~H + R;
  K = P * ~H * Inverse(S);
  X = X + K * Y_tilde;
  P = P - K * H * P;

  AltitudeKalman = X(0);
  VelocityVerticalKalman = X(1);
}

// Kalman with Zero velocity update (ZUPT)
void kalman_2d_ZUPT3(float input, float observation) {
  U3 = { input };
  Y3 = { observation };

  // Prediction
  X3 = F3 * X3 + G3 * U3;
  P3 = F3 * P3 * ~F3 + Q3;  // ~ = transpose

  // Update
  Y_tilde3 = Y3 - H3 * X3;
  S3 = H3 * P3 * ~H3 + R3;
  K3 = P3 * ~H3 * Inverse(S3);
  X3 = X3 + K3 * Y_tilde3;
  P3 = P3 - K3 * H3 * P3;

  // zuptDetected = MagnitudeDetector(input, 5000.0);
  zuptDetected3 = VarDetector3(200);


  if (zuptDetected3) {
    X3(1) = X3(1) * 0.98;        // Reset velocity
    P3(1, 1) = P3(1, 1) * 0.98;  // Reduce velocity uncertainty
  }

  AltitudeKalmaN = X3(0);
  VelocityVerticalKalmaN = X3(1);
}

bool VarDetector3(float threshold) {
  float input = AccZInertial;

  // Update the buffer
  float squaredInput = input * input;
  sum3 = sum3 - samples3[currentIndex3] + input;
  sumSquared3 = sumSquared3 - squaredSamples3[currentIndex3] + squaredInput;
  samples3[currentIndex3] = input;
  squaredSamples3[currentIndex3] = squaredInput;
  currentIndex3 = (currentIndex3 + 1) % N;

  // Calculate mean and variance
  float mean = sum3 / N;
  variance3 = (sumSquared3 / N) - (mean * mean);

  // Return true if variance is below threshold
  return variance3 < threshold;
}

// ============================== SETUP ==============================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_pin, SCL_pin);
  Wire.setClock(400000);
  rgb(10, 10, 0);  // LED indicate config

  kalmanX.setAngle(0);  // Set starting angle
  kalmanY.setAngle(0);

  kalmanX.setQangle(Q_angle);
  kalmanX.setQbias(Q_bias);
  kalmanX.setRmeasure(R_measure);

  kalmanY.setQangle(Q_angle);
  kalmanY.setQbias(Q_bias);
  kalmanY.setRmeasure(R_measure);

  // Initialize MPU6050
  accelgyro.initialize();                  // Arguments are the range: A2G= ±2g, G2000DPS = ±2000deg/s
  accelgyro.setFullScaleGyroRange(0x02);   // Full Scale Range: ±1000°/s
  accelgyro.setFullScaleAccelRange(0x01);  // Full Scale Range: ±4g
  // accelgyro.setDLPFMode(MPU6050_DLPF_BW_10);  // Low-Pass filter: MPU6050_DLPF_BW_10 = Bandwidth 10Hz
  accelgyro.setDLPFMode(0x00);
  gyroscopeRes = 1000.0 / 32768.0;  // Adjust based on setFullScaleGyroRange!
  accelerationRes = 4.0 / 32768.0;  // Adjust based on setFullScaleAccelRange!

  if (!accelgyro.testConnection()) {
    Serial.println("MPU6050 connection failed\nRebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("MPU6050 connection successful!");

  calibrateMPU6050();

  initializeBNP280();

  rgb(0, 0, 20);  // LED indicate config complete

  // Kalman_2D variable init
  X = { 0.0,
        0.0 };
  F = { 1.0, Ts_kalman,
        0.0, 1.0 };
  U = { 0 };
  G = { 0.5 * Ts_kalman * Ts_kalman,
        Ts_kalman };
  Q = G * var_A * ~G;  // Covariance input - Accelerometer

  Y = Y_tilde = { 0.0 };
  H = { 1.0, 0.0 };
  R = { var_baro };  // Covariance state - Barometer
  P = { 0.0, 0.0,
        0.0, 0.0 };
  S = { 0.0 };
  K = { 0.0,
        0.0 };


  // Kalman_2D 2 variable init
  X3 = { 0.0,
         0.0 };
  F3 = { 1.0, Ts_kalman,
         0.0, 1.0 };
  U3 = { 0 };
  G3 = { 0.5 * Ts_kalman * Ts_kalman,
         Ts_kalman };
  Q3 = G3 * var_A * ~G3;  // Covariance input - Accelerometer

  Y3 = Y_tilde3 = { 0.0 };
  H3 = { 1.0, 0.0 };
  R3 = { var_baro };  // Covariance state - Barometer
  P3 = { 0.0, 0.0,
         0.0, 0.0 };
  S3 = { 0.0 };
  K3 = { 0.0,
         0.0 };

  for (int i = 0; i < N; i++) {
    samples3[i] = 0.0f;
    squaredSamples3[i] = 0.0f;
  }
}

// ============================== LOOP ==============================
void loop() {
  // Kalman loop (fast)
  now = micros();
  if (now - lastKalmanUpdate >= Ts_kalman * 1e6) {
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

    KalmanAngleRoll = kalmanX.getAngle(angleAccelX, gxS, dt_kalman);
    KalmanAnglePitch = kalmanY.getAngle(angleAccelY, gyS, dt_kalman);

    AccZInertial = -sin(KalmanAnglePitch * DEG_TO_RAD) * axS + cos(KalmanAnglePitch * DEG_TO_RAD) * sin(KalmanAngleRoll * DEG_TO_RAD) * ayS + cos(KalmanAnglePitch * DEG_TO_RAD) * cos(KalmanAngleRoll * DEG_TO_RAD) * azS;
    AccZInertial = AccZInertial * 9.82 * 100;  // Gravity Sweden + convert to cm
    altitude = bmp.readAltitude(seaLevelhPa) * 100 - refAltitude;
    kalman_2d(AccZInertial - 9.82 * 100, altitude);
    kalman_2d_ZUPT3(AccZInertial - 9.82 * 100, altitude);
  }

  plotCool(now);
}
