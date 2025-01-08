#include <esp_now.h>
#include <WiFi.h>
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

// ==================== CONFIGURATION ====================
#define ESPNOW_WIFI_CHANNEL 6

// Pin definitions
const int joyLX = 2;
const int joyLY = 1;
const int joyRX = 3;
const int joyRY = 4;
const int joyLC = 6;
const int joyRC = 5;
const int buttL = 9;
const int buttR = 7;
const int speaker = 10;
const int cBatRed = 21;
const int cBatGreen = 20;
const int dBatRed = 18;
const int dBatGreen = 19;
const int batMeas = 0;

// Variables
int timer1 = 0;
int timer2 = 0;
bool beepFlag = true;
bool motorsOn = false;
uint32_t data;
bool mute = false;

// Transmission variables
uint8_t throttle = 0;
uint8_t ref_x = 0;
uint8_t ref_y = 0;
uint8_t ref_z = 0;
bool calibration = false;
bool hold = false;
bool unused2 = false;

// Reception variables
float droneBatteryLevel = 12.0;
bool data_avaliable = false;

// Define broadcast address (all 0xFF)
uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Forward declaration of functions
void rgb(int r, int g, int b);
void beep(bool beep);
void checkDroneBattery();
float checkControllerBattery();
bool transmit();

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (len == sizeof(uint32_t)) 
  {
    droneBatteryLevel = (float)*((uint32_t*)data)/1000.0; //We multiply by 1000 before sending so that we can represent our 3 decimal float as a uint32_t
  }
}

void rgb(int r, int g, int b) {
  neopixelWrite(RGB_BUILTIN, r, g, b);  //values 0 to 255
}

void setup() {
  analogReadResolution(8);

  pinMode(buttL, INPUT);
  pinMode(buttR, INPUT);
  pinMode(speaker, OUTPUT);
  pinMode(joyLC, INPUT);
  pinMode(joyRC, INPUT);

  if(digitalRead(buttL) == LOW) { // Enter boot mode
    pinMode(cBatRed, INPUT);
    pinMode(cBatGreen, INPUT);
    pinMode(dBatRed, INPUT);
    pinMode(dBatGreen, INPUT);
    while(1){}; // Stuck here until reset
  } else {
    pinMode(cBatRed, OUTPUT);
    pinMode(cBatGreen, OUTPUT);
    pinMode(dBatRed, OUTPUT);
    pinMode(dBatGreen, OUTPUT);
  }

  // Initialize Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Rebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataReceive);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    Serial.println("Rebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Setup complete.");
  rgb(2, 2, 2);
}

void beep(bool beepOn) {
  beepFlag = !beepFlag;
  if(mute)
  {
    digitalWrite(speaker,LOW);
    return;
  }
  else if (beepOn) {
    digitalWrite(speaker,beepFlag);
  } else {
    digitalWrite(speaker,LOW);
  }
}

void checkDroneBattery() {
  if (droneBatteryLevel < 9.4) {
    digitalWrite(dBatRed,beepFlag); // Use beepFlag to blink
    digitalWrite(dBatGreen,LOW);
    beep(true);
  } else if (droneBatteryLevel < 11.5) {
    digitalWrite(dBatRed,HIGH);
    digitalWrite(dBatGreen,HIGH);
    beep(false);
  } else {
    digitalWrite(dBatRed,LOW);
    digitalWrite(dBatGreen,HIGH);
    beep(false);
  }
}

float checkControllerBattery() {
  float voltage = 2 * analogRead(batMeas) * (3.3/255); // Adjust based on actual circuit
  if (voltage < 4.4) {
    digitalWrite(cBatRed,beepFlag);
    digitalWrite(cBatGreen,LOW);
    beep(true);
  } else if (voltage < 5.4) {
    digitalWrite(cBatRed,HIGH);
    digitalWrite(cBatGreen,HIGH);
    beep(false);
  } else {
    digitalWrite(cBatRed,LOW);
    digitalWrite(cBatGreen,HIGH);
    beep(false);
  }
  return voltage;
}

bool transmit() {
  data = ((throttle << 24) | (ref_x << 16) | (ref_y << 8) | 
           (ref_z & 0b11110000) | ((calibration ? 1 : 0) << 3) |
           ((motorsOn ? 1 : 0) << 2) | ((hold ? 1 : 0) << 1) | 
           (unused2 ? 1 : 0));

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data));
  if (result != ESP_OK) {
    Serial.println("Failed to broadcast message");
    rgb(5,0,0);
    return false;
  } else {
    rgb(0,2,0);
    return true;
  }
}

void loop() {
  if(millis() > timer1 + 300) { // runs every 300ms
    checkControllerBattery();
    timer1 = millis();
  }
  if(millis() > timer2 + 50) { // runs every 100ms
    checkDroneBattery();
    timer2 = millis();
  }

  throttle = analogRead(joyLY);
  ref_x = analogRead(joyRY);
  ref_y = analogRead(joyRX);
  ref_z = analogRead(joyLX); // discard 4 LSBs later if needed

  calibration = !digitalRead(buttL); // active-low

  if(!digitalRead(joyLC)) { // toggles motors on/off
    motorsOn = !motorsOn;
    if(!motorsOn) hold = false; //We want to reset the hold variable when motors turn off
    delay(300); // debounce delay
  }

    if(!digitalRead(joyRC)) { // toggles hold on/off
    hold = !hold;
    delay(300); // debounce delay
  }

  if(!digitalRead(buttR))
  {
    mute = !mute;
    delay(200);
  }

  delay(10);
  transmit();
}
