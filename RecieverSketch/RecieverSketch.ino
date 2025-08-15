

#include <esp_now.h>
#include <WiFi.h>
#include "stepPackets.h"

//setting up servo pins
const int DIR = 12;
const int STEP = 14;
const int DIR1 = 34;
const int STEP1 = 35;

// Structure example to receive data
// Must match the sender structure
#ifndef STEP_PACKETS
#define STEP_PACKETS
typedef struct struct_message {
    char mode[32];
    int b;
    bool d;
} struct_message;
#endif

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len){
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.mode);
  Serial.print("X_DATA ");
  Serial.println(myData.X_DATA);
  Serial.print("Y_DATA ");
  Serial.println(myData.Y_DATA);
  Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  //setting up Stepper pins
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  if(myData.X_DATA > 0){
    digitalWrite(DIR, LOW); //changing direction
  }else if(myData.X_DATA < 0){
    digitalWrite(DIR, HIGH);
  }
  if(myData.Y_DATA > 0){
    digitalWrite(DIR1, LOW); //changing direction
  }else if(myData.Y_DATA < 0){
    digitalWrite(DIR1, HIGH);
  }
  int current = myData.X_DATA;
  int currentY = myData.Y_DATA;
  while(myData.X_DATA == current && myData.X_DATA != 0)
  {
    //Serial.println(i);
    digitalWrite(STEP, HIGH);
    delayMicroseconds(5000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(5000);
  }
  while(myData.Y_DATA == currentY && myData.Y_DATA != 0)
  {
    //Serial.println(i);
    digitalWrite(STEP1, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP1, LOW);
    delayMicroseconds(1000);
  }
  

 
}