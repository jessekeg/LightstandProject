/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#include "stepPackets.h"

//setting up servo pins
const int DIR = 12;
const int STEP = 14;

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
  int current = myData.X_DATA;
  while(myData.X_DATA == current && myData.X_DATA != 0)
  {
    //Serial.println(i);
    digitalWrite(STEP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(1000);
  }
  

 
}