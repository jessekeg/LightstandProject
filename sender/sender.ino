/*
  Jesse Kegley
  Usage:
    Sending data over ESP-NOW protocol in order to control a pair of servo motors
  Pins:
    GPIO 4 - pushButtonIn
*/

#include <esp_now.h>
#include <WiFi.h>
#include "stepPackets.h"
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xa0, 0xb7, 0x65, 0x19, 0x15, 0x8c};

//assigning a pin for the button
const int pushButton = 4;
const int pushButton2 = 3;
const int dirButton = 2;
const int dirButton2 = 5;

#ifndef STEP_PACKETS
#define STEP_PACKETS
//Enumerating a mode type to define which motor to control within the message
enum modes{
  NONE,
  X_MOTOR,
  Y_MOTOR,
  SYNCHRO
};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  modes mode;
  int b;
  bool d;
} struct_message;
#endif

// Create a struct_message called myData
struct_message myData;

//function to generate message to send. Should send however many degrees the stepper should move within myDate
int generateMessage(modes mode_given, bool pushButton, bool pushButton1, bool dirButton, bool dirButton1)
{
  myData.mode = mode_given;
  myData.Type = ACCEL;
  if(mode_given == X_MOTOR || mode_given == SYNCHRO){ //setting x data
    if(pushButton == HIGH && dirButton == HIGH)
    {
      myData.X_DATA = 300; //acceleration of 300
    }
    else if(dirButton == LOW && pushButton == HIGH)
    {
        myData.X_DATA = -300;
    }
    else
    {
      myData.X_DATA = 0;
    }
  }
  if(mode_given == Y_MOTOR || mode_given == SYNCHRO)
  { //setting y data
      if(pushButton2 == HIGH && dirButton2 == HIGH)
      {
          myData.Y_DATA = 300;
      }
      else if(dirButton2 == LOW && pushButton2 == HIGH)
      {
          myData.Y_DATA = -300;
      }
      else
      {
        myData.Y_DATA = 0;
      }
  }
    return 1;
}

  


esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  //setup success and fail pins
  pinMode(pushButton, INPUT);
  pinMode(dirButton, INPUT);
  pinMode(dirButton2, INPUT);
  pinMode(pushButton2, INPUT);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  int PB_State = digitalRead(pushButton); //reading x push button
  int PB2_State = digitalRead(pushButton2);
  int DB_State = digitalRead(dirButton);
  int DB2_State = digitalRead(dirButton2);
  if (PB_State == HIGH)
  {
  Serial.println("PBS HIGH");
  }else{
    Serial.println("PBS LOW");
  }
  if(generateMessage(SYNCHRO, PB_State, PB2_State, DB_State, DB2_State) == 0)
  {
    Serial.println("Error generating message.");
  }
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(15);
}