#include <esp_now.h>
#include <WiFi.h>

#define X_AXIS_PIN 32
#define Y_AXIS_PIN 33
#define SWITCH_PIN 25

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t receiverMacAddress[] = {0xAC,0x67,0xB2,0x36,0x7F,0x28};  //AC:67:B2:36:7F:28

struct MessageData
{
  int xAxisValue;
  int yAxisValue;
  bool switchPressed;
};

MessageData messageData;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t ");
  Serial.println(status);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

void setup() 
{
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Succes: Added peer");
  }  


}
 
void loop() 
{
  messageData.xAxisValue = analogRead(X_AXIS_PIN);
  messageData.yAxisValue = analogRead(Y_AXIS_PIN);  
  messageData.switchPressed = false; 

  if (digitalRead(SWITCH_PIN) == LOW)
  {
    messageData.switchPressed = true;
  }
  
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &messageData, sizeof(messageData));
  if (result == ESP_OK) 
  {
    Serial.println("Sent with success");
  }
  else 
  {
    Serial.println("Error sending the data");
  }    
  
  if (messageData.switchPressed == true)
  {
    delay(1000);
  }
  else
  {
    delay(50);
  }
}
