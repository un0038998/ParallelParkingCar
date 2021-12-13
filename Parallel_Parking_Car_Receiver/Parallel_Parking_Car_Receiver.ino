#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define RIGHT_FRONT_SERVO_PIN  27
#define RIGHT_BACK_SERVO_PIN   26
#define LEFT_FRONT_SERVO_PIN   25
#define LEFT_BACK_SERVO_PIN    33
Servo rightFrontServo;
Servo rightBackServo;
Servo leftFrontServo;
Servo leftBackServo;

//Right motor
int enableRightMotor=22; 
int rightMotorPin1=16;
int rightMotorPin2=17;
//Left motor
int enableLeftMotor=23;
int leftMotorPin1=18;
int leftMotorPin2=19;

#define MAX_MOTOR_SPEED 200

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;

struct MessageData
{
  int xAxisValue;
  int yAxisValue;
  bool switchPressed;
};
MessageData messageData;

int parkCarFlag = false;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&messageData, incomingData, sizeof(messageData));
  String inputData ;
  inputData = inputData + "values " + messageData.xAxisValue + "  " + messageData.yAxisValue + "  " + messageData.switchPressed;
  Serial.println(inputData);


  if (messageData.switchPressed == true)
  {
    if (parkCarFlag == false)
    {
      parkCarFlag = true;
      parkCar(0, 0, 180, 180);   // Park the car. Rotate servos accordingly.
    }
    else
    {
      parkCarFlag = false;
      parkCar(90, 90, 90, 90);  //No Parking. So keep servos in middle position       
    }
  }

  if (messageData.yAxisValue <= 1000)       //Move car Forward
  {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (messageData.yAxisValue >= 3000)   //Move car Backward
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (messageData.xAxisValue >= 3000)  //Move car Right
  {
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (messageData.xAxisValue <= 1000)   //Move car Left
  {
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else                                      //Stop the car
  {
    rotateMotor(0, 0);
  } 
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }  
}

void parkCar(int rightFrontServoValue, int rightBackServoValue, int leftFrontServoValue, int leftBackServoValue)
{
  rightFrontServo.write(rightFrontServoValue);
  rightBackServo.write(rightBackServoValue);
  leftFrontServo.write(leftFrontServoValue);
  leftBackServo.write(leftBackServoValue);  
}

void setUpPinModes()
{
       
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  //Set up PWM for speed
  ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, PWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, PWMSpeedChannel);  
  ledcWrite(PWMSpeedChannel, MAX_MOTOR_SPEED);
  
  rotateMotor(0, 0);

  rightFrontServo.attach(RIGHT_FRONT_SERVO_PIN);
  rightBackServo.attach(RIGHT_BACK_SERVO_PIN);
  leftFrontServo.attach(LEFT_FRONT_SERVO_PIN);
  leftBackServo.attach(LEFT_BACK_SERVO_PIN);

  parkCar(90, 90, 90, 90);  
}


void setup() 
{
  setUpPinModes();
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() 
{
}
