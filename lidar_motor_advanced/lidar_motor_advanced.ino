#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <Grove_I2C_Motor_Driver.h>
#include <Servo.h>

#define I2C_ADDRESS 0x0f
#define ADC_MIC A0
#define FRONT_SERVO_PIN 5
#define REAR_SERVO_PIN 7

TFLI2C sensor;

Servo servoMotor;
Servo servoMotorLidar;

const double EPSILON{0.001};

const int STEADY_DISTANCE_CM{200};
const unsigned int CHECK_MS{100};
const int STOP_DISTANCE_CM{250};
const int START_VOLUME{200};
const int MIN_SPEED{180};

unsigned long lastCheckMs{};
unsigned long now{};

int motorSpeed{};

int servoMotorAngle{};
int servoMotorLidarAngle{};

int calculateSpeed(int currentSpeed, int distance) {
 
  const float sensitivity  = 0.75f;
  const float minGain = 0.2f;
  const float maxGain = 3.0f;
 
  float robotDist = (float)(distance - STEADY_DISTANCE_CM) / STEADY_DISTANCE_CM;
  float gain = 1.0f - sensitivity * robotDist;

  gain = constrain(gain, minGain, maxGain);

  int newSpeed = (int)(currentSpeed * gain);
  return constrain(newSpeed, MIN_SPEED, 255);
}

void startBot() {  
  delay(5000);  //Delay startup
  while (true){

    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      if (command == "START") {
        Serial.println("Robot starting via App");
        break;  
      }
    }
    
    long sum = 0;
    for(int i=0; i<16; i++)
    {
//      sum += analogRead(ADC_MIC);  //until noise picked up by speaker
    }
    sum >>= 4;
    // Serial.println(sum);
    if (sum > START_VOLUME){break;}
    delay(20);
  }

  // Reset to default angles
  steerBot(90);
  steerLidar(90);
  
  Motor.speed(MOTOR1, 150);
  motorSpeed = 150;
  lastCheckMs = millis();
}

void stopBot() {

  Serial.println("Robot stopping via App");
  Motor.speed(MOTOR1, 0);
  motorSpeed = 0;

  // Reset to default values
  steerBot(90);
  steerLidar(90);
}

void steerBot(int angle) {

  if (angle == servoMotorAngle)
    return;
  
  servoMotor.write(angle);
  servoMotorAngle = angle;
}

void steerLidar(int angle) {

  if (angle == servoMotorLidarAngle)
    return;

  servoMotorLidar.write(angle);
  servoMotorLidarAngle = angle;
}

void receiveCommand(String command) {

  if (command == "STOP") {
      stopBot();

      startBot();
  } else if (command.startsWith("FRONT_ANGLE=")) {
    int front_angle{};
    int rear_angle{};

    int parsed_count = sscanf(command.c_str(), "FRONT_ANGLE=%d,REAR_ANGLE=%d", &front_angle, &rear_angle);

    if (parsed_count == 2) {
      steerBot(front_angle);
      
      steerLidar(rear_angle);  
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Motor.begin(I2C_ADDRESS);
  Motor.speed(MOTOR1, 0);
  servoMotor.attach(FRONT_SERVO_PIN);
  servoMotorLidar.attach(REAR_SERVO_PIN);
  startBot();
}

void loop() {

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    receiveCommand(command);
  }

  now = millis();
 
  if (now - lastCheckMs >= CHECK_MS) {
    int dist;
   
   
    if (sensor.getData(dist, 0x10)) {
     
      // Serial.print("dist: ");
      // Serial.print(dist);
     
      if (dist >= STOP_DISTANCE_CM and dist != 9000) {
        motorSpeed = max(motorSpeed - 5, MIN_SPEED); // REDUCE BY INCREMENT
        Motor.speed(MOTOR1, motorSpeed);
      } else {
        // Tweak speed
        motorSpeed = calculateSpeed(motorSpeed, dist);
        Motor.speed(MOTOR1, motorSpeed);
        // Serial.print(" / motor speed: ");
        // Serial.println(motorSpeed);
      }
    } else {
      Serial.println("No output from tf luna");  
    }
   
    lastCheckMs = now;  
  }
}
