#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(0,1);  // Replace with the appropriate RX and TX pins for your Arduino

const int sensor1Pin = 2;  // Pin for sensor 1 (Trigger and Echo)
const int sensor2Pin = 3;  // Pin for sensor 2 (Trigger and Echo)
int pin1 = 9;    // L293D EN1
int pin2 = 10;   // L293D EN2
void handleCommand(char command); 
void runMotor2();
void runMotor1();
void stopMotors();
void stabilizeMotors();
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float pitch, roll;
float targetPitch = 0.0;  
float targetRoll = 0.0;   
float pitchGain = 2.0;    
float rollGain = 2.0;     
float complementaryFilterFactor = 0.98;

bool systemOn = false;
void setup() {
  Serial.begin(9600);
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  mpu.initialize();
  mpu.setDMPEnabled(true);
  bluetoothSerial.begin(9600);
}
void loop() {
  if (bluetoothSerial.available()) {
    char command = bluetoothSerial.read();
    handleCommand(command);
  }

  if (systemOn) {
    autocontrol();
  }
}


void autocontrol() {
  long duration1, distance1;
  long duration2, distance2;

  // Measure distance from Sensor 1
  pinMode(sensor1Pin, OUTPUT);
  digitalWrite(sensor1Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor1Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor1Pin, LOW);
  pinMode(sensor1Pin, INPUT);
  duration1 = pulseIn(sensor1Pin, HIGH);
  distance1 = (duration1 / 2) / 29;

  // Measure distance from Sensor 2
  pinMode(sensor2Pin, OUTPUT);
  digitalWrite(sensor2Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor2Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor2Pin, LOW);
  pinMode(sensor2Pin, INPUT);
  duration2 = pulseIn(sensor2Pin, HIGH);
  distance2 = (duration2 / 2) / 29;
  delay(1000);  // Delay between measurements

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll = atan2(ay, az) * 180.0 / PI;
  // Calculate pitch and roll angles from gyroscope data
  float gyroPitch = pitch + (gy / 131.0) * 0.01;  // 131.0 is the sensitivity scale factor for the gyroscope
  float gyroRoll = roll + (gx / 131.0) * 0.01;
  // Apply complementary filter to combine accelerometer and gyroscope data
  pitch = complementaryFilterFactor * gyroPitch + (1 - complementaryFilterFactor) * pitch;
  roll = complementaryFilterFactor * gyroRoll + (1 - complementaryFilterFactor) * roll;
  Serial.println(pitch);
  Serial.println(roll);

    if (distance1 < 9) {
      Serial.println("akhil1");
      distance1 = 10;
      runMotor2();
    } else if (distance2 < 9) {
      Serial.println("akhil2");
      distance2 = 10;
      runMotor1();
    } else {
     Serial.println("high");
      stabilizeMotors();
    }
}
// }
void runMotor1() {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
}

void runMotor2() {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
}


void stabilizeMotors() {
  // Adjust motor speeds based on the pitch and roll errors
  int motor1Speed = 255 + pitch * pitchGain + roll * rollGain;
  int motor2Speed = 255 + pitch * pitchGain - roll * rollGain;
  
  // Clip motor speeds to a valid range
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);

  Serial.println("motor1Speed");
  Serial.println(motor1Speed);
  Serial.println("motor2Speed");
  Serial.println(motor2Speed);
  analogWrite(pin1, motor1Speed);
  analogWrite(pin2, motor2Speed);
}

void stopMotors() {
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
}
void handleCommand(char command) {
  if (command == '1') {
    systemOn = true;  // Turn on the system
    Serial.println("System turned ON");
  } else if (command == '0') {
    systemOn = false; // Turn off the system
    Serial.println("System turned OFF");
    stopMotors();
  }
}
