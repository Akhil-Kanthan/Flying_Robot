#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(0, 1); // Replace with the appropriate RX and TX pins for your Arduino

const int sensor1Pin = 2; // Pin for sensor 1 (Trigger and Echo)
const int sensor2Pin = 3; // Pin for sensor 2 (Trigger and Echo)
int motor1Pin1 = 9;       // L293D Motor 1 pin 1 (EN1)
int motor2Pin2 = 10;      // L293D Motor 2 pin 2 (EN2)
int motor3Pin1 = 5;       // L293D Motor 3 pin 1 (EN3)
int motor4Pin2 = 6;       // L293D Motor 4 pin 2 (EN4)

void handleCommand(char command);
void runMotors(int motor1Speed, int motor2Speed, int motor3Speed, int motor4Speed);
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

void setup()
{
  Serial.begin(9600);
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);
  mpu.initialize();
  mpu.setDMPEnabled(true);
  bluetoothSerial.begin(9600);
}

void loop()
{
  if (bluetoothSerial.available())
  {
    char command = bluetoothSerial.read();
    handleCommand(command);
  }

  if (systemOn)
  {
    autocontrol();
  }
}

void autocontrol()
{
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
  delay(1000); // Delay between measurements

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll = atan2(ay, az) * 180.0 / PI;
  // Calculate pitch and roll angles from gyroscope data
  float gyroPitch = pitch + (gy / 131.0) * 0.01; // 131.0 is the sensitivity scale factor for the gyroscope
  float gyroRoll = roll + (gx / 131.0) * 0.01;
  // Apply complementary filter to combine accelerometer and gyroscope data
  pitch = complementaryFilterFactor * gyroPitch + (1 - complementaryFilterFactor) * pitch;
  roll = complementaryFilterFactor * gyroRoll + (1 - complementaryFilterFactor) * roll;
  Serial.println(pitch);
  Serial.println(roll);

  if (distance1 < 9)
  {
    Serial.println("Obstacle detected on the right side!");
    distance1 = 10;
    // To be implemented based on your robot's control logic
    // Adjust motor speeds for turning or avoiding obstacles
  }
  else if (distance2 < 9)
  {
    Serial.println("Obstacle detected on the left side!");
    distance2 = 10;
    // To be implemented based on your robot's control logic
    // Adjust motor speeds for turning or avoiding obstacles
  }
  else
  {
    // If no obstacles, stabilize the robot
    stabilizeMotors();
  }
}

void runMotors(int motor1Speed, int motor2Speed, int motor3Speed, int motor4Speed)
{
  // Set the motor directions based on the motor speeds
  digitalWrite(motor1Pin1, motor1Speed >= 0 ? HIGH : LOW);
  digitalWrite(motor2Pin2, motor2Speed >= 0 ? LOW : HIGH);
  digitalWrite(motor3Pin1, motor3Speed >= 0 ? HIGH : LOW);
  digitalWrite(motor4Pin2, motor4Speed >= 0 ? LOW : HIGH);

  // Clip motor speeds to a valid range
  motor1Speed = constrain(abs(motor1Speed), 0, 255);
  motor2Speed = constrain(abs(motor2Speed), 0, 255);
  motor3Speed = constrain(abs(motor3Speed), 0, 255);
  motor4Speed = constrain(abs(motor4Speed), 0, 255);

  // Set motor speeds
  analogWrite(motor1Pin1, motor1Speed);
  analogWrite(motor2Pin2, motor2Speed);
  analogWrite(motor3Pin1, motor3Speed);
  analogWrite(motor4Pin2, motor4Speed);
}

void stabilizeMotors()
{
  // Adjust motor speeds based on the pitch and roll errors
  int motor1Speed = 255 + pitch * pitchGain + roll * rollGain;
  int motor2Speed = 255 + pitch * pitchGain - roll * rollGain;
  int motor3Speed = 255 + pitch * pitchGain + roll * rollGain;
  int motor4Speed = 255 + pitch * pitchGain - roll * rollGain;

  // Apply the same speed to both motors on each side
  runMotors(motor1Speed, motor2Speed, motor3Speed, motor4Speed);
}

void stopMotors()
{
  // Stop all motors
  runMotors(0, 0, 0, 0);
}

void handleCommand(char command)
{
  if (command == '1')
  {
    systemOn = true; // Turn on the system
    Serial.println("System turned ON");
  }
  else if (command == '0')
  {
    systemOn = false; // Turn off the system
    Serial.println("System turned OFF");
    stopMotors();
  }
}
