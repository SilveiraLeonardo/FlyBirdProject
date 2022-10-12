#include <Adafruit_MPU6050.h>
#include <Wire.h>
// Include the ESP32 Arduino Servo Library instead of the original Arduino Servo Library
#include <ESP32Servo.h> 

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  
Servo myservo3; 
Servo myservo4;

int servoPin1 = 33;
int servoPin2 = 15;
int servoPin3 = 32;
int servoPin4 = 14;

Adafruit_MPU6050 mpu;

// conversion from rad/s to degrees/s and vice versa
const float rad2deg = 57.29577;
const float deg2rad = 0.01745;

const float micro2sec = 0.000001;

// from calibration
float gyro_x_cal;
float gyro_y_cal;
float gyro_z_cal;
float acc_x_cal;
float acc_y_cal;

float gyro_x;
float gyro_y;
float gyro_z;

float gyro_angle_x = 0;
float gyro_angle_y = 0;
float gyro_angle_z = 0;

float acc_angle_x;
float acc_angle_y;

unsigned long time_last_measurement; 
unsigned long time_elapsed;

bool firstIteration = true;

// assign your channel in pins
#define THROTTLE_IN_PIN 26
#define PITCH_IN_PIN 4
#define ROLL_IN_PIN 21
#define YAW_IN_PIN 25

// assign channel out pins
// ...

// bit flags indicating the channels have  new signals
#define THROTTLE_FLAG 1
#define PITCH_FLAG 2
#define ROLL_FLAG 4
#define YAW_FLAG 8

// Holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables updated by the ISR and read by loop
volatile uint16_t unThrottleInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;
volatile uint16_t unYawInShared;

// these are used to record the rising edge of a pulse
// they do not need to be volative as they are only used in he ISR
uint32_t ulThrottleStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;
uint32_t ulYawStart;

// contant limits
int pMIN = 1000;
int MINarmed = 1075;
float xy_min = -50; //-50;
float xy_max = 50; //50;
float z_min = -150;
float z_max = 150;
int thrust_min = 900;
int thrust_max = 2000;
int max_thrust = 1800;

// input RC variables
int input_throttle;
float input_pitch;
float input_roll;
float input_yaw = 0;

// error variables
float error_x;
float error_y;
float error_z;
float prev_error_x = 0;
float prev_error_y = 0;

// PID
bool firstIterationPID = true;
float Kp = 3.0; //3.2; //2
float Ki = 0;
float Kd = 26; // smooth again at 35 - 25% de 35 (8.75) => 26.25, final value 26
float Kpz = 3;
float Kiz = 0;
// setting a Kd gain for the yaw is not necessary, because it
// has high drag from the props

unsigned long pid_time_last_measurement = 0; 
unsigned long pid_time_elapsed;
float proportional_x;
float proportional_y;
float proportional_z;
float integral_x = 0.0;
float integral_y = 0.0;
float integral_z = 0.0;
float derivative_x;
float derivative_y;

// velocities
int rearLeft;
int rearRight;
int frontLeft;
int frontRight;

void IRAM_ATTR calcThrottle()
{
  // if the pin is high, it is a rising edge
  if (digitalRead(THROTTLE_IN_PIN)==HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it is a falling edge
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);

    // set the throttle flag to indicate that a new throttle signal has been
    // received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void IRAM_ATTR calcPitch()
{
  // if the pin is high, it is a rising edge
  if (digitalRead(PITCH_IN_PIN)==HIGH)
  {
    ulPitchStart = micros();
  }
  else
  {
    // else it is a falling edge
    unPitchInShared = (uint16_t)(micros() - ulPitchStart);

    // set the throttle flag to indicate that a new throttle signal has been
    // received
    bUpdateFlagsShared |= PITCH_FLAG;
  }
}

void IRAM_ATTR calcRoll()
{
  // if the pin is high, it is a rising edge
  if (digitalRead(ROLL_IN_PIN)==HIGH)
  {
    ulRollStart = micros();
  }
  else
  {
    // else it is a falling edge
    unRollInShared = (uint16_t)(micros() - ulRollStart);

    // set the throttle flag to indicate that a new throttle signal has been
    // received
    bUpdateFlagsShared |= ROLL_FLAG;
  }
}

void IRAM_ATTR calcYaw()
{
  // if the pin is high, it is a rising edge
  if (digitalRead(YAW_IN_PIN)==HIGH)
  {
    ulYawStart = micros();
  }
  else
  {
    // else it is a falling edge
    unYawInShared = (uint16_t)(micros() - ulYawStart);

    // set the throttle flag to indicate that a new throttle signal has been
    // received
    bUpdateFlagsShared |= YAW_FLAG;
  }
}

void setup() {
  // Serial.begin(115200);

  // IMU SETUP
  mpu.begin();
  // set sensitivities
  // AFS_SEL = 1, +-4G
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);  
  // FS_SEL = 1, +-500 degrees/second, 65.5 LSB/°/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set bandwidth - disable it
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  
  // perform gyro calibration
  calibrate_gyro();

  // RC INTERRUPTS SETUP
  attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);
  attachInterrupt(PITCH_IN_PIN, calcPitch, CHANGE);
  attachInterrupt(ROLL_IN_PIN, calcRoll, CHANGE);
  attachInterrupt(YAW_IN_PIN, calcYaw, CHANGE);  

  myservo1.attach(servoPin1, 1000, 2000);
  myservo2.attach(servoPin2, 1000, 2000);  
  myservo3.attach(servoPin3, 1000, 2000);  
  myservo4.attach(servoPin4, 1000, 2000);
  
  // arm the motors
  myservo1.write(pMIN);
  myservo2.write(pMIN);
  myservo3.write(pMIN);
  myservo4.write(pMIN);
  delay(5000);  
}

void loop() {
  // put your main code here, to run repeatedly:
  calculateAngle();
  
  receiveControls();

  calculateErrors();

  calculatePID();

  calculateVelocities();

  runMotors();
  
  // debug();

}

void calibrate_gyro() {
  float x = 0;
  float y = 0;
  float z = 0;
  float acc_x = 0;
  float acc_y = 0;
  int n = 7000;

  sensors_event_t a, g, temp;  
  mpu.getEvent(&a, &g, &temp);

  for (int i=0; i<n; i++){
    x += g.gyro.x;
    y += g.gyro.y;
    z += g.gyro.z;

    acc_x += atan(a.acceleration.y/a.acceleration.z);
    acc_y += -atan(a.acceleration.x/a.acceleration.z);
  }

  gyro_x_cal = (x/n)*rad2deg;
  gyro_y_cal = (y/n)*rad2deg;
  gyro_z_cal = (z/n)*rad2deg;

  acc_x_cal = (acc_x/n)*rad2deg;
  acc_y_cal = (acc_y/n)*rad2deg;
}

void calculateAngle() {
  sensors_event_t a, g, temp;  
  mpu.getEvent(&a, &g, &temp);
  // acceleration is measured in  (m/s2)
  // angular velocity is measured in rad/seconds
    
  gyro_x = g.gyro.x*rad2deg - gyro_x_cal;
  gyro_y = g.gyro.y*rad2deg - gyro_y_cal;
  // in the z axis the target and reference will be the
  // rotational speed, and not the angle
  gyro_z = g.gyro.z*rad2deg - gyro_z_cal;

  time_elapsed = (micros() - time_last_measurement);

  gyro_angle_x += gyro_x*time_elapsed*micro2sec;
  gyro_angle_y += gyro_y*time_elapsed*micro2sec;
  // gyro_angle_z += gyro_z*time_elapsed*micro2sec;

  gyro_angle_x += gyro_angle_y * sin(gyro_z*time_elapsed*micro2sec*deg2rad);
  gyro_angle_y -= gyro_angle_x * sin(gyro_z*time_elapsed*micro2sec*deg2rad);

  time_last_measurement = micros();

  acc_angle_x = atan(a.acceleration.y/a.acceleration.z)*rad2deg - acc_x_cal;
  acc_angle_y = -atan(a.acceleration.x/a.acceleration.z)*rad2deg - acc_y_cal;

  if (firstIteration) {
    gyro_angle_x = acc_angle_x;
    gyro_angle_y = acc_angle_y;
    // gyro_angle_z = 0.0;

    firstIteration = false;            
  } else {
    gyro_angle_x = 0.99*gyro_angle_x + acc_angle_x*0.01;
    gyro_angle_y = 0.99*gyro_angle_y + acc_angle_y*0.01;    
  }
}

void receiveControls(){
  // create local variables to hold copies of the channel inputs
  // static variables persist beyond the function call, preserving their data between // function calls.
  static uint16_t unThrottleIn;
  static uint16_t unPitchIn;
  static uint16_t unRollIn;
  static uint16_t unYawIn;  

  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if (bUpdateFlagsShared)
  {
    // turn interrupts off while we take local copies of the shared variables
    noInterrupts();

    // local copu of which channels were updated
    bUpdateFlags = bUpdateFlagsShared;
    
    // copy the shared values flagged
    if (bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;      
    }
    if (bUpdateFlags & PITCH_FLAG)
    {
      unPitchIn = unPitchInShared;      
    }
    if (bUpdateFlags & ROLL_FLAG)
    {
      unRollIn = unRollInShared;      
    }
    if (bUpdateFlags & YAW_FLAG)
    {
      unYawIn = unYawInShared;      
    }
    // clear shared copy of updated flags
    bUpdateFlagsShared = 0;

    interrupts(); // turn interrupts back on       
  }

  input_throttle = map(unThrottleIn, 1099, 1919, thrust_min, thrust_max);
  input_pitch = map(unPitchIn, 1099, 1919, xy_min, xy_max);
  input_roll = map(unRollIn, 1099, 1919, xy_min, xy_max); 
  input_yaw = map(unYawIn, 1099, 1919, z_min, z_max);

  // failsafes
  if (unThrottleIn < 1000){
    input_throttle = thrust_min;
  }
  else if (unThrottleIn > 2000){
    input_throttle = thrust_min;
  }
  if (unPitchIn < 1000){
    input_pitch = 0;
  }
  else if (unPitchIn > 2000){
    input_pitch = 0;
  }  
  if (unRollIn < 1000){
    input_roll = 0;
  }
  else if (unRollIn > 2000){
    input_roll = 0;
  }  
  if (unYawIn < 1000){
    input_yaw = 0;
  }
  else if (unYawIn > 2000){
    input_yaw = 0;
  }  
}

void calculateErrors() {
  error_x = gyro_angle_x - input_roll;
  error_y = gyro_angle_y - input_pitch;
  error_z = gyro_z - input_yaw;
}

void calculatePID() {
// float Kp = 2;
// float Ki = 0.001;
// float Kd = 0.5;
// float Kpz = 2;
// unsigned long pid_time_last_measurement; 
// unsigned long pid_time_elapsed;
  pid_time_elapsed = micros() - pid_time_last_measurement;

  proportional_x = error_x*Kp;
  proportional_y = error_y*Kp;
  proportional_z = error_z*Kpz;

  if (firstIterationPID) {
    integral_x = 0.0;
    integral_y = 0.0;
    integral_z = 0.0;

    firstIterationPID = false;            
  } else {
    // integral_x += error_x*Ki*pid_time_elapsed*micro2sec;
    // integral_y += error_y*Ki*pid_time_elapsed*micro2sec;
    // integral_z += error_z*Kiz*pid_time_elapsed*micro2sec;

    integral_x += error_x*Ki;
    integral_y += error_y*Ki;
    integral_z += error_z*Kiz;
  }

  // derivative_x = Kd*(error_x - prev_error_x)/(pid_time_elapsed*micro2sec);
  // derivative_y = Kd*(error_y - prev_error_y)/(pid_time_elapsed*micro2sec);

  derivative_x = Kd*(error_x - prev_error_x);
  derivative_y = Kd*(error_y - prev_error_y);

  prev_error_x = error_x;
  prev_error_y = error_y;
  pid_time_last_measurement = micros();
}

void calculateVelocities() {
  if (input_throttle > max_thrust){
    input_throttle = max_thrust;
  }

  // rearLeft = MOTOR 2
  // rearRight = MOTOR 4
  // frontLeft = MOTOR 3
  // frontRight  =  MOTOR 1
  rearLeft = input_throttle - proportional_x - integral_x - derivative_x - proportional_y - integral_y - derivative_y + proportional_z + integral_z;
  
  rearRight = input_throttle + proportional_x + integral_x + derivative_x - proportional_y - integral_y - derivative_y - proportional_z - integral_z;
  
  frontLeft = input_throttle - proportional_x - integral_x - derivative_x + proportional_y + integral_y + derivative_y - proportional_z - integral_z;
  
  frontRight  = input_throttle + proportional_x + integral_x + derivative_x + proportional_y + integral_y + derivative_y + proportional_z + integral_z;

  if (rearLeft < MINarmed){
    rearLeft = MINarmed;
  }
  else if (rearLeft > thrust_max){
    rearLeft = thrust_max;
  }

  if (rearRight < MINarmed){
    rearRight = MINarmed;
  }
  else if (rearRight > thrust_max){
    rearRight = thrust_max;
  }
  
  if (frontLeft < MINarmed){
    frontLeft = MINarmed;
  }
  else if (frontLeft > thrust_max){
    frontLeft = thrust_max;
  }  

  if (frontRight < MINarmed){
    frontRight = MINarmed;
  }
  else if (frontRight > thrust_max){
    frontRight = thrust_max;
  }  
}

void runMotors(){
  // rearLeft = MOTOR 2
  // rearRight = MOTOR 4
  // frontLeft = MOTOR 3
  // frontRight  =  MOTOR 1  
  myservo1.write(frontRight); 
  myservo2.write(rearLeft);
  myservo3.write(frontLeft);
  myservo4.write(rearRight);
}

void debug(){
  // Serial.print(input_throttle);
  // Serial.print(" - ");
  // Serial.print(input_pitch);
  // Serial.print(" - ");
  // Serial.print(input_roll);
  // Serial.print(" - ");
  // Serial.print(input_yaw);  
  // Serial.print(" - ");
  Serial.print(frontRight);
  Serial.print(" - ");
  Serial.print(rearLeft);
  Serial.print(" - ");
  Serial.print(frontLeft);
  Serial.print(" - ");
  Serial.print(rearRight);
  Serial.print(" - ");
  Serial.print(error_x);
  Serial.print(" - ");
  Serial.print(error_y);  
  Serial.print(" - ");
  Serial.print(error_z);
  Serial.print(" - ");
  Serial.print(gyro_angle_x);
  Serial.print(" - ");
  Serial.print(gyro_angle_y);         
  Serial.print(" - ");
  Serial.print(gyro_z);
  Serial.print(" - ");
  Serial.print(input_roll);
  Serial.print(" - ");
  Serial.print(input_pitch);         
  Serial.print(" - ");
  Serial.print(input_yaw);    
  Serial.print(" - ");
  Serial.print(proportional_z);         
  Serial.print(" - ");
  Serial.print(integral_z);     
  Serial.println("");
}
  
