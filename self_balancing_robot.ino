
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <Adafruit_BNO08x.h>
#include <SPI.h>

// pid consts
const float KP = 1000;
const float KI = 0;
const float KD = 100;

// pwm consts
const int pwmM1A_pin27 = 27;
const int pwmM1B_pin12 = 12;
const int pwmM2A_pin13 = 13;
const int pwmM2B_pin14 = 14;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// encoder consts
const int motor1ChA = 25;
const int motor1ChB = 26;
const int motor2ChA = 22;
const int motor2ChB = 21;

volatile int previousState_motor1ChA;
volatile int previousState_motor1ChB;
volatile int previousState_motor2ChA;
volatile int previousState_motor2ChB;

volatile long encoderticks_motor1 = 0;
volatile long encoderticks_motor2 = 0;

volatile bool newEncoderEvent_motor1 = false;
volatile bool newEncoderEvent_motor2 = false;

// spi consts
#define BNO08X_CS   5   // Chip Select pin
#define BNO08X_INT  4   // Interrupt pin
#define BNO08X_RST  2   // Reset pin

// Default ESP32 SPI pins
// MOSI: GPIO23 (DI on BNO085)
// MISO: GPIO19 (SDA on BNO085) 
// SCK:  GPIO18 (SCL on BNO085)

Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue;

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float minoutput, float maxoutput) : 
                kp_(kp), ki_(ki), kd_(kd), minout_(minoutput), maxout_(maxoutput){
    last_time_ = millis();
  }

  float compute(float target, float current_val){

    unsigned long current_time = millis();
    float dt = (current_time - last_time_) / 1000.0; // in seconds
    last_time_ = current_time;

    // P term
    float error = target - current_val;
    float p_term = kp_ * error;

    // I term
    integral += error * dt;
    float i_term = ki_ * integral;

    // D term
    float derivative = (error - previous_error_) / dt;
    float d_term = kd_ * derivative;

    //updating previous error
    previous_error_ = error;

    // total output
    float output = p_term + i_term + d_term;

    // clammping
    if(output > maxout_){
      output = maxout_;
    }
    else if(output < minout_){
      output = minout_;
    }

    return output;

  }
private:
  float kp_;
  float ki_;
  float kd_;
  float minout_;
  float maxout_;

  float integral = 0.0;
  float previous_error_ = 0.0;
  unsigned long last_time_ = 0;
};

PIDController robot_pid(KP, KI, KD, -255.0, 255.0);

void IRAM_ATTR readEncoder_motor1() {
  int currentEncoderAState = digitalRead(motor1ChA);
  int currentEncoderBState = digitalRead(motor1ChB);

  int encoded_state = (previousState_motor1ChA << 3) | (previousState_motor1ChB << 2)
                      | (currentEncoderAState << 1) | (currentEncoderBState);

  switch(encoded_state){
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      encoderticks_motor1++;
      break;
    case 0b0001:
    case 0b1000:
    case 0b1110:
    case 0b0111:
      encoderticks_motor1--;
      break;
    default:
      break;
  }
  previousState_motor1ChA = currentEncoderAState;
  previousState_motor1ChB = currentEncoderBState;
  newEncoderEvent_motor1 = true;
}

void IRAM_ATTR readEncoder_motor2() {
  int currentEncoderAState = digitalRead(motor2ChA);
  int currentEncoderBState = digitalRead(motor2ChB);

  int encoded_state = (previousState_motor2ChA << 3) | (previousState_motor2ChB << 2)
                      | (currentEncoderAState << 1) | (currentEncoderBState);

  switch(encoded_state){
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      encoderticks_motor2++;
      break;
    case 0b0001:
    case 0b1000:
    case 0b1110:
    case 0b0111:
      encoderticks_motor2--;
      break;
    default:
      break;
  }
  previousState_motor2ChA = currentEncoderAState;
  previousState_motor2ChB = currentEncoderBState;
  newEncoderEvent_motor2 = true;
}

void forward(int speed) { // 0 to 255
  //Serial.print("forward speed set to: ");
  //Serial.println(speed);
  ledcWrite(pwmM1A_pin27, speed);
  ledcWrite(pwmM1B_pin12, 0);
  ledcWrite(pwmM2A_pin13, 0);
  ledcWrite(pwmM2B_pin14, speed);
}

void reverse(int speed) { // 0 to 255
  // Serial.print("reverse speed set to: ");
  // Serial.println(speed);
  ledcWrite(pwmM1A_pin27, 0);
  ledcWrite(pwmM1B_pin12, speed);
  ledcWrite(pwmM2A_pin13, speed);
  ledcWrite(pwmM2B_pin14, 0);
}

void brake() {
  Serial.println("Braking");
  ledcWrite(pwmM1A_pin27, 0);
  ledcWrite(pwmM1B_pin12, 0);
  ledcWrite(pwmM2A_pin13, 0);
  ledcWrite(pwmM2B_pin14, 0);
}

void setReports(void) {
  Serial.println("Enabling sensor reports...");
  
  // // Enable accelerometer
  // if (!bno08x.enableReport(SH2_ACCELEROMETER, 100000)) { // 100ms = 10Hz
  //   Serial.println("Could not enable accelerometer");
  // } else {
  //   Serial.println("Accelerometer enabled");
  // }
  
  // // Enable gyroscope
  // if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 100000)) { // 100ms = 10Hz
  //   Serial.println("Could not enable gyroscope");
  // } else {
  //   Serial.println("Gyroscope enabled");
  // }
  
  // // Enable magnetometer
  // if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 100000)) { // 100ms = 10Hz
  //   Serial.println("Could not enable magnetometer");
  // } else {
  //   Serial.println("Magnetometer enabled");
  // }
  
  // Enable rotation vector (quaternion)
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 100000)) { // 100ms = 10Hz
    Serial.println("Could not enable rotation vector");
  } else {
    Serial.println("Rotation vector enabled");
  }

}

void monitorEncoderMotor1Task(void *pvParameters) {
  (void) pvParameters;// to suppress compiler warning about unused parameters
  for(;;) {
    if(newEncoderEvent_motor1) {
      noInterrupts();
      newEncoderEvent_motor1 = false;
      long current_encoderticks_motor1 = encoderticks_motor1;
      interrupts();
      Serial.print("Current encoder motor1 ticks: ");
      Serial.println(current_encoderticks_motor1);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void monitorEncoderMotor2Task(void *pvParameters) {
  (void) pvParameters;// to suppress compiler warning about unused parameters
  for(;;){
    if(newEncoderEvent_motor2) {
      noInterrupts();
      newEncoderEvent_motor2 = false;
      long current_encoderticks_motor2 = encoderticks_motor2;
      interrupts();
      Serial.print("Current encoder motor2 ticks: ");
      Serial.println(current_encoderticks_motor2);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200); //9600
  delay(1000);

  // pwm pin setup
  ledcAttach(pwmM1A_pin27, pwmFreq, pwmResolution);
  ledcAttach(pwmM1B_pin12, pwmFreq, pwmResolution);
  ledcAttach(pwmM2A_pin13, pwmFreq, pwmResolution);
  ledcAttach(pwmM2B_pin14, pwmFreq, pwmResolution);

  // encoder pin setup
  pinMode(motor1ChA, INPUT_PULLUP);
  pinMode(motor1ChB, INPUT_PULLUP);
  pinMode(motor2ChA, INPUT_PULLUP);
  pinMode(motor2ChB, INPUT_PULLUP);

  // get initial previous state in setup
  previousState_motor1ChA = digitalRead(motor1ChA);
  previousState_motor1ChB = digitalRead(motor1ChB);
  previousState_motor2ChA = digitalRead(motor2ChA);
  previousState_motor2ChB = digitalRead(motor2ChB);

  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(motor1ChA), readEncoder_motor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor1ChB), readEncoder_motor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2ChA), readEncoder_motor2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2ChB), readEncoder_motor2, CHANGE);

  // IMU SPI setup
  pinMode(BNO08X_CS, OUTPUT);
  digitalWrite(BNO08X_CS, HIGH); // CS idle high
  pinMode(BNO08X_RST, OUTPUT);
  pinMode(BNO08X_INT, INPUT);

  // Initialize SPI
  SPI.begin();

  // Initialize Sensor
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { 
      delay(5000);
      Serial.println("Retrying connection...");
      if (bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("Connection successful!");
        break;
      }
    }
  }
  
  Serial.println("BNO08x Found!");
  Serial.println();

  // Enable Reports
  setReports();


  // thread
  xTaskCreatePinnedToCore(
    monitorEncoderMotor1Task,          // Task function
    "monitorEncoderMotor1Task",        // Name of the task (for debugging)
    10000,              // Stack size in words (not bytes!), 10000 words is usually generous
    NULL,               // Parameter to pass to the task (none in this case)
    1,                  // Task priority (0 to configMAX_PRIORITIES-1, 1 is low)
    NULL,      // Task handle (optional, pass NULL if not needed)
    0                   // Core to pin the task to (0 or 1). Let's use Core 0.
  );

  xTaskCreatePinnedToCore(
    monitorEncoderMotor2Task,          // Task function
    "monitorEncoderMotor2Task",        // Name of the task (for debugging)
    10000,              // Stack size in words (not bytes!), 10000 words is usually generous
    NULL,               // Parameter to pass to the task (none in this case)
    1,                  // Task priority (0 to configMAX_PRIORITIES-1, 1 is low)
    NULL,      // Task handle (optional, pass NULL if not needed)
    0                   // Core to pin the task to (0 or 1). Let's use Core 0.
  );
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset - reinitializing reports");
    setReports();
  }
  
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  // computing roll, pitch, and yaw from the quaternion
  //extracting components
  double rot_angle = sensorValue.un.rotationVector.real;
  double x_axis = sensorValue.un.rotationVector.i;
  double y_axis = sensorValue.un.rotationVector.j;
  double z_axis = sensorValue.un.rotationVector.k;

  // euler angles
  double roll = atan2(2*(rot_angle * x_axis + y_axis * z_axis), 1 - 2*((x_axis*x_axis) + (y_axis*y_axis)));
  double pitch = asin(2 * (rot_angle * y_axis - z_axis * x_axis));
  double yaw = atan2(2*(rot_angle * z_axis + x_axis * y_axis), 1 - 2*((y_axis*y_axis) + (z_axis*z_axis)));
  // Serial.print("roll: ");
  // Serial.println(roll);
  // Serial.print("pitch: ");
  // Serial.println(pitch);
  // Serial.print("yaw: ");
  // Serial.println(yaw);
  // delay(100);
  float output_speed = robot_pid.compute(0.0, pitch);
  if(output_speed )
  // Serial.print("Output speed: ");
  // Serial.println(output_speed);
  if(output_speed < 0){
    reverse(0);
    forward(abs(output_speed));
  }
  else if(output_speed > 0){
    forward(0);
    reverse(abs(output_speed));
  }

  // Serial.print("Accel - X: ");
  // Serial.print(sensorValue.un.accelerometer.x, 3);
  // Serial.print(" Y: ");
  // Serial.print(sensorValue.un.accelerometer.y, 3);
  // Serial.print(" Z: ");
  // Serial.println(sensorValue.un.accelerometer.z, 3);
  //forward(output_speed);

  // Serial.print("Rot - R: ");
  // Serial.print(sensorValue.un.rotationVector.real, 3);
  // Serial.print(" I: ");
  // Serial.print(sensorValue.un.rotationVector.i, 3);
  // Serial.print(" J: ");
  // Serial.print(sensorValue.un.rotationVector.j, 3);
  // Serial.print(" K: ");
  // Serial.println(sensorValue.un.rotationVector.k, 3);
  
  //  switch (sensorValue.sensorId) {
  //   // case SH2_ACCELEROMETER:
  //   //   Serial.print("Accel - X: ");
  //   //   Serial.print(sensorValue.un.accelerometer.x, 3);
  //   //   Serial.print(" Y: ");
  //   //   Serial.print(sensorValue.un.accelerometer.y, 3);
  //   //   Serial.print(" Z: ");
  //   //   Serial.println(sensorValue.un.accelerometer.z, 3);
  //   //   break;
      
  //   case SH2_GYROSCOPE_CALIBRATED:
  //     Serial.print("Gyro - X: ");
  //     Serial.print(sensorValue.un.gyroscope.x, 3);
  //     Serial.print(" Y: ");
  //     Serial.print(sensorValue.un.gyroscope.y, 3);
  //     Serial.print(" Z: ");
  //     Serial.println(sensorValue.un.gyroscope.z, 3);
  //     break;
      
  //   // case SH2_MAGNETIC_FIELD_CALIBRATED:
  //   //   Serial.print("Mag - X: ");
  //   //   Serial.print(sensorValue.un.magneticField.x, 1);
  //   //   Serial.print(" Y: ");
  //   //   Serial.print(sensorValue.un.magneticField.y, 1);
  //   //   Serial.print(" Z: ");
  //   //   Serial.println(sensorValue.un.magneticField.z, 1);
  //   //   break;
      
  //   case SH2_ROTATION_VECTOR:
  //     Serial.print("Rot - R: ");
  //     Serial.print(sensorValue.un.rotationVector.real, 3);
  //     Serial.print(" I: ");
  //     Serial.print(sensorValue.un.rotationVector.i, 3);
  //     Serial.print(" J: ");
  //     Serial.print(sensorValue.un.rotationVector.j, 3);
  //     Serial.print(" K: ");
  //     Serial.println(sensorValue.un.rotationVector.k, 3);
  //     break;
  //  }

  // for(int i = 0; i < 255; i++){
  //   forward(i);
  //   delay(10);
  // }
  //   for(int i = 255; i > 0; i--){
  //   forward(i);
  //   delay(10);
  // }
  //   for(int i = 0; i < 255; i++){
  //   reverse(i);
  //   delay(10);
  // }
  //   for(int i = 255; i > 0; i--){
  //   reverse(i);
  //   delay(10);
  // }
}
