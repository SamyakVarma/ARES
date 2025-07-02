#include <stdio.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>
#include "Kalman.h"
#include <math.h>
#include <string.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// === STABILIZER ===
MPU6050 mpu;
Kalman kalmanX;
Kalman kalmanY;

uint32_t timer;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
float imu_roll, imu_pitch;
float gyroXrate, gyroYrate;

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define MAX_SERVO_ANGLE 180

const int servoChannels[3] = {0, 1, 2};
float L[4] = {0.04, 0.04, 0.065, 0.065};

float pz_min = 0.0532;
float pz_max = 0.0732;
float phi_max = 20;

float base_height = 0.09;
float stabilization_gain = 1.0;
float max_correction_angle = 15.0;

// === MOTORS ===
#define LEFT_MOTOR_IN1 16
#define LEFT_MOTOR_IN2 17
#define RIGHT_MOTOR_IN1 18
#define RIGHT_MOTOR_IN2 19

// Servo channels on PCA9685 (0-15)
#define SERVO_FRONT_LEFT 3
#define SERVO_MIDDLE_LEFT 4
#define SERVO_REAR_LEFT 5
#define SERVO_FRONT_RIGHT 6
#define SERVO_MIDDLE_RIGHT 7
#define SERVO_REAR_RIGHT 8

#define SERVO_MIN 150  // Min pulse length count (out of 4096)
#define SERVO_MAX 600  // Max pulse length count (out of 4096)
#define SERVO_CENTER 375  // Center position
#define SERVO_FREQ 50 

#define MAX_LINEAR_SPEED 255  // Maximum PWM value for motors
#define MAX_ANGULAR_SPEED 1.0 // Maximum angular velocity (rad/s)
#define WHEELBASE 0.3

#define PWM_FREQ     1000  // 1 KHz
#define PWM_RES      8     // 8-bit resolution (0-255)
// PWM channels for each motor pin
#define LEFT_IN1_CH  0
#define LEFT_IN2_CH  1
#define RIGHT_IN1_CH 2
#define RIGHT_IN2_CH 3

float front_angle, rear_angle;
float linear_x = 0.0, angular_z = 0.0;

uint16_t angleToPWM(float angle) {
  int us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  return us * 4096 / 20000; // 20ms period at 50Hz
}

// ===============================
// ========== IK ================
// ===============================
void inverseKinematics(float n[3], float Pz, float angles[3]) {
  float A = (L[0] + L[1]) / Pz;
  float B = (pow(Pz, 2) + pow(L[2], 2) - pow(L[0] + L[1], 2) - pow(L[3], 2)) / (2 * Pz);
  float C = A*A + 1;
  float D = 2 * (A*B - (L[0] + L[1]));
  float E = B*B + pow(L[0] + L[1], 2) - pow(L[2], 2);
  float discriminant = D*D - 4*C*E;
  if (discriminant < 0) {
    Serial.println("Warning: Invalid IK solution");
    return;
  }
  float Pmx = (-D + sqrt(discriminant)) / (2*C);
  float Pmz = sqrt(pow(L[2], 2) - pow(Pmx, 2) + 2 * (L[0] + L[1]) * Pmx - pow(L[0] + L[1], 2));

  // Servo A
  float a_m_x = (L[3] / sqrt(n[0]*n[0] + n[2]*n[2])) * n[2];
  float a_m_z = Pz + (L[3] / sqrt(n[0]*n[0] + n[2]*n[2])) * (-n[0]);
  float A_a = (L[0] - a_m_x) / a_m_z;
  float B_a = (a_m_x*a_m_x + a_m_z*a_m_z - L[2]*L[2] - L[0]*L[0] + L[1]*L[1]) / (2 * a_m_z);
  float C_a = A_a*A_a + 1;
  float D_a = 2 * (A_a * B_a - L[0]);
  float E_a = B_a*B_a + L[0]*L[0] - L[1]*L[1];
  discriminant = D_a*D_a - 4*C_a*E_a;
  if (discriminant < 0) return;
  float ax = (-D_a + sqrt(discriminant)) / (2*C_a);
  float az = sqrt(pow(L[1], 2) - pow(ax, 2) + 2*L[0]*ax - L[0]*L[0]);
  if (a_m_z < Pmz) az = -az;
  angles[0] = 90.0 - atan2(ax - L[0], az) * 180.0 / M_PI;

  // Servo B
  float n_sum_b = sqrt(n[0]*n[0] + 3*n[1]*n[1] + 4*n[2]*n[2] + 2*sqrt(3)*n[0]*n[1]);
  float b_m_x = (L[3] / n_sum_b) * -n[2];
  float b_m_y = (L[3] / n_sum_b) * -sqrt(3)*n[2];
  float b_m_z = Pz + (L[3] / n_sum_b) * (sqrt(3)*n[1] + n[0]);
  float A_b = -(b_m_x + sqrt(3)*b_m_y + 2*L[0]) / b_m_z;
  float B_b = (b_m_x*b_m_x + b_m_y*b_m_y + b_m_z*b_m_z + L[1]*L[1] - L[2]*L[2] - L[0]*L[0]) / (2 * b_m_z);
  float C_b = A_b*A_b + 4;
  float D_b = 2*A_b*B_b + 4*L[0];
  float E_b = B_b*B_b + L[0]*L[0] - L[1]*L[1];
  discriminant = D_b*D_b - 4*C_b*E_b;
  if (discriminant < 0) return;
  float bx = (-D_b - sqrt(discriminant)) / (2*C_b);
  float by = sqrt(3) * bx;
  float bz = sqrt(pow(L[1], 2) - 4*bx*bx - 4*L[0]*bx - L[0]*L[0]);
  if (b_m_z < Pmz) bz = -bz;
  angles[1] = 90.0 - atan2(sqrt(bx*bx + by*by) - L[0], bz) * 180.0 / M_PI;

  // Servo C
  float n_sum_c = sqrt(n[0]*n[0] + 3*n[1]*n[1] + 4*n[2]*n[2] - 2*sqrt(3)*n[0]*n[1]);
  float c_m_x = (L[3] / n_sum_c) * -n[2];
  float c_m_y = (L[3] / n_sum_c) * sqrt(3)*n[2];
  float c_m_z = Pz + (L[3] / n_sum_c) * (-sqrt(3)*n[1] + n[0]);
  float A_c = -(c_m_x - sqrt(3)*c_m_y + 2*L[0]) / c_m_z;
  float B_c = (c_m_x*c_m_x + c_m_y*c_m_y + c_m_z*c_m_z + L[1]*L[1] - L[2]*L[2] - L[0]*L[0]) / (2 * c_m_z);
  float C_c = A_c*A_c + 4;
  float D_c = 2*A_c*B_c + 4*L[0];
  float E_c = B_c*B_c + L[0]*L[0] - L[1]*L[1];
  discriminant = D_c*D_c - 4*C_c*E_c;
  if (discriminant < 0) return;
  float cx = (-D_c - sqrt(discriminant)) / (2*C_c);
  float cy = -sqrt(3) * cx;
  float cz = sqrt(pow(L[1], 2) - 4*cx*cx - 4*L[0]*cx - L[0]*L[0]);
  if (c_m_z < Pmz) cz = -cz;
  angles[2] = 90.0 - atan2(sqrt(cx*cx + cy*cy) - L[0], cz) * 180.0 / M_PI;
}

// ===============================
// === Set Platform Posture ====
// ===============================
void setStabilizationPosture(float theta, float phi, float Pz) {
  phi = constrain(phi, -phi_max, phi_max);
  Pz = constrain(Pz, pz_min, pz_max);

  float z = cos(radians(phi));
  float r = sin(radians(phi));
  float x = r * cos(radians(theta));
  float y = r * sin(radians(theta));
  float n[3] = {x, y, z};

  float angles[3];
  inverseKinematics(n, Pz, angles);

  for (int i = 0; i < 3; i++) {
    angles[i] = angles[i] + 90; 
    angles[i] = constrain(angles[i], 10, 180);
    angles[i] = map(angles[i], 10, 180, 180, 10);
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(angles[i]);
    uint16_t pwm_val = angleToPWM(angles[i]);
    pwm.setPWM(servoChannels[i], 0, pwm_val);
  }
}

// ===============================
// === IMU Update and Filtering ==
// ===============================
void updateIMU() {
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  unsigned long now = micros();
  float dt = (now - timer) / 1000000.0;
  timer = now;

  gyroXrate = gyroX / 131.0;
  gyroYrate = gyroY / 131.0;

  float accRoll  = atan2(accY, accZ) * RAD_TO_DEG;
  float accPitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  imu_roll  = kalmanX.getAngle(accRoll, gyroXrate, dt);
  imu_pitch = kalmanY.getAngle(accPitch, gyroYrate, dt);
}

// ===============================
// === Main Stabilization Loop ===
// ===============================
void performStabilization() {
  updateIMU();

  float correction_roll = -imu_roll * stabilization_gain;
  float correction_pitch = -imu_pitch * stabilization_gain;

  correction_roll = constrain(correction_roll, -max_correction_angle, max_correction_angle);
  correction_pitch = constrain(correction_pitch, -max_correction_angle, max_correction_angle);

  float phi = sqrt(correction_roll * correction_roll + correction_pitch * correction_pitch);
  float theta = atan2(correction_pitch, correction_roll) * RAD_TO_DEG;

  setStabilizationPosture(theta, phi, base_height);
}

void initializeHardware() {
  Serial.println("Initializing hardware...");
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  mpu.setXAccelOffset(-1727);
  mpu.setYAccelOffset(15);
  mpu.setZAccelOffset(1303);
  mpu.setXGyroOffset(63);
  mpu.setYGyroOffset(69);
  mpu.setZGyroOffset(71);

  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  imu_roll = atan2(accY, accZ) * RAD_TO_DEG;
  imu_pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(imu_roll);
  kalmanY.setAngle(imu_pitch);
  timer = micros();

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  Serial.println("PCA9685 initialized");
  delay(500);
  setStabilizationPosture(0, 0, base_height);
  delay(1000);
  Serial.println("Stewart Platform IMU Stabilizer Ready");

  ledcAttachChannel(LEFT_MOTOR_IN1, PWM_FREQ, PWM_RES, LEFT_IN1_CH);
  ledcAttachChannel(LEFT_MOTOR_IN2, PWM_FREQ, PWM_RES, LEFT_IN2_CH);
  ledcAttachChannel(RIGHT_MOTOR_IN1, PWM_FREQ, PWM_RES, RIGHT_IN1_CH);
  ledcAttachChannel(RIGHT_MOTOR_IN2, PWM_FREQ, PWM_RES, RIGHT_IN2_CH);

  centerAllServos();

  stopMotors();
  Serial.println("Hardware initialization complete");
}

void setServoAngle(uint8_t servo_channel, float angle) {
  angle = constrain(angle, -90, 90);

  int pulse = map((int)(angle * 10), -900, 900, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(servo_channel, 0, pulse);
}

void centerAllServos() {
  for(int i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, SERVO_CENTER);
  }
  delay(500); // Allow time for servos to reach position
}

void stopMotors() {
  ledcWrite(LEFT_IN1_CH, 0);
  ledcWrite(LEFT_IN2_CH, 0);
  ledcWrite(RIGHT_IN1_CH, 0);
  ledcWrite(RIGHT_IN2_CH, 0);
}

void setMotorSpeed(bool is_left_side, int speed) {
  speed = constrain(speed, -255, 255);
  int abs_speed = abs(speed);
  
  if(is_left_side) {
    if(speed > 0) {
      ledcWrite(LEFT_MOTOR_IN1, abs_speed);
      ledcWrite(LEFT_MOTOR_IN2, 0);
    } else if(speed < 0) {
      ledcWrite(LEFT_MOTOR_IN1, 0);
      ledcWrite(LEFT_MOTOR_IN2, abs_speed);
    } else {
      ledcWrite(LEFT_MOTOR_IN1, 0);
      ledcWrite(LEFT_MOTOR_IN2, 0);
    }
  } else {
    if(speed > 0) {
      ledcWrite(RIGHT_MOTOR_IN1, abs_speed);
      ledcWrite(RIGHT_MOTOR_IN2, 0);
    } else if(speed < 0) {
      ledcWrite(RIGHT_MOTOR_IN1, 0);
      ledcWrite(RIGHT_MOTOR_IN2, abs_speed);
    } else {
      ledcWrite(RIGHT_MOTOR_IN1, 0);
      ledcWrite(RIGHT_MOTOR_IN2, 0);
    }
  }
}

void calculateSteeringAngles(float linear_x, float angular_z) {
  if(abs(linear_x) < 0.01 && abs(angular_z) < 0.01) //STOP
  {
    front_angle = 0;
    rear_angle = 0;
    return;
  }
  
  if(abs(linear_x) < 0.01) //Pure Rotation
  {
    front_angle = (angular_z > 0) ? 45 : -45;
    rear_angle = (angular_z > 0) ? -45 : 45;
    return;
  }
  
  if(abs(angular_z) < 0.01) //Pure translation
  {
    front_angle = 0;
    rear_angle = 0;
    return;
  }
  
  // Ackermann steering calculation
  float turning_radius = linear_x / angular_z;
  
  // Calculate steering angles (simplified for 6-wheel)
  front_angle = atan(WHEELBASE / turning_radius) * 180.0 / PI;
  rear_angle = -atan(WHEELBASE / turning_radius) * 180.0 / PI;
  
  // Constrain angles
  front_angle = constrain(front_angle, -45, 45);
  rear_angle = constrain(rear_angle, -45, 45);
}

void executeMovement(float linear_x, float angular_z) {
  // Calculate motor speeds
  int base_speed = (int)(linear_x * MAX_LINEAR_SPEED);
  int turn_speed = (int)(angular_z * MAX_LINEAR_SPEED * 0.5);
  
  int left_speed = base_speed - turn_speed;
  int right_speed = base_speed + turn_speed;
  
  // Set motor speeds
  setMotorSpeed(true, left_speed);   // Left side motors
  setMotorSpeed(false, right_speed); // Right side motors
  
  // Calculate and set steering angles
  calculateSteeringAngles(linear_x, angular_z);
  
  // Set servo positions
  setServoAngle(SERVO_FRONT_LEFT, front_angle);
  setServoAngle(SERVO_FRONT_RIGHT, front_angle);
  setServoAngle(SERVO_MIDDLE_LEFT, 0);  // Middle wheels typically don't steer
  setServoAngle(SERVO_MIDDLE_RIGHT, 0);
  setServoAngle(SERVO_REAR_LEFT, rear_angle);
  setServoAngle(SERVO_REAR_RIGHT, rear_angle);
  
  // Status LED
  //digitalWrite(LED_PIN, (abs(linear_x) > 0.01 || abs(angular_z) > 0.01) ? HIGH : LOW);
}

void readSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove leading/trailing whitespace

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0) {
      String lin = input.substring(0, spaceIndex);
      String ang = input.substring(spaceIndex + 1);
      linear_x = lin.toFloat();
      angular_z = ang.toFloat();
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initializeHardware();

}

void loop() {
  // put your main code here, to run repeatedly:
  readSerialInput();
  executeMovement(linear_x, angular_z);
  performStabilization();
  delay(10);

}