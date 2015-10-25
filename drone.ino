/* much code from the following sites
 *  
 *  https://github.com/strangedev/Arduino-Quadcopter
 *  400Hz PWM code from somewhere T_T (don't remember)
 *  
 */

#include <Wire.h>
#include <PID_v1.h>v
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Console.h>
#include <FileIO.h>

//#define VERBOSE_DEBUG

#define PID_GYRO_P            3.9
#define PID_GYRO_I            14
#define PID_GYRO_D            1

#define PID_ACCEL_P           1
#define PID_ACCEL_I           0.8
#define PID_ACCEL_D           3

#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1

#define PID_SAMPLE_PERIOD     10

#define PRESAMPLE_COUNT     1500
#define PRESAMPLE_STABLE_CHECK  30
#define PRESAMPLE_STABLE_TOLERANCE  100.0

#define REPOSITION_PERIOD_MS  30ul
#define MOVE_DURATION_MS      2000

#define ESC_MIN               88
#define ESC_WORKING_MIN       160
#define ESC_MAX               460
#define ESC_ARM_DELAY         5000
  
#define ESC_A_REG OCR1C // PIN 11
#define ESC_B_REG OCR1B // PIN 10
#define ESC_C_REG OCR1A // PIN 9
#define ESC_D_REG OCR3A // PIN 5

#define ESC_REG_MIN 1400
#define ESC_REG_MAX 3800

#define TAKEOFF_Z_ACCEL       (-0.05)
#define TAKEOFF_THROTTLE_STEP 0.03
#define TAKEOFF_GOUP_DELAY    5000

#define PID_XY_INFLUENCE    30.0
#define PID_THROTTLE_INFLUENCE  50.0

#define UPDOWN_MULT_FACTOR  (0.1)
#define MOVE_MULT_FACTOR    (10)


double orig_accel_z = 0;
double orig_yaw = 0;
double adj_accel_z = 0, adj_gyro_x = 0, adj_gyro_y = 0;
double accel_z, gyro_x, gyro_y;

double hover_throttle = ESC_WORKING_MIN;
boolean hover_found = false;

unsigned long repos_last_time;
unsigned long repos_remaining_time;

double v_ac, v_bd, velocity, bal_axes;
double zero_value = 0.0;

int presample_count  = PRESAMPLE_COUNT;
boolean resample_sensor = false;

MPU6050 mpu;
Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
int16_t accel_data[3];
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};
double yaw;

uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t in_error;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

volatile bool mpuInterrupt = false;    //interrupt flag

int did_takeoff = 0;
int doing_takeoff = 0;

PID xPID(&gyro_x, &v_ac, &adj_gyro_x, PID_GYRO_P, PID_GYRO_I, PID_GYRO_D, DIRECT);
PID yPID(&gyro_y, &v_bd,  &adj_gyro_y, PID_GYRO_P, PID_GYRO_I, PID_GYRO_D, DIRECT);
PID vPID(&accel_z, &velocity, &adj_accel_z, PID_ACCEL_P, PID_ACCEL_I, PID_ACCEL_D, REVERSE);
PID yawPID(&yaw, &bal_axes, &zero_value, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);

void setup() {
  Serial.begin(115200);

  initServo();
  
  arm(0);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Bridge.begin();
  Console.begin();
  FileSystem.begin();
  delay(5000);
  digitalWrite(13, LOW);
  
  xPID.SetMode(AUTOMATIC);
  xPID.SetSampleTime(PID_SAMPLE_PERIOD);
  xPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  yPID.SetMode(AUTOMATIC);
  yPID.SetSampleTime(PID_SAMPLE_PERIOD);
  yPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  vPID.SetMode(AUTOMATIC);
  vPID.SetOutputLimits(-PID_THROTTLE_INFLUENCE, PID_THROTTLE_INFLUENCE);

  initMPU();
}

void loop() {
  while(!mpuInterrupt && fifoCount < packetSize){

    /* Do nothing while MPU is not working
     * This should be a VERY short period
     */
      
  }
  if (!getYPR())
    return;  
  count_presample();
  
  if((did_takeoff || doing_takeoff) && !in_error)
    position_adjust();
  
  if (Console.available()) {
    process();
  }
}

void reset_pid_output(void)
{
  v_ac = 0.0;
  v_bd = 0.0;
  bal_axes = 0.0;
  velocity = 0.0;
}

void initServo(void)
{
  DDRB |= (1 << 7) | (1 << 6) | (1 << 5);
  ICR1 = 0x1387;  // 400Hz
  TCCR1A = 0b10101010;
  TCCR1B = 0b00011010;
  DDRC |= (1 << PC6); // motor3  PWM Port.
  ICR3 = 0x1387; 
  TCCR3A = 0b10101010;
  TCCR3B = 0b00011010;
}

void count_presample(void)
{
  if (in_error) return;
  
  if (presample_count > PRESAMPLE_STABLE_CHECK) {
    presample_count --;
    if (presample_count % 100 == 50) digitalWrite(13, LOW);
    if (presample_count % 100 == 1) digitalWrite(13, HIGH);
  } else if (presample_count == PRESAMPLE_STABLE_CHECK) {
    presample_count --;
    resample_sensor = true;
  } else if (presample_count < PRESAMPLE_STABLE_CHECK && presample_count >= 0) {
    presample_count --;
    if (accel_z > PRESAMPLE_STABLE_TOLERANCE || accel_z < -PRESAMPLE_STABLE_TOLERANCE) in_error = 1;
    
    if (in_error) {
      digitalWrite(13, HIGH);
      presample_count = PRESAMPLE_COUNT;
      in_error = 0;
      initMPU();
    }
  }
}

void print_sensors(void) {
#ifdef VERBOSE_DEBUG
  Serial.print(F("t : "));
  Serial.println(millis());
#endif
  Serial.print(F("gx : "));
  Serial.println(gyro_x);
#ifdef VERBOSE_DEBUG
  Serial.print(F("gy : "));
  Serial.println(gyro_y);
  Serial.print(F("adj_accel_z : "));
  Serial.println(adj_accel_z);
  Serial.print(F("accel_z : "));
  Serial.println(accel_z);
#endif
}

void initMPU(){
  int i;
  
  Wire.begin();
  mpu.initialize();
  in_error = mpu.dmpInitialize();
  if(in_error == 0){
    mpu.setDMPEnabled(true);
    attachInterrupt(4, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println(F("E."));
    digitalWrite(13, 1);
  }
}

inline void dmpDataReady() {
    mpuInterrupt = true;
}

void set_servos(void)
{
  int va, vb, vc, vd;
  int regVal;

  va = (hover_throttle + velocity) * ((100.0 + v_ac)/100.0) * ((100.0+bal_axes)/100.0) + 0.5;
  vb = (hover_throttle + velocity) * ((100.0 + v_bd)/100.0) * (abs(-100.0+bal_axes)/100.0) + 0.5;
  vc = (hover_throttle + velocity) * (abs(-100.0 + v_ac)/100.0) * ((100.0+bal_axes)/100.0) + 0.5;
  vd = (hover_throttle + velocity) * (abs(-100.0 + v_bd)/100.0) * (abs(-100.0+bal_axes)/100.0) + 0.5;
  
  if (va > ESC_MAX) va = ESC_MAX;
  if (vb > ESC_MAX) vb = ESC_MAX;
  if (vc > ESC_MAX) vc = ESC_MAX;
  if (vd > ESC_MAX) vd = ESC_MAX;

  if (va < ESC_WORKING_MIN) va = ESC_WORKING_MIN;
  if (vb < ESC_WORKING_MIN) vb = ESC_WORKING_MIN;
  if (vc < ESC_WORKING_MIN) vc = ESC_WORKING_MIN;
  if (vd < ESC_WORKING_MIN) vd = ESC_WORKING_MIN;
  
  regVal = map(va, ESC_MIN, ESC_MAX, ESC_REG_MIN, ESC_REG_MAX);
  ESC_A_REG = regVal;
  regVal = map(vb, ESC_MIN, ESC_MAX, ESC_REG_MIN, ESC_REG_MAX);
  ESC_B_REG = regVal;
  regVal = map(vc, ESC_MIN, ESC_MAX, ESC_REG_MIN, ESC_REG_MAX);
  ESC_C_REG = regVal;
  regVal = map(vd, ESC_MIN, ESC_MAX, ESC_REG_MIN, ESC_REG_MAX);
  ESC_D_REG = regVal;

  Serial.print(F("ac : "));
  Serial.println(v_ac);
#ifdef VERBOSE_DEBUG
  Serial.print(F("bd : "));
  Serial.println(v_bd);
  Serial.print(F("vel : "));
  Serial.println(velocity);
  Serial.println();
#endif
}

void reset_adjust_variables(void)
{
  adj_accel_z = 0;
  adj_gyro_x = adj_gyro_y  = 0;
  repos_last_time = millis();
}

void position_adjust(void)
{
  unsigned long current_time;

  if (!hover_found && accel_z < TAKEOFF_Z_ACCEL) {
    hover_found = true;
    doing_takeoff = 0;
    did_takeoff = 1;
    Serial.print(F("Hover found :"));
    Serial.println(hover_throttle);
  }

  if (repos_last_time == 0) repos_last_time = millis();
  current_time = millis();
  
  if (current_time - repos_last_time > repos_remaining_time) {
    repos_remaining_time = 0;
    reset_adjust_variables();
  } else {
    repos_remaining_time -= current_time - repos_last_time;
    repos_last_time = current_time;
  }
  
  if (hover_found) {
    xPID.Compute();
    yPID.Compute();
    //vPID.Compute();
  } else {
    hover_throttle += TAKEOFF_THROTTLE_STEP;
  }
  
  print_sensors();  
  set_servos();
}

boolean getYPR(){
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if((mpuIntStatus & 0x10) || fifoCount >= 1024){
    mpu.resetFIFO(); 
    Serial.print(F("rf : "));
    Serial.println(millis());
    return false;
  } else if(mpuIntStatus & 0x02){
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (resample_sensor) {
      resample_sensor = false;
      orig_yaw = ypr[0] * 180 / M_PI;
      orig_accel_z = ((double)accel_data[2]) / 100.0;
        
      Serial.println(F("R."));
    }

    ypr[2] = ypr[2] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[0] = ypr[0] * 180 / M_PI;

    if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
    if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
    if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];

    yprLast[0] = ypr[0];
    yprLast[1] = ypr[1];
    yprLast[2] = ypr[2];

    gyro_x = - ypr[2];
    gyro_y = ypr[1];
    yaw = ypr[0] - orig_yaw;

    mpu.dmpGetAccel(accel_data, fifoBuffer);
    accel_z = ((double)accel_data[2]) / 100.0 - orig_accel_z;

    return true;
  }
}

void arm(int delay_req)
{
  ESC_A_REG = ESC_REG_MIN;
  ESC_B_REG = ESC_REG_MIN;
  ESC_C_REG = ESC_REG_MIN;
  ESC_D_REG = ESC_REG_MIN;
  
  if (delay_req)
    delay(ESC_ARM_DELAY);
}

void process(void)
{
  char command = Console.read();
  Serial.println(command);
  if (presample_count > 0) {
    Serial.println(F("NR."));
    return;
  }
  
  if (command == 'x') {
    repos_remaining_time = 0;
    did_takeoff = 0;
    doing_takeoff = 0;
    reset_adjust_variables();
    arm(0);
  }

  if (doing_takeoff) {
    return;
  }
  
  double param = 1.0;
  if(command == 'p') {
    if (!did_takeoff) {
      hover_throttle = ESC_WORKING_MIN;
      hover_found = false;
      doing_takeoff = 1;
      did_takeoff = 0;
      reset_pid_output();
      reset_adjust_variables();
      repos_remaining_time = TAKEOFF_GOUP_DELAY;
      resample_sensor = true;
      adj_accel_z = - param * UPDOWN_MULT_FACTOR;
    } else {
      velocity += 5;
      repos_remaining_time = MOVE_DURATION_MS;
      adj_accel_z = - param * UPDOWN_MULT_FACTOR;
    }
  } if (command == 'l') {
    velocity -= 5;
    repos_remaining_time = MOVE_DURATION_MS;
    adj_accel_z = param * UPDOWN_MULT_FACTOR;
  }else if (command == 'w') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = -param * MOVE_MULT_FACTOR;
    adj_gyro_y = -param * MOVE_MULT_FACTOR;
  } else if (command == 's') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = param * MOVE_MULT_FACTOR;
    adj_gyro_y = param * MOVE_MULT_FACTOR;
  } else if (command == 'a') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = -param * MOVE_MULT_FACTOR;
    adj_gyro_y = param * MOVE_MULT_FACTOR;
  } else if (command == 'd') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = param * MOVE_MULT_FACTOR;
    adj_gyro_y = -param * MOVE_MULT_FACTOR;
  }
}
