#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define REPOSITION_PERIOD_MS  30ul
#define MOVE_DURATION_MS      1000

#define ESC_MIN               22
#define ESC_WORKING_MIN       70
#define ESC_MAX               150
#define ESC_ARM_DELAY         5000
  
#define ESC_A 9
#define ESC_B 8
#define ESC_C 6
#define ESC_D 5

#define TAKEOFF_Z_ACCEL      100
#define TAKEOFF_STEP_DELAY     500
#define TAKEOFF_GOUP_DELAY    5000

#define PID_XY_INFLUENCE    20.0

#define GYRO_READ_AVERAGE_COUNT  20

double orig_accel_z;
double orig_gyro_x, orig_gyro_y;
double adj_accel_z;
double adj_gyro_x, adj_gyro_y;
double accel_z;
double gyro_x, gyro_y;

unsigned long repos_last_time;
unsigned long repos_remaining_time;

double v_ac, v_bd, velocity;

Servo a, b, c, d;
PID xPID(&gyro_x, &v_ac, &adj_gyro_x, 1, 1, 1, REVERSE);
PID yPID(&gyro_y, &v_bd,  &adj_gyro_y, 1, 1, 1, DIRECT);
PID vPID(&accel_z, &velocity, &adj_accel_z, 20, 10, 1, REVERSE);

MPU6050 mpu;
Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

volatile bool mpuInterrupt = false;    //interrupt flag

int did_take_off = 0;


void setup() {
  Serial.begin(115200);

  Serial.println(F("initializing Motors")); 
  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  delay(100);
  
  Serial.println(F("Arming Motors"));
  arm(0);
  
  Serial.println(F("initializing PID"));
  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  yPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  vPID.SetMode(AUTOMATIC);
  vPID.SetOutputLimits(ESC_WORKING_MIN, ESC_MAX);

  Serial.println(F("initializing MPU6050"));
  initMPU();
}

void loop() {
  if (Serial.available()) {
    process();
  }

  while(!mpuInterrupt && fifoCount < packetSize){
     
    /* Do nothing while MPU is not working
     * This should be a VERY short period
     */
      
  }

  getYPR();
  
  if(did_take_off)
    position_adjust();
}

void initMPU(){
  int i;
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(4, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

inline void dmpDataReady() {
    mpuInterrupt = true;
}

void set_servos(void)
{
  double va, vb, vc, vd;
  
  va = velocity * ((100.0 + v_ac)/100.0);
  vb = velocity * ((100.0 + v_bd)/100.0);
  vc = velocity * (abs(-100.0 + v_ac)/100.0);
  vd = velocity * (abs(-100.0 + v_bd)/100.0);

  if (va > ESC_MAX) va = ESC_MAX;
  if (vb > ESC_MAX) vb = ESC_MAX;
  if (vc > ESC_MAX) vc = ESC_MAX;
  if (vd > ESC_MAX) vd = ESC_MAX;

  if (va < ESC_WORKING_MIN) va = ESC_WORKING_MIN;
  if (vb < ESC_WORKING_MIN) vb = ESC_WORKING_MIN;
  if (vc < ESC_WORKING_MIN) vc = ESC_WORKING_MIN;
  if (vd < ESC_WORKING_MIN) vd = ESC_WORKING_MIN;
  
  a.write(va);
  b.write(vb);
  c.write(vc);
  d.write(vd);

  Serial.print(F("v_ac :"));
  Serial.println(v_ac);
  Serial.print(F("v_bd :"));
  Serial.println(v_bd);
  Serial.print(F("va :"));
  Serial.println(va);
  Serial.print(F("vb :"));
  Serial.println(vb);
  Serial.print(F("vc :"));
  Serial.println(vc);
  Serial.print(F("vd :"));
  Serial.println(vd);
  Serial.println();
}

void reset_adjust_variables(void)
{
  adj_accel_z = 0;
  adj_gyro_x = adj_gyro_y  = 0;
  repos_last_time = millis();
}

void print_adjust_variables()
{
  if (adj_accel_z) {
    Serial.print(F("adj_accel_z : "));
    Serial.println(adj_accel_z);
  }

  if (adj_gyro_x) {
    Serial.print(F("adj_gyro_x : "));
    Serial.println(adj_gyro_x);
  }

  if (adj_gyro_y) {
    Serial.print(F("adj_gyro_y : "));
    Serial.println(adj_gyro_y);
  }
}

int first_sample = 1;

void position_adjust(void)
{
  unsigned long current_time;

  if (repos_last_time == 0) repos_last_time = millis();
  current_time = millis();
  Serial.print(F("repos_remaining_time : "));
  Serial.println(repos_remaining_time);
  
  if (current_time - repos_last_time > repos_remaining_time) {
    repos_remaining_time = 0;
    reset_adjust_variables();
  } else {
    repos_remaining_time -= current_time - repos_last_time;
    repos_last_time = current_time;
  }

  print_adjust_variables();

 if (first_sample) {
    orig_accel_z = accel_z;
    orig_gyro_x = gyro_x;
    orig_gyro_y = gyro_y;
    first_sample = 0;
    Serial.print(F("orig_accel_z : "));
    Serial.println(
    return;
  }

  Serial.print(F("accel_z : "));
  Serial.println(accel_z);
  Serial.print(F("gyro_x : "));
  Serial.println(gyro_x);
  Serial.print(F("gyro_y : "));
  Serial.println(gyro_y);
  
  xPID.Compute();
  yPID.Compute();
  vPID.Compute();

  Serial.println(velocity);
  
  set_servos();
}

void getYPR(){
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
      mpu.resetFIFO(); 
    }else if(mpuIntStatus & 0x02){
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }

    gyro_x = ypr[2] - orig_gyro_x;
    gyro_y = ypr[1] - orig_gyro_y;
    accel_z = q.z - orig_accel_z;;
}

void arm(int delay_req)
{
  unsigned long take_off_time;
    
  a.write(ESC_MIN);
  b.write(ESC_MIN);
  c.write(ESC_MIN);
  d.write(ESC_MIN);
  Serial.println(ESC_MIN);
  Serial.println();
  
  if (delay_req)
    delay(ESC_ARM_DELAY);
}

void process(void)
{
  char command = Serial.read();
  Serial.println(command);
  int param = 0.3;
  if(command == 'p') {
    if (!did_take_off) {
      reset_adjust_variables();
      repos_remaining_time = TAKEOFF_GOUP_DELAY;
      adj_accel_z = -param;
      did_take_off = 1;
    } else {
      repos_remaining_time = MOVE_DURATION_MS;
      adj_accel_z = -param;
    }
  } if (command == 'l') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_accel_z = -param;
  }else if (command == 'w') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = -param;
    adj_gyro_y = -param;
  } else if (command == 's') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = param;
    adj_gyro_y = param;
  } else if (command == 'a') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = -param;
    adj_gyro_y = param;
  } else if (command == 'd') {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = param;
    adj_gyro_y = -param;
  } else if (command == 'x') {
    repos_remaining_time = 0;
    did_take_off = 0;
    reset_adjust_variables();
    arm(1);
  }
}
