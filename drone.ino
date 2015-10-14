#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Console.h>
#include <FileIO.h>

//#define VERBOSE_DEBUG

#define PID_GYRO_P            5
#define PID_GYRO_I            10
#define PID_GYRO_D            3

#define PID_ACCEL_P           10
#define PID_ACCEL_I           10
#define PID_ACCEL_D           2

#define PID_GYRO_SAMPLE_PERIOD     10

#define PRESAMPLE_COUNT     1500
#define PRESAMPLE_STABLE_CHECK  30
#define PRESAMPLE_STABLE_TOLERANCE  0.02

#define REPOSITION_PERIOD_MS  30ul
#define MOVE_DURATION_MS      2000

#define ESC_MIN               22
#define ESC_WORKING_MIN       75
#define ESC_MAX               150
#define ESC_ARM_DELAY         5000
  
#define ESC_A 9
#define ESC_B 8

#define ESC_C 6
#define ESC_D 5

#define TAKEOFF_Z_ACCEL       (-0.08)
#define TAKEOFF_THROTTLE_STEP 0.03
#define TAKEOFF_GOUP_DELAY    5000

#define PID_XY_INFLUENCE    20.0
#define PID_THROTTLE_INFLUENCE  70.0

#define UPDOWN_MULT_FACTOR  (0.3)
#define MOVE_MULT_FACTOR    (0.1)


double orig_accel_z = 0;
double orig_gyro_x = 0, orig_gyro_y = 0;
double adj_accel_z = 0, adj_gyro_x = 0, adj_gyro_y = 0;
double accel_z, gyro_x, gyro_y;

double hover_throttle = ESC_WORKING_MIN;
boolean hover_found = false;

unsigned long repos_last_time;
unsigned long repos_remaining_time;

double v_ac, v_bd, velocity;

int presample_count  = PRESAMPLE_COUNT;
boolean resample_sensor = false;

Servo a, b, c, d;
PID xPID(&gyro_x, &v_ac, &adj_gyro_x, PID_GYRO_P, PID_GYRO_I, PID_GYRO_D, DIRECT);
PID yPID(&gyro_y, &v_bd,  &adj_gyro_y, PID_GYRO_P, PID_GYRO_I, PID_GYRO_D, DIRECT);
PID vPID(&accel_z, &velocity, &adj_accel_z, PID_ACCEL_P, PID_ACCEL_I, PID_ACCEL_D, REVERSE);

MPU6050 mpu;
Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
int16_t accel_data[3];
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t in_error;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

volatile bool mpuInterrupt = false;    //interrupt flag

int did_takeoff = 0;
int doing_takeoff = 0;

void setup() {
  Serial.begin(115200);

  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  delay(100);
  
  arm(0);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Bridge.begin();
  Console.begin();
  FileSystem.begin();
  digitalWrite(13, LOW);
  
  xPID.SetMode(AUTOMATIC);
  xPID.SetSampleTime(PID_GYRO_SAMPLE_PERIOD);
  xPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  yPID.SetMode(AUTOMATIC);
  yPID.SetSampleTime(PID_GYRO_SAMPLE_PERIOD);
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
  getYPR();  
  count_presample();
  if((did_takeoff || doing_takeoff) && !in_error)
    position_adjust();
  
  if (Console.available()) {
    process();
  }
}

void count_presample(void)
{
  if (in_error) return;
  
  if (presample_count > PRESAMPLE_STABLE_CHECK) {
    presample_count --;
    if (presample_count % 100 == 0) Serial.println(presample_count / 100);
    if (presample_count % 100 == 50) digitalWrite(13, LOW);
    if (presample_count % 100 == 1) digitalWrite(13, HIGH);
  } else if (presample_count == PRESAMPLE_STABLE_CHECK) {
    presample_count --;
    resample_sensor = true;
  } else if (presample_count < PRESAMPLE_STABLE_CHECK && presample_count >= 0) {
    presample_count --;
    if (gyro_x > PRESAMPLE_STABLE_TOLERANCE || gyro_x < -PRESAMPLE_STABLE_TOLERANCE) in_error = 1;
    if (gyro_y > PRESAMPLE_STABLE_TOLERANCE || gyro_y < -PRESAMPLE_STABLE_TOLERANCE) in_error = 1;
    if (accel_z > PRESAMPLE_STABLE_TOLERANCE || accel_z < -PRESAMPLE_STABLE_TOLERANCE) in_error = 1;

    if (in_error) {
      digitalWrite(13, HIGH);
      presample_count = PRESAMPLE_COUNT;
      in_error = 0;
      orig_gyro_x = orig_gyro_y = orig_accel_z = 0;
      initMPU();
    }
  }
}

void print_sensors(void) {
#ifdef VERBOSE_DEBUG
  Serial.print(F("t : "));
  Serial.println(millis());
  Serial.print(F("gx : "));
  Serial.println(gyro_x);
  Serial.print(F("gy : "));
  Serial.println(gyro_y);
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

  va = (hover_throttle + velocity) * ((100.0 + v_ac)/100.0) + 0.5;
  vb = (hover_throttle + velocity) * ((100.0 + v_bd)/100.0) + 0.5;
  vc = (hover_throttle + velocity) * (abs(-100.0 + v_ac)/100.0) + 0.5;
  vd = (hover_throttle + velocity) * (abs(-100.0 + v_bd)/100.0) + 0.5;
  
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

#ifdef VERBOSE_DEBUG
  Serial.print(F("ac : "));
  Serial.println(v_ac);
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
    vPID.Compute();
  } else {
    hover_throttle += TAKEOFF_THROTTLE_STEP;
  }
  
  print_sensors();  
  set_servos();
}

void getYPR(){
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if((mpuIntStatus & 0x10) || fifoCount >= 1024){
    mpu.resetFIFO(); 
    Serial.print(F("rf : "));
    Serial.println(millis());
  } else if(mpuIntStatus & 0x02){
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (resample_sensor) {
      resample_sensor = false;
      orig_gyro_x = ypr[2];
      orig_gyro_y = ypr[1];
      orig_accel_z = ((double)accel_data[2]) / 10000.0;
        
      Serial.println(F("R."));
      Serial.println(orig_accel_z);
      Serial.println(orig_gyro_x);
      Serial.println(orig_gyro_y);
    }

    gyro_x = orig_gyro_x - ypr[2] ;
    gyro_y = ypr[1] - orig_gyro_y;

    mpu.dmpGetAccel(accel_data, fifoBuffer);
    accel_z = ((double)accel_data[2]) / 10000.0 - orig_accel_z;
  }
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
      reset_adjust_variables();
      repos_remaining_time = TAKEOFF_GOUP_DELAY;
      resample_sensor = true;
      adj_accel_z = - param * UPDOWN_MULT_FACTOR;
    } else {
      repos_remaining_time = MOVE_DURATION_MS;
      adj_accel_z = - param * UPDOWN_MULT_FACTOR;
    }
  } if (command == 'l') {
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
