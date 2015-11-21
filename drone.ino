/*
 *  
 *  https://github.com/strangedev/Arduino-Quadcopter
 *  http://andrea-toscano.com/400hz-pwm-on-atmega32u4-for-multirotors-without-using-servo-library/
 *  
 */

#include <I2Cdev.h>
#include <Wire.h>
#include <PID_v1.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PinChangeInt.h>

//#define VERBOSE_DEBUG

#define PID_GYRO_P            4.7
#define PID_GYRO_I            0.9
#define PID_GYRO_D            2.5

#define SDA_PIN     (20)
#define SCL_PIN     (21)

#define RC_1  A9
#define RC_2  A10
#define RC_3  A11
#define RC_4  A12
#define RC_5  A13
#define RC_6  A14
#define RC_7  A15

#define PID_SAMPLE_PERIOD     5

#define PRESAMPLE_COUNT     500
#define PRESAMPLE_STABLE_CHECK  30
#define PRESAMPLE_STABLE_TOLERANCE  100.0

#define ESC_MIN               88
#define ESC_WORKING_MIN       160
#define ESC_MAX               460
#define ESC_ARM_DELAY         500

/* RC configuration */
#define RC_HIGH_CH1            1844
#define RC_LOW_CH1             1032
#define RC_HIGH_CH2            1856
#define RC_LOW_CH2             1032
#define RC_HIGH_CH3            1800
#define RC_LOW_CH3             1032
#define RC_HIGH_CH4           1840
#define RC_LOW_CH4             1032
#define RC_HIGH_CH5            1900
#define RC_LOW_CH5             1700


#define RC_ROUNDING_BASE      50

/* Fligh parameters */
#define PITCH_MIN             -15
#define PITCH_MAX             15
#define ROLL_MIN      -15
#define ROLL_MAX      15
#define YAW_MIN       -30
#define YAW_MAX       30

#define MPU_INT_PIN 2

#define ESC_A    12
#define ESC_B    11
#define ESC_C    8
#define ESC_D    7
#define ESC_T    6

#define ESC_A_REG  OCR1B
#define ESC_B_REG  OCR1A
#define ESC_C_REG  OCR4C
#define ESC_D_REG  OCR4B
#define ESC_T_REG  OCR4A

#define ESC_REG_HIGH  3800
#define ESC_REG_LOW   2100

#define PID_XY_INFLUENCE    30.0

double adj_accel_z = 0, adj_gyro_x = 0, adj_gyro_y = 0;
double accel_z, gyro_x, gyro_y;

double v_ac, v_bd, velocity, bal_axes;

int presample_count  = PRESAMPLE_COUNT;
boolean resample_sensor = false;

MPU6050 mpu;
Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;
int16_t accel_data[3];
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool interruptLock = false;

uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t in_error;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

volatile bool mpuInterrupt = false;    //interrupt flag

double ch1, ch2, ch3, ch4;
int ch5, ch6, ch7;
unsigned long rcLastChange1 = micros();
unsigned long rcLastChange2 = micros();
unsigned long rcLastChange3 = micros();
unsigned long rcLastChange4 = micros();
unsigned long rcLastChange5 = micros();
unsigned long rcLastChange6 = micros();
unsigned long rcLastChange7 = micros();

double ch1Last = 0.0;
double ch2Last = 0.0;
double ch4Last = 0.0;
double velocityLast = 0.0;

bool engine_on = true;

double pid_gyro_p = PID_GYRO_P;
double pid_gyro_i = PID_GYRO_I;
double pid_gyro_d = PID_GYRO_D;

PID xPID(&gyro_x, &v_ac, &adj_gyro_x, pid_gyro_p, pid_gyro_i, pid_gyro_d, DIRECT);
PID yPID(&gyro_y, &v_bd,  &adj_gyro_y, pid_gyro_p, pid_gyro_i, pid_gyro_d, DIRECT);

void setup() {
  resetI2C();
  Wire.begin();
  Wire.setClock(400000L);
  
  initRC();
  
  Serial.begin(115200);
  
  pinMode(13, OUTPUT);

  Serial.println(F("Init Servos.."));
  initServo();

  Serial.println(F("Arming.."));
  arm(1);

  Serial.println(F("Init PID.."));
  xPID.SetMode(AUTOMATIC);
  xPID.SetSampleTime(PID_SAMPLE_PERIOD);
  xPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  yPID.SetMode(AUTOMATIC);
  yPID.SetSampleTime(PID_SAMPLE_PERIOD);
  yPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
    
  Serial.println(F("MPU init.."));
  initMPU();
}

void loop() {
  while(!mpuInterrupt && fifoCount < packetSize){
    
  }

  if (!getYPR())
    return;
    
  count_presample();
  
  if(presample_count < 0 && !in_error)
    position_adjust();
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
  pinMode(ESC_A, OUTPUT);
  pinMode(ESC_B, OUTPUT);
  pinMode(ESC_C, OUTPUT);
  pinMode(ESC_D, OUTPUT);

  delay(500);

  ICR1 = 0x1387;
  TCCR1A = 0b10101010;
  TCCR1B = 0b00011010;
  ICR4 = 0x1387;
  TCCR4A = 0b10101010;
  TCCR4B = 0b00011010;
  
  ESC_A_REG = ESC_REG_LOW;
  ESC_B_REG = ESC_REG_LOW;
  ESC_C_REG = ESC_REG_LOW;
  ESC_D_REG = ESC_REG_LOW;
  ESC_T_REG = ESC_REG_LOW;
}

void count_presample(void)
{
  if (in_error) return;
  
  if (presample_count > PRESAMPLE_STABLE_CHECK) {
    presample_count --;
    if (presample_count % 100 == 50) digitalWrite(13, LOW);
    if (presample_count % 100 == 1) digitalWrite(13, HIGH);
    if (presample_count % 100 == 0) Serial.println(presample_count / 100);
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

void initMPU(){
  int i;
  
  mpu.initialize();
  in_error = mpu.dmpInitialize();

#if 0
  mpu.setXAccelOffset(-197);
  mpu.setYAccelOffset(-1983);
  mpu.setZAccelOffset(676);
  mpu.setXGyroOffset(76);
  mpu.setYGyroOffset(-32);
  mpu.setZGyroOffset(69);
#endif

  if(in_error == 0){
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.setDMPEnabled(true);
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

  va = ESC_MIN + (velocity - ESC_MIN) * ((100.0 + v_ac)/100.0) * ((100.0+bal_axes)/100.0) + 0.5;
  vb = ESC_MIN + (velocity - ESC_MIN) * ((100.0 + v_bd)/100.0) * (abs(-100.0+bal_axes)/100.0) + 0.5;
  vc = ESC_MIN + (velocity - ESC_MIN) * (abs(-100.0 + v_ac)/100.0) * ((100.0+bal_axes)/100.0) + 0.5;
  vd = ESC_MIN + (velocity - ESC_MIN) * (abs(-100.0 + v_bd)/100.0) * (abs(-100.0+bal_axes)/100.0) + 0.5;

  if (va > ESC_MAX) va = ESC_MAX;
  if (vb > ESC_MAX) vb = ESC_MAX;
  if (vc > ESC_MAX) vc = ESC_MAX;
  if (vd > ESC_MAX) vd = ESC_MAX;

  if (va < ESC_MIN) va = ESC_MIN;
  if (vb < ESC_MIN) vb = ESC_MIN;
  if (vc < ESC_MIN) vc = ESC_MIN;
  if (vd < ESC_MIN) vd = ESC_MIN;

  if (!engine_on) {
    va = ESC_MIN; 
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  }

  regVal = map(va, ESC_MIN, ESC_MAX, ESC_REG_LOW, ESC_REG_HIGH);
  ESC_A_REG = regVal;
  regVal = map(vb, ESC_MIN, ESC_MAX, ESC_REG_LOW, ESC_REG_HIGH);
  ESC_B_REG = regVal;
  regVal = map(vc, ESC_MIN, ESC_MAX, ESC_REG_LOW, ESC_REG_HIGH);
  ESC_C_REG = regVal;
  regVal = map(vd, ESC_MIN, ESC_MAX, ESC_REG_LOW, ESC_REG_HIGH);
  ESC_D_REG = regVal;


#if 0  
  
  Serial.print(F("gx : "));
  Serial.println(gyro_x);
  Serial.print(F("gy : "));
  Serial.println(gyro_y); 
  Serial.print(F("bal_axes: "));
  Serial.println(bal_axes);

  Serial.print(F("t : "));
  Serial.println(millis());
 
  Serial.print(F("adj_accel_z : "));
  Serial.println(adj_accel_z);
  Serial.print(F("accel_z : "));
  Serial.println(accel_z);
  Serial.print(F("ac : "));
  Serial.println(v_ac);
  Serial.print(F("bd : "));
  Serial.println(v_bd);
  Serial.print(F("vel : "));
  Serial.println(velocity);
  Serial.print("va:"); Serial.println(va);
  Serial.print("vb:"); Serial.println(vb);
  Serial.print("vc:"); Serial.println(vc);
  Serial.print("vd:"); Serial.println(vd);
#endif
}

void reset_adjust_variables(void)
{
  adj_accel_z = 0;
  adj_gyro_x = adj_gyro_y  = 0;
}

void position_adjust(void)
{
#if 0
  Serial.print("CH1:");Serial.println(ch1);
  Serial.print("CH2:");Serial.println(ch2);
  Serial.print("CH3:");Serial.println(ch3);
  Serial.print("CH4:");Serial.println(ch4);
#endif

  double temp_p, temp_i, temp_d;
  
  acquireLock();
  ch1 = floor(ch1/RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch2 = floor(ch2/RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch3 = floor(ch3/RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch4 = floor(ch4/10) * 10;
  
  ch2 = map(ch2, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH4, RC_HIGH_CH4, YAW_MIN, YAW_MAX);
  velocity = map(ch3, RC_LOW_CH3, RC_HIGH_CH3, ESC_MIN, ESC_MAX);

  if ((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if ((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if ((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;
  if ((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;

  if (ch1 < ROLL_MIN + 10 && velocity < ESC_MIN + 10) engine_on = false;

  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  velocityLast = velocity;

  adj_gyro_x = - ch2 - ch1;
  adj_gyro_y = - ch2 + ch1;
  bal_axes = ch4;

  temp_p = (ch6 - 1000 + 15) / 30 * 30;
  temp_i = (ch7 - 1000 + 25) / 50 * 50;
  temp_d = (ch5 - 1000 + 25) / 50 * 50;
  
  releaseLock();

  temp_p /= 50.0;
  if (temp_p >= 0 && temp_p < 1000 / 50 && temp_p != pid_gyro_p) {
    pid_gyro_p = temp_p;
    Serial.print(F("P:"));Serial.println(pid_gyro_p);
    xPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
    yPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
  }

  temp_i /= 1000.0;
  temp_i -= 0.05;
  if (temp_i >= 0 && temp_i < 1000.0 / 1000.0 && temp_i != pid_gyro_i) {
    pid_gyro_i = temp_i;
    Serial.print(F("I:"));Serial.println(pid_gyro_i);
    xPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
    yPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
  }
  
  temp_d /= 100.0;
  temp_d -= 0.5;
  if (temp_d >= 0 && temp_d < 1000.0 / 100.0 && temp_d != pid_gyro_d) {
    pid_gyro_d = temp_d;
    Serial.print(F("D:"));Serial.println(pid_gyro_d);
    xPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
    yPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
  }

#if 0
  Serial.print(F("ch1:")); Serial.println(ch1);
  Serial.print(F("ch2:")); Serial.println(ch2);
  Serial.print(F("ch4:")); Serial.println(ch4);
#endif

  xPID.Compute();
  yPID.Compute();
  
  set_servos();
}

boolean getYPR(){
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if((mpuIntStatus & 0x10) || fifoCount >= 1024){
    Serial.print(F("fifocount : "));Serial.println(fifoCount);
    mpu.resetFIFO(); 
    return false;
  } else if(mpuIntStatus & 0x02){
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    do {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    } while (fifoCount > packetSize);
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (resample_sensor) {
      resample_sensor = false;
        
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

    gyro_x = ypr[2];
    gyro_y = - ypr[1];

    return true;
  }
}

void arm(int delay_req)
{
  ESC_A_REG = ESC_REG_LOW;
  ESC_B_REG = ESC_REG_LOW;
  ESC_C_REG = ESC_REG_LOW;
  ESC_D_REG = ESC_REG_LOW;
  
  if (delay_req)
    delay(ESC_ARM_DELAY);
}

void initRC() {
  pinMode(RC_1, INPUT);
  pinMode(RC_2, INPUT);
  pinMode(RC_3, INPUT);
  pinMode(RC_4, INPUT);
  pinMode(RC_5, INPUT);
  pinMode(RC_6, INPUT);
  pinMode(RC_7, INPUT);
  PCintPort::attachInterrupt(RC_1, rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(RC_3, rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(RC_4, rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(RC_5, rcInterrupt5, CHANGE);
  PCintPort::attachInterrupt(RC_6, rcInterrupt6, CHANGE);
  PCintPort::attachInterrupt(RC_7, rcInterrupt7, CHANGE);
}

void rcInterrupt1() {
  if (!interruptLock) ch1 = micros() - rcLastChange1;
  rcLastChange1 = micros();
}

void rcInterrupt2() {
  if (!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

void rcInterrupt3() {
  if (!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

void rcInterrupt4() {
  if (!interruptLock) ch4 =  micros() - rcLastChange4;
  rcLastChange4 = micros();
}

void rcInterrupt5() {
  if (!interruptLock) ch5 =  micros() - rcLastChange5;
  rcLastChange5 = micros();
}

void rcInterrupt6() {
  if (!interruptLock) ch6 =  micros() - rcLastChange6;
  rcLastChange6 = micros();
}

void rcInterrupt7() {
  if (!interruptLock) ch7 =  micros() - rcLastChange7;
  rcLastChange7 = micros();
}

inline void acquireLock() {
  interruptLock = true;
}

inline void releaseLock() {
  interruptLock = false;
}

void resetI2C(void)
{
  // reset I2C bus
  // This ensures that if the nav6 was reset, but the devices
  // on the I2C bus were not, that any transfer in progress do
  // not hang the device/bus.  Since the MPU-6050 has a 1024-byte
  // fifo, and there are 8 bits/byte, 10,000 clock edges
  // are generated to ensure the fifo is completely cleared
  // in the worst case.

  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, OUTPUT);

  // Clock through up to 1000 bits
  int x = 0;
  for ( int i = 0; i < 10000; i++ ) {

    digitalWrite(SCL_PIN, HIGH);
    digitalWrite(SCL_PIN, LOW);
    digitalWrite(SCL_PIN, HIGH);

    x++;
    if ( x == 8 ) {
      x = 0;
      // send a I2C stop signal
      pinMode(SDA_PIN, OUTPUT);
      digitalWrite(SDA_PIN, HIGH);
      digitalWrite(SDA_PIN, LOW);
      pinMode(SDA_PIN, INPUT);
    }
  }

  // send a I2C stop signal
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SDA_PIN, LOW);
}
