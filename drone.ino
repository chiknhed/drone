/*
 *  
 *  https://github.com/strangedev/Arduino-Quadcopter
 *  http://andrea-toscano.com/400hz-pwm-on-atmega32u4-for-multirotors-without-using-servo-library/
 *  
 */

#define EMPL_TARGET_ATMEGA328

#include <Wire.h>
#include <I2Cdev.h>
#include <PID_v1.h>
#include <math.h>
#include <helper_3dmath.h>
#include <PinChangeInt.h>
extern "C" {
  #include <inv_mpu.h>
  #include <inv_mpu_dmp_motion_driver.h>
}

//#define VERBOSE_DEBUG

#define PID_GYRO_P            4.7
#define PID_GYRO_I            0.9
#define PID_GYRO_D            2.5

#define SDA_PIN     (20)
#define SCL_PIN     (21)

#define DEFAULT_MPU_HZ     (200)
#define MAX_NAV6_MPU_RATE   (500)
#define MIN_NAV6_MPU_RATE   (4)

#define RC_1  A9
#define RC_2  A10
#define RC_3  A11
#define RC_4  A12
#define RC_5  A13
#define RC_6  A14
#define RC_7  A15

#define PID_SAMPLE_PERIOD     1

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

double adj_gyro_x = 0, adj_gyro_y = 0;
double accel_z, gyro_x, gyro_y;

double v_ac, v_bd, velocity, bal_axes;

unsigned char dmp_on;
volatile bool new_gyro;


/* Conversion Factors */
const float degrees_to_radians = M_PI / 180.0;
const float radians_to_degrees = 180.0 / M_PI;

/* MPU Calibration */
#define STARTUP_CALIBRATION_DELAY_MS        19000
#define CALIBRATED_OFFSET_AVERAGE_PERIOD_MS  1000

#define NAV6_CALIBRATION_STATE_WAIT 0 // Waiting for MPU to complete internal calibration
#define NAV6_CALIBRATION_STATE_ACCUMULATE  1 // Accumulate Yaw/Pitch/Roll offsets
#define NAV6_CALIBRATION_STATE_COMPLETE    2 // Normal Operation

int calibration_state = NAV6_CALIBRATION_STATE_WAIT;
int calibration_accumulator_count = 0;
float yaw_accumulator = 0.0;
float quaternion_accumulator[4] = { 0.0, 0.0, 0.0, 0.0 };
float calibrated_yaw_offset = 0.0;
float calibrated_quaternion_offset[4] = { 0.0, 0.0, 0.0, 0.0 };

/* Gyro / Accel / DMP State */
float ypr[3] = {0, 0, 0};
unsigned long sensor_timestamp;

struct FloatVectorStruct {
  float x;
  float y;
  float z;
};
struct FloatVectorStruct gravity;

unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000


volatile bool interruptLock = false;

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
  
  Serial.begin(115200);
  Serial.flush();

  Serial.println(F("MPU init.."));
  Serial.flush();
  initMPU();

  Serial.println(F("Init PID.."));
  Serial.flush();
  xPID.SetMode(AUTOMATIC);
  xPID.SetSampleTime(PID_SAMPLE_PERIOD);
  xPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  yPID.SetMode(AUTOMATIC);
  yPID.SetSampleTime(PID_SAMPLE_PERIOD);
  yPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  
  Serial.println(F("RC Interrupts Init.."));
  initRC();

  Serial.println(F("Init Servos.."));
  initServo();

  Serial.println(F("Arming.."));
  arm(1);
}

void loop() {
  if (new_gyro && dmp_on) {
    short gyro[3], accel[3], sensors;
    unsigned char more = 0;
    long quat[4];
    //float euler[3];
    /* This function gets new data from the FIFO when the DMP is in
     * use. The FIFO can contain any combination of gyro, accel,
     * quaternion, and gesture data. The sensors parameter tells the
     * caller which data fields were actually populated with new data.
     * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
     * the FIFO isn't being filled with accel data.
     * The driver parses the gesture data to determine if a gesture
     * event has occurred; on an event, the application will be notified
     * via a callback (assuming that a callback function was properly
     * registered). The more parameter is non-zero if there are
     * leftover packets in the FIFO.
     */
    int success = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                                &more);
    
    if (!more)
      new_gyro = 0;

    if ( ( success == 0 ) && ( (sensors & INV_XYZ_ACCEL) != 0 ) && ( (sensors & INV_WXYZ_QUAT) != 0 ) ) {

      Quaternion q( (float)(quat[0] >> 16) / 16384.0f,
                    (float)(quat[1] >> 16) / 16384.0f,
                    (float)(quat[2] >> 16) / 16384.0f,
                    (float)(quat[3] >> 16) / 16384.0f);

      // Calculate Yaw/Pitch/Roll
      // Update client with yaw/pitch/roll and tilt-compensated magnetometer data

      getGravity(&gravity, &q);
      dmpGetYawPitchRoll(ypr, &q, &gravity);

      boolean accumulate = false;
      if ( calibration_state == NAV6_CALIBRATION_STATE_WAIT ) {

        digitalWrite(13, HIGH);

        if ( millis() >= STARTUP_CALIBRATION_DELAY_MS ) {

          calibration_state = NAV6_CALIBRATION_STATE_ACCUMULATE;
        }
      }
      if ( calibration_state == NAV6_CALIBRATION_STATE_ACCUMULATE ) {

        accumulate = true;
        if ( millis() >= (STARTUP_CALIBRATION_DELAY_MS + CALIBRATED_OFFSET_AVERAGE_PERIOD_MS) ) {

          accumulate = false;
          calibrated_yaw_offset = yaw_accumulator / calibration_accumulator_count;
          calibrated_quaternion_offset[0] = quaternion_accumulator[0] / calibration_accumulator_count;
          calibrated_quaternion_offset[1] = quaternion_accumulator[1] / calibration_accumulator_count;
          calibrated_quaternion_offset[2] = quaternion_accumulator[2] / calibration_accumulator_count;
          calibrated_quaternion_offset[3] = quaternion_accumulator[3] / calibration_accumulator_count;
          calibration_state = NAV6_CALIBRATION_STATE_COMPLETE;
          digitalWrite(13, LOW);
        }
        else {

          calibration_accumulator_count++;

        }
      }

      float x = ypr[0] * radians_to_degrees;
      float y = ypr[1] * radians_to_degrees;
      float z = ypr[2] * radians_to_degrees;

      if ( accumulate ) {

        yaw_accumulator += x;
        quaternion_accumulator[0] += q.w;
        quaternion_accumulator[1] += q.x;
        quaternion_accumulator[2] += q.y;
        quaternion_accumulator[3] += q.z;

      }

      gyro_x = z;
      gyro_y = - y;
      accel_z = 0;

      if (calibration_state == NAV6_CALIBRATION_STATE_COMPLETE) position_adjust();
    }
    else {

      /* The following debug print outs are useful
         if DMP fifo streaming is not working or
         data is being lost.
         If modifying code, it's a good idea to check that
         the "FIFO OVERFLOW" error does not occur, since
         this case can occur if too many cycles are used
         to keep up with the sensor data stream. */
      if ( success == -1 )
      {
        Serial.println(F("DMP DISABLED!!!"));
      }
      else if ( success == -2 )
      {
        Serial.println(F("I2C READ ERROR"));
      }
      else if ( success == -3 )
      {
        Serial.println(F("FIFO OVERFLOW ERROR!!!"));
      }
      else if ( success == -4 )
      {
        Serial.println(F("NO_SENSORS"));
      }
      else if ( success == -6 )
      {
        Serial.println(F("CORRUPTED_QUATERNION"));
      }
    }
  }
}

void disable_mpu() {
  mpu_set_dmp_state(0);
  dmp_on = 0;
}

void enable_mpu() {
  mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
  dmp_on = 1;
}

void getGravity(struct FloatVectorStruct *v, Quaternion *q) {

  v -> x = 2 * (q -> x * q -> z - q -> w * q -> y);
  v -> y = 2 * (q -> w * q -> x + q -> y * q -> z);
  v -> z = q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z;
}

void dmpGetYawPitchRoll(float *data, Quaternion *q, struct FloatVectorStruct *gravity) {

  // yaw: (about Z axis)
  data[0] = atan2(2 * q -> x * q -> y - 2 * q -> w * q -> z, 2 * q -> w * q -> w + 2 * q -> x * q -> x - 1);
  // pitch: (nose up/down, about Y axis)
  data[1] = atan(gravity -> x / sqrt(gravity -> y * gravity -> y + gravity -> z * gravity -> z));
  // roll: (tilt left/right, about X axis)
  data[2] = atan(gravity -> y / sqrt(gravity -> x * gravity -> x + gravity -> z * gravity -> z));
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
void initMPU() {
  // MPU-6050 Initialization

  // Initialize the MPU:
  //
  // Gyro sensitivity:      2000 degrees/sec
  // Accel sensitivity:     2 g
  // Gyro Low-pass filter:  42Hz
  // DMP Update rate:       10Hz

  boolean mpu_initialized = false;
  while (!mpu_initialized) {
    if (initialize_mpu()) {
      mpu_initialized = true;
      Serial.println(F("Success"));
      enable_mpu();
    } else {
      Serial.println(F("Failed"));
      mpu_force_reset();
      delay(100);
      Serial.println(F("Re-initializing"));
    }
  }
  Serial.println(F("MPU Initialized"));
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

boolean initialize_mpu() {
  int result;
  struct int_param_s int_param;

  /* Set up gyro.
   * Every function preceded by mpu_ is a driver function and can be found
   * in inv_mpu.h.
   */
  pinMode(2, INPUT);
  int_param.cb = gyro_data_ready_cb;
  int_param.pin = digitalPinToInterrupt(2);
  result = mpu_init(&int_param);
  if (result != 0) {
    Serial.println(F("mpu_init failed!"));
    return false;
  }

  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  /* Read back configuration in case it was set improperly. */
  mpu_get_sample_rate(&dmp_update_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);

  /* To initialize the DMP:
   * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
   *    inv_mpu_dmp_motion_driver.h into the MPU memory.
   * 2. Push the gyro and accel orientation matrix to the DMP.
   * 3. Register gesture callbacks. Don't worry, these callbacks won't be
   *    executed unless the corresponding feature is enabled.
   * 4. Call dmp_enable_feature(mask) to enable different features.
   * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
   * 6. Call any feature-specific control functions.
   *
   * To enable the DMP, just call mpu_set_dmp_state(1). This function can
   * be called repeatedly to enable and disable the DMP at runtime.
   *
   * The following is a short summary of the features supported in the DMP
   * image provided in inv_mpu_dmp_motion_driver.c:
   * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
   * 200Hz. Integrating the gyro data at higher rates reduces numerical
   * errors (compared to integration on the MCU at a lower sampling rate).
   * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
   * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
   * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
   * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
   * an event at the four orientations where the screen should rotate.
   * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
   * no motion.
   * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
   * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
   * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
   * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
   */
  result = dmp_load_motion_driver_firmware();
  if ( result != 0 ) {
    Serial.println("firmware load failure");
    return false;
  }
  dmp_set_orientation(0x88);

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  return true;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void) {
  new_gyro = true;
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
  temp_p /= 50.0;
  if (temp_p >= 0 && temp_p < 1000 / 50 && temp_p != pid_gyro_p) {
    pid_gyro_p = temp_p;
    Serial.print(F("P:"));Serial.println(pid_gyro_p);
    xPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
    yPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
  }

  temp_i = (ch7 - 1000 + 25) / 50 * 50;
  temp_i /= 1000.0;
  temp_i -= 0.05;
  if (temp_i >= 0 && temp_i < 1000.0 / 1000.0 && temp_i != pid_gyro_i) {
    pid_gyro_i = temp_i;
    Serial.print(F("I:"));Serial.println(pid_gyro_i);
    xPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
    yPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
  }

  temp_d = (ch5 - 1000 + 25) / 50 * 50;
  temp_d /= 100.0;
  temp_d -= 0.5;
  if (temp_d >= 0 && temp_d < 1000.0 / 100.0 && temp_d != pid_gyro_d) {
    pid_gyro_d = temp_d;
    Serial.print(F("D:"));Serial.println(pid_gyro_d);
    xPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
    yPID.SetTunings(pid_gyro_p, pid_gyro_i, pid_gyro_d);
  }
  releaseLock();

#if 0
  Serial.print("CH5:"); Serial.println(ch5);
  Serial.print(F("ch1:"));
  Serial.println(ch1);
  Serial.print(F("ch2:"));
  Serial.println(ch2);
#endif

  xPID.Compute();
  yPID.Compute();
  
  set_servos();
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

