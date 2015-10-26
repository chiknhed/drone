/* much code from the following sites
 *
 *  https://github.com/strangedev/Arduino-Quadcopter
 *  400Hz PWM code from somewhere T_T (don't remember)
 *  Nav6 source code
 *
 */

#define EMPL_TARGET_ATMEGA328
#include <Wire.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}
#include <Console.h>

//#define VERBOSE_DEBUG

#define PID_GYRO_P            1.7
#define PID_GYRO_I            1.5
#define PID_GYRO_D            35

#define PID_ACCEL_P           1
#define PID_ACCEL_I           0.8
#define PID_ACCEL_D           3

#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1

#define SDA_PIN               2
#define SCL_PIN               2
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ    (20)
#define MAX_NAV6_MPU_RATE (100)
#define MIN_NAV6_MPU_RATE (4)
/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define PID_SAMPLE_PERIOD     10

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

#define PID_XY_INFLUENCE    50.0
#define PID_THROTTLE_INFLUENCE  50.0

#define UPDOWN_MULT_FACTOR  (0.1)
#define MOVE_MULT_FACTOR    (10)


double orig_accel_z = 0;
double orig_yaw = 0;
double adj_accel_z = 0, adj_gyro_x = 0, adj_gyro_y = 0;
double accel_z, gyro_x, gyro_y, yaw;

double hover_throttle = ESC_WORKING_MIN;
boolean hover_found = false;

unsigned long repos_last_time;
unsigned long repos_remaining_time;

double v_ac, v_bd, velocity, bal_axes;
double zero_value = 0.0;

int did_takeoff = 0;
int doing_takeoff = 0;

/* NAV6 code */
unsigned char new_gyro, dmp_on;
volatile boolean compass_data_ready = false;
void compassDataAvailable() {
  compass_data_ready = true;
}

/*****************************************
* Conversion Factors
*****************************************/

// angle in radians = angle in degrees * Pi / 180
const float degrees_to_radians = M_PI / 180.0;
// angle in degrees = angle in radians * 180 / Pi
const float radians_to_degrees = 180.0 / M_PI;

/*****************************************
* MPU Calibration
*****************************************/

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

/******************************************
* Magnetometer State
******************************************/

int16_t mag_x = 0;
int16_t mag_y = 0;
int16_t mag_z = 0;

float compass_heading_radians = 0.0;
float compass_heading_degrees = 0.0;

/****************************************
* Gyro/Accel/DMP State
****************************************/

float temp_centigrade = 0.0;  // Gyro/Accel die temperature
float ypr[3] = { 0, 0, 0 };
long curr_mpu_temp;
unsigned long sensor_timestamp;

struct FloatVectorStruct {
  float x;
  float y;
  float z;
};

struct FloatVectorStruct gravity;

unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000

PID xPID(&gyro_x, &v_ac, &adj_gyro_x, PID_GYRO_P, PID_GYRO_I, PID_GYRO_D, DIRECT);
PID yPID(&gyro_y, &v_bd,  &adj_gyro_y, PID_GYRO_P, PID_GYRO_I, PID_GYRO_D, DIRECT);
PID vPID(&accel_z, &velocity, &adj_accel_z, PID_ACCEL_P, PID_ACCEL_I, PID_ACCEL_D, REVERSE);
PID yawPID(&yaw, &bal_axes, &zero_value, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);

void setup() {
  Wire.begin();

  Serial.begin(115200);

  initServo();

  arm(0);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Bridge.begin();
  Console.begin();
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
      // Send a Yaw/Pitch/Roll/Heading update

      x -= calibrated_yaw_offset;

      if ( x < -180 ) x += 360;
      if ( x > 180 ) x -= 360;
      if ( y < -180 ) y += 360;
      if ( y > 180 ) y -= 360;
      if ( z < -180 ) z += 360;
      if ( z > 180 ) z -= 360;

      float linear_acceleration_x;
      float linear_acceleration_y;
      float linear_acceleration_z;
      float q1[4];
      float q2[4];
      float q_product[4];
      float q_conjugate[4];
      float q_final[4];
      float world_linear_acceleration_x;
      float world_linear_acceleration_y;
      float world_linear_acceleration_z;

      // calculate linear acceleration by
      // removing the gravity component from raw acceleration values

      linear_acceleration_x = (((float)accel[0]) / (32768.0 / accel_fsr)) - gravity.x;
      linear_acceleration_y = (((float)accel[1]) / (32768.0 / accel_fsr)) - gravity.y;
      linear_acceleration_z = (((float)accel[2]) / (32768.0 / accel_fsr)) - gravity.z;

      // Calculate world-frame acceleration

      q1[0] = quat[0] >> 16;
      q1[1] = quat[1] >> 16;
      q1[2] = quat[2] >> 16;
      q1[3] = quat[3] >> 16;

      q2[0] = 0;
      q2[1] = linear_acceleration_x;
      q2[2] = linear_acceleration_y;
      q2[3] = linear_acceleration_z;

      // Rotate linear acceleration so that it's relative to the world reference frame

      // http://www.cprogramming.com/tutorial/3d/quaternions.html
      // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
      // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
      // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

      // P_out = q * P_in * conj(q)
      // - P_out is the output vector
      // - q is the orientation quaternion
      // - P_in is the input vector (a*aReal)
      // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])

      // calculate quaternion product
      // Quaternion multiplication is defined by:
      //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
      //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
      //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
      //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2

      q_product[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]; // new w
      q_product[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]; // new x
      q_product[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]; // new y
      q_product[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]; // new z

      q_conjugate[0] = q1[0];
      q_conjugate[1] = -q1[1];
      q_conjugate[2] = -q1[2];
      q_conjugate[3] = -q1[3];

      q_final[0] = q_product[0] * q_conjugate[0] - q_product[1] * q_conjugate[1] - q_product[2] * q_conjugate[2] - q_product[3] * q_conjugate[3]; // new w
      q_final[1] = q_product[0] * q_conjugate[1] + q_product[1] * q_conjugate[0] + q_product[2] * q_conjugate[3] - q_product[3] * q_conjugate[2]; // new x
      q_final[2] = q_product[0] * q_conjugate[2] - q_product[1] * q_conjugate[3] + q_product[2] * q_conjugate[0] + q_product[3] * q_conjugate[1]; // new y
      q_final[3] = q_product[0] * q_conjugate[3] + q_product[1] * q_conjugate[2] - q_product[2] * q_conjugate[1] + q_product[3] * q_conjugate[0]; // new z

      world_linear_acceleration_x = q_final[1];
      world_linear_acceleration_y = q_final[2];
      world_linear_acceleration_z = q_final[3];

      gyro_x = - z;
      gyro_y = y;
      accel_z = world_linear_acceleration_z;
#if 0
      int num_bytes = IMUProtocol::encodeYPRUpdate(protocol_buffer, x, y, z, compass_heading_degrees);
      Serial.write((unsigned char *)protocol_buffer, num_bytes);
#endif
    }
  }
  else {

    /* The following debug print outs are useful
       if DMP fifo streaming is not working or
       data is being lost.

       If modifying code, it's a good idea to check that
       the "FIFO OVERFLOW" error does not occur, since
       this case can occur if too many cycles are used
       to keep up with the sensor data stream.

    if ( success == -1 )
    {
      Serial.println("DMP DISABLED!!!");
    }
    else if ( success == -2 )
    {
      Serial.println("I2C READ ERROR");
    }
    else if ( success == -3 )
    {
      Serial.println("FIFO OVERFLOW ERROR!!!");
    }
    else if ( success == -4 )
    {
      Serial.println("NO_SENSORS");
    }
    else if ( success == -6 )
    {
      Serial.println("CORRUPTED_QUATERNION");
    }
    */
  }

  if (did_takeoff || doing_takeoff)
    position_adjust();

  if (Console.available()) {
    process();
  }
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
  pinMode(13, OUTPUT);

  digitalWrite(13, LOW);

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
      digitalWrite(SDA_PIN, HIGH);
      digitalWrite(SDA_PIN, LOW);
    }
  }

  // send a I2C stop signal
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SDA_PIN, LOW);
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
}void disable_mpu() {
    mpu_set_dmp_state(0);
    dmp_on = 0;
}

void enable_mpu() {

    mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
    dmp_on = 1;
}  

boolean run_mpu_self_test(boolean& gyro_ok, boolean& accel_ok) {
  
    int result;
    long gyro[3], accel[3];
    boolean success = false;

    gyro_ok = false;
    accel_ok = false;
    result = mpu_run_self_test(gyro, accel);
    if ( ( result & 0x1 ) != 0 ) {
      // Gyro passed self test
      gyro_ok = true;
      float sens;
      mpu_get_gyro_sens(&sens);
      gyro[0] = (long)(gyro[0] * sens);
      gyro[1] = (long)(gyro[1] * sens);
      gyro[2] = (long)(gyro[2] * sens);
      dmp_set_gyro_bias(gyro);
    }
    if ( ( result & 0x2 ) != 0 ) {
      // Accelerometer passed self test
      accel_ok = true;
      unsigned short accel_sens;
      mpu_get_accel_sens(&accel_sens);
      accel[0] *= accel_sens;
      accel[1] *= accel_sens;
      accel[2] *= accel_sens;
      dmp_set_accel_bias(accel);
    }

    success = gyro_ok && accel_ok;
  
    return success;
}

void getEuler(float *data, Quaternion *q) {

    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
}

void getGravity(struct FloatVectorStruct *v, Quaternion *q) {

    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
}

void dmpGetYawPitchRoll(float *data, Quaternion *q, struct FloatVectorStruct *gravity) {
  
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
}

void print_sensors(void) {
  Serial.print(F("t : "));
  Serial.println(millis());
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
    digitalWrite(13, HIGH);
    if (initialize_mpu()) {
      mpu_initialized = true;
      enable_mpu();
    } else {
      digitalWrite(13, LOW);
      mpu_force_reset();
      delay(100);
    }
  }
  digitalWrite(13, LOW);
}

boolean initialize_mpu() {
  int result;
  struct int_param_s int_param;

  /* Set up gyro.
   * Every function preceded by mpu_ is a driver function and can be found
   * in inv_mpu.h.
   */
  int_param.cb = gyro_data_ready_cb;
  int_param.pin = 0;
  result = mpu_init(&int_param);

  if (result != 0) {
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
    Serial.print("E.");
    return false;
  }
  dmp_set_orientation(0x88);

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  return true;
}

void set_servos(void)
{
  int va, vb, vc, vd;
  int regVal;

  va = ESC_MIN + (hover_throttle + velocity - ESC_MIN) * ((100.0 + v_ac) / 100.0) * ((100.0 + bal_axes) / 100.0) + 0.5;
  vb = ESC_MIN + (hover_throttle + velocity - ESC_MIN) * ((100.0 + v_bd) / 100.0) * (abs(-100.0 + bal_axes) / 100.0) + 0.5;
  vc = ESC_MIN + (hover_throttle + velocity - ESC_MIN) * (abs(-100.0 + v_ac) / 100.0) * ((100.0 + bal_axes) / 100.0) + 0.5;
  vd = ESC_MIN + (hover_throttle + velocity - ESC_MIN) * (abs(-100.0 + v_bd) / 100.0) * (abs(-100.0 + bal_axes) / 100.0) + 0.5;

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
  if (command == 'p') {
    if (!did_takeoff) {
      hover_throttle = ESC_WORKING_MIN;
      hover_found = false;
      doing_takeoff = 1;
      did_takeoff = 0;
      reset_pid_output();
      reset_adjust_variables();
      repos_remaining_time = TAKEOFF_GOUP_DELAY;
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
  } else if (command == 'w') {
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

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void) {
  new_gyro = 1;
}

