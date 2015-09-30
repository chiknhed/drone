#include <Wire.h>
#include <MPU6050.h>
#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>
#include <Servo.h>
#include <PID_v1.h>

#define NO_MOTOR_DEBUG
#ifdef NO_MOTOR_DEBUG
#define REPOSITION_PERIOD_MS  100ul
#else
#define REPOSITION_PERIOD_MS  30ul
#endif

#define MOVE_DURATION_MS      1000

#define ESC_MIN               22
#define ESC_MAX               115
#define ESC_ARM_DELAY         5000

#define ESC_A 5
#define ESC_B 4
#define ESC_C 3
#define ESC_D 2

#define TAKEOFF_Z_ACCEL      100
#define TAKEOFF_STEP_DELAY     500
#define TAKEOFF_GOUP_DELAY    5000
#define TAKEOFF_GOUP_ADJUST    400
#define TAKEOFF_HOVER_DELAY   1000

#define PID_AGG_P           1.0
#define PID_AGG_I           1.0
#define PID_AGG_D           1.0
#define PID_CONS_P          0.1
#define PID_CONS_I          0.1
#define PID_CONS_D          0.1
#define PID_CONS_THRESH_GYRO 200
#define PID_CONS_THRESH_ACCEL 200
#define PID_XY_INFLUENCE    30

double orig_accel_z;
int orig_gyro_x, orig_gyro_y;
double adj_accel_z;
double adj_gyro_x, adj_gyro_y;
double accel_z;
double gyro_x, gyro_y;

double v_ac, v_bd, velocity;

YunServer server;
Servo a, b, c, d;
PID xPID(&gyro_y, &v_ac, &adj_gyro_x, PID_AGG_P, PID_AGG_I, PID_AGG_D, DIRECT);
PID yPID(&gyro_x, &v_bd, &adj_gyro_y, PID_AGG_P, PID_AGG_I, PID_AGG_D, DIRECT);
PID vPID(&accel_z, &velocity, &adj_accel_z, PID_AGG_P, PID_AGG_I, PID_AGG_D, DIRECT);

int did_take_off = 0;

void reset_sensor(void)
{
  mpu6050.ReadAvrRegisters(1000);

  orig_accel_z = mpu6050.GetAccelZ();
  orig_gyro_x = mpu6050.GetGyroX();
  orig_gyro_y = mpu6050.GetGyroY();

  Serial.println(F("Starting Values :"));
  Serial.print(F("Accel (x y z)\n"));
  Serial.print(orig_accel_z, DEC); Serial.println();
  Serial.print(F("Gyro (x y z)\n"));
  Serial.print(orig_gyro_x, DEC); Serial.println();
  Serial.print(orig_gyro_y, DEC); Serial.println();
}

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Bridge.begin();
  digitalWrite(13, LOW);

  server.listenOnLocalhost();
  server.begin();

  Serial.println(F("initializing MPU6050"));
  mpu6050.begin();

  Serial.println(F("initializing Motors")); 
  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  delay(100);

  Serial.println(F("initializing PID"));
  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  yPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  vPID.SetMode(AUTOMATIC);
  vPID.SetOutputLimits(ESC_MIN, ESC_MAX);
}

void set_servos(void)
{
  double va, vb, vc, vd;
  
  va = velocity - v_ac - v_bd;
  vb = velocity - v_ac + v_bd;
  vc = velocity + v_ac - v_bd;
  vd = velocity + v_ac + v_bd;

  if (va > ESC_MAX) va = ESC_MAX;
  if (vb > ESC_MAX) vb = ESC_MAX;
  if (vc > ESC_MAX) vc = ESC_MAX;
  if (vd > ESC_MAX) vd = ESC_MAX;

  if (va < ESC_MIN) va = ESC_MIN;
  if (vb < ESC_MIN) vb = ESC_MIN;
  if (vc < ESC_MIN) vc = ESC_MIN;
  if (vd < ESC_MIN) vd = ESC_MIN;
  
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
}

void position_adjust(void)
{
  double gap;

  mpu6050.ReadAvrRegisters(100);

  accel_z = mpu6050.GetAccelZ() - orig_accel_z;
  gyro_x = mpu6050.GetGyroX() - orig_gyro_x;
  gyro_y = mpu6050.GetGyroY() - orig_gyro_y;

  Serial.println(accel_z);
  Serial.println(gyro_x);
  Serial.println(gyro_y);

  gap = abs(gyro_x - adj_gyro_x);
  if (gap < PID_CONS_THRESH_GYRO) {
    xPID.SetTunings(PID_CONS_P, PID_CONS_I, PID_CONS_D);
  } else {
    xPID.SetTunings(PID_AGG_P, PID_AGG_I, PID_AGG_D);
  }

  gap = abs(gyro_y - adj_gyro_y);
  if (gap < PID_CONS_THRESH_GYRO) {
    yPID.SetTunings(PID_CONS_P, PID_CONS_I, PID_CONS_D);
  } else {
    yPID.SetTunings(PID_AGG_P, PID_AGG_I, PID_AGG_D);
  }

  gap = abs(accel_z - adj_accel_z);
  if (gap < PID_CONS_THRESH_ACCEL) {
    vPID.SetTunings(PID_CONS_P, PID_CONS_I, PID_CONS_D);
  } else {
    vPID.SetTunings(PID_AGG_P, PID_AGG_I, PID_AGG_D);
  }

  xPID.Compute();
  yPID.Compute();
  vPID.Compute();

  Serial.println(velocity);
  
  set_servos();
}

void printStatus(YunClient client)
{
  client.println(accel_z);
  client.println(gyro_x);
  client.println(gyro_y);
  client.println();
}

void take_off()
{
  unsigned long take_off_time;
  reset_sensor();
  reset_adjust_variables();

  Serial.println(F("Taking off"));
  velocity = ESC_MIN;
  v_ac = 0;
  v_bd = 0;
  set_servos();
  Serial.println(ESC_MIN);
  Serial.println();
  delay(ESC_ARM_DELAY);

  Serial.println(F("Go up"));
  take_off_time = millis();    /* tricky -_-;;*/
  while (millis() - take_off_time < TAKEOFF_GOUP_DELAY) {
    adj_accel_z = 100;
    position_adjust();
    delay(REPOSITION_PERIOD_MS);
  }
}

void process(YunClient client)
{
  String command = client.readStringUntil('/');
  Serial.println(command);
  
  int param = client.parseInt();
  Serial.println(param);
  if(command == "up") {
    if (!did_take_off) {
      take_off();
      did_take_off = 1;
    } else {
      adj_accel_z = param;
    }
  } if (command == "down") {
    adj_accel_z = -param;
  }else if (command == "forward") {
    adj_gyro_x = -param;
    adj_gyro_y = -param;
  } else if (command == "backward") {
    adj_gyro_x = param;
    adj_gyro_y = param;
  } else if (command == "left") {
    adj_gyro_x = -param;
    adj_gyro_y = param;
  } else if (command == "right") {
    adj_gyro_x = param;
    adj_gyro_y = -param;
  }

  printStatus(client);
}

void loop() {
  digitalWrite(13, LOW);
  YunClient client = server.accept();

  if (client) {
    digitalWrite(13, HIGH);
    process(client);
    client.stop();
    digitalWrite(13, LOW);
  }

  if(did_take_off)
    position_adjust();
  
}
