#include <Wire.h>
#include <MPU6050.h>
#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>
#include <Servo.h>
#include <PID_v1.h>

#define REPOSITION_PERIOD_MS  30ul
#define MOVE_DURATION_MS      2000

#define GYRO_FACTOR           1.0
#define ACCEL_FACTOR          1.0
#define BALANCE_FACTOR           1.0

#define ESC_MIN               22
#define ESC_WORKING_MIN       70
#define ESC_MAX               150
#define ESC_ARM_DELAY         5000
  
#define ESC_A 9
#define ESC_B 8
#define ESC_C 7
#define ESC_D 6

#define TAKEOFF_Z_ACCEL      100
#define TAKEOFF_STEP_DELAY     500
#define TAKEOFF_GOUP_DELAY    5000

#define PID_XY_INFLUENCE    20.0

#define GYRO_READ_AVERAGE_COUNT  20

double orig_accel_z;
int orig_gyro_x, orig_gyro_y;
double adj_accel_z;
double adj_gyro_x, adj_gyro_y;
double accel_z;
double gyro_x, gyro_y;

unsigned long repos_last_time;
unsigned long repos_remaining_time;

double v_ac, v_bd, velocity;

YunServer server;
Servo a, b, c, d;
PID xPID(&gyro_x, &v_ac, &adj_gyro_x, 0.001, 0.0001, 0.005, DIRECT);
PID yPID(&gyro_y, &v_bd,  &adj_gyro_y, 0.001, 0.0001, 0.005, DIRECT);
PID vPID(&accel_z, &velocity, &adj_accel_z, 0.001, 0.001, 0.005, DIRECT);

int did_take_off = 0;

void reset_sensor(void)
{
  mpu6050.ReadAvrRegisters(1000);

  orig_accel_z = mpu6050.GetAccelZ();
  orig_gyro_x = mpu6050.GetGyroX();

  Serial.println(F("Starting Values :"));
  Serial.print(F("Accel z : "));
  Serial.print(orig_accel_z, DEC); Serial.println();
  Serial.print(F("Gyro x y "));
  Serial.print(orig_gyro_x, DEC); Serial.print(F(" "));
  Serial.print(orig_gyro_y, DEC); Serial.println(F("\n"));

  repos_last_time = 0;
}

void setup() {
  Serial.begin(115200);

  Serial.println(F("initializing MPU6050"));
  mpu6050.begin();

  Serial.println(F("initializing Motors")); 
  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  delay(100);
  
  Serial.println(F("Arming Motors"));
  arm(0);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Bridge.begin();
  digitalWrite(13, LOW);

  server.listenOnLocalhost();
  server.begin();
  
  Serial.println(F("initializing PID"));
  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  yPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(-PID_XY_INFLUENCE, PID_XY_INFLUENCE);
  vPID.SetMode(AUTOMATIC);
  vPID.SetOutputLimits(ESC_WORKING_MIN, ESC_MAX);
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

void position_adjust(void)
{
  unsigned long current_time;

  if (repos_last_time == 0) repos_last_time = millis();
  current_time = millis();
  Serial.print(F("Current Time : "));
  Serial.println(current_time);
  
  if (current_time - repos_last_time > repos_remaining_time) {
    repos_remaining_time = 0;
    reset_adjust_variables();
  } else {
    repos_remaining_time -= current_time - repos_last_time;
  }

  print_adjust_variables();

  mpu6050.ReadAvrRegisters(GYRO_READ_AVERAGE_COUNT);

  accel_z = (mpu6050.GetAccelZ() - orig_accel_z) * ACCEL_FACTOR;
  gyro_x = (mpu6050.GetGyroX() - orig_gyro_x) * GYRO_FACTOR;
  gyro_y = (mpu6050.GetGyroY() - orig_gyro_y) * GYRO_FACTOR;

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

void printStatus(YunClient client)
{
  client.println(accel_z);
  client.println(gyro_x);
  client.println(gyro_y);
  client.println();
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

void process(YunClient client)
{
  String command = client.readStringUntil('/');
  Serial.println(command);
  
  int param = client.parseInt();
  Serial.println(param);
  if(command == "up") {
    if (!did_take_off) {
      reset_sensor();
      reset_adjust_variables();
      repos_remaining_time = TAKEOFF_GOUP_DELAY;
      adj_accel_z = -param;
      did_take_off = 1;
    } else {
      repos_remaining_time = MOVE_DURATION_MS;
      adj_accel_z = -param;
    }
  } if (command == "down") {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_accel_z = -param;
  }else if (command == "forward") {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = -param;
    adj_gyro_y = -param;
  } else if (command == "backward") {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = param;
    adj_gyro_y = param;
  } else if (command == "left") {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = -param;
    adj_gyro_y = param;
  } else if (command == "right") {
    repos_remaining_time = MOVE_DURATION_MS;
    adj_gyro_x = param;
    adj_gyro_y = -param;
  } else if (command == "emg") {
    repos_remaining_time = 0;
    did_take_off = 0;
    reset_adjust_variables();
    arm(1);
  }
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
