#include <Wire.h>
#include <MPU6050.h>
#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>
#include <TimerOne.h>

#define REPOSITION_PERIOD_MS  30
#define MOVE_DURATION_MS      1000

YunServer server;

int orig_accel_x, orig_accel_y, orig_accel_z;
int orig_gyro_x, orig_gyro_y, orig_gyro_z;
int adj_accel_x, adj_accel_y, adj_accel_z;
int adj_gyro_x, adj_gyro_y, adj_gyro_z;
int accel_x, accel_y, accel_z;
int gyro_x, gyro_y, gyro_z;

unsigned int last_position_adjust;
int adjust_remain_ms;

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Bridge.begin();
  digitalWrite(13, LOW);

  server.listenOnLocalhost();
  server.begin();

  mpu6050.begin();

  mpu6050.ReadAvrRegisters(1000);

  orig_accel_x = mpu6050.GetAccelX();
  orig_accel_y = mpu6050.GetAccelY();
  orig_accel_z = mpu6050.GetAccelZ();
  orig_gyro_x = mpu6050.GetGyroX();
  orig_gyro_y = mpu6050.GetGyroY();
  orig_gyro_z = mpu6050.GetGyroZ();

  Serial.println(F("Starting Values :"));
  Serial.print(F("Accel (x y z)\n"));
  Serial.print(orig_accel_x, DEC); Serial.println();
  Serial.print(orig_accel_y, DEC); Serial.println();
  Serial.print(orig_accel_z, DEC); Serial.println();
  Serial.print(F("Gyro (x y z)\n"));
  Serial.print(orig_gyro_x, DEC); Serial.println();
  Serial.print(orig_gyro_y, DEC); Serial.println();
  Serial.print(orig_gyro_z, DEC); Serial.println();

  Timer1.initialize(REPOSITION_PERIOD_MS * 1000);
  Timer1.attachInterrupt(position_adjust);
}

void position_adjust(void)
{
  adjust_remain_ms -= (millis() - last_position_adjust);
  if (adjust_remain_ms < 0) {
      adjust_remain_ms = 0;
      adj_accel_x = adj_accel_y = adj_accel_z = 0;
      adj_gyro_x = adj_gyro_y = adj_gyro_z = 0;
  }
  last_position_adjust = millis();

  mpu6050.ReadAvrRegisters(20);

  accel_x = mpu6050.GetAccelX() - orig_accel_x + adj_accel_x;
  accel_y = mpu6050.GetAccelY() - orig_accel_y + adj_accel_y;
  accel_y = mpu6050.GetAccelZ() - orig_accel_z + adj_accel_z;
  gyro_x = mpu6050.GetGyroX() - orig_gyro_x + adj_gyro_x;
  gyro_y = mpu6050.GetGyroY() - orig_gyro_y + adj_gyro_y;
  gyro_z = mpu6050.GetGyroZ() - orig_gyro_z + adj_gyro_z;
}

void printStatus(YunClient client)
{
  client.println(accel_x);
  client.println(accel_y);
  client.println(accel_z);
  client.println(gyro_x);
  client.println(gyro_y);
  client.println(gyro_z);
  client.println();
}

void process(YunClient client)
{
  String command = client.readStringUntil('/');
  if (client.read() == '/') {
    int param = client.parseInt();
    if(command == "up") {
      adjust_remain_ms = MOVE_DURATION_MS;
      adj_accel_z = param;
    } else if (command == "forward") {
      adjust_remain_ms = MOVE_DURATION_MS;
      adj_gyro_x = -param;
      adj_gyro_y = -param;
    } else if (command == "backward") {
      adjust_remain_ms = MOVE_DURATION_MS;
      adj_gyro_x = param;
      adj_gyro_y = param;
    } else if (command == "left") {
      adjust_remain_ms = MOVE_DURATION_MS;
      adj_gyro_x = -param;
      adj_gyro_y = param;
    } else if (command == "right") {
      adjust_remain_ms = MOVE_DURATION_MS;
      adj_gyro_x = param;
      adj_gyro_y = -param;
    }
  }

  printStatus(client);
}

void loop() {
  digitalWrite(13, LOW);
  YunClient client = server.accept();

  digitalWrite(13, HIGH);

  if (client) {
    process(client);
    client.stop();
  }
 
  digitalWrite(13, LOW);
}
