#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Servo esc;
Servo esc1;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

double Kp = 0, Ki = 0, Kd = 500;

double input, output, setpoint, degreeValue;

unsigned long  currentTime, previousTime;
double elapsedTime;

double error, lastError, pid_p, pid_i = 0, pid_d;


void setup(void)
{
  esc.writeMicroseconds(2000);
  esc1.writeMicroseconds(2000);
  delay(2000);
  esc.writeMicroseconds(1000);
  esc1.writeMicroseconds(1000);
  delay(2000);

  Serial.begin(115200);

  while (!Serial) delay(10);

  Serial.println("Orientation Sensor Test \t");

  if (!bno.begin()) while (1);

  pinMode(2, OUTPUT);
  digitalWrite(2, 1);
  esc.attach(9);
  esc1.attach(10);

  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

void loop(void)
{
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print(event.orientation.y, 4);Serial.print("  ");

  displayCalStatus();

  degreeValue = event.orientation.y;

  pidController();

  Serial.print(pid_p);Serial.print("  ");
  Serial.print(pid_i);Serial.print("  ");
  Serial.print(pid_d);Serial.print("  ");
  Serial.println(output);

  esc.writeMicroseconds(1500 + output);
  esc1.writeMicroseconds(1500 - output);
}

void pidController() {
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);

  input = degreeValue;
  setpoint = 0;

  error = setpoint - input;

  pid_p = Kp * error;
  pid_i = pid_i + (error * Ki);
  pid_d = Kd * ((error - lastError) / elapsedTime);

  output = pid_p + pid_i + pid_d;
  //  output = constrain(output, -400, 400);

  if (output > 500)
    output = 500;
  else if (output < -500)
    output = -500;

  lastError = error;
  previousTime = currentTime;
}

void displaySensorStatus(void)
{
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
}

void displayCalStatus(void)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}
