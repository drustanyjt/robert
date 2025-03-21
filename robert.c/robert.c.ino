#include <Servo.h>

#define MOT_H 4
#define MOT_L 3
#define TRIG_PIN 12
#define ECHO_PIN 13
#define ONLY_ONCE 0
#define SERVO_PIN 11
#define SERVO_1_START 500
#define SERVO_2_END 2400


const float SPEED_OF_SOUND = 0.0345;
unsigned long START_TIME = 0;
bool IS_LAUNCHED = false;

Servo servo;

enum MotorDirection {
  FORWARD=0,
  STOP=1,
  BACKWARD=2
};

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOT_H, OUTPUT);
  pinMode(MOT_L, OUTPUT);
  START_TIME = millis();

  servo.attach(SERVO_PIN, 500, 2400); // values calibrated on 28 feb
  launcher_reset();
  Serial.begin(9600);
}

void spin_motors(int md) {
  switch (md) {
  	case 0:
      digitalWrite(MOT_H, HIGH);
      digitalWrite(MOT_L, LOW);
      Serial.println("Motors forward");
  	  break;
    case 1:
      digitalWrite(MOT_H, LOW);
      digitalWrite(MOT_L, LOW);
      Serial.println("Motors stop");
  	  break;
    case 2:
      digitalWrite(MOT_H, LOW);
      digitalWrite(MOT_L, HIGH);
      Serial.println("Motors back");
  }
}

void test_motor_loop() {
  spin_motors(FORWARD);
  delay(1000); // Wait for 1000 millisecond(s)
  spin_motors(STOP);
  delay(1000); // Wait for 1000 millisecond(s)
  spin_motors(BACKWARD);
  delay(1000); // Wait for 1000 millisecond(s)
}

void stop_bf_obstacle_loop() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  int microsecs = pulseIn(ECHO_PIN, HIGH);
  float cms = microsecs*SPEED_OF_SOUND/2;
  Serial.println(cms);
  if (cms < 30) {
    Serial.println("Obstacle detected");
    spin_motors(STOP);
  } else {
    spin_motors(FORWARD);
  }
  delay(10);
}

void timed_forward_then_reverse_loop() {
  int forward_time = 5;
  int stop_time = 1;
  int reverse_time = 5;
  spin_motors(FORWARD);
  delay(forward_time * 1000);
  spin_motors(STOP);
  delay(stop_time * 1000);
  spin_motors(BACKWARD);
  delay(reverse_time * 1000);
  spin_motors(STOP);
  delay(stop_time * 1000);
}

bool is_elapsed_time_gt(unsigned long ms) {
  unsigned long elapsed_time = millis() - START_TIME;
  return elapsed_time > ms;
}

void launcher_reset()
{
  servo.write(180);
}

void launcher_shoot()
{
  servo.write(0);
}

void timed_fw_launch_bw_loop()
{
  int forward_time = 2;
  int launch_time = 10;
  int stop_time = 10;
  int reverse_time = forward_time;
  spin_motors(FORWARD);
  delay(forward_time * 1000);
  spin_motors(STOP);
  delay(forward_time * 1000);
  launcher_shoot();
  delay(launch_time * 1000);
  launcher_reset();
  spin_motors(BACKWARD);
  delay(reverse_time * 1000);
  spin_motors(STOP);
  delay(stop_time * 1000);
}

void launch_loop()
{
  launcher_reset();
  delay(5000);
  launcher_shoot();
  delay(10000);
}

void loop()
{
  // stop_bf_obstacle_loop();
  // test_motor_loop();
  // timed_forward_then_reverse_loop();
  // spin_motors(BACKWARD);
  timed_fw_launch_bw_loop();
  // launcher_reset();
  // launch_loop();
  // delay(10000);
  // timed_fw_launch_bw_loop();
}