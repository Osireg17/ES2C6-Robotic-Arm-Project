#include <Arduino.h>

const int TRIG_PIN = 7; const int ECHO_PIN = 12;
const int DISTANCE_THRESHOLD = 15;
float duration_us, distance_cm; // duration in microseconds, distance in centimeters

const int pwm = 3; //pulse width modulation
const int dir = 8; // direction
const int forcePin = A4; const int ledPin = 13;
int forceValue = 0; uint8_t motorSpeed = 60;
bool bHigh = false;  // boolean high
bool bChangeDir = false; // boolean change direction
const int buttonPin = 2; int buttonState = 0;

#include <Servo.h>
Servo servo_x_axis; Servo servo_y_axis; 
Servo servo_z_axis; Servo servo_clamp;
int x_axis_degree = servo_x_axis.read();
int y_axis_degree = servo_y_axis.read();
int z_axis_degree = servo_z_axis.read();
int clamp_degree = servo_clamp.read();

#define left_joystick_x A0
#define left_joystick_y A1
#define right_joystick_x A2
#define right_joystick_y A3



void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  pinMode(ledPin, OUTPUT); // Sets the ledPin as an Output
  pinMode(pwm, OUTPUT); // Sets the pwm as an Output
  pinMode(dir, OUTPUT); // Sets the dir as an Output
  pinMode(buttonPin, INPUT_PULLUP);
  servo_z_axis.attach(11); // servo 1
  servo_y_axis.attach(5); // servo 2
  servo_x_axis.attach(6); // servo 3
  servo_clamp.attach(9); // servo 4
}

void loop() {
  
  // These two Q  1`will be used to control the gripper and run at the same time
  joyStick(); // This is the code for the joystick
  gripper(); // This is the code for the gripper


}

void joyStick() {

  int left_joystick_x_value = analogRead(left_joystick_x);
  int left_joystick_y_value = analogRead(left_joystick_y);
  int right_joystick_x_value = analogRead(right_joystick_x);
  int right_joystick_y_value = analogRead(right_joystick_y);

  if(left_joystick_x_value < 340){
    y_axis_degree -= 2;
  }else if (left_joystick_x_value > 660){
    y_axis_degree += 2;
  }

  if(left_joystick_y_value < 340){
    x_axis_degree -= 2;
  }else if(left_joystick_y_value > 680){
    x_axis_degree += 2;
  }

  if(right_joystick_x_value < 340){
    z_axis_degree -= 2;
  }else if(right_joystick_x_value > 680){
    z_axis_degree += 2;
  }

  if(right_joystick_y_value < 340){
    clamp_degree -= 2;
  }else if(right_joystick_y_value > 680){
    clamp_degree += 2;
  }

  z_axis_degree = min(145, max(15, z_axis_degree));
  y_axis_degree = min(175, max(40, y_axis_degree));
  x_axis_degree = min(180, max(5, x_axis_degree));
  clamp_degree = min(90, max(40, clamp_degree));

  Serial.print("x_axis_degree: "); 
  Serial.println(x_axis_degree);
  Serial.print("y_axis_degree: ");
  Serial.println(y_axis_degree);
  Serial.print("z_axis_degree: ");
  Serial.println(z_axis_degree);
  Serial.print("clamp_degree: ");
  Serial.println(clamp_degree);

  servo_clamp.write(clamp_degree);
  servo_x_axis.write(x_axis_degree);
  servo_y_axis.write(y_axis_degree);
  servo_z_axis.write(z_axis_degree);

}

void gripper() {

  buttonState = digitalRead(buttonPin);
  forceValue = analogRead(forcePin);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, LOW);
  duration_us = pulseIn(ECHO_PIN, HIGH);
  distance_cm = 0.017 * duration_us;

  if((forceValue < 100) && (buttonState == 1)){
    if((distance_cm < DISTANCE_THRESHOLD) && (buttonState == 1)){
      if(bHigh){
        bChangeDir = false;
      }
      else{
        bChangeDir = true;
      }
      bHigh = !bHigh;
  }else{
    if(bHigh){
      bChangeDir = true;
    }else{
      bChangeDir = false;
    }
    bHigh = !bHigh;
  }
}else{
  if(bHigh){
    bChangeDir = true;
  }else{
    bChangeDir = false;
  }
  bHigh = !bHigh;
}

if(bChangeDir){
  analogWrite(pwm, 0);
  delay(250);
}
if((bHigh)){
  digitalWrite(ledPin, HIGH);
  digitalWrite(dir, LOW);
}
else{
  digitalWrite(ledPin, LOW);
  digitalWrite(dir, HIGH);
}
analogWrite(pwm, motorSpeed);
}
