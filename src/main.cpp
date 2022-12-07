#include <Arduino.h>

//The code used to set up the ultrasonic sensor at the top of the robot arm
const int TRIG_PIN = 7; const int ECHO_PIN = 8; const int ledPin  = 13;
const int DISTANCE_THRESHOLD = 15; // this is the maximum distance the sensor can detect
float duration_us, distance_cm;

int forceValue = 0;
const int FORCE_THRESHOLD = 1000; // this is the maximum force the sensor can detect
const int FORCE_PIN = A4;

#include <Servo.h>
Servo servo_x_axis; Servo servo_y_axis; Servo servo_z_axis; Servo servo_clamp;
int x_axis_degree = servo_x_axis.read(); int y_axis_degree = servo_y_axis.read(); // This reads the current position of the servo
int z_axis_degree = servo_z_axis.read(); int clamp_degree = servo_clamp.read();

#define left_joystick_x A0
#define left_joystick_y A1
#define right_joystick_x A2
#define right_joystick_y A3

// The code for the gripper now
const int pwm = 3; const int dir = 8;
uint8_t motorSpeed = 50; // Defining the speed of the motor using 8 bit unsigned integer
bool bHigh = false; // Defining the direction of the motor
bool bChangeDir = false; // Defining the direction of the motor

const int buttonPin = 2; // the number of the pushbutton pin
int buttonState = 0; // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT); pinMode(ledPin, OUTPUT);
  pinMode(pwm, OUTPUT); pinMode(dir, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  servo_z_axis.attach(11); // servo 1
  servo_y_axis.attach(5); // servo 2
  servo_x_axis.attach(6); // servo 3
  servo_clamp.attach(9); // servo 4
}

void loop() {
  
  // These two will be used to control the gripper and run at the same time
  joyStick(); // This is the code for the joystick
  gripper(); // This is the code for the gripper


}

void joyStick() {

  int left_joystick_x_value = analogRead(left_joystick_x);
  int left_joystick_y_value = analogRead(left_joystick_y);
  int right_joystick_x_value = analogRead(right_joystick_x);
  int right_joystick_y_value = analogRead(right_joystick_y);

  if(left_joystick_x_value < 340){
    y_axis_degree -= 4;
  }else if (left_joystick_x_value > 660){
    y_axis_degree += 4;
  }

  if(left_joystick_y_value < 340){
    x_axis_degree -= 4;
  }else if(left_joystick_y_value > 680){
    x_axis_degree += 4;
  }

  if(right_joystick_x_value < 340){
    z_axis_degree -= 4;
  }else if(right_joystick_x_value > 680){
    z_axis_degree += 4;
  }

  if(right_joystick_y_value < 340){
    clamp_degree -= 4;
  }else if(right_joystick_y_value > 680){
    clamp_degree += 4;
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
  forceValue = analogRead(FORCE_PIN);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, LOW);
  duration_us = pulseIn(ECHO_PIN, HIGH);
  distance_cm = 0.017 * duration_us;

  if (buttonState == LOW) {
    if(bHigh){ // if false then change
      bChangeDir = false;
    }
    else{
      bChangeDir = true;
    }
    bHigh = !bHigh;
  }else if(distance_cm > DISTANCE_THRESHOLD){
    if(bHigh){
      bChangeDir = false;
    }
    else{
      bChangeDir = true;
    }
    bHigh = !bHigh;
}else{
  if(bHigh){
    bChangeDir = false;
  }
  else{
    bChangeDir = true;
  }
  bHigh = !bHigh;
}

if(bChangeDir){
  analogWrite(pwm, 0);
}
if((bHigh == true)){
  digitalWrite(ledPin, LOW);
  digitalWrite(dir, LOW);
}
else{
  digitalWrite(ledPin, HIGH);
  digitalWrite(dir, HIGH);
}
analogWrite(pwm, motorSpeed);
}
