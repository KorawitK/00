#include <ESP32Servo.h>
#include <PS4Controller.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;


int servo1;
int servo2;
int servo3;
int fo;
int back;
int left;
int right;
int turn_left;
int turn_right;
int y;
int x;
int z;
boolean buttonState;
boolean lastState;
boolean state = 0;
boolean buttonState1;
boolean lastState1;
boolean state1 = 0;
boolean buttonState2;
boolean lastState2;
boolean state2 = 0;

// DC Motor_Backward_Left
int Motor1_IN1 = 33;
int Motor1_IN2 = 32;
int Motor1_EN1 = 25;
// DC Motor_Forward_Left
int Motor2_IN3 = 18;
int Motor2_IN4 = 19;
int Motor2_EN2 = 5;
// DC Motor_Forward_Right
int Motor3_IN5 = 23;
int Motor3_IN6 = 22;
int Motor3_EN3 = 21;
// DC Motor_Backward_Right
int Motor4_IN7 = 2;
int Motor4_IN8 = 4;
int Motor4_EN4 = 15;

void setup() {
  Serial.begin(115200);
  PS4.begin();
  myservo1.attach(13);
  myservo2.attach(12);
  myservo3.attach(14);

  pinMode(Motor1_IN1, OUTPUT);
  pinMode(Motor1_IN2, OUTPUT);
  pinMode(Motor1_EN1, OUTPUT);
  pinMode(Motor2_IN3, OUTPUT);
  pinMode(Motor2_IN4, OUTPUT);
  pinMode(Motor2_EN2, OUTPUT);
  pinMode(Motor3_IN5, OUTPUT);
  pinMode(Motor3_IN6, OUTPUT);
  pinMode(Motor3_EN3, OUTPUT);
  pinMode(Motor4_IN7, OUTPUT);
  pinMode(Motor4_IN8, OUTPUT);
  pinMode(Motor4_EN4, OUTPUT);
  pinMode(39, INPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(36, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);

  digitalWrite(Motor1_IN1, LOW);
  digitalWrite(Motor1_IN2, LOW);
  digitalWrite(Motor2_IN3, LOW);
  digitalWrite(Motor2_IN4, LOW);
  digitalWrite(Motor3_IN5, LOW);
  digitalWrite(Motor3_IN6, LOW);
  digitalWrite(Motor4_IN7, LOW);
  digitalWrite(Motor4_IN8, LOW);

  myservo1.write(50);
  myservo2.write(0);
  myservo3.write(180);
}

void loop() {
  //    if (PS4.LStickX()>80) {
  //      Serial.printf("Right Stick x at %d\n", PS4.RStickX());
  //    }
  //    if (PS4.LStickY()>80) {
  //      Serial.printf("Right Stick y at %d\n", PS4.RStickY());
  //    }

  int button = PS4.Circle();
  int button1 = PS4.Triangle();
  int button2 = PS4.Square();

  buttonState = button;
  if ((buttonState == 0) && (lastState == 1))
    state = !state;
  lastState = buttonState;
  //  Serial.printf("Circle  %d\n", state);

  buttonState1 = button1;
  if ((buttonState1 == 0) && (lastState1 == 1))
    state1 = !state1;
  lastState1 = buttonState1;
  //  Serial.printf("Triangle %d\n", state1);

  buttonState2 = button2;
  if ((buttonState2 == 0) && (lastState2 == 1))
    state2 = !state2;
  lastState2 = buttonState2;
  //  Serial.printf("Square %d\n", state2);

  if (state == 1) {
    myservo3.write(140);
    //    Serial.println("myservo3");
  } else {
    myservo3.write(70);
    //    Serial.println("Un.myservo3");
  }

  if (state1 == 1) {
    myservo2.write(140);
    //    Serial.println("myservo2");
  } else {
    myservo2.write(50);
    //    Serial.println("Un.myservo2");
  }

  if (state2 == 1) {
    myservo1.write(60);
    //    Serial.println("myservo1");
  } else {
    myservo1.write(100);
    //    Serial.println("Un.myservo1");
  }

  fo = map(PS4.LStickY(), 60, 127, 78, 255);
  back = map(PS4.LStickY(), -127, -60, 255, 78);
  right = map(PS4.LStickX(), 40, 127, 78, 255);
  left = map(PS4.LStickX(), -127, -40, 255, 78);
  turn_right = map(PS4.RStickX(), 40, 127, 78, 255);
  turn_left = map(PS4.RStickX(), -127, -40, 255, 78);

  if (PS4.LStickY() >= 60) {
    analogWrite(Motor1_EN1, fo);
    analogWrite(Motor2_EN2, fo);
    analogWrite(Motor3_EN3, fo);
    analogWrite(Motor4_EN4, fo);
    digitalWrite(Motor1_IN1, HIGH);
    digitalWrite(Motor1_IN2, LOW);
    digitalWrite(Motor2_IN3, HIGH);
    digitalWrite(Motor2_IN4, LOW);
    digitalWrite(Motor3_IN5, HIGH);
    digitalWrite(Motor3_IN6, LOW);
    digitalWrite(Motor4_IN7, HIGH);
    digitalWrite(Motor4_IN8, LOW);
    delay(2);
    // Serial.printf("Left Stick y at %d\n", PS4.LStickY());
    // Serial.println(fo);
    //  Serial.printf("fo");
  }

  else if (PS4.LStickY() <= -60) {
    analogWrite(Motor1_EN1, back);
    analogWrite(Motor2_EN2, back);
    analogWrite(Motor3_EN3, back);
    analogWrite(Motor4_EN4, back);
    digitalWrite(Motor1_IN1, LOW);
    digitalWrite(Motor1_IN2, HIGH);
    digitalWrite(Motor2_IN3, LOW);
    digitalWrite(Motor2_IN4, HIGH);
    digitalWrite(Motor3_IN5, LOW);
    digitalWrite(Motor3_IN6, HIGH);
    digitalWrite(Motor4_IN7, LOW);
    digitalWrite(Motor4_IN8, HIGH);
    delay(2);
    // Serial.printf("Left Stick y at %d\n", PS4.LStickY());
    // Serial.println(back);
    //  Serial.printf("back");
  }

  else if (PS4.LStickX() >= 40) {
    analogWrite(Motor1_EN1, right);
    analogWrite(Motor2_EN2, right);
    analogWrite(Motor3_EN3, right);
    analogWrite(Motor4_EN4, right);
    digitalWrite(Motor1_IN1, LOW);
    digitalWrite(Motor1_IN2, HIGH);
    digitalWrite(Motor2_IN3, HIGH);
    digitalWrite(Motor2_IN4, LOW);
    digitalWrite(Motor3_IN5, LOW);
    digitalWrite(Motor3_IN6, HIGH);
    digitalWrite(Motor4_IN7, HIGH);
    digitalWrite(Motor4_IN8, LOW);
    delay(2);
    // Serial.printf("Left Stick x at %d\n", PS4.LStickX());
    // Serial.println(right);
    //  Serial.printf("right");
  }

  else if (PS4.LStickX() <= -40) {
    analogWrite(Motor1_EN1, left);
    analogWrite(Motor2_EN2, left);
    analogWrite(Motor3_EN3, left);
    analogWrite(Motor4_EN4, left);
    digitalWrite(Motor1_IN1, HIGH);
    digitalWrite(Motor1_IN2, LOW);
    digitalWrite(Motor2_IN3, LOW);
    digitalWrite(Motor2_IN4, HIGH);
    digitalWrite(Motor3_IN5, HIGH);
    digitalWrite(Motor3_IN6, LOW);
    digitalWrite(Motor4_IN7, LOW);
    digitalWrite(Motor4_IN8, HIGH);
    delay(2);
    // Serial.printf("Left Stick x at %d\n", PS4.LStickX());
    // Serial.println(left);
    //  Serial.printf("left");

  }

  else if (PS4.RStickX() <= -40) {
    analogWrite(Motor1_EN1, turn_left);
    analogWrite(Motor2_EN2, turn_left);
    analogWrite(Motor3_EN3, turn_left);
    analogWrite(Motor4_EN4, turn_left);
    digitalWrite(Motor1_IN1, LOW);
    digitalWrite(Motor1_IN2, HIGH);
    digitalWrite(Motor2_IN3, LOW);
    digitalWrite(Motor2_IN4, HIGH);
    digitalWrite(Motor3_IN5, HIGH);
    digitalWrite(Motor3_IN6, LOW);
    digitalWrite(Motor4_IN7, HIGH);
    digitalWrite(Motor4_IN8, LOW);
    delay(2);
    // Serial.printf("Rigth Stick x at %d\n", PS4.RStickX());
    // Serial.println(turn_left);
    //  Serial.printf("turn_left");
  }

  else if (PS4.RStickX() >= 40) {
    analogWrite(Motor1_EN1, turn_right);
    analogWrite(Motor2_EN2, turn_right);
    analogWrite(Motor3_EN3, turn_right);
    analogWrite(Motor4_EN4, turn_right);
    digitalWrite(Motor1_IN1, HIGH);
    digitalWrite(Motor1_IN2, LOW);
    digitalWrite(Motor2_IN3, HIGH);
    digitalWrite(Motor2_IN4, LOW);
    digitalWrite(Motor3_IN5, LOW);
    digitalWrite(Motor3_IN6, HIGH);
    digitalWrite(Motor4_IN7, LOW);
    digitalWrite(Motor4_IN8, HIGH);
    delay(2);
    // Serial.printf("Ridth Stick x at %d\n", PS4.RStickX());
    // Serial.println(turn_right);
    //  Serial.printf("turn_right");
  }

  else {
    digitalWrite(Motor1_IN1, LOW);
    digitalWrite(Motor1_IN2, LOW);
    digitalWrite(Motor2_IN3, LOW);
    digitalWrite(Motor2_IN4, LOW);
    digitalWrite(Motor3_IN5, LOW);
    digitalWrite(Motor3_IN6, LOW);
    digitalWrite(Motor4_IN7, LOW);
    digitalWrite(Motor4_IN8, LOW);
  }
}
