#include <PS4Controller.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define LmY 15
#define LmX 0
#define Linear1 34
#define Linear2 12

#define dirPinX 17
#define StepPinX 16
#define endPinX 4

#define dirPinY 19
#define StepPinY 18
#define endPinY 5


#define BallC 2
int fo;
int back;
int left;
int right;
int turn_left;
int turn_right;

// DC Motor_Forward_Left
#define Motor4_IN7 14
#define Motor4_IN8 27

// DC Motor_Backward_Left
#define Motor1_IN1 13  //13
#define Motor1_IN2 23  //12

// DC Motor_Forward_Right
#define Motor2_IN3 32  //32
#define Motor2_IN4 33  //33

// DC Motor_Backward_Right
#define Motor3_IN5 25  // 25
#define Motor3_IN6 26  // 26

#define ALinearAcc 3 //3
#define BLinearAcc 1  //1

#define servo1 0
#define servo2 1
#define servo3 2
#define servo4 3
#define servo5 4
#define servo6 5
#define servoMin 195
#define servoMax 650

int Ty;
int Tx;
//StepperX
boolean buttonStateX;
boolean lastStateX;
boolean stateX = 0;
//StepperY
boolean buttonStateY;
boolean lastStateY;
boolean stateY = 0;
//Servo1,2
boolean buttonState_S12;
boolean lastState_S12;
boolean state_S12 = 0;
//Servo3,4
boolean buttonState_S34;
boolean lastState_S34;
boolean state_S34 = 0;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

boolean buttonStateBall;
boolean lastStateBall;
boolean stateBall = 0;

boolean buttonStateBall2;
boolean lastStateBall2;
boolean stateBall2 = 0;

boolean buttonStateBall_Arm;
boolean lastStateBall_Arm;
boolean stateBall_Arm = 0;

bool Run = false;

void setup() {
  // Serial.begin(115200);
  PS4.begin();
  pinMode(StepPinX, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(endPinX, OUTPUT);
  pinMode(StepPinY, OUTPUT);
  pinMode(dirPinY, OUTPUT);
  pinMode(endPinY, OUTPUT);
  pinMode(LmY, INPUT_PULLUP);
  pinMode(LmX, INPUT_PULLUP);
  pinMode(Linear1, INPUT_PULLUP);
  pinMode(Linear2, INPUT_PULLUP);
  pinMode(ALinearAcc, OUTPUT);
  pinMode(BLinearAcc, OUTPUT);
  pinMode(BallC, OUTPUT);

  pinMode(Motor1_IN1, OUTPUT);
  pinMode(Motor1_IN2, OUTPUT);
  pinMode(Motor2_IN3, OUTPUT);
  pinMode(Motor2_IN4, OUTPUT);
  pinMode(Motor3_IN5, OUTPUT);
  pinMode(Motor3_IN6, OUTPUT);
  pinMode(Motor4_IN7, OUTPUT);
  pinMode(Motor4_IN8, OUTPUT);

  analogWrite(Motor1_IN1, 0);
  analogWrite(Motor1_IN2, 0);
  analogWrite(Motor2_IN3, 0);
  analogWrite(Motor2_IN4, 0);
  analogWrite(Motor3_IN5, 0);
  analogWrite(Motor3_IN6, 0);
  analogWrite(Motor4_IN7, 0);
  analogWrite(Motor4_IN8, 0);

  pwm.begin();
  pwm.setPWMFreq(60);
  delay(20);
  // HOME();
}

bool LMS;
bool Lm_X = false;
bool Lm_Y = false;
bool Lm_L = false;
bool State_LM;


void loop() {
  Connect();
  if (Run == true) {
    motion();
    //StepperX
    int buttonX = PS4.Square();
    buttonStateX = buttonX;
    if ((buttonStateX == 0) && (lastStateX == 1))
      stateX = !stateX;
    lastStateX = buttonStateX;
    //StepperY
    int buttonY = PS4.Triangle();
    buttonStateY = buttonY;
    if ((buttonStateY == 0) && (lastStateY == 1))
      stateY = !stateY;
    lastStateY = buttonStateY;
    //Servo1,2
    int button_S12 = PS4.Cross();
    buttonState_S12 = button_S12;
    if ((buttonState_S12 == 0) && (lastState_S12 == 1))
      state_S12 = !state_S12;
    lastState_S12 = buttonState_S12;
    //Servo3,4
    int button_S34 = PS4.Circle();
    buttonState_S34 = button_S34;
    if ((buttonState_S34 == 0) && (lastState_S34 == 1))
      state_S34 = !state_S34;
    lastState_S34 = buttonState_S34;

    int buttonBall = PS4.Down();
    buttonStateBall = buttonBall;
    if ((buttonStateBall == 0) && (lastStateBall == 1))
      stateBall = !stateBall;
    lastStateBall = buttonStateBall;

    int buttonBall2 = PS4.Up();
    buttonStateBall2 = buttonBall2;
    if ((buttonStateBall2 == 0) && (lastStateBall2 == 1))
      stateBall2 = !stateBall2;
    lastStateBall2 = buttonStateBall2;


    int buttonBall_Arm = PS4.L1();
    buttonStateBall_Arm = buttonBall_Arm;
    if ((buttonStateBall_Arm == 0) && (lastStateBall_Arm == 1))
      stateBall_Arm = !stateBall_Arm;
    lastStateBall_Arm = buttonStateBall_Arm;

    //Stepper
    if (PS4.Triangle() == 1) {
      StepperY();
    } else if (PS4.Square() == 1) {
      StepperX();
    } else {
      Tx = 0;
      Ty = 0;
    }

    //Servo1,2
    if (PS4.Cross() == 1 && state_S12 == 0) {
      Servo12();
      Serial.println("servo1_0");
    } else if (PS4.Cross() == 1 && state_S12 == 1) {
      Servo12();
      Serial.println("servo1_180");
    }
    //Servo3,4
    if (PS4.Circle() == 1 && state_S34 == 0) {
      Servo34(50);
      Serial.println("servo3_0");
    } else if (PS4.Circle() == 1 && state_S34 == 1) {
      Servo34(60);
      Serial.println("servo3_180");
    }

    //Servo5
    if (PS4.Down() == 1 && stateBall == 0) {
      Servo5();
      Serial.println("servo5_0");
    } else if (PS4.Down() == 1 && stateBall == 1) {
      Servo5();
      Serial.println("servo5_180");
    }

    //Servo_Armball
    int Limit_linear2 = digitalRead(Linear2);
    if (PS4.L1() == 1 && stateBall_Arm == 0 && Limit_linear2 == LOW) {
      ServoArm_ball();
      Serial.println("servo5_0");
    } else if (PS4.L1() == 1 && stateBall_Arm == 1 && Limit_linear2 == LOW) {
      ServoArm_ball();
      Serial.println("servo5_180");
    }

    //Ball Cannon
    if (PS4.Up() == 1) {
      if (stateBall2 == 1) {
        digitalWrite(BallC, HIGH);
      } else if (stateBall2 == 0) {
        digitalWrite(BallC, LOW);
      }
    }

    //LinearAcc//
    if (PS4.Right() == 1 || PS4.Left() == 1) {
      LinearAcc();
    } else {
      analogWrite(ALinearAcc, 0);
      analogWrite(BLinearAcc, 0);
      delay(10);
      Serial.println("StopLA");
    }

    //Home
    if (PS4.PSButton() == 1) {
      Lm_X = false;
      Lm_Y = false;
      Lm_L = false;
      State_LM == false;
      Serial.println("HOME");
      HOME();
    }

    Serial.println("Run");
  } else if (Run == false) {
    // HOME();
    motion();
    Serial.println("StopRun");
  }
}



void Servo12() {
  if (state_S12 == 1) {
    for (int pulse = 425; pulse > 195; pulse -= 1) {
      pwm.setPWM(servo1, 0, pulse);
      pwm.setPWM(servo2, 0, pulse);
      Serial.println("180");
    }
  }
  if (state_S12 == 0) {
    for (int pulse = 195 ; pulse < 425; pulse += 1) {
      pwm.setPWM(servo1, 0, pulse);
      pwm.setPWM(servo2, 0, pulse);
      Serial.println("0");
    }
  }
}

void Servo34(int degree) {
  if (state_S34 == 1) {
    for (int pulse = 650; pulse > 155; pulse -= 1) {
      pwm.setPWM(servo3, 0, pulse);
      pwm.setPWM(servo4, 0, pulse);
    }
  }
  if (state_S34 == 0) {
    for (int pulse = 150; pulse < 650; pulse += 1) {
      pwm.setPWM(servo3, 0, pulse);
      pwm.setPWM(servo4, 0, pulse);
      Serial.println(pulse);
    }
  }
}

void Servo5() {
  if (stateBall == 1) {
    for (int pulse = 390; pulse > 195; pulse -= 1) {
      pwm.setPWM(servo5, 0, pulse);
      Serial.println("180");
    }
  }
  if (stateBall == 0) {
    for (int pulse = 195; pulse < 390; pulse += 1) {
      pwm.setPWM(servo5, 0, pulse);
      Serial.println("0");
    }
  }
}

void ServoArm_ball() {
  if (stateBall_Arm == 0) {
    for (int pulse = 560; pulse > 195; pulse -= 1) {  //195
      pwm.setPWM(servo6, 0, pulse);
      Serial.println("180");
    }
  }
  if (stateBall_Arm == 1) {
    for (int pulse = 195; pulse < 560; pulse += 1) {
      pwm.setPWM(servo6, 0, pulse);
      Serial.println("0");
    }
  }
}

void Connect() {
  if (PS4.isConnected()) {
    Run = true;
  } else {
    Run = false;
  }
}

void HOME() {
  int Limit_X = digitalRead(LmX);
  if (Limit_X == HIGH && Lm_X == false) {
    while (Limit_X == HIGH) {
      digitalWrite(dirPinX, LOW);
      digitalWrite(StepPinX, HIGH);
      delayMicroseconds(1000);
      digitalWrite(StepPinX, LOW);
      delayMicroseconds(1000);
      Limit_X = digitalRead(LmX);
      Serial.println("HOME X");
    }
    digitalWrite(endPinX, LOW);
  } else if (Limit_X == LOW) {
    Lm_X = true;
  }

   int Limit_Y = digitalRead(LmY);
  if (Limit_Y == HIGH && Lm_Y == false) {
    while (Limit_Y == HIGH) {
      digitalWrite(dirPinY, HIGH);
      digitalWrite(StepPinY, HIGH);
      delayMicroseconds(1000);
      digitalWrite(StepPinY, LOW);
      delayMicroseconds(1000);
      Limit_Y = digitalRead(LmY);
      Serial.println("HOME Y");
    }
    digitalWrite(endPinY, LOW);
  } else if (Limit_Y == LOW) {
    Lm_Y = true;
  }


  int Limit_LA = digitalRead(Linear1);
  if (Limit_LA == HIGH && Lm_L == false) {
    while (Limit_LA == HIGH) {
      analogWrite(ALinearAcc, 255);
      analogWrite(BLinearAcc, 0);
      delay(10);
      Limit_LA = digitalRead(Linear1);
      Serial.println("HOME LA");
    }
  } else if (Limit_LA == LOW) {
    Lm_L = true;
  }
}

void LinearAcc() {
  int Limit_linear1 = digitalRead(Linear1);
  int Limit_linear2 = digitalRead(Linear2);
  if (PS4.Left() == 1 && Limit_linear2 == HIGH) {
    LMS = true;
  } else if (PS4.Right() == 1 && Limit_linear1 == HIGH) {
    LMS = false;
  }
  if (PS4.Left() == 1) {
    if (Limit_linear2 == LOW && LMS == true) {
      analogWrite(ALinearAcc, 0);
      analogWrite(BLinearAcc, 0);
      delay(10);
      Serial.println("StopLML");
    } else if (PS4.Left() == 1) {
      analogWrite(ALinearAcc, 0);
      analogWrite(BLinearAcc, 255);
      delay(10);
      Serial.println("L");
    }
  }

  if (PS4.Right() == 1) {
    if (Limit_linear1 == LOW && LMS == false) {
      analogWrite(ALinearAcc, 0);
      analogWrite(BLinearAcc, 0);
      delay(10);
      Serial.println("StopLMR");
    } else if (PS4.Right() == 1) {
      analogWrite(ALinearAcc, 255);
      analogWrite(BLinearAcc, 0);
      delay(10);
      Serial.println("R");
    }
  }
}
bool enter1 = true;
bool enter2 = true;
void StepperX() {
  if (stateX == 1 && Tx != 3800) {
    enter2 = true;
    while (Tx < 3800) {
      if (enter1 == true) {
        digitalWrite(dirPinX, LOW);
        for (int j = 800; j < 1050; j++) {
          digitalWrite(StepPinX, HIGH);
          delayMicroseconds(1200 - j);
          digitalWrite(StepPinX, LOW);
          delayMicroseconds(1200 - j);
        }
        enter1 = false;
      }

      Tx++;
      digitalWrite(StepPinX, HIGH);
      delayMicroseconds(150);
      digitalWrite(StepPinX, LOW);
      delayMicroseconds(150);
    }
  } else if (stateX == 0 && Tx != 3800) {
    enter1 = true;
    while (Tx < 3800) {
      if (enter2 == true) {
        digitalWrite(dirPinX, HIGH);
        for (int j = 800; j < 1050; j++) {

          digitalWrite(StepPinX, HIGH);
          delayMicroseconds(1200 - j);
          digitalWrite(StepPinX, LOW);
          delayMicroseconds(1200 - j);
        }
        enter2 = false;
      }
      Tx++;
      digitalWrite(StepPinX, HIGH);
      delayMicroseconds(150);
      digitalWrite(StepPinX, LOW);
      delayMicroseconds(150);
    }
  }
}

bool enter3 = true;
bool enter4 = true;
void StepperY() {
  if (stateY == 1 && Ty != 1300) {
    enter3 = true;
    while (Ty < 1300) {
      if (enter3 == true) {
        digitalWrite(dirPinY, HIGH);
        for (int j = 700; j < 1040; j++) {
          digitalWrite(StepPinY, HIGH);
          delayMicroseconds(1200 - j);
          digitalWrite(StepPinY, LOW);
          delayMicroseconds(1200 - j);
        }
        enter3 = false;
      }

      Ty++;
      digitalWrite(StepPinY, HIGH);
      delayMicroseconds(160);
      digitalWrite(StepPinY, LOW);
      delayMicroseconds(160);
    }
    // digitalWrite(endPinY, HIGH);
  } else if (stateY == 0 && Ty != 1300) {
    enter4 = true;
    // digitalWrite(endPinY, LOW);
    while (Ty < 1300) {
      if (enter4 == true) {
        digitalWrite(dirPinY, LOW);
        for (int j = 700; j < 1040; j++) {

          digitalWrite(StepPinY, HIGH);
          delayMicroseconds(1200 - j);
          digitalWrite(StepPinY, LOW);
          delayMicroseconds(1200 - j);
        }
        enter4 = false;
      }
      Ty++;
      digitalWrite(StepPinY, HIGH);
      delayMicroseconds(160);
      digitalWrite(StepPinY, LOW);
      delayMicroseconds(160);
    }
  }
}

void motion() {
  fo = map(PS4.LStickY(), 50, 127, 0, 255);
  back = map(PS4.LStickY(), -127, -50, 255, 0);
  right = map(PS4.LStickX(), 40, 127, 0, 255);
  left = map(PS4.LStickX(), -127, -40, 255, 0);
  turn_right = map(PS4.RStickX(), 40, 127, 0, 255);
  turn_left = map(PS4.RStickX(), -127, -40, 255, 0);
  if (PS4.LStickY() >= 40) {
    analogWrite(Motor1_IN1, fo);
    analogWrite(Motor1_IN2, 0);
    analogWrite(Motor2_IN3, fo);
    analogWrite(Motor2_IN4, 0);
    analogWrite(Motor3_IN5, fo);
    analogWrite(Motor3_IN6, 0);
    analogWrite(Motor4_IN7, fo);
    analogWrite(Motor4_IN8, 0);
    delay(10);
  } else if (PS4.LStickY() <= -40) {
    analogWrite(Motor1_IN1, 0);
    analogWrite(Motor1_IN2, back);
    analogWrite(Motor2_IN3, 0);
    analogWrite(Motor2_IN4, back);
    analogWrite(Motor3_IN5, 0);
    analogWrite(Motor3_IN6, back);
    analogWrite(Motor4_IN7, 0);
    analogWrite(Motor4_IN8, back);
    delay(10);
  } else if (PS4.LStickX() >= 40) {
    analogWrite(Motor1_IN1, right);
    analogWrite(Motor1_IN2, 0);
    analogWrite(Motor2_IN3, right);
    analogWrite(Motor2_IN4, 0);
    analogWrite(Motor3_IN5, 0);
    analogWrite(Motor3_IN6, right);
    analogWrite(Motor4_IN7, 0);
    analogWrite(Motor4_IN8, right);
    delay(10);
  } else if (PS4.LStickX() <= -40) {
    analogWrite(Motor1_IN1, 0);
    analogWrite(Motor1_IN2, left);
    analogWrite(Motor2_IN3, 0);
    analogWrite(Motor2_IN4, left);
    analogWrite(Motor3_IN5, left);
    analogWrite(Motor3_IN6, 0);
    analogWrite(Motor4_IN7, left);
    analogWrite(Motor4_IN8, 0);
    delay(10);
  } else if (PS4.RStickX() <= -40) {
    analogWrite(Motor1_IN1, 0);
    analogWrite(Motor1_IN2, turn_left);
    analogWrite(Motor2_IN3, turn_left);
    analogWrite(Motor2_IN4, 0);
    analogWrite(Motor3_IN5, turn_left);
    analogWrite(Motor3_IN6, 0);
    analogWrite(Motor4_IN7, 0);
    analogWrite(Motor4_IN8, turn_left);
    delay(10);
  } else if (PS4.RStickX() >= 40) {
    analogWrite(Motor1_IN1, 0);
    analogWrite(Motor1_IN1, turn_right);
    analogWrite(Motor1_IN2, 0);
    analogWrite(Motor2_IN3, 0);
    analogWrite(Motor2_IN4, turn_right);
    analogWrite(Motor3_IN5, 0);
    analogWrite(Motor3_IN6, turn_right);
    analogWrite(Motor4_IN7, turn_right);
    analogWrite(Motor4_IN8, 0);
    delay(10);
  } else if (Run == false) {
    analogWrite(Motor1_IN1, 0);
    analogWrite(Motor1_IN2, 0);
    analogWrite(Motor2_IN3, 0);
    analogWrite(Motor2_IN4, 0);
    analogWrite(Motor3_IN5, 0);
    analogWrite(Motor3_IN6, 0);
    analogWrite(Motor4_IN7, 0);
    analogWrite(Motor4_IN8, 0);
    delay(10);
  } else {
    analogWrite(Motor1_IN1, 0);
    analogWrite(Motor1_IN2, 0);
    analogWrite(Motor2_IN3, 0);
    analogWrite(Motor2_IN4, 0);
    analogWrite(Motor3_IN5, 0);
    analogWrite(Motor3_IN6, 0);
    analogWrite(Motor4_IN7, 0);
    analogWrite(Motor4_IN8, 0);
    delay(10);
  }
  Serial.println("mo");
}