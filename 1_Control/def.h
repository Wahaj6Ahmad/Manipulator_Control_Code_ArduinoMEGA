#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <stdio.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define m1s1 52 //motor 1 signal 1 is connected to digital pin 2 
#define m1s2 53
#define p1 A0   //pot 1 signal
#define m2s1 48
#define m2s2 49
#define p2 A2
#define m3s1 44
#define m3s2 45
#define p3 A4
#define m4 14  //m4 is connected to port 15 on PCA9685a 
#define m5 8  //m5 is connected to port 7 on PCA9685a 
#define m6 0   //m6 is connected to port x on PCA9685a 

#define min_p1 255 //minimum analog reading from pot 1
#define max_p1 940  //maximum analog reading from pot 1
#define min_p2 138 //backwards
#define max_p2 844 //forward
#define min_p3 60 //0 @ 720 pot value.
#define max_p3 800 //180 degres

#define min_s4 110 //minimum signal value for servo/motor 4
#define max_s4 500 //maximum signal value for servo/motor 4
#define min_s5 90
#define max_s5 495

#define min_d1 0  //minimum angle for motor 1
#define max_d1 180  //maximum angle for motor 1
#define min_d2 0
#define max_d2 180
#define min_d3 0
#define max_d3 160
#define min_d4 0
#define max_d4 180
#define min_d5 0
#define max_d5 180
#define max_error 20
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
bool m1flag = true;
bool m2flag = true;
bool m3flag = true;
bool init_flag = false;
double desiredPot[3] = {0.0};
double currentPot[3] = {0.0};
double error[3] = {0.0};
double desiredSignal = 0;

struct structA{double beta,alpha;}; //store values of angles in this struct to be used
structA servoAngle;
structA findAngle(float,float,float len1 = 26,float len2 = 30);
float x1,y1=0;

structA findAngle(float Y,float Z,float len1 = 26,float len2 = 30)
{
   double maxD = len1 + len2;
   structA s;
   double d = sqrt(square(Y)+square(Z));
   if (Y < 0 || Z < 0 || d >= maxD)
   {
    Serial.println("'finding angle' -- Input is out of range");
    s.alpha = 90; //if input is out of range, set L shaped position
    s.beta = 90; //FOR OUR LINK THIS SHOULD BE 180//
    return s;
   }
   else if (Y >= 0 && Z > 0 && d <= maxD)
   {
    double phi1 = (180/M_PI)*acos(Y/d);
    double theta1 = (180/M_PI)*(acos((square(len1)+square(d)-square(len2))/(2*len1*d)));
    double theta2 = (180/M_PI)*(acos((square(len1)-square(d)+square(len2))/(2*len1*len2)));
    s.alpha = (int)(theta1 + phi1);
    s.beta = abs((int) theta2 - 180);
  
    Serial.print("maxD = ");
    Serial.println(maxD);
    Serial.print("d = ");
    Serial.println(d);
    Serial.print("phi1 = ");
    Serial.println(phi1);
    Serial.print("theta 1 = ");
    Serial.println(theta1);
    Serial.print("theta2 = ");
    Serial.println(theta2);
    Serial.print("Alpha = ");
    Serial.println(s.alpha);
    Serial.print("Beta = ");
    Serial.println(s.beta);
      
    return s;
   }
   
}
void pinMod() //sets pins to INPUT or OUTPUT
{
  pinMode(m1s1, OUTPUT);
  pinMode(m1s2, OUTPUT);
  pinMode(m2s1, OUTPUT);
  pinMode(m2s2, OUTPUT);
  pinMode(m3s1, OUTPUT);
  pinMode(m3s2, OUTPUT);

  pinMode(p1, INPUT);
  pinMode(p2, INPUT);
  pinMode(p3, INPUT);
}
void readPot()
{

  Serial.print("Pot 1 = ");
  Serial.println(analogRead(p1));
  Serial.print("Pot 2 = ");
  Serial.println(analogRead(p2));
  Serial.print("Pot 3 = ");
  Serial.println(analogRead(p3));
  delay(500);
}
void moveM1(char where)
{
  if (analogRead(p1) > min_p1 && analogRead(p1) < max_p1)
  {
    if (where == 'l') //move left
    {
      digitalWrite(m1s1, LOW);
      digitalWrite(m1s2, HIGH);
      Serial.println("Motor 1 moving Left");
    }
    else if (where == 'r') //move right
    {
      digitalWrite(m1s1, HIGH);
      digitalWrite(m1s2, LOW);
      Serial.println("Motor 1 moves right");
    }
  }
  else
  {
    digitalWrite(m1s1, LOW);
    digitalWrite(m1s2, LOW);
  }
}

void moveM2(char where)
{
  if (analogRead(p2) > min_p2 && analogRead(p2) < max_p2)
  {
    if (where == 'f') //move forwards
    {
      digitalWrite(m2s1, HIGH);
      digitalWrite(m2s2, LOW);
      Serial.println("Motor 2 moving Forwards");
    }
    else   if (where == 'b') //move backwards
    {
      digitalWrite(m2s1, LOW);
      digitalWrite(m2s2, HIGH);
      Serial.println("Motor 2 moves backwards");
    }
  }
  else
  {
    digitalWrite(m2s1, LOW);
    digitalWrite(m2s2, LOW);
  }
}

void moveM3(char where)
{
  if (analogRead(p3) > (min_p3-20) && analogRead(p3) < (max_p3+200))
  {
    if (where == 'b') //move backwards
    {
      digitalWrite(m3s1, HIGH);
      digitalWrite(m3s2, LOW);
      Serial.println("Motor 3 moving Backwards");
    }
    else   if (where == 'f') //move forwards
    {
      digitalWrite(m3s1, LOW);
      digitalWrite(m3s2, HIGH);
      Serial.println("Motor 3 moves forwards");
    }
  }
  else
  {
    digitalWrite(m3s1, LOW);
    digitalWrite(m3s2, LOW);
  }
}

void poseArm(double th1, double th2, double th3, double th4)
{
  desiredPot[0] = map(th1, min_d1, max_d1, min_p1, max_p1);
  desiredPot[1] = map(th2, min_d2, max_d2, min_p2, max_p2);
  desiredPot[2] = map(th3, min_d3, max_d3, min_p3, max_p3);

  currentPot[0] = analogRead(p1);
  currentPot[1] = analogRead(p2);
  currentPot[2] = analogRead(p3);

  error[0] = currentPot[0] - desiredPot[0]; //sign dictates direction
  error[1] = currentPot[1] - desiredPot[1];
  error[2] = currentPot[2] - desiredPot[2];

  Serial.print("Desired Pot1 = "); Serial.println(desiredPot[0]);
  Serial.print("Current Pot1 = "); Serial.println(currentPot[0]);
  Serial.print("Error 1 = "); Serial.println(error[0]);
  Serial.print("Desired Pot2 = "); Serial.println(desiredPot[1]);
  Serial.print("Current Pot2 = "); Serial.println(currentPot[1]);
  Serial.print("Error 2 = "); Serial.println(error[1]);
  Serial.print("Desired Pot3 = "); Serial.println(desiredPot[2]);
  Serial.print("Current Pot3 = "); Serial.println(currentPot[2]);
  Serial.print("Error 3 = "); Serial.println(error[2]);

  if (abs(error[0]) > max_error && m1flag == true)
  {
    if (error[0] < 0) moveM1('r');
    else if (error[0] > 0)moveM1('l');
  }
  else if(abs(error[0]) < max_error) 
  {
    m1flag = false;
    digitalWrite(m1s1,LOW);
    digitalWrite(m1s2,LOW);
  }
  if (abs(error[1]) > max_error && m2flag == true) //opposite forward backward conditionbecause the pot at motor 1 is connected opposite
  {
    if (error[1] < 0) moveM2('b');
    else if (error[0] > 0) moveM2('f');
  }
  else if(abs(error[1]) < max_error)
  {
    m2flag = false;
    digitalWrite(m2s1,LOW);
    digitalWrite(m2s2,LOW);
  }
  if (abs(error[2]) > max_error && m3flag == true)
  {
    if (error[2] < 0) moveM3('f');
    else if (error[2] > 0) moveM3('b');
  }
else if(abs(error[2]) < max_error)
{
  m3flag = false;
  digitalWrite(m3s1,LOW);
  digitalWrite(m3s2,LOW);
}
  desiredSignal = map(th4, min_d4, max_d4, min_s4, max_s4);
  pwm.setPWM(m4, 0, desiredSignal);

}

void goToAngle(int motor, int angle) //which motor goes to which angle
{
  if (motor == 1)
  {
    int desiredPot1 = map(angle, min_d1, max_d1, min_p1, max_p1);
    int currentPot1 = analogRead(p1);
    while (abs(currentPot1 - desiredPot1) > max_error)
    {
      if (currentPot1 < desiredPot1)
      {
        digitalWrite(m1s1, HIGH);
        digitalWrite(m1s2, LOW);
        Serial.println("INIT - Motor 1 moves right");
      }
      else if (currentPot1 > desiredPot1)
      {
        digitalWrite(m1s1, LOW);
        digitalWrite(m1s2, HIGH);
        Serial.println("INIT - Motor 1 moving Left");
      }
    }
  }
  if (motor == 2)
  {
    int desiredPot2 = map(angle, min_d2, max_d2, min_p2, max_p2);
    int currentPot2 = analogRead(p2);
    while (abs(currentPot2 - desiredPot2) > max_error)
    {
      if (currentPot2 < desiredPot2) //backwards
      {
        digitalWrite(m2s1, LOW);
        digitalWrite(m2s2, HIGH);
        Serial.println("INIT - Motor 2 moves backwards");
      }
      else if (currentPot2 > desiredPot2)
      {
        digitalWrite(m2s1, HIGH);
        digitalWrite(m2s2, LOW);
        Serial.println("INIT - Motor 2 moving forward");
      }
    }
  }
  if (motor == 3)
  {
    int desiredPot3 = map(angle, min_d3, max_d3, min_p3, max_p3);
    int currentPot3 = analogRead(p3);
    while (abs(currentPot3 - desiredPot3) > max_error)
    {
      if (currentPot3 < desiredPot3) //forward
      {
        digitalWrite(m3s1, LOW);
        digitalWrite(m3s2, HIGH);
        Serial.println("INIT - Motor 3 moves forward");
      }
      else if (currentPot3 > desiredPot3) //backwards
      {
        digitalWrite(m3s1, HIGH);
        digitalWrite(m3s2, LOW);
        Serial.println("INIT - Motor 3 moving backwards");
      }
    }
  }
}

void initiate()
{
  goToAngle(3, 90);
  goToAngle(2, 90);
  goToAngle(3, 90);
  pwm.setPWM(m4, 0, map(90, min_d4, max_d4, min_s4, max_s4));
}
