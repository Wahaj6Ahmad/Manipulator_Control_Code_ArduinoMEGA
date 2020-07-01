#include "def.h"
int desiredAngle = 0;
int currentAngle = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  pinMod(); //Function that sets pinModes to INPUTS and OUTPUTS
}

void loop() {
  //Inverse Kinematic Code to find angles
  servoAngle = findAngle(24, 16);
  //Function that drives the arm to the input angles for each link
  poseArm(90, servoAngle.alpha, servoAngle.beta, 90);
}
