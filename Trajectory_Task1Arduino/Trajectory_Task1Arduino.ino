#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define Joint4Pin 10
#define GripperPin 11
#define L2 9.5
#define L3 18.5
#define FinalTime 5.0
#define RadiansToDegrees 180/PI

// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Joint4;
Servo Gripper;

// Starting Joint Angles
int Joint1Angle = 90;
int Joint2Angle = 90;
int Joint3Angle = 90;
int Joint4Angle = 180;
int GripperOpen = 80; // Open gripper; Need to tune value
int GripperClose = 170; // Close gripper; Need to tune value

// Joint Angle Offsets
int Joint1Offset = 15; // Your value may be different
int Joint2Offset = 39; // Your value may be different
int Joint3Offset = 0; // Your value may be different
int Joint4Offset = -90; // Your value may be different

// the positve direction for x is left - to go right, change to negative sign
double initX = 0.0;
double finalX = -10.0;

double initY = 18.0;
double finalY = 5.0;

// the postive direction for z is down - to go up, change to negative sign
double initZ = -9.5;
double finalZ = -5.0;

int Joint1AnglePrev = 90;
int Joint2AnglePrev = 90;
int Joint3AnglePrev = 90;



void setup() {
  Serial.begin(9600);
  Joint1.attach(Joint1Pin);
  Joint2.attach(Joint2Pin);
  Joint3.attach(Joint3Pin);
  Joint4.attach(Joint4Pin);
  
  Joint1.write(Joint1Angle+Joint1Offset);
  Joint2.write(Joint2Angle+Joint2Offset);
  Joint3.write(Joint3Angle+Joint3Offset);
  Gripper.attach(GripperPin);
  Joint4.write(Joint4Angle+Joint4Offset);
  Gripper.write(GripperOpen); // Open gripper
  delay(5000); // Wait 5 seconds before going into loop()

}

void loop() {
  startingPostion();
  delay(10000);
  doApproach();
  delay(10000);
  doReturn();
  delay(10000);
}


int getTheta1(double x, double y, double z){
  int theta1 = (atan2(y,x))*(RadiansToDegrees);
  if (!isValidAngle(theta1))
  {
    return Joint1AnglePrev;
  } else {
    Joint1AnglePrev = theta1;
    return abs(theta1);
  }  
}

int getTheta2(double x, double y, double z){
  double r = sqrt(pow(x,2)+pow(y,2));
  double w = sqrt(pow(r,2)+pow(z,2));
  double A = pow(L2,2)+pow(r,2)+pow(z,2)-pow(L3,2);
  double B = 2*L2*w;
  if (!isValidAngle(atan2(z,r)) || !isValidAngle(acos(A/B)))
  {
    return Joint2AnglePrev;
  } else {
    int theta2 = (acos(A/B)-atan2(z,r)) * RadiansToDegrees;
    Joint2AnglePrev = theta2;
    return theta2;
  }
}

int getTheta3(double x, double y, double z){
  double r = sqrt(pow(x,2)+pow(y,2));  
  double C = pow(r,2)+pow(z,2)-pow(L2,2)-pow(L3,2);
  double D = 2*L2*L3;
  if (!isValidAngle((acos(C/D)))){
    return Joint3AnglePrev;
  }
  else {
    int theta3 = (acos(C/D)*RadiansToDegrees);
    Joint3AnglePrev = theta3;
    return theta3;
  }
}

bool isValidAngle(double angle){
  if (isnan(angle) || isinf(angle))
  {
    return false;
  }
  else {
    return true;
  }
}

double getTrajectoryatT(double initial, double finalPos, double t){
  if (t >= FinalTime)
  {
    return finalPos;
  }
  return initial + (3.0/pow(FinalTime,2) * (finalPos - initial) * pow(t,2)) - (2.0/pow(FinalTime,3) * (finalPos-initial) * pow(t,3));
}

void doApproach(){
  // move to position in FinalTime seconds
  double x;
  double y;
  double z;
  Serial.println("Approach");

  for (double i = 0; i <= FinalTime; i = i + 0.1)
  {
    x = getTrajectoryatT(initX, finalX, i);
    y = getTrajectoryatT(initY, finalY, i);
    z = getTrajectoryatT(initZ, finalZ, i);

    // Calculate angles

    moveToPosition(x,y,z);
  }  
}

void doReturn(){
  double x;
  double y;
  double z;

  // move in FinalTime seconds
  Serial.println("Return");
  
  for (double i = 0; i <= FinalTime; i = i + 0.1)
  {
    x = getTrajectoryatT(finalX, initX, i);
    y = getTrajectoryatT(finalY, initY, i);
    z = getTrajectoryatT(finalZ, initZ, i);

    // Calculate angles

    moveToPosition(x,y,z);
  }
}

void startingPostion() {
  Joint1.write(90+Joint1Offset);
  Joint2.write(90+Joint2Offset);
  Joint3.write(90+Joint3Offset);
  Gripper.attach(GripperPin);
  Joint4.write(180+Joint4Offset);
  Gripper.write(GripperOpen); // Open gripper
}

void moveToPosition(double x, double y, double z){
  Joint1Angle = getTheta1(x,y,z); 
  Joint2Angle = getTheta2(x,y,z); 
  Joint3Angle = getTheta3(x,y,z); 


  Serial.print("Joint 1: ");
  Serial.print(Joint1Angle);
  Serial.print(", Joint 2: ");
  Serial.print(Joint2Angle);
  Serial.print(", Joint 3: ");
  Serial.print(Joint3Angle);

  Serial.print("     X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  Joint1.write(Joint1Angle+Joint1Offset);
  Joint2.write(Joint2Angle+Joint2Offset);
  Joint3.write(Joint3Angle+Joint3Offset);
  Joint4.write(Joint4Angle+Joint4Offset);
  delay(100);
}
