#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define Joint4Pin 10
#define GripperPin 11
#define L1 9.5
#define L2 18.0
#define FinalTime 5.0

// Control pins
int Joint1ControlPin = A1;
int Joint2ControlPin = A2;
int Joint3ControlPin = A3;

// Control values
int Joint1Control = 512; // middle value between 0 and 1024
int Joint2Control = 512; // middle value between 0 and 1024
int Joint3Control = 512; // middle value between 0 and 1024


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
double startX, currentX = 0.0;

double startY, currentY = 18.0;

// the postive direction for z is down - to go up, change to negative sign
double startZ, currentZ = -9.5;

double startingPosition[3] = {0.0, 18.0, -9.5};
double objectInit[3] = {10.0,10.0,5.0};
double objectTarget[3] = {-10.0,10.0,5.0};

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
  doGripper(GripperOpen);
  // go to object
  moveTo(objectInit);
  // grab the object
  doGripper(GripperClose);
  Serial.println("back to start");
  // move to start position
  moveTo(startingPosition);
  // place object
  moveTo(objectTarget);
  doGripper(GripperOpen);
  delay(10000);
  moveTo(startingPosition);
}

int getTheta1(double x, double y, double z){
  int theta1 = (atan2(y,x))*(180/PI);
  if (!isValidAngle(theta1))
  {
    return Joint1AnglePrev;
  } else {
    Joint1AnglePrev = theta1;
    return theta1;
  }  
}

int getTheta2(double x, double y, double z){
  double r = sqrt(pow(x,2)+pow(y,2));
  double w = sqrt(pow(r,2)+pow(z,2));
  double A = pow(L1,2)+pow(r,2)+pow(z,2)-pow(L2,2);
  double B = 2*L1*w;
  if (!isValidAngle(atan2(z,r)) || !isValidAngle(acos(A/B)))
  {
    return Joint2AnglePrev;
  } else {
    int theta2 = -(atan2(z,r)-acos(A/B))*(180/PI);
    Joint2AnglePrev = theta2;
    return theta2;
  }
}

int getTheta3(double x, double y, double z){
  double r = sqrt(pow(x,2)+pow(y,2));  
  double C = pow(r,2)+pow(z,2)-pow(L1,2)-pow(L2,2);
  double D = 2*L1*L2;
  if (!isValidAngle((acos(C/D)))){
    return Joint3AnglePrev;
  }
  else {
    int theta3 = (acos(C/D)*(180/PI));
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

void moveToPosition(double x, double y, double z){
  // this takes the Cartesian Coordinates from the trajectory and uses inverse kinematics to get to the respective position
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

void doGripper(int angle){
  Gripper.write(angle+Joint4Offset);
}

void moveTo( double endingPos[3]){
  double x,y,z;

  for (double i = 0; i < FinalTime; i = i + 0.1)
  {
    x = getTrajectoryatT(currentX, endingPos[0], i);
    y = getTrajectoryatT(currentY, endingPos[1], i);
    z = getTrajectoryatT(currentZ, endingPos[2], i);
    moveToPosition(x,y,z);
  }

  currentX = endingPos[0];
  currentY = endingPos[1];
  currentZ = endingPos[2];
}
