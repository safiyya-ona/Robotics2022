#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define Joint4Pin 10
#define GripperPin 11
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
int GripperOpen = 15; // Open gripper;
int GripperClose = 120; // Close gripper; 
// Joint Angle Offsets
int Joint1Offset = 15; 
int Joint2Offset = 39; 
int Joint3Offset = 0; 
int Joint4Offset = -90;


// cartesian coordinates for x, y and z

// the positve direction for x is left - to go right, change to negative sign
int x = 19;

int y = 19;

// the postive direction for z is down - to go up, change to negative sign
int z = -10;

double L1 =9.5;
double L2 =18;

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

  int teta1 = getTheta1(x,y,z);
  int teta2 = getTheta2again(x,y,z);
  int teta3 = getTheta3(x,y,z);
 
  Joint4.write(Joint4Angle+Joint4Offset);
  Joint1.write(teta1 +Joint1Offset);
  Joint2.write(teta2 +Joint2Offset);
  Joint3.write(teta3 +Joint3Offset);
  Serial.print("theta1: ");
  Serial.print(teta1);
  Serial.print(" theta2: ");
  Serial.print(teta2);
  Serial.print(" theta3: ");  
  Serial.println(teta3);
  
  delay(10);

}


int getTheta1(double x, double y, double z){
  int theta1 = (atan2(y,x))*(180/PI);
  return theta1;
}

int getTheta2(double x, double y, double z){
  double r = sqrt(pow(x,2)+pow(y,2));
  double w = sqrt(pow(r,2)+pow(z,2));
  float A = pow(L1,2)+pow(r,2)+pow(z,2)-pow(L2,2);
  float B = 2*L1*w;
  int theta2 = -(atan2(z,r)-acos(A/B))*(180/PI);

  Serial.print("r: ");
  Serial.print(r);
  Serial.print(" w: ");
  Serial.print(w);
  Serial.print(" A: ");
  Serial.print(A);
  Serial.print(" B: ");
  Serial.println(B);
  return theta2;
}

int getTheta3(double x, double y, double z){
  double r = sqrt(pow(x,2)+pow(y,2));  
  float C = pow(r,2)+pow(z,2)-pow(L1,2)-pow(L2,2);
  float D = 2*L1*L2;
  int theta3 = (acos(C/D)*(180/PI));
  Serial.print("r: ");
  Serial.print(r);
  Serial.print(" C: ");
  Serial.print(C);
  Serial.print(" D: ");
  Serial.println(D);
  return theta3;
}

int getTheta2again(double x, double y, double z){
  double r = sqrt(pow(x,2)+pow(y,2));
  double w = sqrt(pow(r,2)+pow(z,2));

  double beta = atan2(z,r);
  if (beta < 0)
  {
    beta = -1 *  beta;
  }
  double psi = acos((pow(L2,2) - pow(L1,2) - pow(w,2)) / (-2 * L1 * w));

    Serial.print("r: ");
  Serial.print(r);
  Serial.print(" w: ");
  Serial.print(w);
  Serial.print(" psi: ");
  Serial.print(psi);
  Serial.print(" beta : ");
  Serial.println(beta);
  return (beta + psi) * (180 / PI);
}
