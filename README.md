# self-driving-robot
code that leads arduiono robot with ultrasonic sensors around a maze

#include <Servo.h>
#include <NewPing.h>


#define TRIGGER_PIN  A2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 250 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
// NewPing setup of pins and maximum distance.
#define TRIGGER_PIN13 A0
#define ECHO_PIN13 A0
#define TRIGGER_PIN1 A1
#define ECHO_PIN1 A1
Servo servoRight;
Servo servoLeft;
Servo servoArm;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
NewPing sonar1(TRIGGER_PIN13, ECHO_PIN13, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);

int servoPinArm = 11;
int servoPinWheelRight = 12;
int servoPinWheelLeft = 13;
void setup() {
  // put your setup code here, to run once:
  servoRight.attach(servoPinWheelRight);
  servoLeft.attach(servoPinWheelLeft);
  servoArm.attach(servoPinArm);
  //pinMode(trigPin, OUTPUT); 
  //pinMode(echogozPin, INPUT);
  Serial.begin(9600);
  servoArm.write(180);
  
   //goForwards();
}

void turnLeft(){
  servoRight.writeMicroseconds(1500);
  servoLeft.writeMicroseconds(1300);
}

void turnRight(){
  servoRight.writeMicroseconds(1700);
  servoLeft.writeMicroseconds(1500);
}

void goForwards(){
  servoRight.writeMicroseconds(1700);  
  servoLeft.writeMicroseconds(1300); 
}

void goStop(){
  servoRight.writeMicroseconds(1500);  
  servoLeft.writeMicroseconds(1500); 
}

void goBackwards(){
  servoRight.writeMicroseconds(1300);  
  servoLeft.writeMicroseconds(1700); 
}


void goLeft(){
  servoRight.writeMicroseconds(1300);  
  servoLeft.writeMicroseconds(1300); 
}

void goRight(){
  servoRight.writeMicroseconds(1700);  
  servoLeft.writeMicroseconds(1700); 
}


void loop() {
  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  unsigned int iS = sonar1.ping();//leftsonar
  unsigned int kS = sonar2.ping();//rightsonar
  
  
  Serial.print("Ping: ");
  Serial.print(iS); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
 
 

//if(iS / US_ROUNDTRIP_CM > kS/ US_ROUNDTRIP_CM){
 // turnRight();
//}
 //else{
 // turnLeft();
// }



 if (uS / US_ROUNDTRIP_CM < 30){
    goStop();
    delay(100);
    if (iS / US_ROUNDTRIP_CM < 30){
      turnRight();
      delay(1400);
    }
    else{
      turnLeft();
      delay(1400);      
    }
 }
 else{
    goForwards();
    delay(100);
 }

 //if( uS / US_ROUNDTRIP_CM == 10){
   //turnLeft();
   //goForwards()
  //}
  
  //else if (iS/US_ROUNDTRIP_CM > 0){
    //();  
  //}
  
  //else {
    //goRight();
  //}
}
