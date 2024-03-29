#define enA 10//Enable1 L298 Pin enA 
#define in1 9 //Motor1  L298 Pin in1 
#define in2 8 //Motor1  L298 Pin in1 
#define in3 7 //Motor2  L298 Pin in1 
#define in4 6 //Motor2  L298 Pin in1 
#define enB 5 //Enable2 L298 Pin enB 
#define L_S A0 //ir sensor Left
#define R_S A1 //ir sensor Right
#define echo A2    //Echo pin
#define trigger A3 //Trigger pin
#define servo A5
#define rxPin  12
#define txPin  13
#include <SoftwareSerial.h>
#include <Servo.h>

// Set up a new SoftwareSerial object
SoftwareSerial myserial (rxPin, txPin);
myservo.attach(11);
int count=3;

int Set=15;
int distance_L, distance_F, distance_R; 
void setup(){ // put your setup code here, to run once
Serial.begin(9600); // start serial communication at 9600bps
pinMode(R_S, INPUT); // declare if sensor as input  
pinMode(L_S, INPUT); // declare ir sensor as input
pinMode(echo, INPUT );// declare ultrasonic sensor Echo pin as input
pinMode(trigger, OUTPUT); // declare ultrasonic sensor Trigger pin as Output  
pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 
analogWrite(enA, 200); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
analogWrite(enB, 200); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed 
pinMode(servo, OUTPUT);
pinMode(rxPin, INPUT);
pinMode(txPin, OUTPUT);
myserial.begin(9600);
myservo.write(90);
delay(100);
distance_F = Ultrasonic_read();
Serial.println(distance_F);
delay(500);
}
void loop(){  
//==============================================
//     Line Follower and Obstacle Avoiding
//==============================================  
char plant_id='';
if(myserial.available()){
 plant_id=myserial.read();
 
}
//_______________________to plant one__________________________________________________________________________
if(plant_id =='1'){ 
distance_F = Ultrasonic_read();
Serial.print("D F=");Serial.println(distance_F);
//if Right Sensor and Left Sensor are at White color then it will call forword function
 if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
  if(distance_F > Set){forword();}
                  else{Check_side();}  
 }  
 
//if Right Sensor is Black and Left Sensor is White then it will call turn Right function
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();}  
//if Right Sensor is White and Left Sensor is Black then it will call turn Left function
else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();} 
//IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){turnLeft();} 
 //if t joint arises
 else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==3){turnLeft();count=2;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==2){turnRight();turnRight();count=1;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==1){turnRight();count=0;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==0){stop();} 
    
delay(10);
    
delay(10);
}
//_______________________to plant two__________________________________________________________________________
if(plant_id =='2'){ 
distance_F = Ultrasonic_read();
Serial.print("D F=");Serial.println(distance_F);
//if Right Sensor and Left Sensor are at White color then it will call forword function
 if(((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0))||((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1))){
  if(distance_F > Set){forword();}
                  else{Check_side();}  
 }  
 
//if Right Sensor is Black and Left Sensor is White then it will call turn Right function
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();}  
//if Right Sensor is White and Left Sensor is Black then it will call turn Left function
else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();}
 //if t joint
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==3){forward();count=2;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==2){turnRight();turnRight();count=1;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==1){forward();count=0;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==0){stop();} 
    
delay(10);
    
delay(10);
}}
//_______________________to plant three__________________________________________________________________________
if(plant_id =='3'){ 
distance_F = Ultrasonic_read();
Serial.print("D F=");Serial.println(distance_F);
//if Right Sensor and Left Sensor are at White color then it will call forword function
 if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
  if(distance_F > Set){forword();}
                  else{Check_side();}  
 }  
 
//if Right Sensor is Black and Left Sensor is White then it will call turn Right function
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();}  
//if Right Sensor is White and Left Sensor is Black then it will call turn Left function
else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();} 
//IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==3){turnRight();count=2;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==2){turnRight();turnRight();count=1;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==1){turnLeft();count=0;} 
 //IF T joint arises
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1) && count==0){stop();} 
    
delay(10);
}}
//**********************Ultrasonic_read****************************
long Ultrasonic_read(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (echo, HIGH);
  return time / 29 / 2;
}
void compareDistance(){
    if(distance_L > distance_R){
  turnLeft();
  delay(500);
  forword();
  delay(600);
  turnRight();
  delay(500);
  forword();
  delay(600);
  turnRight();
  delay(400);
  }
  else{
  turnRight();
  delay(500);
  forword();
  delay(600);
  turnLeft();
  delay(500);
  forword();
  delay(600);  
  turnLeft();
  delay(400);
  }
}
void Check_side(){
    Stop();
    delay(100);
 myservo.write(30)
  delay(500)
    distance_R = Ultrasonic_read();
    Serial.print("D R=");Serial.println(distance_R);
    delay(100);
  myservo.write(150);
    delay(500);
    distance_L = Ultrasonic_read();
    Serial.print("D L=");Serial.println(distance_L);
    delay(100);
  myservo.write(90);
    delay(300);
    compareDistance();
}
void forword(){  //forword
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, HIGH); //Left Motor forword Pin 
digitalWrite(in3, HIGH); //Right Motor forword Pin 
digitalWrite(in4, LOW); //Right Motor backword Pin 
}
void backword(){ //backword
digitalWrite(in1, HIGH); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, HIGH); //Right Motor backword Pin 
}
void turnRight(){ //turnRight
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, HIGH); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, HIGH); //Right Motor backword Pin 
}
void turnLeft(){ //turnLeft
digitalWrite(in1, HIGH); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, HIGH); //Right Motor forword Pin 
digitalWrite(in4, LOW); //Right Motor backword Pin 
}
void Stop(){ //stop
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, LOW); //Right Motor backword Pin 
}




