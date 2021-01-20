#include <Servo.h>  //Libraries for Servo motor and ultrasonic sensor
#include <NewPing.h>  

#define speedPinR 9 // motor Right Enable + 2 direction Pin
#define RightMotorDirPin1 11
#define RightMotorDirPin2 12
#define speedPinL 6 // motor Left Enable + 2 direction Pin
#define LeftMotorDirPin1 5
#define LeftMotorDirPin2 4

#define SERVO_PIN 10

#define Echo_PIN 8  //ultrasonic sensors pin
#define Trig_PIN 13

#define SPEED 255
#define max_dist 200 //maximum distance for ultrasonic sensor, if distance >200 return 0

#define IRSensorL 3 //IR left sensor
#define IRSensorR 2 //IR right sensor

int cm;
bool turning;
int counter;


NewPing sonar(Trig_PIN, Echo_PIN, max_dist);
Servo servo;

 void followLine(){
    //Reading sensors, if HIGH (BLACK Line) or LOW (No Line), if black line on Sensor Right, we go Right and contrary
    int valueLeft= digitalRead(IRSensorL);
    int valueRight = digitalRead(IRSensorR); 

    if(valueLeft == HIGH && valueRight == LOW){     
      go_Left();      
  } else if (valueLeft == LOW &&  valueRight == HIGH){
      go_Right();      
  } else{
      go_Advance();
  }
    }

void set_Motorspeed(){  //set_Motorspeed(int value, int value) to change the speed of motors
  analogWrite(speedPinL, 255);
  analogWrite(speedPinR, 255);
  }

void go_Advance(){
    digitalWrite(RightMotorDirPin1, LOW);
    digitalWrite(RightMotorDirPin2, HIGH);
    digitalWrite(LeftMotorDirPin1, LOW);
    digitalWrite(LeftMotorDirPin2, HIGH);
    set_Motorspeed();
  }

 void go_Back(){
    digitalWrite(RightMotorDirPin1, HIGH);
    digitalWrite(RightMotorDirPin2, LOW);
    digitalWrite(LeftMotorDirPin1, HIGH);
    digitalWrite(LeftMotorDirPin2, LOW);
    set_Motorspeed();
  }

  void go_Right(){  //one motor forward, second motor backward
    digitalWrite(RightMotorDirPin1, LOW);
    digitalWrite(RightMotorDirPin2, HIGH);
    digitalWrite(LeftMotorDirPin1, HIGH);
    digitalWrite(LeftMotorDirPin2, LOW);
    set_Motorspeed();
  }

   void go_Left(){ //one motor forward, second motor backward
    digitalWrite(RightMotorDirPin1, HIGH);
    digitalWrite(RightMotorDirPin2, LOW);
    digitalWrite(LeftMotorDirPin1, LOW);
    digitalWrite(LeftMotorDirPin2, HIGH);
    set_Motorspeed();
  }
  
   void stop_Stop(){
    digitalWrite(RightMotorDirPin1, LOW);
    digitalWrite(RightMotorDirPin2, LOW);
    digitalWrite(LeftMotorDirPin1, LOW);
    digitalWrite(LeftMotorDirPin2, LOW);
    set_Motorspeed();
  }

int readDistance(){
  int cm = sonar.ping_cm();
  return cm;
  }

void setup() {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  pinMode(IRSensorL, INPUT);
  pinMode(IRSensorR, INPUT);
  
  servo.attach(SERVO_PIN);
  servo.write(103); //Our servo motor works bad, 103 = 90 degree
  delay(200);
  Serial.begin(9600);
  stop_Stop();
  counter = 0;
}

void loop() {
  
      int counter2 = 0;
      cm = readDistance();  //we read the distance continuosly, and increase the counter to avoid false values
      
      if(cm<=5 && cm != 0){
      counter++;
      }
      
     if(counter >= 10){ //when the counter reach the target we start the avoiding step
      counter=0; //reset the counter
      stop_Stop();
      turning = true;
      
      go_Left();
      delay(380); //tune the delay in order to have 90° turning
      stop_Stop();
      delay(500);
      servo.write(0); //turn the servo on the right to look at the obstacle
      delay(500);

      while(turning){
        delay(100);
        cm = readDistance(); //while turning we check if the obstacle is still ahead by reading the distance
        if (cm < 20 && cm != 0){ //if we detect the obstacle we go ahead
          go_Advance();
          delay(50);
          stop_Stop();
         }      
          else {
              counter2++; //if we do not detect the obstacle we increase a counter to be sure that there is not anymore an obstacle
            }
        if(counter2 > 10){ // when counter reaches the target, we go forward to let also the body of the car pass the obstacle
          go_Advance();
          delay(500);
          stop_Stop();
          turning = false;
          }
      }

      go_Right(); //then we go right, and we repeat the operation
      delay(310);
      go_Advance();
      delay(500);
      stop_Stop();
      delay(500);
      turning = true;
      counter2 = 0;
     
        while(turning){
        delay(100);
        cm = readDistance();
         if (cm < 20 && cm != 0){
          go_Advance();
          delay(50);
          stop_Stop();
          }      
          else {
              counter2++;
            }
            if(counter2 > 10){
              go_Advance();
              delay(500);
              stop_Stop();
              turning = false;
              }
      }
      go_Right();
      delay(305);
      stop_Stop();
      delay(500);
      turning = true;
      
      while(turning){ // on the last turn, we continue to go forward until we see the line, then we come back to follow the line
         int valueLeft= digitalRead(IRSensorL);
         int valueRight = digitalRead(IRSensorR); 
         if(valueLeft == LOW || valueRight == LOW){
            go_Advance();
            delay(50);
            stop_Stop();
          } else{ // line found
            turning = false;
            go_Left();
            delay(300);
            stop_Stop();  
            servo.write(103);
            }
              }     
    } else {
      followLine();
      }
}
