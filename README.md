/*
  Firmware for the "2WD Ultrasonic Motor Robot Car Kit"

  Stephen A. Edwards

  Hardware configuration :
  A pair of DC motors driven by an L298N H bridge motor driver
  An HC−SR04 ultrasonic range sensor mounted atop a small hobby servo */
#include <Servo.h>
Servo servo;

// Ultrasonic Module pins
const int trigPin = 13; // 10 microsecond high pulse causes chirp , wait 50 us
const int echoPin = 12; // Width of high pulse indicates distance

// Servo motor that aims ultrasonic sensor .
const int servoPin = 11; // PWM output for hobby servo

// Motor control pins : L298N H bridge
const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 5; // Left motor Direction 2
const int in3Pin = 4; // Right motor Direction 1
const int in4Pin = 2; // Right motor Direction 2
const int enBPin = 3; // Right motor PWM speed control

// Set motor speed: 255 full ahead, −255 full reverse , 0 stop void go( enum Motor m, int speed)
enum Motor { LEFT, RIGHT };
{
  digitalWrite (m == LEFT ? in1Pin : in3Pin , speed > 0 ? HIGH : LOW );
  digitalWrite (m == LEFT ? in2Pin : in4Pin , speed <= 0 ? HIGH : LOW );
  analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? −speed : speed );
}

// Initial motor test :
// left motor forward then back // right motor forward then back
void testMotors()
{
  static int speed[8] = {128, 255, 128, 0, −128, −255, −128, 0};
  go(RIGHT, 0);
  for (unsigned char i = 0 ; i < 8 ; i++)
    go(LEFT, speed[i ]), delay (200);
  for (unsigned char i = 0 ; i < 8 ; i++)
    go(RIGHT, speed[i ]), delay (200);
}

// Read distance from the ultrasonic sensor , return distance in mm //
// Speed of sound in dry air , 20C is 343 m/s
// pulseIn returns time in microseconds (10ˆ−6)
// 2d=p*10ˆ−6s*343m/s=p*0.00343m=p*0.343mm/us
unsignedint readDistance()
{
  digitalWrite ( trigPin , HIGH );
  delayMicroseconds (10);
  digitalWrite ( trigPin , LOW );
  unsigned long period = pulseIn ( echoPin, HIGH );
  return period * 343 / 2000;
}

#define NUM ANGLES 7
unsigned char sensorAngle[NUM ANGLES] = { 60, 70, 80, 90, 100, 110, 120 };
unsigned int distance [NUM ANGLES];
// Scan the area ahead by sweeping the ultrasonic sensor left and right
// and recording the distance observed . This takes a reading , then
// sends the servo to the next angle . Call repeatedly once every 50 ms or so .
void readNextDistance()
{
  static unsigned char angleIndex = 0;
  static signed char step = 1;
  distance [angleIndex] = readDistance ();
  angleIndex += step ;
  if (angleIndex == NUM ANGLES − 1) step = −1;
  else if (angleIndex == 0) step = 1;
  servo . write ( sensorAngle[angleIndex] );
}

// Initial configuration
//
// Configure the input and output pins
// Center the servo
// Turn off the motors
// Test the motors
// Scan the surroundings once
//
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite ( trigPin , LOW);
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  servo . attach ( servoPin );
  servo . write (90);
  go(LEFT, 0);
  go(RIGHT, 0);
  testMotors ();
  // Scan the surroundings before starting
  servo . write ( sensorAngle[0] );
  delay (200);
  for (unsignedchar i = 0 ; i < NUM ANGLES; i++)
    readNextDistance (), delay (200);
}

// Mainloop:
//
// Get the next sensor reading
// If anything appears to be too close , back up
// Otherwise, go forward
//
void loop() {
  readNextDistance ();
  // See if something is too close at any angle
  unsigned char tooClose = 0;
  for (unsigned char i = 0 ; i < NUM ANGLES ; i++)
    if ( distance [i] < 300)
      tooClose = 1;

  if ( tooClose ) {
    // Something's nearby: back up left
    go(LEFT, −180);
    go(RIGHT, −80);
  }
  else {
    // Nothing in our way: go forward
    go(LEFT, 255);
    go(RIGHT, 255);
  }
  // Check the next direction in 50 ms
  delay (50);
}

/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
//Define L298N Dual H-Bridge Motor Controller Pins
#define speedPinR 3   //  RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  4  //  Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2  2   // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6     //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  7   // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  5   //Left Motor direction pin 1 to MODEL-X IN4

/*From left to right, connect to D3,A1-A5 ,D10*/
#define LFSensor_1 A1
#define LFSensor_3 A3
#define LFSensor_5 A5


#define SPEED   180 //motor in   speed

 char sensor[5];
 /*read sensor value string, 1 stands for black, 0 starnds for white, i.e 010 means the first sensor(from left) detect white line, middle sensor detect black and right sensor detect white*/
 
String read_sensor_values()
{   int sensorvalue=32;
  sensor[1]= digitalRead(LFSensor_1);
 
  sensor[3]=digitalRead(LFSensor_3);
 
  sensor[5]=digitalRead(LFSensor_5);

  sensorvalue +=sensor[1]*16+sensor[3]*4+sensor[5];
 
  String senstr= String(sensorvalue,BIN);
  Serial.println(senstr);
  return senstr.substring(1,6);
}

void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Left(void)  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
}
void go_Right(void)  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}

void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
}
/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

void setup()
{
 pinMode(speedPinL,OUTPUT); //left motor PWM pin
 pinMode(speedPinR,OUTPUT); //rignt motor PWM  pin
 pinMode(RightDirectPin1,OUTPUT); //left motor direction pin1
 pinMode(RightDirectPin2,OUTPUT); //left motor direction pin2
 pinMode(LeftDirectPin1,OUTPUT); //right motor direction Pin 1
 pinMode(LeftDirectPin2,OUTPUT);  //right motor direction Pin 2

  /*line follow sensors */
 pinMode(LFSensor_1,INPUT);
 pinMode(LFSensor_3,INPUT);
 pinMode(LFSensor_5,INPUT);
 Serial.begin(9600);
}

void auto_tracking(){
 String sensorval= read_sensor_values();
 //Serial.println(sensorval);
 if (sensorval=="000" or sensorval=="001" or sensorval=="011")
 { 
  //The black line is in the right of the car, need right turn 
      go_Right();  //Turn right
    set_Motorspeed(0,SPEED);
    delay(200);
    stop_Stop();
    }

 if (sensorval=="100" or sensorval=="110"){ //The black line is on the left of the car, need  left turn 
  
     go_Left();  //Turn left
       set_Motorspeed(SPEED,0);
           delay(200);
    stop_Stop();
    }
 
 if (sensorval=="111") {
     stop_Stop();   //The car front touch stop line, need stop
     set_Motorspeed(0,0);
        stop_Stop();
    }
    
 if (sensorval=="010) {
    go_Advance(); //Continue forward
    set_Motospeed(100,100);
    delay(100);
    }
    
}

void loop(){
 
auto_tracking();
}

