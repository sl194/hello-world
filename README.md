/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.Define L298N Dual H-Bridge Motor Controller Pins*/
#define speedPinR 3 // RIGHT PWM pin connect MODEL-X ENA 
#define RightDirectPin1 4 // Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2 2 // Right Motor direction pin 2 to MODEL-X IN2 
#define speedPinL 6 // Left PWM pin connect MODEL-X ENB 
#define LeftDirectPin1 7 // Left Motor direction pin 1 to MODEL-X IN3 
#define LeftDirectPin2 5 //Left Motor direction pin 1 to MODEL-X IN4

/*From left to right, connect to D3,A1-A5 ,D10*/
#define LFSensor_2 A2 
#define LFSensor_3 A3 
#define LFSensor_4 A4

#define SPEED 255 //motor in speed

char sensor[3]= {digitalRead(LFSensor_2),digitalRead(LFSensor_3),digitalRead(LFSensor_4)};
/*read (sensor value string, 1 stands for black, 0 stands for white) i.e.010 means the first sensor(from left) detect white line, middle sensor detect black and right sensor detect white)*/


void go_Advance(void) //Forward 
{ 
  digitalWrite(RightDirectPin1, HIGH); 
  digitalWrite(RightDirectPin2,LOW); 
  digitalWrite(LeftDirectPin1,HIGH); 
  digitalWrite(LeftDirectPin2,LOW); } 
  
void go_Left(void) //Turn left 
{ 
  digitalWrite(RightDirectPin1, HIGH); 
  digitalWrite(RightDirectPin2,LOW); 
  digitalWrite(LeftDirectPin1,LOW); 
  digitalWrite(LeftDirectPin2,HIGH); 
  } 

void go_Right(void) //Turn right 
{ 
  digitalWrite(RightDirectPin1, LOW); 
  digitalWrite(RightDirectPin2,HIGH); 
  digitalWrite(LeftDirectPin1,HIGH); 
  digitalWrite(LeftDirectPin2,LOW); 
  }


void stop_Stop() //Stop 
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
  pinMode(speedPinR,OUTPUT); //rignt motor PWM pin 
  pinMode(RightDirectPin1,OUTPUT); //left motor direction pin1 
  pinMode(RightDirectPin2,OUTPUT); //left motor direction pin2 
  pinMode(LeftDirectPin1,OUTPUT); //right motor direction Pin 1 
  pinMode(LeftDirectPin2,OUTPUT); //right motor direction Pin 2


/*line follow sensors */ 
pinMode(LFSensor_2,INPUT); 
pinMode(LFSensor_3,INPUT); 
pinMode(LFSensor_4,INPUT); 
Serial.begin(9600); }

void auto_tracking()
 
{
  if ((sensor[0]==0 and sensor[1]==0 and sensor[2]==0) or (sensor[0]==0 and sensor[1]==0 and sensor[2]==1) or (sensor[0]==0 and sensor[1]==1 and sensor[2]==1))
{ 
  //The black line is in the right of the car, need right turn 
  go_Right(); //Turn right 
  set_Motorspeed(0,SPEED); 
  delay(200); 
  stop_Stop(); 
}

if ((sensor[0]==1 and sensor[1]==0 and sensor[2]==0) or (sensor[0]==1 and sensor[1]==1 and sensor[2]==0))
{ 
  //The black line is on the left of the car, need left turn
 go_Left();  //Turn left
   set_Motorspeed(SPEED,0);
   delay(200);
stop_Stop();
}

if (sensor[0]==1 and sensor[1]==1 and sensor[2]==1) 
{ 
  stop_Stop(); 
 //The car front touch stop line, need stop 
set_Motorspeed(0,0); 
stop_Stop(); 
}

if (sensor[0]==0 and sensor[1]==1 and sensor[2]==0) 
{ 
  go_Advance(); //Continue forward 
set_Motorspeed(100,100); 
delay(100); 
}

}

void loop () 
{ auto_tracking();
}
