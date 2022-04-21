 #include <Servo.h>

#include <SPI.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <Wire.h>
#include "apintI2C.h"

#include <AccelStepper.h>
#include <MultiStepper.h>

#define FSRLEFT 0 
#define FSRRIGHT 1
int fsrLeft,fsrRight;

#define STEP 6
#define DIR 7
AccelStepper rotStepper(AccelStepper::DRIVER, STEP, DIR);
int rotDirection;
int ogDir;

#define LEFTHOOK 9
#define RIGHTHOOK 10
Servo lefthook; 
Servo righthook;

enum state{INIT, ORBIT, TRACK, ALIGN, PICKUP, CLIMB, DROP, STANDBY, RTB};
state curstate = INIT;

Pixy2 pixy;
bool enPixy = true;
#define TARGET_CODE 10  

#define STEPPER_AVAIL 22
char UIbuffer[8]={};
char StepBuffer[MAX_LEN];

int curmillis, prevmillis=0;
int verStep = 1000;
stepPos curPos;
struct relpos{
  bool detected=false;
  int x;
  int y;
  int angle;
} lastknown;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //start i2c
  Serial.begin(9600); //debug out
  pixy.init();

  //Stepper ready line
  pinMode(STEPPER_AVAIL, INPUT);
  //DC motor and encoder
  
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  rotStepper.setMaxSpeed(100);
  rotStepper.setAcceleration(50);
  rotDirection=rotStepper.currentPosition();
  ogDir = rotDirection; 
  lefthook.attach(LEFTHOOK);
  righthook.attach(RIGHTHOOK);
  lefthook.write(70);
  righthook.write(70);
}

void loop() {
  curmillis=millis();
  if(rotStepper.isRunning()){
    enPixy=false;
 
  }
  else{
    enPixy=true;
  }
  if(enPixy){
   pixy.ccc.getBlocks(1);
   if (pixy.ccc.numBlocks)
    {
      
      //Serial.print("Detected ");
      //Serial.println(pixy.ccc.numBlocks);
      
      for (int i=0; i<pixy.ccc.numBlocks; i++)
      {
        if(pixy.ccc.blocks[i].m_signature==TARGET_CODE){
          pixy.ccc.blocks[i].print();
          lastknown.x=pixy.ccc.blocks[i].m_x;
          lastknown.y=pixy.ccc.blocks[i].m_y;
          lastknown.angle=pixy.ccc.blocks[i].m_angle;
          lastknown.detected = true;
          break;
        }
      }
    }
  }
  
  if(curstate==INIT){
    sendToStepper(STEP_ORBIT, 0, 0, 0);
    curstate = ORBIT;
  }
  else if(curstate == STANDBY){
    
  }
  else if(curstate == ORBIT){
    if(lastknown.detected){
      curstate = TRACK;
      sendToStepper(STEP_STOP, 0, 0, 0);
    }

  }
  else if(curstate == TRACK){
    Serial.println("track");
    if(curmillis-prevmillis >=100){
      if(lastknown.x>185&&lastknown.x<195&&lastknown.y>130&&lastknown.y<140){
        curstate = ALIGN;
      }
      if(digitalRead(STEPPER_AVAIL)==HIGH){
        if(lastknown.detected){ 
          Serial.println("sending");
          StepBuffer[0]=STEP_MOVE;
          StepBuffer[1]=highByte(lastknown.x);
          StepBuffer[2]=lowByte(lastknown.x);
          StepBuffer[3]=0;
          StepBuffer[4]=0;
          StepBuffer[5]=highByte(lastknown.y);
          StepBuffer[6]=lowByte(lastknown.y);
          
          prevmillis = curmillis;
          Wire.beginTransmission(STEPPER_ADDR);
          Wire.write(StepBuffer, 7);
          Wire.endTransmission();
        
        }
      }
      else{
        Serial.println("Stepper busy");
      
      }
    }
  }
  else if(curstate == ALIGN){
   Serial.println("ALIGN");
   //enPixy =false;
   if(lastknown.detected){ 
     if(!rotStepper.isRunning()){
      if((lastknown.angle>5)||(lastknown.angle<-5)){
        Serial.print("Angle: ");
        Serial.println(lastknown.angle);
        
         rotDirection=(int)(lastknown.angle/1.8);
         Serial.println(rotDirection);
         rotStepper.move(rotDirection);
         prevmillis = curmillis;
         curstate=PICKUP;
  
      }
  
      else{
        curstate=PICKUP;
        //rotStepper.stop();
        Serial.println("aligned");
        //enPixy =false;
      }
     }
   }
  }
  else if(curstate==PICKUP){
    enPixy =false;
       fsrLeft = analogRead(FSRLEFT);  
       fsrRight =analogRead(FSRRIGHT); 
       Serial.print(fsrLeft);
       Serial.print("//");
       Serial.println(fsrRight);
       if(fsrLeft>240 && fsrRight>160){
          sendToStepper(STEP_STOP, 0, 0, 0);
          Serial.println("hook");
          lefthook.write(150);
          righthook.write(150);
          curstate = CLIMB;
       }
       else if(digitalRead(STEPPER_AVAIL)==HIGH){
          sendToStepper(STEP_DESC, 0, 0, 0);
        }
  }
  else if(curstate==CLIMB){
     sendToStepper(STEP_CLIMB, 0, 5000, 0);
     Serial.println("Sent Climb"); 
     curstate = RTB;
     prevmillis = curmillis;
    
  }
  else if(curstate == RTB){
    if(digitalRead(STEPPER_AVAIL)==HIGH){
      if(curmillis-prevmillis>3000){
       curstate = DROP;
       prevmillis = curmillis;
       rotStepper.move(-rotDirection);
      }
    }
    
  }
  else if(curstate == DROP){
       fsrLeft = analogRead(FSRLEFT);  
       fsrRight =analogRead(FSRRIGHT); 
       //FSR threshold
       Serial.print(fsrLeft);
       Serial.print("//");
       Serial.println(fsrRight);

      //sendToStepper(STEP_STOP, 0, 0, 0);
      if(fsrLeft>250 && fsrRight>160){
          sendToStepper(STEP_STOP, 0, 0, 0); 
          Serial.println("unhook");
          lefthook.write(70);
          righthook.write(70);
          sendToStepper(STEP_CLIMB, 0, 10000, 0);
          curstate = STANDBY;
       }
       else if(digitalRead(STEPPER_AVAIL)==HIGH){
          sendToStepper(STEP_DESC, 0, 0, 0);
          Serial.println("Sent Descend");

      }
    
  }
 
  rotStepper.run();
  
  
}

void sendToStepper(byte cmd, int x, int y, int z){
  Serial.println("sending");
  StepBuffer[0]=cmd;
  StepBuffer[1]=highByte(x);
  StepBuffer[2]=lowByte(x);
  StepBuffer[3]=highByte(y);
  StepBuffer[4]=lowByte(y);
  StepBuffer[5]=highByte(z);
  StepBuffer[6]=lowByte(z); 
  
  //prevmillis = curmillis;
  Wire.beginTransmission(STEPPER_ADDR);
  Wire.write(StepBuffer, 7);
  Wire.endTransmission();
}
