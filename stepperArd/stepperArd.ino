#include <AccelStepper.h>
#include <MultiStepper.h>

#include <Wire.h>
#include "apintI2C.h"

#define XSTEP   8
#define XDIR    9
#define YSTEP   10
#define YDIR    11 
#define ZSTEP   12
#define ZDIR    13
#define X_LIMIT_MAX_PIN 7
#define X_LIMIT_MIN_PIN 6
#define Y_LIMIT_MAX_PIN 5
#define Y_LIMIT_MIN_PIN 4
#define Z_LIMIT_MAX_PIN 3
#define Z_LIMIT_MIN_PIN 2
#define ASSERT_AVAIL 22
bool initialize;
bool cmdrec=false;
bool orbit = false;
bool xmin = false, xmax= false, zmin= false, zmax= false, ymax= false;
int orbitphase = 0;
int curmillis =0, prevmillis =0;
stepPos curPos;
stepPos moveVec;

int incomingByte;
/*
int xcount=0, ycount=0, zcount=0;
unsigned long curMicros;
unsigned long prevStepMicros = 0;
unsigned long microsBetweenSteps = 550; // microseconds*/
AccelStepper xStepper(AccelStepper::DRIVER, XSTEP, XDIR);
AccelStepper yStepper(AccelStepper::DRIVER, YSTEP, YDIR);
AccelStepper zStepper(AccelStepper::DRIVER, ZSTEP, ZDIR);
byte recBuffer[MAX_LEN];
byte sendBuffer[MAX_LEN];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(receiveCommand);
  Wire.onRequest(requestCommand);
  initialize = true;
  pinMode(X_LIMIT_MAX_PIN, INPUT);
  pinMode(X_LIMIT_MIN_PIN, INPUT);
  pinMode(Y_LIMIT_MAX_PIN, INPUT);
  pinMode(Z_LIMIT_MAX_PIN, INPUT);
  pinMode(Z_LIMIT_MIN_PIN, INPUT);
  xStepper.setMaxSpeed(1000);
  xStepper.setAcceleration(5000);
  yStepper.setMaxSpeed(1000);
  yStepper.setAcceleration(5000);
  zStepper.setMaxSpeed(1000);
  zStepper.setAcceleration(5000);
  
  pinMode(XSTEP, OUTPUT);
  pinMode(YSTEP, OUTPUT);
  pinMode(ZSTEP, OUTPUT);
  pinMode(XDIR, OUTPUT);
  pinMode(YDIR, OUTPUT); 
  pinMode(ZDIR, OUTPUT);
  pinMode(ASSERT_AVAIL, OUTPUT);
  digitalWrite(ASSERT_AVAIL, HIGH);

}

void loop() {
  curmillis = millis();
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
      if(incomingByte=='q'){ 
          yStepper.move(1000);
          Serial.println("Moving y +ve");
      }
      else if(incomingByte=='e'){
          yStepper.move(-1000);
          Serial.println("Moving y -ve");
      }
      else if(incomingByte=='w'){
          zStepper.move(1000);
          Serial.println("Moving z +ve");
      }
      else if(incomingByte=='s'){
          zStepper.move(-1000);
          Serial.println("Moving z -ve");
      }
      else if(incomingByte=='a'){
          xStepper.move(1000);
          Serial.println("Moving x +ve");
      }
      else if(incomingByte=='d'){
          xStepper.move(-1000);
          Serial.println("Moving -ve");
      }
      
  }
  if(curmillis-prevmillis > 200){
    prevmillis=curmillis;
    
   if(digitalRead(X_LIMIT_MAX_PIN)){
      xmax =true;
    }
    else if(digitalRead(X_LIMIT_MIN_PIN)){
      xmin =true;
    }
    if(digitalRead(Y_LIMIT_MAX_PIN)){
      ymax = true;
    }
  
    if(digitalRead(Z_LIMIT_MAX_PIN)){
      zmax = true;
    }
    else if(digitalRead(Z_LIMIT_MIN_PIN)){
      zmin = true;
    }
  }
  if(initialize){
    initialize = false;
  }
  
  else if(orbit){
    if(orbitphase==0){
      if(xmax){
        orbitphase=1;
        xStepper.stop();
      }
      else{
        xStepper.move(500);
      }
    }
    else if(orbitphase==1){
      if(zmax){
        orbitphase=2;
        zStepper.stop();
      }
      else{
        zStepper.move(500);
      }
    }
    else if(orbitphase==2){
      if(xmin){
        orbitphase=3;
        xStepper.stop();
      }
      else{
        xStepper.move(-500);
      }
    }
    else if(orbitphase==3){
      if(zmin){
        orbitphase=0;
        zStepper.stop();  
      }
      else{
        zStepper.move(-500);
      }
    }
  }
  else if(cmdrec){
    cmdrec=false;
    if(recBuffer[0]== STEP_MOVE){
      moveVec.x = (int)(recBuffer[1]<<8|recBuffer[2]);
      moveVec.y = recBuffer[3]<<8|recBuffer[4];
      moveVec.z = recBuffer[5]<<8|recBuffer[6];
      if(moveVec.x){
        xStepper.stop();
        int movex = (moveVec.x-190)*30;
        xStepper.move(movex);
        Serial.print("target x: ");
        Serial.println(movex);
      }

      
      /*if(moveVec.y){
        int movey = moveVec.y*100;
        yStepper.move(movey);
        Serial.print("target y: ");
        Serial.println(movey);
      }*/

      if(moveVec.z){
        int movez = (moveVec.z-135)*-10;
        zStepper.move(movez);
        Serial.print("target z: ");
        Serial.println(movez);
      }

    }
    else if(recBuffer[0]== STEP_STOP){
      xStepper.stop();
      yStepper.stop();
      zStepper.stop();
    }
    else if(recBuffer[0]== STEP_CLIMB){
      Serial.println("climbing");
      yStepper.move(recBuffer[3]<<8|recBuffer[4]);
    }
    else if(recBuffer[0]== STEP_DESC){
      Serial.println("DESC");
      yStepper.move(-1000);
    }
  }
  if(xStepper.isRunning()||yStepper.isRunning()||zStepper.isRunning()){
    digitalWrite(ASSERT_AVAIL, LOW);
    Serial.println("running");
  }
  else{
    digitalWrite(ASSERT_AVAIL, HIGH);
    Serial.println("ready");
  }
  
  if(xmax){
    xmax= false;
    xStepper.stop();
    xStepper.move(-100);
  }
  else if(xmin){
    xmin = false;
    xStepper.stop();
    xStepper.move(100);
    curPos.x = 0;
  }
  if(ymax){
    ymax =false;
    yStepper.stop();
    yStepper.move(-100);
  }/*
  else if(digitalRead(Y_LIMIT_MIN_PIN)){
    yStepper.stop();
    yStepper.move(100);
    curPos.y = 0;
  }*/
  if(zmax){
    zmax=false;
    zStepper.stop();
    zStepper.move(-40);
  }
  else if(zmin){
    zmin = false;
    zStepper.stop();
    zStepper.move(40);
    curPos.z = 0;
  }
  xStepper.run();
  yStepper.run();
  zStepper.run();
}


void receiveCommand(int nbytes){
  Serial.println("command received");
  while(Wire.available()){
    Wire.readBytes(recBuffer, nbytes);
  }
  digitalWrite(ASSERT_AVAIL, LOW);
  if(recBuffer[0]==STEP_ORBIT){
    orbit = true;
  }
  else{
    orbit = false;
    cmdrec= true;
  }
  
}

void requestCommand(){
  Serial.println("req received");
  sendBuffer[0]= STEP_STATUS;
  sendBuffer[1]= highByte(curPos.x);
  sendBuffer[2]= lowByte(curPos.x);
  sendBuffer[3]= highByte(curPos.y);
  sendBuffer[4]= lowByte(curPos.y);
  sendBuffer[5]= highByte(curPos.z);
  sendBuffer[6]= lowByte(curPos.z);
  Wire.write(sendBuffer, MAX_LEN);
}
