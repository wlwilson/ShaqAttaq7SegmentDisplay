#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_7segment matrix1 = Adafruit_7segment();
Adafruit_7segment matrix2 = Adafruit_7segment();
Adafruit_7segment matrix3 = Adafruit_7segment();
Adafruit_7segment matrix4 = Adafruit_7segment();

int DX0pin = 2;//VALUE LSB
int DX1pin = 3;
int DX2pin = 4;
int DX3pin = 5;//VALUE MSB
int DX4pin = 6;//LOC LSB
int DX5pin = 7;
int DX6pin = 8;
int DX7pin = 9;//LOC MSB

int AX4pin = 11; //latch when LOW 
int MRBarPin = 12; //blank

int val = 0;
int digit = 0;
int clk = 0;
int lastclk = 0;
int blank = 1;
int DX0 = 0;
int DX1 = 0;
int DX2 = 0;
int DX3 = 0;
int DX4 = 0;
int DX5 = 0;
int DX6 = 0;
int DX7 = 0;
int PDX[8] = {0,0,0,0,0,0,0,0};
int DX[8] = {0,0,0,0,0,0,0,0};
int MR = 0;
boolean dots = false;
int counter=0;
int D[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char * outSt = "";
void setup() {
  
  pinMode(DX0pin, INPUT);
  pinMode(DX1pin, INPUT);
  pinMode(DX2pin, INPUT);
  pinMode(DX3pin, INPUT);
  pinMode(DX4pin, INPUT);
  pinMode(DX5pin, INPUT);
  pinMode(DX6pin, INPUT);
  pinMode(DX7pin, INPUT);
  pinMode(AX4pin, INPUT);
  pinMode(MRBarPin, INPUT);

  matrix1.begin(0x70);
  matrix2.begin(0x71);
  matrix3.begin(0x72);
  matrix4.begin(0x73);
}

void loop() {
  // if AX4pin = HIGH, latch DX0-DX7
  clk = digitalRead(AX4pin);
  //if(clk != lastclk){
    if(clk==HIGH){
        DX0 = digitalRead(DX0pin);
        DX1 = digitalRead(DX1pin);
        DX2 = digitalRead(DX2pin);
        DX3 = digitalRead(DX3pin);
        DX4 = digitalRead(DX4pin);
        DX5 = digitalRead(DX5pin);
        DX6 = digitalRead(DX6pin);
        DX7 = digitalRead(DX7pin);
        //MR = digitalRead(MRBarPin);
  
      val = DX3*8 + DX2*4 + DX1*2 + DX0*1;
      digit = DX7*8 + DX6*4 + DX5*2 + DX4*1;
    
      if(val<10){
        D[digit] = val;
      }else{
        D[digit]=17;
      }
      counter=(counter+1)%2;
    }
  //}
  lastclk=clk;
  blank = digitalRead(MRBarPin);
  if(!blank){
    //matrix1.clear();
    //matrix2.clear();
    //matrix3.clear();
    //matrix4.clear();
  }
  else{
    matrix1.writeDigitNum(0,D[0],dots);
    matrix1.writeDigitNum(1,D[1],dots);
    matrix1.writeDigitNum(3,D[2],dots);

    matrix2.writeDigitNum(0,D[3],dots);
    matrix2.writeDigitNum(1,D[4],dots);
    matrix2.writeDigitNum(3,D[5],dots);

    matrix3.writeDigitNum(0,D[6],dots);
    matrix3.writeDigitNum(1,D[7],dots);
    matrix3.writeDigitNum(3,D[8],dots);

    matrix4.writeDigitNum(0,D[9],dots);
    matrix4.writeDigitNum(1,D[10],dots);
    matrix4.writeDigitNum(3,D[11],dots);
  }
  //if(!(D[0]>=10 && D[1]>=10 && D[2]>=10)){// || (D[0]<10 && D[1]<10 && D[2]<10))
  matrix1.writeDisplay();
  //}
  matrix2.writeDisplay();
  matrix3.writeDisplay();
  matrix4.writeDisplay();
  
}
  
