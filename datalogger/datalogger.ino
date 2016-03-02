#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>
#include "Timer.h"
Timer t;
String dataString = "";

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //I2C display at adres 0x27

boolean engineRunning = false;

int analogMagnet;
int MAGNETPIN = 0;
int defaultMagnet;

const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ambientTemperature;

const int SECTORS = 2;  //Sector 0,1,2
const int LAPS = 4;    //laps to keep so they can be displayed.

//Two arrarys who will be used as queues.
unsigned long laptimes[LAPS+2];    //4+2
unsigned long sectortimes[SECTORS+1];
int laptimesFirst = 0;    //location where next value will be written to.
int laptimesLast = 0;     //location of last item in queue.
int laptimesSaved = 0;    //to keep track of wich items are already saved. (= written to SD card)
int sectortimesFirst = 0; //sectortimes get cleared every lap, so a queue is not needed.
int sectortimesSaved = 0;
int currentLap = 0;       //Current sector can be determined from sectortimesLast and First.
int currentSector = 0;
int lapsToDisplay;
unsigned long laptime;
unsigned long sectortime;


unsigned long startTime;    //used in stopwatch(), to keep track of time
unsigned long elapsedTime;
boolean magnetState;
boolean previousMagnetState = false;
boolean stopwatchRunning = false;




void setup(){
  Wire.begin();
  
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);      // PWR_MGMT_1 register
  Wire.write(0);         // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
      
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);      //GYRO_CONFIG register
  Wire.write(0);         //0: 250°/S, 8: 500°/S, 16: 1000°/S, 24: 2000°/S
  Wire.endTransmission(true); 
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);     //ACCEL_CONFIG register
  Wire.write(8);        //0: 2G, 8: 4G, 16: 8G, 24: 16G
  Wire.endTransmission(true);
  
  lcd.begin(20,4);
  lcd.backlight();
  lcd.clear();
  
  int timerId;
  timerId = t.every(1000, updateDisplay);
  
  if(!SD.begin())
    error("SD card failed");
  calibrate(); 
}

void loop(){
    updateTimeTables();     //needs to run as fast as possible because stopwatch() only returns sectortime while driving over a magnet strip.
    updateAccelerometer();
    saveData();
    t.update();    
    
}

void calibrate(){
  //Determine default magnet value, dismiss first values.
  for(int i=0; i<100; i++)
   defaultMagnet = analogRead(MAGNETPIN);
  defaultMagnet = 0;
  for(int i=0; i<50;i++)
    defaultMagnet += analogRead(MAGNETPIN);    
  defaultMagnet = defaultMagnet / 50;  
  
}

//Displays the error message and goes into an infinite delay(), reset needed.
void error(String message){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(message);
  while(1)
    delay(1000);
}

//Saves the current data to the SD card.
void saveData(){
  dataString = "";
  dataString += (String)millis();
  dataString += ",";
  dataString += (String)ambientTemperature;
  dataString += ",";
  dataString += (String)AcX;
  dataString += ",";
  dataString += (String)AcY;
  dataString += ",";
  dataString += (String)AcZ;
  dataString += ",";
  dataString += (String)GyX;
  dataString += ",";
  dataString += (String)GyY;
  dataString += ",";
  dataString += (String)GyZ;
  dataString += ",";
  dataString += (String)currentLap;
  dataString += ",";
  dataString += (String)currentSector;
  dataString += ",";
  
  if(sectortimesSaved != sectortimesFirst){        //There is a cell (sector) that is not yet saved. 
    dataString += (String)sectortimes[sectortimesSaved];
    sectortimesSaved = sectortimesSaved + 1;
  }
  dataString += ",";
  if(laptimesSaved != laptimesFirst){              //Easier than "if the distance between Saved and First > 1". 
    dataString += (String)laptimes[laptimesSaved];
    laptimesSaved = (laptimesSaved+1)%(LAPS+2);
  }

  File dataFile = SD.open("datalog.txt", FILE_WRITE); 
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }
  else {
    error("can't open file");
  }
}



//updates Accleration, Gyro and temperature values over I2C.
void updateAccelerometer(){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
   
    ambientTemperature = Tmp/340.00+34.53; 
}


//Writes current data to the display.
void updateDisplay(){

    lcd.print(" ");    
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(ambientTemperature);
    if(ambientTemperature < 10);
    lcd.print(" ");
    
    lcd.setCursor(6,0);
    lcd.print(" S:");
    lcd.print((currentSector + 1) * stopwatchRunning);
    
    for(int i=0;i<4;i++){
      lcd.setCursor(10,i);
      lcd.print("|");
    }
    
    lapsToDisplay = 0;    //if laptimesFirst == laptimesLast then there are no laps to display, the two following if() clauses will be skipped so lapsToDisplay remains 0.
    if(laptimesFirst > laptimesLast)
      lapsToDisplay = laptimesFirst - laptimesLast;
    if(laptimesFirst < laptimesLast)
      lapsToDisplay = (LAPS + 2) - (laptimesLast - laptimesFirst);
    if(lapsToDisplay > LAPS)
      laptimesLast = (laptimesLast+1)%(LAPS+2);
    
      
    unsigned long timeCopy;
    int minutes, seconds, milliseconds; 
    for(int j=lapsToDisplay; j > 0; j--){ 
      timeCopy = laptimes[(laptimesLast + j - 1)%(LAPS+2)];
      milliseconds = (int) ((timeCopy) % 1000u);
      seconds = (int) ((timeCopy / 1000u) % 60u);
      minutes = (int) ((timeCopy / (1000u*60u)) % 60u);      
      
      lcd.setCursor(11,lapsToDisplay - j);
      if(minutes<10)
        lcd.print(0);
      lcd.print(minutes);
      lcd.print(":");
      if(seconds<10)
        lcd.print(0);
      lcd.print(seconds);
        
      lcd.print(".");
      if(milliseconds<100)
        lcd.print(0);
      if(milliseconds<10)
        lcd.print(0);
      lcd.print(milliseconds);     
    }
}


//returns true if currently passing a magnet strip
boolean passingMagnet(){
    analogMagnet = analogRead(MAGNETPIN);    //No field: 506-510
    if(analogMagnet > (defaultMagnet + 1) or analogMagnet < (defaultMagnet - 1))
      return true;
    else
      return false;
}

//Returns time passed between magnet strips while passing magnet strip, else returns 0
unsigned long stopwatch(){
  magnetState = passingMagnet();
  if(magnetState == false && previousMagnetState == true){
    previousMagnetState = magnetState;
    return 0;
  }
  else if(magnetState == true && previousMagnetState == false && stopwatchRunning == false){
    //Detects rising edge, if true then passed a magnetstrip while clock is not running - start clock
   startTime = millis();
   stopwatchRunning = true;
   previousMagnetState = magnetState;
   return 0;
  }  
  else if(magnetState == true && previousMagnetState == false && stopwatchRunning == true){
    //Detects rising edge, if true then passed a magnetstrip while clock is running - return elapsed time and restart clock
    elapsedTime = millis() - startTime;
    startTime = millis();
    previousMagnetState = magnetState;
    return elapsedTime;
  }

  else if(magnetState == true && previousMagnetState == true && stopwatchRunning == true){
    return elapsedTime;
  }    
  else{
    return 0; // no magnet strip passed
  }
}


void updateTimeTables(){  
  sectortime = stopwatch();
  
   if(sectortimesFirst==SECTORS+1 && sectortimesFirst == sectortimesSaved){  //if we reached the end of the array AND the last sector has been saved.
     currentLap = currentLap + 1;
     sectortimesFirst = 0;
     sectortimesSaved = 0;
     currentSector = sectortimesFirst;
     
     laptime = 0;
     for(int i=0;i < SECTORS+1;i++){
       laptime += sectortimes[i];
     }
     laptimes[laptimesFirst] = laptime;
     laptimesFirst = (laptimesFirst+1)%(LAPS+2);
  } 
  if(sectortime != 0 && sectortimesFirst == 0 && sectortime != sectortimes[SECTORS] && sectortime > 1000){                                  //we cant check sectortime != sectortimes[sectortimesFirst - 1] when sectortimesFirst == 0. (see next if() )
    sectortimes[sectortimesFirst] = sectortime;
    sectortimesFirst ++;
    currentSector = sectortimesFirst;
  }
    
  if(sectortime != 0  && sectortimesFirst != 0 && sectortime != sectortimes[sectortimesFirst-1] && sectortime > 1000){          //if stopwatch is returning a time that is different from the last one (stopwatch() keeps returning time while passingMagnet() is true) and this time > 300.
    sectortimes[sectortimesFirst] = sectortime;                                                       
    sectortimesFirst ++;  
    currentSector = sectortimesFirst;
  }  


}



