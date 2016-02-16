#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>
#include "Timer.h"
Timer t;
String dataString = "";

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //I2C display at adres 0x27

int engineTemperature;
int ambientTemperature;
int TEMPERATUREPIN = 1;

unsigned int rpm;
unsigned int v;
int RPMPIN = 2;
int analogRpm;
boolean sparkState;
boolean previousSparkState = false;
boolean engineRunning = false;
unsigned long sparkInterval;
unsigned long sparkStart;


float GEARRATIO = float(1); //engine/axle

int analogMagnet;
int MAGNETPIN = 0;

const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float totalG;

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
  Wire.endTransmission(false);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  //GYRO_CONFIG register
  Wire.write(0);     //0: 250°/S, 8: 500°/S, 10: 1000°/S, 18: 2000°/S
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  //ACCEL_CONFIG register
  Wire.write(0);        //0: 2G, 8: 4G, 10: 8G, 18: 16G
  
  Wire.endTransmission(true);
  
  lcd.begin(20,4);
  lcd.backlight();
  lcd.clear();
  
  int timerId;
//  timerId = t.every(300, updateDisplay);
//  timerId = t.every(300, updateEngineTemperature);
    timerId = t.every(300, updateAccelerometer);
    timerId = t.every(200, saveData);
  
  if(!SD.begin())
    error("SD card failed"); 
}

void loop(){
    //updateRPM();            //needs to run as fast as possible because at high RPM the square wave will have a period of a few milliseconds.
    updateTimeTables();     //needs to run as fast as possible because stopwatch() only returns sectortime while driving over a magnet strip.
    t.update();    
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(AcX);
    lcd.setCursor(0,1);
    lcd.print(AcY);
    lcd.setCursor(0,2);
    lcd.print(AcZ);
    
    lcd.setCursor(10,0);
    lcd.print(GyX);
    lcd.setCursor(10,1);
    lcd.print(GyY);
    lcd.setCursor(10,2);
    lcd.print(GyZ);
    
    lcd.setCursor(15,0);
    lcd.print(analogMagnet);
    
    lcd.setCursor(6,3);
    lcd.print(" S:");
    lcd.print((currentSector + 1) * stopwatchRunning);
    delay(1000);
    
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
  dataString += (String)rpm;
  dataString += ",";
  dataString += (String)engineTemperature;
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

//Returns true if there is a spark, behaves like a Schmitt Trigger. (Hysteresis)
//Limits to be determined. (Check max. input  x/1024 first)
//1024/5 = 204,8.  Hysteresis over 1/5th of the range: 0 -> 409,6 -> 614,4 -> 1024
boolean spark(){
  analogRpm = analogRead(RPMPIN);
  if(previousSparkState == true && analogRpm >= 410)
    return true;
  if(previousSparkState == true && analogRpm < 410)
    return false;
  if(previousSparkState == false && analogRpm >= 615)
    return true;
  if(previousSparkState == false && analogRpm < 615)
    return false;
    
  else
    return false;
}

//updates the current rpm based on spark(), measures time between positive flanks. (same principle as stopwatch() )
void updateRPM(){  
  sparkState = spark();
  if(sparkState == false && previousSparkState == true){
    previousSparkState = sparkState;
  }
  else if(sparkState == true && previousSparkState == false && engineRunning == false){
   sparkStart = millis();
   engineRunning = true;
   previousSparkState = sparkState;
  }  
  else if(sparkState == true && previousSparkState == false && engineRunning == true){
    sparkInterval = millis() - sparkStart;
    sparkStart = millis();
    previousSparkState = sparkState;
    if(sparkInterval > 6u)                         //This prevents some readings, by doing this we limit the maximum measurable RPM to 20 000.
      rpm=(unsigned int)(120000/sparkInterval);    // (60 * 2 * 1000)/sparkInterval
  }
  else{}
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
    totalG =  sqrt(AcX*AcX + AcY*AcY + AcZ*AcZ);
}


//Writes current data to the display.
void updateDisplay(){
    lcd.setCursor(0,0);
    lcd.print("RPM:");
    lcd.print(rpm);
    if(rpm<10)
      lcd.print(" ");
    if(rpm<100)
      lcd.print(" ");
    if(rpm<1000)
      lcd.print(" ");
    if(rpm<10000)
      lcd.print(" ");
    if(rpm<100000)
      lcd.print(" ");
     
    v=(unsigned int)(rpm*GEARRATIO); //*bandomtrek
    lcd.setCursor(0,1);
    lcd.print("SPEED:");
    lcd.print(v);   
    if(v < 100)
      lcd.print(" ");
    if(v < 1000)
      lcd.print(" ");    
    lcd.setCursor(0,2);
    lcd.print("T:");
    lcd.print(ambientTemperature);
    if(ambientTemperature < 10);
      lcd.print(" ");
    lcd.setCursor(5,2);
    lcd.print("T:");
    lcd.print(engineTemperature);
    if(engineTemperature<100)
      lcd.print(" ");
    if(engineTemperature<10)
      lcd.print(" ");
     
    lcd.setCursor(0,3);
    lcd.print("G:");
    lcd.print(totalG);
    
    lcd.setCursor(6,3);
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

//updates engine temperature
void updateEngineTemperature(){
     engineTemperature =(int)(analogRead(TEMPERATUREPIN)/ 2.048); 
}

//returns true if currently passing a magnet strip
boolean passingMagnet(){
    analogMagnet = analogRead(MAGNETPIN);    //No field: 506-510
    if(analogMagnet >=513 or analogMagnet <= 507)
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
  
  if(sectortime != 0 && sectortimesFirst == 0 && sectortime > 100){                                  //we cant check sectortime != sectortimes[sectortimesFirst - 1] when sectortimesFirst == 0. (see next if() )
    sectortimes[sectortimesFirst] = sectortime;
    sectortimesFirst ++;
    currentSector = sectortimesFirst;
  }
    
  if(sectortime != 0  && sectortime != sectortimes[sectortimesFirst-1] && sectortime > 300){          //if stopwatch is returning a time that is different from the last one (stopwatch() keeps returning time while passingMagnet() is true) and this time > 300.
    sectortimes[sectortimesFirst] = sectortime;                                                       
    sectortimesFirst ++;  
    currentSector = sectortimesFirst;
  } 
    
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

}



