#include <Arduino.h>
#include <TMCStepper.h>     // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <TimeLib.h>
#include <DS3232RTC.h>

#define EN_PIN_AZ           2 
#define DIR_PIN_AZ          3 
#define STEP_PIN_AZ         4 
#define SERIAL_PORT_AZ Serial3 // To write to registries
#define ENDSTOP_PIN_AZ  8

#define EN_PIN_ALT           5 
#define DIR_PIN_ALT         6 
#define STEP_PIN_ALT        7 
#define SERIAL_PORT_ALT Serial1 // To write to registries
#define ENDSTOP_PIN_ALT  9

#define R_SENSE 0.11f // SilentStepStick series use 0.11
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define STALL_VALUE     100 // [0..255] Gotta figure out stallguard

TMC2209Stepper driverAz(&SERIAL_PORT_AZ, R_SENSE, DRIVER_ADDRESS); // Create TMC driver for AZ
TMC2209Stepper driverAlt(&SERIAL_PORT_ALT, R_SENSE, DRIVER_ADDRESS); // Create TMC driver for ALT
using namespace TMC2208_n;
AccelStepper stepperAz = AccelStepper(stepperAz.DRIVER, STEP_PIN_AZ, DIR_PIN_AZ);
AccelStepper stepperAlt = AccelStepper(stepperAlt.DRIVER, STEP_PIN_ALT, DIR_PIN_ALT);
DS3232RTC myRTC;

#define MAXANGLE = 330
double azSteps = 555.555555555;
double altSteps = 550.555555555;
int microSteps = 16;

float azPos = 0;
float altPos = 0;

int accel;
long maxSpeed;
int speedChangeDelay;
bool dir = false;
float latitude;
float longitude;
double altitude;
double radAltitude;
double azimuth;
double ut = 10.0;
double millisAfterUtRead = 00.00;
bool safety = true;

double ra = 0;
double dec = 0;
double radLatitude;
double radDec;
double radHourAngle;
double hourAngle;
int expPosAz;
int expPosAlt;
int difference;


String RcvTxt = "Nothing Input";
String raStr;
String decStr;

double Rad(double angle){
  return (angle * (M_PI/180.0));
}

double Deg(double angle){
   return (angle * (180.0/M_PI));
}

void moveAxis(String axis, int steps) //(az, alt)(neg, pos)
{
  //Serial.println(axis);
  if (axis == "alt"){
    //Serial.println(stepperAlt.distanceToGo());
      if (stepperAlt.distanceToGo() == 0) {
        stepperAlt.disableOutputs();
        delay(10);
        stepperAlt.moveTo(steps); // Move 100 deg
        stepperAlt.enableOutputs();
        while (stepperAlt.distanceToGo() != 0){
          stepperAlt.run();
          //Serial.print("alt: ");
          //Serial.println(steps);
          }
        altPos = steps;
        
      } 
  }
  
  if (axis == "az"){
      //Serial.println("az moving");
      if (stepperAz.distanceToGo() == 0) {
        stepperAz.disableOutputs();
        delay(10);
        stepperAz.moveTo(steps); // Move 100 deg
        stepperAz.enableOutputs();
        while (stepperAz.distanceToGo() != 0){
          stepperAz.run();
          //Serial.print("az: ");
          //Serial.println(steps);
          }
        azPos = steps;

      }
    }

  }

void InputToInstructions(){
  RcvTxt="";
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer    
    RcvTxt += c; //makes the string readString
  }
  switch(RcvTxt.charAt(0)){
  case 'm':
    raStr = RcvTxt.substring(1,9);
    ra = raStr.toFloat();
    decStr = RcvTxt.substring(10,9);
    dec = decStr.toFloat();
    Serial.println("RA: " + raStr);
    Serial.println( " DEC" + decStr); // Recieved and processed
  break;
  case 'p':
    latitude = RcvTxt.substring(1,9).toFloat();
    longitude = RcvTxt.substring(10,9).toFloat();
    Serial.println(""); // Recieved and processed
  break;
}
}

double dayFrac(){
  double currMillis = millis() - millisAfterUtRead;
  return (currMillis/(1000.0*3600.0*24));
}

double UtNow(){

  double currMillis = millis() - millisAfterUtRead;
  return (ut + (currMillis/(1000.0*3600.0)));
}

void initialization()
{
  long homing = -1;
  Serial.println("homing...");
  pinMode(ENDSTOP_PIN_AZ, INPUT_PULLUP ); //Limit switch must be connected to gnd when closed
  stepperAz.setMaxSpeed(100.0);       // slow speed for homing
  stepperAz.setAcceleration(100.0);
  Serial.println(digitalRead( ENDSTOP_PIN_AZ ));
  stepperAz.setCurrentPosition(0);
  while( digitalRead( ENDSTOP_PIN_AZ ) == HIGH ){
    stepperAz.moveTo(homing);
    stepperAz.run();
    homing--;
    }
  stepperAz.setCurrentPosition(0);
  homing = -1;
  delay(500);
  pinMode(ENDSTOP_PIN_ALT, INPUT_PULLUP );
  stepperAlt.setMaxSpeed(100.0);
  stepperAlt.setAcceleration(100.0);
  while( digitalRead( ENDSTOP_PIN_ALT ) == HIGH ){
    stepperAlt.moveTo(homing);
    stepperAlt.run();
    homing--;
  }
  stepperAlt.setCurrentPosition(0);
  Serial.println("Homing complete");
  stepperAlt.disableOutputs();
  stepperAz.disableOutputs();
  stepperAz.setMaxSpeed(3000);
  stepperAz.setAcceleration(200);
  stepperAlt.setMaxSpeed(3000);
  stepperAlt.setAcceleration(200);
  stepperAlt.enableOutputs();
  stepperAz.enableOutputs();

}

double Lst(){  // in degrees
   double totalLst = 100.46 + 0.985647 * dayFrac() + longitude + 15*UtNow();
   return (totalLst - (int)(totalLst/360));
}

void CelestialToEquatorial(){
  radDec = Rad(dec);
  radLatitude = Rad(latitude);
  hourAngle = Lst() - ra;
  //Serial.println(hourAngle);
  if (hourAngle < 0){
    hourAngle = hourAngle + 360.0;
  }else if (!(hourAngle<360)){
    hourAngle = hourAngle - 360.0;
  }
  radHourAngle = Rad(hourAngle);
  double sinAlt = Rad(
    sin(radDec)*sin(radLatitude) +
    cos(radDec)*cos(radLatitude)*cos(radHourAngle)
  );
  //Serial.println(sinAlt);
  radAltitude = asin(sinAlt);
  altitude = Deg(radAltitude);
  //Serial.println(sinAlt);


  double cosA = ((sin(radDec)-(sin(radAltitude)*sin(radLatitude)))/
          (cos(radAltitude)*cos(radLatitude))); 
  if (sin(radHourAngle)<0){
    azimuth = Deg(acos(cosA));
  } else{
    azimuth = 360.0 - Deg(acos(cosA));
  }
  delay(100);
}

void setup()
{
    Serial.begin(9600);
    while(!Serial){
      delay(1000);
    };
    Serial.println("Start...");


    ///////////////////////////////////////////////////////////////////////////
    ///////////////////Setting Pins for AccelStepper///////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // AZ axis
    pinMode(EN_PIN_AZ, OUTPUT);
    pinMode(STEP_PIN_AZ, OUTPUT);
    pinMode(DIR_PIN_AZ, OUTPUT);
    digitalWrite(EN_PIN_AZ, LOW); 

    // ALT axis
    pinMode(EN_PIN_ALT, OUTPUT);
    pinMode(STEP_PIN_ALT, OUTPUT);
    pinMode(DIR_PIN_ALT, OUTPUT);
    digitalWrite(EN_PIN_ALT, LOW); 
    //-----------------------------------------------------------------------//
    //-----------------------------------------------------------------------//


    ///////////////////////////////////////////////////////////////////////////
    ///////////////////Serial Comm for setting registries//////////////////////
    ///////////////////////////////////////////////////////////////////////////
    SERIAL_PORT_AZ.begin(19200);
    SERIAL_PORT_ALT.begin(19200);
    Serial.println("serial opened...");

    // Writing to registries for AZ axis
    driverAz.beginSerial(19200);
    driverAz.begin();
    driverAz.toff(5); 
    driverAz.I_scale_analog(false);
    driverAz.rms_current(1600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    //driverAz.en_spreadCycle(false);
    driverAz.pwm_autoscale(true); // Needed for stealthChop
    driverAz.microsteps(microSteps);
    //driver.TCOOLTHRS(0xFFFFF); // 20bit max
    //driver.semin(5);
    //driver.semax(2);
    //driver.sedn(0b01);
    //driver.SGTHRS(STALL_VALUE); Gotta figure this out for stallguard

    // Writing to registries for ALT axis
    driverAlt.beginSerial(19200);
    driverAlt.begin();
    driverAlt.toff(5); 
    driverAlt.I_scale_analog(false);
    driverAlt.rms_current(1600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    //driverAlt.en_spreadCycle(false);
    driverAlt.pwm_autoscale(true); // Needed for stealthChop
    driverAlt.microsteps(microSteps);
    // printout for troubleshooting if needed
    Serial.println("reg values set...");
    //-----------------------------------------------------------------------//
    //-----------------------------------------------------------------------//


    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////AccelStepper Settings////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // Az axis
    stepperAz.setMaxSpeed(4000);
    stepperAz.setAcceleration(1000);
    stepperAz.setEnablePin(EN_PIN_AZ);
    stepperAz.setPinsInverted(false, false, true);
    stepperAz.enableOutputs();

    // Alt Axis
    stepperAlt.setMaxSpeed(4000);
    stepperAlt.setAcceleration(1000);
    stepperAlt.setEnablePin(EN_PIN_ALT);
    stepperAlt.setPinsInverted(false, false, true);
    stepperAlt.enableOutputs();
    //-----------------------------------------------------------------------//
    //-----------------------------------------------------------------------//

    Serial.println("Testing driver connection through Microsteps");
    while (!((int)driverAz.microsteps() == microSteps)){
        driverAz.microsteps(microSteps);
        Serial.println(driverAz.microsteps());
        Serial.println("Az Driver not connected - Test 1");
        delay(1000);
    }
    while (!((int)driverAlt.microsteps() == microSteps)){
        Serial.println("Alt Driver not connected - Test 2");
        delay(1000);
    }

    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////Sync RTC for UT time/////////////////////////////
    ///////////////////////////////////////////////////////////////////////////    

    myRTC.begin();
    setSyncProvider(myRTC.get);   // the function to get the time from the RTC
    if(timeStatus() != timeSet)
    {
        Serial.println("Unable to sync with the RTC");
    }
    else
    {
        Serial.println("RTC has set the system time");
    }
    //read time
    time_t t = myRTC.get();
    //read milliseconds for tracking -> ds3231 only has second accuracy
    millisAfterUtRead = millis();
    //calculate utc time in decimal
    ut = hour(t) + (minute(t)/60) + (second(t)/3600);

    //-----------------------------------------------------------------------//
    //-----------------------------------------------------------------------//


    //-------------->>> Add requests for information here

   /* while (ra == 0){
      Serial.println("Enter Ra");
      ra = Serial.readString().toDouble();}
      while (dec == 0){
      Serial.println("Input Dec");
      dec = Serial.readString().toDouble();
      delay(1000);
    }*/
    ra = 088.80000;
    dec = 007.40000;
    Serial.println("ra: "+ (String)ra + " Dec = "+ (String)dec);
    initialization();

    while (!Serial.available()) //wait for input
    {
      Serial.println('Input RADEC mxxxx.xxxxyyyy.yyyy');
    }
    InputToInstructions();
}

void loop(){
  //Serial.println("Loop!");
  CelestialToEquatorial();
  expPosAz = (long) (azimuth * azSteps * microSteps);
  expPosAlt = (long) (altitude * altSteps * microSteps);
  //Serial.println(expPosAlt);


  if (expPosAlt != altPos){
    moveAxis("alt",expPosAlt);
    Serial.print("Moving Altitude to:");
    Serial.println(expPosAlt);
  }
  if(expPosAz != azPos){
    moveAxis("az",expPosAz);
    Serial.print("Moving Azimuth to:");
    Serial.println(expPosAz);
  }
  if (Serial.available() >0){
    RcvTxt = Serial.readString();
    InputToInstructions();    
    /*
    Commands that can be recieved:
    'm':move m
    'p': recieve position, lat = +041.2414  lon = -077.0419
    */
  }

}