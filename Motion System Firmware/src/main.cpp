#include <Arduino.h>
#include <TMCStepper.h>     // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>

#define EN_PIN_AZ           2 // Enable
#define DIR_PIN_AZ          3 // Direction
#define STEP_PIN_AZ         4 // Step
#define SERIAL_PORT_AZ Serial2 // TMC2208/TMC2224 HardwareSerial port
#define R_SENSE 0.11f // SilentStepStick series use 0.11
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define STALL_VALUE     100 // [0..255] Gotta figure out stallguard

#define EN_PIN_ALT           5 // Enable
#define DIR_PIN_ALT         6 // Direction
#define STEP_PIN_ALT        7 // Step
#define SERIAL_PORT_ALT Serial1 // TMC2208/TMC2224 HardwareSerial port






TMC2209Stepper driverAz(&SERIAL_PORT_AZ, R_SENSE, DRIVER_ADDRESS); // Create TMC driver for AZ
TMC2209Stepper driverAlt(&SERIAL_PORT_ALT, R_SENSE, DRIVER_ADDRESS); // Create TMC driver
using namespace TMC2208_n;
AccelStepper stepperAz = AccelStepper(stepperAz.DRIVER, STEP_PIN_AZ, DIR_PIN_AZ);
AccelStepper stepperAlt = AccelStepper(stepperAlt.DRIVER, STEP_PIN_ALT, DIR_PIN_ALT);

#define MAXANGLE = 330
#define ENDSTOP_AZ_MAX 6
#define ENDSTOP_ALT_MAX 7
float azSteps = 555.555;
float altSteps = 555.555;
int microSteps = 2;

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
double ut = 0; // Figure out how to do this - ut time now, has to be in hours.decimal
double millisAfterUtRead;

double ra;
double dec;
double radLatitude = Rad(latitude);
double radDec = Rad(dec);
double radHourAngle;
int expPosAz;
int expPosAlt;
int difference;


String RcvTxt = "";
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
  if (axis == "alt"){
      if (stepperAlt.distanceToGo() == 0) {
        stepperAlt.disableOutputs();
        delay(10);
        stepperAlt.moveTo(steps); // Move 100 deg
        stepperAlt.enableOutputs();
  } else if (axis == "az"){
      if (stepperAz.distanceToGo() == 0) {
        stepperAz.disableOutputs();
        delay(10);
        stepperAz.moveTo(steps); // Move 100 deg
        stepperAz.enableOutputs();
      }
    }
  }
}

double dayFrac(){
  double currMillis = millis() - millisAfterUtRead;
  return (currMillis/(1000*3600*24));
}

double UtNow(){
  double currMillis = millis() - millisAfterUtRead;
  return (ut + (currMillis/(1000*3600)));
}

void initialization()
{

}


double Lst(){  // in degrees
   double totalLst = 100.46 + 0.985647 * dayFrac() + longitude + 15*UtNow();
   return (totalLst - (int)(totalLst/360));
}

float CelestialToEquatorial(){
  double hourAngle = Lst() - ra;
  radHourAngle = Rad(hourAngle);
  if (hourAngle < 0){
    hourAngle = hourAngle + 360.0;
  }else if (!(hourAngle<360)){
    hourAngle = hourAngle - 360.0;
  }
  double sinAlt = (
    sin(radDec)*sin(radLatitude) +
    cos(radDec)*cos(radLatitude)*hourAngle
  );
  radAltitude = asin(sinAlt);
  altitude = Deg(radAltitude);


  double cosA = ((sin(radDec)-(sin(radAltitude)*sin(radLatitude)))/
          (cos(radAltitude)*cos(radLatitude))); 
  if (sin(radHourAngle)<0){
    azimuth = Deg(acos(cosA));
  } else{
    azimuth = 360.0 - Deg(acos(cosA));
  }




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
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////
    ///////////////////Serial Comm for setting registries//////////////////////
    ///////////////////////////////////////////////////////////////////////////
    SERIAL_PORT_AZ.begin(19200);
    SERIAL_PORT_ALT.begin(19200);
    Serial.println("serial opened...");

    // Writing to registries for AZ axis
    driverAz.beginSerial(19200);
    driverAz.begin();
    driverAz.toff(4); 
    driverAz.rms_current(400);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
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
    driverAlt.toff(4); 
    driverAlt.rms_current(400);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    driverAlt.pwm_autoscale(true); // Needed for stealthChop
    driverAlt.microsteps(microSteps);
    // printout for troubleshooting if needed
    Serial.println("reg values set...");
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////
    //////////////////////////AccelStepper Settings////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // Az axis
    stepperAz.setMaxSpeed(400);
    stepperAz.setAcceleration(1000);
    stepperAz.setEnablePin(EN_PIN_AZ);
    stepperAz.setPinsInverted(false, false, true);
    stepperAz.enableOutputs();

    // Alt Axis
    stepperAlt.setMaxSpeed(400);
    stepperAlt.setAcceleration(1000);
    stepperAlt.setEnablePin(EN_PIN_ALT);
    stepperAlt.setPinsInverted(false, false, true);
    stepperAlt.enableOutputs();
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    Serial.println("Testing driver connection through Microsteps");
    while (!(driverAz.microsteps() == microSteps)){
        Serial.println("Az Driver not connected - Test 1");
        delay(1000);
    }
    while (!(driverAlt.microsteps() == microSteps)){
        Serial.println("Alt Driver not connected - Test 2");
        delay(1000);
    }



}

void loop()
{
  CelestialToEquatorial();
  expPosAz = azimuth * azSteps * microSteps;
  expPosAlt = altitude * altSteps * microSteps;
  if (expPosAlt != altPos){
    moveAxis("alt",expPosAlt);
  } else if(expPosAz != azPos){
    moveAxis("az",expPosAz);
  }
  if (Serial.available() >0){
    while (Serial.available() >0){ // check the readString() function for this
      RcvTxt = RcvTxt +  Serial.read();
    }
    /*
    Commands that can be recieved:
    'm':move
    'p': recieve position, lat = 41.241489  lon = -77.041924
    */
    switch(RcvTxt.charAt(0)){
      case 'm':
        raStr = RcvTxt.substring(1,9);
        ra = raStr.toFloat();
        decStr = RcvTxt.substring(10,9);
        dec = decStr.toFloat();
        Serial.println("RA: " + raStr + "DEC" + decStr); // Recieved and processed
      break;
      case 'p':
        latitude = RcvTxt.substring(1,9).toFloat();
        longitude = RcvTxt.substring(10,9).toFloat();
        Serial.println("200"); // Recieved and processed
      break;
    }
  }

}