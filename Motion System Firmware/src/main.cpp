#include <Arduino.h>
#include <TMCStepper.h>     // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <SoftwareSerial.h> // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
#include <Streaming.h>      // for the << operator I think

#define EN_PIN 2                         // Enable - PURPLE
#define DIR_PIN 3                        // Direction - WHITE
#define STEP_PIN 4                       // Step - ORANGE
#define SW_SCK 5                         // Software Slave Clock (SCK) - BLUE
#define SERIAL_PORT Serial1              // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00              // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f                    // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)


TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS); // Create TMC driver

#define MAXANGLE = 330
#define ENDSTOP_AZ_MAX 6
#define ENDSTOP_ALT_MAX 7
float azESteps = 555.555;
float altESteps = 555.555;

float azPos = 0;
float altPos = 0;

int accel;
long maxSpeed;
int speedChangeDelay;
bool dir = false;

float calculateAngle(int esteps, String axis)
{
  if (axis == "az")
  {
    return (esteps * azESteps);
  }
  if (axis == "alt")
  {
    return (esteps * altESteps);
  }
}

void moveAxis(String axis, float angle, String direction)
{
  // Insert code here

  if (direction == "neg")
  {
    if (axis == "az")
    {
      azPos = azPos - angle;
    }
    else if (axis == "alt")
    {
      altPos = altPos - angle;
    }

    if (direction == "pos")
    {
      if (axis == "az")
      {
        azPos = azPos + angle;
      }
      else if (axis == "alt")
      {
        altPos = altPos + angle;
      }
    }
  }
}

void moveSingleStep(String axis, String Direction)
{
  // Insert code here
}

void northLoc(float lat, float dirFront)
{                                        // latitude and direction to front
  moveAxis("alt", lat, "pos");           // Moves up from straight ahead
  moveAxis("az", (0 - dirFront), "neg"); // Moves to North
}

void initialization()
{
  // Move to endstop for AZ
  while (digitalRead(ENDSTOP_AZ_MAX != 1))
  {
    moveSingleStep("az", "neg");
  }
  while (digitalRead(ENDSTOP_ALT_MAX != 1))
  {
    moveSingleStep("alt", "neg");
  }

  // Currently assuming facing east and level
}
void computeposition()
{
  float lat = 41.2493292;
  float lon = -77.0027671;
}
void setup()
{

  Serial.begin(11520);          // initialize hardware serial for debugging
  TMCdriver.beginSerial(11520); // Initialize UART

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  TMCdriver.begin();          // UART: Init SW UART (if selected) with default 115200 baudrate
  TMCdriver.toff(5);          // Enables driver in software
  TMCdriver.rms_current(500); // Set motor RMS current
  TMCdriver.microsteps(256);  // Set microsteps

  TMCdriver.en_spreadCycle(false);
  TMCdriver.pwm_autoscale(true); // Needed for stealthChop

  pinMode(ENDSTOP_AZ_MAX, INPUT_PULLUP);
  pinMode(ENDSTOP_ALT_MAX, INPUT_PULLUP);

  float fov = 2;
  float hRes = 10;
  float vRes = 10;
  float integrationTime = 10; // in seconds
}

void loop()
{
  accel = 10000;          // Speed increase/decrease amount
  maxSpeed = 50000;       // Maximum speed to be reached
  speedChangeDelay = 100; // Delay between speed changes

  for (long i = 0; i <= maxSpeed; i = i + accel)
  {                       // Speed up to maxSpeed
    TMCdriver.VACTUAL(i); // Set motor speed
    Serial << TMCdriver.VACTUAL() << endl;
    delay(100);
  }

  for (long i = maxSpeed; i >= 0; i = i - accel)
  { // Decrease speed to zero
    TMCdriver.VACTUAL(i);
    Serial << TMCdriver.VACTUAL() << endl;
    delay(100);
  }

  dir = !dir;           // REVERSE DIRECTION
  TMCdriver.shaft(dir); // SET DIRECTION
}