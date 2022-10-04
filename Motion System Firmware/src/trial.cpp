#include <Arduino.h>
#include <TMCStepper.h> // TMCstepper - https://github.com/teemuatlut/TMCStepper

#define EN_PIN 2            // Enable - PURPLE
#define DIR_PIN 3           // Direction - WHITE
#define STEP_PIN 4          // Step - ORANGE
#define SW_SCK 5            // Software Slave Clock (SCK) - BLUE
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS); // Create TMC driver

void setup()
{
    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); // Enable driver in hardware

    driver.begin();          //  SPI: Init CS pins and possible SW SPI pins
    driver.toff(5);          // Enables driver in software
    driver.rms_current(600); // Set motor RMS current
    driver.microsteps(2);    // Set microsteps to 1/16th

    driver.pwm_autoscale(true); // Needed for stealthChop
}

bool shaft = false;

void loop()
{
    // Run 5000 steps and switch direction in software
    for (uint16_t i = 0; i < 10000; i++)
    {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(160);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(160);
    }
    shaft = !shaft;
    driver.shaft(shaft);
}