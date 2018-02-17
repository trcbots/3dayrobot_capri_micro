
#include <SoftwareSerial.h>
//#include "RunningAverage.h"


#include <Servo.h>
//#include <SabertoothSimplified.h>


// #include "serial_command.h"
#include "linda.h"

Linda l;
// SerialCommand sc;

unsigned int timeDiff;

void setup() {

    Serial.begin(9600);
    Serial.println("Initialising!");

    l.Init();

}


void loop() {
  // This is where all of the driverless car goodness happens

  // process_command() MUST be called in the main loop
  l.process_command();

}
