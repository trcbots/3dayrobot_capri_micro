
#include <SoftwareSerial.h>
#include <Servo.h>
#include "linda.h"

Linda l;

unsigned int timeDiff;

void setup() {

    Serial.begin(9600);
    Serial.println("Initialising!");

    l.Init();

}


void loop() {
  // This is where all of the driverless car goodness happens

  // process_command() MUST be called in the main loop
  // l.process_command();

}
