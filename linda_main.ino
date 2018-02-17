
#include <SoftwareSerial.h>
#include <Servo.h>


#include "linda.h"

FireNugget fn;

unsigned int timeDiff;

void setup() {
    Serial.begin(9600);
    Serial.println("Initialising!");

    fn.Init();
}


void loop() {
  // This is where all of the driverless car goodness happens

  // process_command() MUST be called in the main loop
//  fn.process_command();
}
