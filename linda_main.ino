
#include <SoftwareSerial.h>
#include "RunningAverage.h"


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

    /* ENGINE AUTOSTART (disabled for now..., RC operator can do this manually)
    delay(3000);

    // Turn on the ignition
    // (i.e. turn the ignition to the ON state, but not start the engine, yet)
    Serial.println("Ignition!");
    l.set_current_state_ID(IGNITION_STATE);

    delay(250);

    // Ok, lets start the engine!
    Serial.println("Engine!");
    l.set_current_state_ID(ENGINE_START_STATE);
    */

    // Lets go into the Remote Control Teleoperation state!
    // This control the car based on PWM commands read from the RC reciever
    l.Init();
    l.set_current_state_ID(HALT_STATE);
    //l.set_current_state_ID(AUTONOMOUS_SWITCH_STATE);

    // Please note that AI state is currently untested!
}


void loop() {
  // This is where all of the driverless car goodness happens

  // process_command() MUST be called in the main loop
  l.process_command();

}
