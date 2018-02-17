
#include <Servo.h>
#include "linda.h"

FireNugget fn;
SerialCommand _sc;
SerialCommand* sc = &_sc;

unsigned int timeDiff;

void setup() {
    Serial.begin(9600);
    
    Serial.println("Initialising!");
    Serial.println("Serial Read Data Enabled");

    fn.Init();
}


void loop() {
    sc->ReadData();
    // This is where all of the driverless car goodness happens
    // process_command() MUST be called in the main loop
    // fn.process_command();
    
//    Serial.println(sc.message_type);
    if(sc->message_time - timeDiff > 500) {
        if ( sc->message_type != -1 ) {
            Serial.print("valid message:");
            Serial.print(sc->message_type);
            Serial.print(",");
            Serial.print(sc->message_ignition);
            Serial.print(",");
            Serial.print(sc->message_engine_start);
            Serial.print(",");
            Serial.print(sc->message_steering);
            Serial.print(",");
            Serial.print(sc->message_velocity);
            Serial.print(",");
            Serial.println(sc->message_gear);
    
            fn.process_command(sc);
            sc->Reset();
        } 
    }
}
