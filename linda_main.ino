
#include <Servo.h>
#include "linda.h"

FireNugget fn;
PWMCommand _sc;
PWMCommand* sc = &_sc;

unsigned int timeDiff;

// serial packet protocol
//bool processing_message;

void setup() {
    // serial packet protocol

    Serial.begin(9600);
    
    Serial.println("Initialising!");
    Serial.println("Serial Read Data Enabled");

    fn.Init();
}


void loop() {
    // This is where all of the driverless car goodness happens
    // process_command() MUST be called in the main loop
    // fn.process_command();

    // process packet in buffer
    // if(sc->message_time - timeDiff > 500) {
    // PWM input pins from RC Reciever
    sc->message_type = 1;
    sc->message_ignition =          pulseIn(RC_PIN_7, HIGH);
    sc->message_engine_start =      pulseIn(RC_PIN_8, HIGH);
    sc->message_steering =          pulseIn(RC_PIN_1, HIGH);
    sc->message_velocity =          pulseIn(RC_PIN_2, HIGH);
    sc->message_gear =              pulseIn(RC_PIN_6, HIGH);
    
    sc->message_failsafe =          pulseIn(RC_PIN_7, HIGH);
    
//    Serial.print(sc->message_type);
//    Serial.print(",");
//    Serial.print(sc->message_ignition);
//    Serial.print(",");
//    Serial.print(sc->message_engine_start);
//    Serial.print(",");
//    Serial.print(sc->message_steering);
//    Serial.print(",");
//    Serial.print(sc->message_velocity);
//    Serial.print(",");
//    Serial.print(sc->message_gear);
//    Serial.print(",");
//    Serial.println(sc->message_failsafe);

      fn.process_command(sc);
}
