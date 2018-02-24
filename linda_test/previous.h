#include <Servo.h>
#include "ignition_controller.h"

#define DEBUG 1

IgnitionController ignitionController(true);

uint8_t  ignition_status = 0;
uint16_t expected_ignition_status = 0;
uint16_t expected_start_status = 0;

uint8_t  kill_status = 0;
uint8_t  debug = 1;

uint8_t  nextMillis = 0; 
uint8_t  rate = 200;

String command;

/* 
	Timing mechanisms if we want to only allow commands after a certain
	period of time
*/
uint16_t ignition_interval = 5000; //ms
uint16_t start_time = 0;

void setup() {
    Serial.begin(9600);
    ignitionController.setup();
}


void loop() {
    logic();    
}

void logic() {
    
    if (Serial.available()) {
        Serial.print("Recieved message: ");
        command = Serial.readStringUntil('\n');
        Serial.println(command);


        // IGNITION AND START TEST
        
        if (command == "ignit") {
            expected_ignition_status = 1;
        }

        if (command == "start" && ignition_status == 1) {
            expected_start_status = 1;
        }

        if (command == "stop") {
            expected_ignition_status = 0;
        }
    }

    if (ignition_status == 0 && expected_ignition_status == 1) {
        ignitionController.ignition();
        if (DEBUG) Serial.println("Ignit relay");
        delay(1000);
        ignition_status = 1;
    }

    if (ignition_status == 1 && expected_start_status == 1) {
        ignitionController.start();
        if (DEBUG) Serial.println("Start relay");
        expected_start_status = 0;
    }
      
    if (expected_ignition_status == 0 && ignition_status == 1) {
    	  ignitionController.stop();
        expected_ignition_status = 0;
        ignition_status = 0;
    }

    Serial.print("expected ignit status: ");
    Serial.println(expected_ignition_status);
}


void reset() {
    expected_ignition_status = 0;
    expected_start_status = 0; 
}


