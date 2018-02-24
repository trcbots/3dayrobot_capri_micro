#include <Servo.h>
#include "accelerator_controller.h"
#include "brake_controller.h"
#include "data_parser.h"
#include "rc_parser.h"
#include "gear_controller.h"
#include "ignition_controller.h"

#define DEBUG 1

IgnitionController 		ignitionController(true);
BrakeController    		brakeController(false);
GearController     	  gearController(false);
AcceleratorController acceleratorController(false);
DataParser         		dataParser(true);

uint16_t steering_position;
uint16_t brake_position;
uint16_t accelerator_position;
uint16_t gear_position;
uint8_t  autonomy_status = 0;
uint8_t  ignition_status = 0;
uint8_t  kill_status = 0;
uint8_t  debug = 1;

uint8_t  nextMillis = 0; 
uint8_t  rate = 200;

String command;

/* 
	Timing mechanisms if we want to only allow commands after a certain
	period of time
*/
uint8_t ignition_interval = 5000; //ms
uint16_t start_time = 0;

//FireNugget fn;
//PWMCommand _sc;
//PWMCommand* sc = &_sc;

//unsigned int timeDiff;
// serial packet protocol
//bool processing_message;

void setup() {
    // serial packet protocol

    Serial.println("Initialising!");

    Serial.begin(9600);
    Serial.println("Serial Read Data Enabled");


    dataParser.setup();
    ignitionController.setup();
    brakeController.setup();
    gearController.setup();
    acceleratorController.setup();
  
//    fn.Init();
}


void loop() {
    // This is where all of the driverless car goodness happens
    // process_command() MUST be called in the main loop
    // fn.process_command();

    // process packet in buffer
    // if(sc->message_time - timeDiff > 500) {
    // PWM input pins from RC Reciever
//
//    sc->message_type = 1;
//    sc->message_ignition =          pulseIn(RC_PIN_7, HIGH);
//    sc->message_engine_start =      pulseIn(RC_PIN_8, HIGH);
//    sc->message_steering =          pulseIn(RC_PIN_1, HIGH);
//    sc->message_velocity =          pulseIn(RC_PIN_2, HIGH);
//    sc->message_gear =              pulseIn(RC_PIN_6, HIGH);
//    
//    sc->message_failsafe =          pulseIn(RC_PIN_7, HIGH);
    
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

//      fn.process_command(sc);
    dataParser.loop(100);
    logic();
    brakeController.loop(100);
    gearController.loop(100);
    ignitionController.loop(100);
    acceleratorController.loop(100);
}

void logic() {
    if (Serial.available()) {
        command = Serial.readStringUntil('\n');
        dataParser.parseExternalData(command);

        uint16_t expected_steering_status       = dataParser.getExpectedSteeringPosition();
        uint16_t expected_accelerator_status    = dataParser.getExpectedAcceleratorPosition();
        uint16_t expected_ignition_status       = dataParser.getExpectedIgnitionStatus();

        Serial.print("expected_steering: ");
        Serial.print(expected_steering_status);

        Serial.print("\texpected_accel: ");
        Serial.print(expected_accelerator_status);

        Serial.print("\texpected_init_status: ");
        Serial.println(expected_ignition_status);



        // ignition commands
        uint16_t expected_ignition_status = dataParser.getExpectedIgnitionStatus();
        ignition_status = ignitionController.getCurrentStatus();

        if (ignition_status == 0 && expected_ignition_status == 1) {
            ignitionController.start();
            if (DEBUG) Serial.println("Ignit on");
            delay(1000);
            //ignitionController.run();
        } else if (ignition_status == 1 && expected_ignition_status == 1) {
            ignitionController.run();
            if (DEBUG) Serial.println("Ignit run");
        } else if (ignition_status == 1 && expected_ignition_status == 0) {
            if (DEBUG) Serial.println("Ignit stop");
            ignitionController.stop();
        }


        // brake commands
        uint16_t expected_brake_status = dataParser.getExpectedBrakePosition();
        brake_position = brakeController.getCurrentPosition();

        if (brake_position == 0 && expected_brake_status == 1) {
            if (DEBUG) Serial.println("Brake on");
            brakeController.start();
        } else if (brake_position == 1 && expected_brake_status == 1) {
            if (DEBUG) Serial.println("Brake run");
            brakeController.run();
        } else if (brake_position == 1 && expected_brake_status == 0) {
            if (DEBUG) Serial.println("Brake on");
            brakeController.stop();
        }


        // accelerator commands

        uint16_t expected_accelerator_status = dataParser.getExpectedAcceleratorPosition();
        accelerator_position = acceleratorController.getCurrentPosition();

        if (accelerator_position == 0 && expected_accelerator_status == 1) {
            acceleratorController.start();
            if (DEBUG) Serial.println("Accel on");
        } else if (accelerator_position == 1 && expected_accelerator_status == 1) {
            if (DEBUG) Serial.println("Accel run");
            acceleratorController.run();
        } else if (accelerator_position == 1 && expected_accelerator_status == 0) {
            if (DEBUG) Serial.println("Accel stop");
            acceleratorController.stop();
        }

        // gear commands

        uint16_t expected_gear_position = dataParser.getExpectedGearPosition();
        gear_position = gearController.getCurrentPosition();

        Serial.println("Checking gears");
        if (expected_gear_position != gear_position) {
            gearController.setTargetPosition(expected_gear_position);
            if (expected_gear_position == 312) {
                Serial.println("Reverse");
            } else if (expected_gear_position == 457) {
                Serial.println("Drive");
            } else if (expected_gear_position == 367) {
                Serial.println("Neutral");
            }
        }
    }
}
