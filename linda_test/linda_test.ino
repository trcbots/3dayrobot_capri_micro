#include <Servo.h>
#include "steering_controller.h"
#include "ignition_controller.h"

// #include "accelerator_controller.h"
// #include "brake_controller.h"
// #include "data_parser.h"
// #include "rc_parser.h"
// #include "gear_controller.h"

#define DEBUG 1
#define STEERING_SERVO_MIN_POWER    0
#define STEERING_SERVO_MAX_POWER    11
#define STEERING_FEEDBACK_PIN       A5

// Define the allowable range of motion for the steering actuator
#define STEERING_FULL_LEFT          60     // full left lock
#define STEERING_CENTRE             410    // steering in centre
#define STEERING_FULL_RIGHT         710    // full right lock

#define STEERING_SENSITIVITY        1


Servo _steering_servo;
Servo* steering_servo_ = &_steering_servo;

SteeringController *steer_motor_;

IgnitionController ignitionController(true);

                                          
uint16_t steering_position;
//uint16_t brake_position;
//uint16_t accelerator_position;
//uint16_t gear_position;
//uint8_t  autonomy_status = 0;

uint8_t  ignition_status = 0;
uint16_t expected_ignition_status = 0;
uint16_t expected_start_status = 0;

uint16_t steering_command_pos_ = 410;

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

    steer_motor_ = new SteeringController(steering_servo_, STEERING_FEEDBACK_PIN, 
                                      STEERING_FULL_LEFT, STEERING_FULL_RIGHT,
                                      STEERING_SERVO_MIN_POWER, STEERING_SERVO_MAX_POWER,
                                      STEERING_SENSITIVITY,
                                      0.5, 0.0, 0.0);
    
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


// ========================================================================

        // STEERING TEST

        if (command == "centre") {
            steering_command_pos_ = 410;
        }

        if (command == "left") {
            steering_command_pos_ = 300;
        }

        if (command == "right") {
            steering_command_pos_ = 500;
        }

        steer_motor_->SetTargetPosition(steering_command_pos_);
    }
    

}


void reset() {
    expected_ignition_status = 0;
    expected_start_status = 0; 
}


