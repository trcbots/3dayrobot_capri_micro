#include <Servo.h>
#include "data_parser.h"
#include "steering_controller.h"
#include "ignition_controller.h"

// #include "rc_parser.h"
// #include "accelerator_controller.h"
// #include "brake_controller.h"

// #include "gear_controller.h"

#define DEBUG 1
#define STEERING_SERVO_MIN_POWER    0
#define STEERING_SERVO_MAX_POWER    11
#define STEERING_FEEDBACK_PIN       A5

#define STEERING_MOTOR_DRIVER_PIN   11

// Define the allowable range of motion for the steering actuator
//#define STEERING_FULL_LEFT          60     // full left lock
//#define STEERING_CENTRE             410    // steering in centre
//#define STEERING_FULL_RIGHT         710    // full right lock

#define STEERING_FULL_LEFT          250     // full left lock
#define STEERING_CENTRE             410    // steering in centre
#define STEERING_FULL_RIGHT         550    // full right lock

#define JETSON_STEERING_RIGHT       0
#define JETSON_STEERING_CENTRE      70
#define JETSON_STEERING_LEFT        140

#define STEERING_SENSITIVITY        1

// DECLARE DATA PARSER (FROM JETSON)
DataParser dataParser(true);

// DECLARE IGNITION
IgnitionController ignitionController(true);

// DECLARE STEERING
Servo _steering_servo;
Servo* steering_servo_ = &_steering_servo;
SteeringController *steer_motor_;
                                       
uint16_t steering_position;
uint8_t  autonomy_status = 0;
//uint16_t brake_position;
//uint16_t accelerator_position;
//uint16_t gear_position;


uint8_t  ignition_status = 0;
uint16_t expected_ignition_status = 0;
uint16_t expected_start_status = 0;

uint16_t expected_steering_status = JETSON_STEERING_CENTRE;
uint16_t expected_accelerator_status = 0;                   /// TO DO!!

uint16_t steering_command_pos = STEERING_CENTRE;

uint8_t  kill_status = 0;
uint8_t  debug = 1;

uint8_t  nextMillis = 0; 
uint8_t  rate = 200;

/* 
	Timing mechanisms if we want to only allow commands after a certain
	period of time
*/
uint16_t ignition_interval = 5000; //ms
uint16_t start_time = 0;

void setup() {
    _steering_servo.attach(STEERING_MOTOR_DRIVER_PIN);
    
    Serial.begin(9600);

    steer_motor_ = new SteeringController(steering_servo_, STEERING_FEEDBACK_PIN, 
                                      STEERING_FULL_LEFT, STEERING_FULL_RIGHT,
                                      STEERING_SERVO_MIN_POWER, STEERING_SERVO_MAX_POWER,
                                      STEERING_SENSITIVITY,
                                      2.0, 0.0, 0.0);
    
    ignitionController.setup();
}


void loop() {
    logic();    
}

void logic() {

// =========================== JETSON TEST ==============================

    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        Serial.print("Recieved message: ");
        Serial.println(command);

        // DATA PARSING FROM JETSON (CHECK FIRST)
        if (autonomy_status == 1) {
            dataParser.parseExternalData(command);

            expected_steering_status       = dataParser.getExpectedSteeringPosition();
            expected_accelerator_status    = dataParser.getExpectedAcceleratorPosition();
            expected_ignition_status       = dataParser.getExpectedIgnitionStatus();
      
            Serial.print("expected_steering: ");
            Serial.print(expected_steering_status);
      
            Serial.print("\t expected_accel: ");
            Serial.print(expected_accelerator_status);
      
            Serial.print("\t expected_ignit_status: ");
            Serial.println(expected_ignition_status);

            // map expected values from jetson to command positions
            if (expected_steering_status >= 70) {         // LEFT
                steering_command_pos = map(expected_steering_status, 70, 140, STEERING_CENTRE, STEERING_FULL_LEFT);
            } else if (expected_steering_status < 70) {   // RIGHT
                steering_command_pos = map(expected_steering_status, 70, 0, STEERING_CENTRE, STEERING_FULL_RIGHT);
            }

            Serial.print("mapped_steering: ");
            Serial.print(steering_command_pos);
        }
        

        // AUTONOMY MODE
        if (command == "on" and autonomy_status == 0) {
            autonomy_status = 1;
            Serial.println("AUTONOMOUS MODE ON");
        }
    
        if (command == "off" and autonomy_status == 1) {
            autonomy_status = 0;
            Serial.println("AUTONOMOUS MODE OFF");
        }
        

        // IGNITION AND START COMMANDS
       
        if (command == "ignit") {
            expected_ignition_status = 1;
        }

        if (command == "start" && ignition_status == 1) {
            expected_start_status = 1;
        }

        if (command == "stop") {
            expected_ignition_status = 0;
            if (autonomy_status == 1) {
                autonomy_status == 0;
            }
        }

        
        // STEERING COMMANDS

        if (command == "centre") {
            steering_command_pos = 410;
        }
    
        if (command == "left") {
            steering_command_pos = 250;
        }
    
        if (command == "right") {
            steering_command_pos = 550;
        }
    }

    
    // SET IGNITION STATUS
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
        if (DEBUG) Serial.println("Stopping engine");
        ignitionController.stop();
        expected_ignition_status = 0;
        ignition_status = 0;
    }
    

    // SET STEER MOTOR
    steer_motor_->SetTargetPosition(steering_command_pos);

}


void reset() {
    expected_ignition_status = 0;
    expected_start_status = 0; 
}


