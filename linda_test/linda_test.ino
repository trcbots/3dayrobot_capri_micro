#include <Servo.h>
#include "data_parser.h"
#include "rc_receiver.h"
#include "ignition_controller.h"
#include "steering_controller.h"
#include "brake_controller.h"

// #include "gear_controller.h"

// IGNITION and START
#define RC_IGNITION_OFF             40
#define RC_IGNITION_ON              115

#define RC_START_OFF                40
#define RC_START_ON                 115

#define RC_DEADZONE                 50

//  STEERING
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

#define RC_STEERING_RIGHT           1010
#define RC_STEERING_CENTRE          1450
#define RC_STEERING_LEFT            1860

#define STEERING_SENSITIVITY        1


// THROTTLE
#define RC_THROTTLE_MAX             1910
#define RC_THROTTLE_IDLE            1470
#define RC_BRAKE_MAX                1070

#define THROTTLE_IDLE               0     // min throttle
#define THROTTLE_MAX                30    // max throttle

#define THROTTLE_SERVO_PIN          3


// BRAKE
#define BRAKE_MIN                   635   
#define BRAKE_MAX                   383   // max brake

#define BRAKE_FEEDBACK_PIN          A3  // brake actuator position (10 bit analog signal)
#define BRAKE_MOTOR_DRIVER_PIN      9

#define BRAKE_SERVO_MIN_POWER       0
#define BRAKE_SERVO_MAX_POWER       90
#define BRAKE_BUFFER                50

#define BRAKE_SENSITIVITY           1



// DECLARE DATA PARSER (FROM JETSON)
DataParser dataParser(true);

// DECLARE RC PARSER
RCReceiver RCReceiver(true);

// DECLARE IGNITION
IgnitionController ignitionController(true);

// DECLARE STEERING
Servo _steering_servo;
Servo* steering_servo_ = &_steering_servo;
SteeringController *steer_motor_;

// DECLARE BRAKE
Servo _brake_servo;
Servo* brake_servo_ = &_brake_servo;
BrakeController *brake_motor_;

// DECLARE THROTTLE
Servo _throttle_servo_;
                                       
uint16_t steering_position;
uint8_t  autonomy_status = 0;
uint8_t  rc_status = 0;
//uint16_t brake_position;
//uint16_t accelerator_position;
//uint16_t gear_position;


uint8_t  ignition_status = 0;

// expected values from jetson
uint16_t expected_ignition_status = 0;
uint16_t expected_start_status = 0;
uint16_t expected_steering_status = JETSON_STEERING_CENTRE;
uint16_t expected_accelerator_status = 0;

uint16_t steering_command_pos = STEERING_CENTRE;
uint16_t throttle_command_pos = THROTTLE_IDLE;
uint16_t brake_command_pos = BRAKE_MIN;

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

int printcounter = 0;
int ignitioncounter = 0;
int startcounter = 0;


void setup() {
    Serial.begin(9600);

    //steering
    _steering_servo.attach(STEERING_MOTOR_DRIVER_PIN);
    steer_motor_ = new SteeringController(steering_servo_, STEERING_FEEDBACK_PIN, 
                                          STEERING_FULL_LEFT, STEERING_FULL_RIGHT,
                                          STEERING_SERVO_MIN_POWER, STEERING_SERVO_MAX_POWER,
                                          STEERING_SENSITIVITY,
                                          2.0, 0.0, 0.0);

    // brake
    _brake_servo.attach(BRAKE_MOTOR_DRIVER_PIN);
    brake_motor_ = new BrakeController(   brake_servo_, BRAKE_FEEDBACK_PIN,
                                          BRAKE_MIN, BRAKE_MAX + BRAKE_BUFFER,
                                          BRAKE_SERVO_MIN_POWER, BRAKE_SERVO_MAX_POWER,
                                          BRAKE_SENSITIVITY,
                                          1, 0.0, 0.0);
    
    //throttle
    _throttle_servo_.attach(THROTTLE_SERVO_PIN);
    delay(5);
    _throttle_servo_.write(0);
    
    ignitionController.setup();
}

void loop() {
    logic();
}

void logic() {

    // ============================= RC TEST ================================
    RCReceiver.readRCData();
        
    uint8_t rc_autonomy_status        = RCReceiver.getExpectedAutonomyStatus();
    uint8_t rc_ignition_status        = RCReceiver.getExpectedIgnitionStatus();
    uint8_t rc_start_status           = RCReceiver.getExpectedStartStatus();
    uint16_t rc_steering_position     = RCReceiver.getExpectedSteeringPosition();
    uint16_t rc_velocity              = RCReceiver.getExpectedVelocity();
    uint16_t rc_gear                  = RCReceiver.getExpectedGearPosition();

    printcounter++;
    if (DEBUG && printcounter == 5) {        
        Serial.print("Autonomy: ");
        Serial.print(rc_autonomy_status);
        Serial.print("\tIgnition: ");
        Serial.print(rc_ignition_status);
        Serial.print("\tStart: ");
        Serial.print(rc_start_status);
        Serial.print("\tSteering: ");
        Serial.print(rc_steering_position);
        Serial.print("\tVelocity: ");
        Serial.print(rc_velocity);
        Serial.print("\tGear: ");
        Serial.println(rc_gear);  
    }
    
    // kill switch
    if (rc_ignition_status <= (RC_IGNITION_OFF + 20)) {
        expected_ignition_status = 0;
    }
    
    if (rc_status == 1) {
    
        // map RC values to controllers

        // ignition
        if (ignitioncounter >= 10) {
            expected_ignition_status = 1;
        }
    
        if (rc_ignition_status >= (RC_IGNITION_ON - 20)) {
            ignitioncounter++;
        } else {
            expected_ignition_status = 0;
            ignitioncounter = 0;
        }
 
        // start
        if (startcounter >= 10) {
            expected_start_status = 1;
        }
        
        if (rc_start_status >= (RC_START_ON - 50)) {
            startcounter++;
        } else {
            expected_start_status = 0;
            startcounter = 0;
        }
    
        // steering
        if (rc_steering_position >= RC_STEERING_CENTRE) {     // left
            steering_command_pos = map(rc_steering_position, RC_STEERING_CENTRE, RC_STEERING_LEFT, STEERING_CENTRE, STEERING_FULL_LEFT);
        } else if (rc_steering_position < RC_STEERING_CENTRE) {               // right
            steering_command_pos = map(rc_steering_position, RC_STEERING_CENTRE, RC_STEERING_RIGHT, STEERING_CENTRE, STEERING_FULL_RIGHT);
        }

        // throttle
        if (rc_velocity >= RC_THROTTLE_IDLE + RC_DEADZONE) {
            throttle_command_pos = map(rc_velocity, RC_THROTTLE_IDLE + RC_DEADZONE, RC_THROTTLE_MAX, THROTTLE_IDLE, THROTTLE_MAX);
        // idle
        } else {
            throttle_command_pos = THROTTLE_IDLE;
        }
        
        // brake
        if (rc_velocity <= RC_THROTTLE_IDLE - RC_DEADZONE) {
            brake_command_pos = map(rc_velocity, RC_BRAKE_MAX, RC_THROTTLE_IDLE - RC_DEADZONE, BRAKE_MAX, BRAKE_MIN);
        } else {
            brake_command_pos = BRAKE_MIN;
        }
        
        
        
    }

    if (DEBUG && printcounter == 5) {
        printcounter = 0;
        
        Serial.print("expected_ignit_status: ");
        Serial.print(expected_ignition_status);
    
        Serial.print("\t expected_start_status: ");
        Serial.print(expected_start_status);
        
        Serial.print("\t steering_command_pos: ");
        Serial.print(steering_command_pos);
          
        Serial.print("\t throttle_command_pos: ");
        Serial.print(throttle_command_pos);

        Serial.print("\t brake_command_pos: ");
        Serial.println(brake_command_pos);
    }
    

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
            if (expected_steering_status >= JETSON_STEERING_CENTRE) {           // LEFT
                steering_command_pos = map(expected_steering_status, JETSON_STEERING_CENTRE, JETSON_STEERING_LEFT, STEERING_CENTRE, STEERING_FULL_LEFT);
            } else if (expected_steering_status < JETSON_STEERING_CENTRE) {     // RIGHT
                steering_command_pos = map(expected_steering_status, JETSON_STEERING_CENTRE, JETSON_STEERING_RIGHT, STEERING_CENTRE, STEERING_FULL_RIGHT);
            }

            Serial.print("mapped_steering: ");
            Serial.print(steering_command_pos);
        }

        // RC MODE
        if (command == "rc_on" and rc_status == 0) {
            autonomy_status = 0;
            rc_status = 1;
            Serial.println("RC MODE ON");
        }

        if (command == "rc_off" and rc_status == 1) {
            autonomy_status = 0;
            rc_status = 0;
            Serial.println("RC MODE OFF");
        }
        

        // AUTONOMY MODE
        if (command == "ai_on" and autonomy_status == 0) {
            rc_status = 0;
            autonomy_status = 1;
            Serial.println("AUTONOMOUS MODE ON");
        }
    
        if (command == "ai_off" and autonomy_status == 1) {
            autonomy_status = 0;
            rc_status = 1;
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

    // SET ACTUATORS
    
    steer_motor_->SetTargetPosition(steering_command_pos);
    brake_motor_->SetTargetPosition(constrain(brake_command_pos, BRAKE_MAX, BRAKE_MIN));
    _throttle_servo_.write(constrain(throttle_command_pos, THROTTLE_IDLE, THROTTLE_MAX));

}


void reset() {
    expected_ignition_status = 0;
    expected_start_status = 0; 
}


//bool checkFailsafes() {
//    // This function will check all failsafes
//    // If it is not safe to drive: the car will be switched to HALT_STATE
//    // Pin 13 will be ON when it is safe to drive, otherwise OFF.
//
//    // The failsafes include: a watchdog timer (i.e. an automatic shutdown if a command hasn't been recieved within 250ms)
//    // Also included is PWM switch from the RC reciever.
//
//    // Note that: the RC PWM switch failsafe should be connected in series with the emergency stop switch at the rear of the car
//
//    Serial.println("Checking failsafes!");
//    bool watchdogValid = ((millis() - last_command_timestamp_) < WATCHDOG_TIMEOUT);
//    bool rcFailsafeValid = sc->message_failsafe >= 1500;
//
//    Serial.print("Dutycycle for failsafe=");
//    Serial.print(sc->message_failsafe);
//
//    Serial.print(", watchdog_valid=");
//    Serial.println(watchdogValid);
//
//    bool safeToDrive = (watchdogValid && rcFailsafeValid);
//
//    if (!safeToDrive and !(current_engine_state_ == OFF_STATE)) {
//        set_engine_state(OFF_STATE, sc);
//    }
//
//    digitalWrite(FAILSAFE_LED_PIN, safeToDrive);
//    return safeToDrive;
//}


