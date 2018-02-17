#include <Servo.h>
#include "motor_controller.h"
#include "serial_command.h"

#include "RoboClaw.h"

// These are the names of the states that the car can be in

// FSM1 : Engine States
#define OFF_STATE               0           // engine is off
#define IGNITION_STATE          1           // turn on ignition relay
#define ENGINE_START_STATE      2           // turn on the start relay for 2 seconds
#define ENGINE_RUNNING_STATE    3           // engine is running and receptive to control
#define GEAR_CHANGE_STATE       4           // gear is being changed

// FSM2 : Control States
#define RC_TELEOP_STATE         5           // recieving signals from RC
#define AI_READY_STATE          6           // signals governed by AI

// FSM3 : Switches ---> fsm is probably overkill
#define NEUTRAL_SWITCH_STATE    7           // switch to put car in neutral
#define AUTONOMOUS_SWITCH_STATE 8           // switch to turn on or off autonomous mode

/************************ ARDUINO PIN DEFINITIONS ********************************/
// PWM input pins from RC Reciever
// #define RC_IGNITION_SERIAL_PIN                 10          // RC PIN 7     // this should be a button
// #define RC_ENGINE_START_SERIAL_PIN             11          // RC PIN 8     // this should be a button 
#define RC_FAILSAFE_PIN                           11       // RC PIN 7     // this should be a button
// #define THROTTLE_SERIAL_PIN                     5          // RC PIN 2
// #define STEERING_SERIAL_PIN                     6          // RC PIN 1
// #define GEAR_PIN_SERIAL                   9          // RC PIN 6

// Digital output pins
#define ENGINE_START_RELAY_PIN               8          // ENGINE START RELAY OUTPUT
#define IGNITION_RELAY_PIN                   7          // IGNITION RELAY OUTPUT
#define FAILSAFE_LED_PIN                    13          // OUTPUT TO LED ON THE ARDUINO BOARD
#define THROTTLE_SERVO_PIN                   3          // THROTTLE SERVO MOTOR SIGNAL (OUTPUT)

// Analog input pins
#define NEUTRAL_CONTROL_SWITCH_PIN              A0  // state of neutral control switch (on | off)
#define AUTONOMOUS_CONTROL_SWITCH_PIN           A1  // state of analogue control switch (on | off)
#define BRAKE_ACTUATOR_POSITION_SENSOR_PIN      A3  // brake actuator position (10 bit analog signal)
#define GEAR_ACTUATOR_POSITION_SENSOR_PIN       A4  // gear actuator position (10 bit analog signal)
#define STEERING_ACTUATOR_POSITION_SENSOR_PIN   A5  // steering actuator position (10 bit analog signal)

// Motor driver Pins (UART Serial)

/************************ DRIVE CONTROL DEFINEs **********************************/
/* These parameters adjust how the car will behave.
   They will need to be changed according to the particular vehicle.
   However, most values provided should be fairly suitable for  configurations.
*/

#define ENGINE_STARTER_CRANKING_TIME 1400

// Sensitivity values define how responsive the actuators are to a given input
#define AUTO_BRAKE_SENSITIVITY 0.0 // 0.0 disables this
#define AUTO_THROTTLE_SENSITIVITY 1.0
#define AUTO_STEERING_SENSITIVITY 0.0

#define THROTTLE_SENSITIVITY 0.1

// How much delta_v (from t-1 to t) will trigger the brake engage
// MUST BE NEGATIVE! 
#define AUTO_BRAKE_ENGAGE_THRESH -5

// Max power applies a constraint to the driver output speed.
// Important note: set these low for testing so you don't destroy anything
#define BRAKE_MAX_POWER         0               // FIND OUT WHAT THIS IS
#define GEAR_MAX_POWER          0               // FIND OUT WHAT THIS IS
#define STEERING_MAX_POWER      0               // FIND OUT WHAT THIS IS

// PID values for each motor driver
// Important note: These values are optional

// Velocity PID coefficients.
#define BRAKE_Kp            0.5
#define BRAKE_Ki            0
#define BRAKE_Kd            0

#define GEAR_Kp             0.5
#define GEARK_Ki            0
#define GEAR_Kd             0

#define STEERING_Kp         0.5
#define STEERING_Ki         0
#define STEERING_Kd         0

#define QPPS                44000           // FIND OUT WHAT THIS IS 

// Gear positions define where the gear actuator has to travel to engage a specified gear
#define GEAR_PARK_ADC           0
#define GEAR_REVERSE_ADC        200
#define GEAR_NEUTRAL_ADC        300
#define GEAR_DRIVE_ADC          400    

// How close should the analog feedback reading be to the actual position, as confirmation that we are actually in the specified gear
// An absolute difference threshold
#define GEAR_FEEDBACK_TOLERENCE      50

// ALLOWABLE RANGE ON INPUTS FROM ADC:
// Define the allowable range of motion for the brake actuator
#define BRAKE_MIN_ADC               200     // brake not depressed
#define BRAKE_MAX_ADC               900     // maximum brake depression

// Define the allowable range of motion for the throttle servo actuator
#define THROTTLE_MIN_ADC            200     // throttle not depressed 
#define THROTTLE_MAX_ADC            900     // maximum throttle depression

// Define the allowable range of motion for the steering actuator
#define STEERING_FULL_LEFT_ADC      200     // full left lock
#define STEERING_FULL_RIGHT_ADC     900     // full right lock


// ALLOWABLE RANGE ON OUTPUTS TO MOTOR CONTROLLER:
// Define the limits on Steering PWM input from the RC Reciever
// In RC Mode: these values will get mapped to STEERING_FULL_LEFT and STEERING_FULL_RIGHT respectively
#define STEERING_FULL_LEFT_SERIAL      1100 //+ RC_DEADZONE
#define STEERING_FULL_RIGHT_SERIAL     1900 //- RC_DEADZONE

// Define the limits on Throttle PWM input from the RC Reciever
// In RC Mode: these values will get mapped to THROTTLE_SERVO_ZERO_POSITION and THROTTLE_SERVO_FULL_POSITION respectively
#define THROTTLE_MIN_SERIAL            1100 //+ RC_DEADZONE
#define THROTTLE_MAX_SERIAL            1900 //- RC_DEADZONE

// Define the limits on Brake PWM input from the RC Receiver
// In RC Mode: these values will get mapped to BRAKE_SERVO_ZERO_POSITION and BRAKE_SERVO_FULL_POSITION respectively
#define BRAKE_MIN_SERIAL               1100 //+ RC_DEADZONE
#define BRAKE_MAX_SERIAL               1900 //- RC_DEADZONE

// RC stick DEADZONEs are optionally used to adjust the ergonomics of RC control
// 0.0 values will disable them
#define RC_DEADZONE                 0 //Don't worry about DEADZONEs, set to zero to ignore!

// PWM input thresholds on the RC 3-way switch, these will map to gear positions
#define GEAR_PARK_SERIAL               300
#define GEAR_REVERSE_SERIAL            600
#define GEAR_DRIVE_SERIAL              900

// PWM input thresholds on the ignition and start switches, relays will be activated if the thresholds are reached
#define IGNITION_SERIAL                512           
#define START_SERIAL                   512

// Motor IDs
#define BRAKE       1
#define GEAR        2
#define STEERING    3

/**********************************************************************************/

// ^^ for each analog input pin map to 10 bit int with read PWM
// this will represent command position

// If a command from the RC or AI has not been recieved within WATCHDOG_TIMEOUT ms, will be switched to HALT state.
#define WATCHDOG_TIMEOUT 250        // milliseconds

static inline int8_t sgn(int val)
// Get the sign of an integer
{
    if (val < 0)
        return -1;
    if (val == 0)
        return 0;
    return 1;
}

class FireNugget {
    public:
        FireNugget() {
            // Initialise pins
            pinMode(RC_FAILSAFE_PIN, INPUT);

            // Initialise class member variables
            pinMode(FAILSAFE_LED_PIN, OUTPUT);
            pinMode(ENGINE_START_RELAY_PIN, OUTPUT);
            digitalWrite(ENGINE_START_RELAY_PIN, LOW);

            pinMode(IGNITION_RELAY_PIN, OUTPUT);
            digitalWrite(IGNITION_RELAY_PIN, LOW);
        }

        void Init() {
            // initialise roboclaw controllers
            Serial.begin(115200);

            // Don't need these anymore 
            //Serial1.begin(9600);
            //Serial2.begin(9600);

            RoboClaw _roboclaw1(&Serial1, 100);
            RoboClaw _roboclaw2(&Serial2, 100);
            _roboclaw1.begin(38400);
            _roboclaw2.begin(38400);

            roboclaw1_ = &_roboclaw1;
            roboclaw2_ = &_roboclaw2;

            // Initialise the servo motor
            throttle_servo_.attach(THROTTLE_SERVO_PIN);
            delay(5);
            throttle_servo_.write(0);

            // TODO: remove integrated roboclaw PID controllers (just run speed PID control on the arduino)
             brake_motor_ = new MotorController(BRAKE, roboclaw1_, BRAKE_ACTUATOR_POSITION_SENSOR_PIN, 
                                        BRAKE_MIN_ADC, BRAKE_MAX_ADC, 0.5, false, false,
                                        0.5, 0.0, 0.0, 44000);

            gear_motor_ = new MotorController(GEAR, roboclaw1_, GEAR_ACTUATOR_POSITION_SENSOR_PIN, 
                                        GEAR_PARK_ADC, GEAR_DRIVE_ADC, 0.5, false, true,
                                        0.5, 0.0, 0.0, 44000);

            steer_motor_ = new MotorController(STEERING, roboclaw2_, STEERING_ACTUATOR_POSITION_SENSOR_PIN, 
                                        STEERING_FULL_LEFT_ADC, STEERING_FULL_RIGHT_ADC, 0.5, false, false, 
                                        0.5, 0.0, 0.0, 44000);
        }

        void startEngine() {
            // Engine AUTOSTART functionallity
            // Used in AI mode ONLY
            digitalWrite(ENGINE_START_RELAY_PIN, HIGH);
            delay(ENGINE_STARTER_CRANKING_TIME);
            digitalWrite(ENGINE_START_RELAY_PIN, LOW);
        }

        void stopEngine() {
            // Will stop the engine
            digitalWrite(IGNITION_RELAY_PIN, LOW);
        }

        void process_command(int cmd_x_velocity , int cmd_theta ) {
            // This is the main function for the RC car control
            // It decides what action to do based on the current state and command input
            // RUNS REPEATEDLY, IT MUST BE CALLED FROM THE MAIN LOOP

            // Note: if in RC_TELEOP_STATE, commanded velocities will be ignored, PWM values will be read instead

            last_command_timestamp_ = millis();
            Serial.println("Processing command");

            // Will be changed into the HALT state if it is not safe to drive.
            checkFailsafes();

            // FSM1 : Engine States
            switch (current_engine_state_) {
                case OFF_STATE:
                    // engine is off
                    if (sc.message_ignition > 512) {               // ingition signal is recieved
                        set_engine_state(IGNITION_STATE);
                    }
                    break;

                case IGNITION_STATE:
                    // ignition relay on
                    if (sc.message_engine_start > 512) {           // start signal is recieved
                        delay(2000);
                        set_engine_state(ENGINE_START_STATE);
                    }
                    break;

                case ENGINE_START_STATE:
                    // this will only run once
                    startEngine();                                  // assume successful
                    set_engine_state(ENGINE_RUNNING_STATE);
                    break;

                case ENGINE_RUNNING_STATE:
                    // engine is running and receptive to control
                    if (sc.message_engine_start > 512) {            // off signal is recieved
                        set_engine_state(OFF_STATE);
                    } else if (sc.message_gear > 512) {                                  // gear change is requested
                        set_engine_state((GEAR_CHANGE_STATE - current_gear_pos_) > 50);
                    }
                    break;

                case GEAR_CHANGE_STATE:
                    // gear has been changed
                    set_engine_state(ENGINE_RUNNING_STATE);
                    break;
            }
                
            // FSM2 : Control States
            switch (current_control_state_) {
                case RC_TELEOP_STATE:
                    // recieving signals from RC and send to motor controller PID and throttle serve
                    
                    // Receive serial command from XBox Controller and parse
                    sc.ReadData();
                    if ( sc.message_type == 1 ) {
                        Serial.println("Recieving");
                        // to-do: parse the message from the serial

                        // convert velocity to throttle / brake
                        if (sc.message_velocity > 512) {
                            unmapped_brake_ = BRAKE_MIN_SERIAL;            // brakes off
                            unmapped_throttle_ = sc.message_velocity;
                        } else {        // not considering reverse right now just braking for < 512
                            unmapped_brake_ = BRAKE_MAX_SERIAL;            // brakes on (MAX)
                            unmapped_throttle_ = THROTTLE_MIN_SERIAL;
                        }
                    } else if ( sc.message_type == 2 ) {
                        Serial.println("Jetson Command received in RC Mode");
                        return;
                    } else {
                        Serial.println("No command from Serial received");
                        return;
                    }

                    // BRAKE
                    brake_command_pos_ = map(unmapped_brake_, BRAKE_MIN_SERIAL, BRAKE_MAX_SERIAL, BRAKE_MIN_ADC, BRAKE_MAX_ADC);
                    brake_motor_->SetTargetPosition(brake_command_pos_);
                    
                    // GEAR
                    gear_command_pos_ = map(sc.message_gear, GEAR_PARK_SERIAL, GEAR_DRIVE_SERIAL, GEAR_PARK_ADC, GEAR_DRIVE_ADC);
                    gear_motor_->SetTargetPosition(gear_command_pos_);

                    // STEERING
                    steering_command_pos_ = map(sc.message_steering, STEERING_FULL_LEFT_SERIAL, STEERING_FULL_RIGHT_SERIAL, STEERING_FULL_LEFT_ADC, STEERING_FULL_RIGHT_ADC);
                    steer_motor_->SetTargetPosition(steering_command_pos_);

                    // THROTTLE
                    throttle_command_pos_ = map(unmapped_throttle_, THROTTLE_MIN_SERIAL, THROTTLE_MAX_SERIAL, THROTTLE_MIN_ADC, THROTTLE_MAX_ADC);
                    throttle_servo_.write(int(throttle_command_pos_));

                case AI_READY_STATE:
                    // signals governed by AI
                    // CURRENTLY AI_READY_STATE ONLY SUPPORTS FORWARD GEAR (i.e. no reverse)!!!!

                    throttle_command_pos_ = AUTO_THROTTLE_SENSITIVITY * abs(cmd_x_velocity);
                    steering_command_pos_ = AUTO_STEERING_SENSITIVITY * cmd_theta;
                    brake_command_pos_ = 0.0;

                    // When going forward: we want to brake if the current velocity command is AUTO_BRAKE_ENGAGE_THRESH less than the previous velocity command
                    // Thus, we find the delta between the current command velocity and the previous command velocity
                    // TODO: Replace ai_previous_velocity_ with actual feedback measurements from the GPS
                    int delta_v = cmd_x_velocity - ai_previous_velocity_;

                    // Check if we want to change gear
                    if (sgn(cmd_x_velocity) != sgn(ai_previous_velocity_)) {
                        // If so, which gear do we want?
                        switch (sgn(cmd_x_velocity)) {
                            case 1:
                                gear_command_pos_ = GEAR_DRIVE_ADC;
                                break;
                            case 0:
                                gear_command_pos_ = GEAR_PARK_ADC;
                                break;
                            case -1:
                                gear_command_pos_ = GEAR_REVERSE_ADC;
                                break;
                        }
                        set_engine_state(GEAR_CHANGE_STATE);
                    }

                    if (delta_v < AUTO_BRAKE_ENGAGE_THRESH) {
                        // Don't apply throttle when applying brakes
                        brake_command_pos_ = AUTO_BRAKE_SENSITIVITY * delta_v;
                        throttle_command_pos_ = 0.0;
                    }

                    // Store previous velocity state
                    ai_previous_velocity_ = cmd_x_velocity;

                    brake_motor_->SetTargetPosition(brake_command_pos_);
                    steer_motor_->SetTargetPosition(steering_command_pos_);
                    throttle_servo_.write(int(throttle_command_pos_));
            }

            // FSM3 : Switch States
            switch (current_switch_state_) {
                case NEUTRAL_SWITCH_STATE:
                    // switch to put car in neutral in ON

                case AUTONOMOUS_SWITCH_STATE:
                    Serial.println("Autonomous State Selected");

                    // Output the steering angle to the console
                    Serial.print("#");
                    Serial.print(double(analogRead(STEERING_ACTUATOR_POSITION_SENSOR_PIN)));
                    Serial.println("!");

                    // Receive serial command from Jetson and parse
                    sc.ReadData();
                    //if(sc.message_time - timeDiff > 500)
                    if ( sc.message_type != -1 ) {
                        // to-do: parse the message from the serial

                    } else {
                        Serial.println("No command from Jetson received");
                        return;
                    }

                    // Send command to the steering controller
                    // Send command to the brake motor controller
                    // Send command to the throttle controller
                    // Send command to the gear controller

                    break;
            } 
        }

        bool set_engine_state(int new_engine_state) {
            switch (new_engine_state) {
                case OFF_STATE:
                    if (current_engine_state_ == ENGINE_RUNNING_STATE) {
                        
                    }
                    break;
                
                case IGNITION_STATE:
                    if (current_engine_state_ == OFF_STATE) {
                        // Ensure that we are in park before engaging ignition
                        // CHECKS
                        // -- car in park
                        if (abs(gear_motor_->get_current_pos() - GEAR_PARK_ADC) > GEAR_FEEDBACK_TOLERENCE) {
                            Serial.println("Ignition command received, not in park.");
                            return false;
                        } else {
                            // Once the car is in park, we can start the ignition
                            Serial.println("Car in park, turning on ignition");
                            digitalWrite(IGNITION_RELAY_PIN, HIGH);
                            main_relay_on_ = 1;
                            return true;
                        }
                    }
                    break;
                
                case ENGINE_START_STATE:
                    // may want some additional logic here

                    break;

                case ENGINE_RUNNING_STATE:

                    break;

                case GEAR_CHANGE_STATE:

                    // set throttle to zero
                    throttle_servo_.write(0);
                    delay(300);
                    // engage full brake
                    brake_motor_->SetTargetPosition(1000);
                    // wait for the car to stop
                    delay(3500);
                    Serial.println("Changing Gear to: " + String(gear_command_pos_));

                    // change gear
                    gear_motor_->SetTargetPosition(gear_command_pos_);
                    break;

                return true;
            }

            Serial.print("Changing state to: ");
            Serial.println(new_engine_state);

            // Change to desired state
            current_engine_state_ = new_engine_state;
            return true;
        }

        bool set_control_state(int new_control_state) {
            Serial.print("Changing state to: ");
            Serial.println(new_control_state);

            // Change to desired state
            current_control_state_ = new_control_state;
            return true;
        }

        unsigned long read_pwm_value(int pwm_pin) {
            // Read a value from a PWM input
            unsigned long pwm_value = pulseIn(pwm_pin, HIGH);
            return pwm_value;
        }

        bool checkFailsafes() {
            // This function will check all failsafes
            // If it is not safe to drive: the car will be switched to HALT_STATE
            // Pin 13 will be ON when it is safe to drive, otherwise OFF.

            // The failsafes include: a watchdog timer (i.e. an automatic shutdown if a command hasn't been recieved within 250ms)
            // Also included is PWM switch from the RC reciever.

            // Note that: the RC PWM switch failsafe should be connected in series with the emergency stop switch at the rear of the car

            Serial.println("Checking failsafes!");
            bool watchdogValid = ((millis() - last_command_timestamp_) < WATCHDOG_TIMEOUT);
            bool rcFailsafeValid = read_pwm_value(RC_FAILSAFE_PIN) >= 0.5;

            Serial.print("Dutycycle for failsafe=");
            Serial.print(read_pwm_value(RC_FAILSAFE_PIN));

            Serial.print(", watchdog_valid=");
            Serial.println(watchdogValid);

            bool safeToDrive = (watchdogValid && rcFailsafeValid);

            if (!safeToDrive) {
                set_engine_state(OFF_STATE);
            }

            digitalWrite(FAILSAFE_LED_PIN, safeToDrive);
            return safeToDrive;
        }

    private:
        int current_engine_state_;       // current state in engine FSM
        int current_control_state_;      // current state in control FSM
        int current_switch_state_;


        long unmapped_brake_;
        long unmapped_throttle_;
        
        // command positions
        long brake_command_pos_;
        long gear_command_pos_;
        long steering_command_pos_;
        long throttle_command_pos_;

        long last_command_timestamp_;
        bool main_relay_on_;
        // bool engine_currently_running;

        int current_gear_pos_;

        int ai_previous_velocity_;
        
        SerialCommand sc;
        
        Servo throttle_servo_;
        MotorController *brake_motor_;
        MotorController *gear_motor_;
        MotorController *steer_motor_;

        RoboClaw* roboclaw1_;
        RoboClaw* roboclaw2_;

};
