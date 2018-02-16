#include <Servo.h>
#include "motor_controller.h"
#include "serial_command.h"

#include "RoboClaw.h"

// These are the names of the states that the car can be in

// FSM1 : Engine States
#define HALT_STATE              0           // engine is off
#define IGNITION_STATE          1           // turn on ignition relay
#define ENGINE_START_STATE      2           // turn on the start relay for 2 seconds
#define ENGINE_RUNNING_STATE    3           // engine is running and receptive to control

// FSM2 : Control States
#define RC_TELEOP_STATE         4           // recieving signals from RC
#define AI_READY_STATE          5           // signals governed by AI

// FSM3 : Switches ---> fsm is probably overkill
#define NEUTRAL_SWITCH_STATE    6           // switch to put car in neutral
#define AUTONOMOUS_SWITCH_STATE 7           // switch to turn on or off autonomous mode


/************************ ARDUINO PIN DEFINITIONS ********************************/
// PWM input pins from RC Reciever
#define RC_ENGINE_START_PWM_PIN             11 // RC PIN 8
#define RC_IGNITION_PWM_PIN                 10 // RC PIN 7
#define RC_FAILSAFE_PIN    RC_IGNITION_PWM_PIN // RC PIN 7
#define THROTTLE_PWM_PIN                     5 // RC PIN 2
#define STEERING_PWM_PIN                     6 // RC PIN 1
#define THROTTLE_SERVO_PIN                   3 // THROTTLE SERVO MOTOR SIGNAL (OUTPUT)
#define RC_GEAR_SWITCH_PIN                   9 // RC PIN 6

// Digital output pins
#define ENGINE_START_RELAY_PIN  8           // ENGINE START RELAY OUTPUT
#define IGNITION_RELAY_PIN      7           // IGNITION RELAY OUTPUT
#define FAILSAFE_LED_PIN       13           // OUTPUT TO LED ON THE ARDUINO BOARD

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

// Sensitivity values define how responsive the actuators are to a given input


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
#define GEAR_PARK           100
#define GEAR_REVERSE        200
#define GEAR_NEUTRAL        300
#define GEAR_DRIVE          400    

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
#define STEERING_FULL_LEFT_PWM      0 + RC_DEADZONE
#define STEERING_FULL_RIGHT_PWM     1024 - RC_DEADZONE

// Define the limits on Throttle PWM input from the RC Reciever
// In RC Mode: these values will get mapped to THROTTLE_SERVO_ZERO_POSITION and THROTTLE_SERVO_FULL_POSITION respectively
#define THROTTLE_MIN_PWM            0 + RC_DEADZONE
#define THROTTLE_MAX_PWM            1024 - RC_DEADZONE

// Define the limits on Brake PWM input from the RC Receiver
// In RC Mode: these values will get mapped to BRAKE_SERVO_ZERO_POSITION and BRAKE_SERVO_FULL_POSITION respectively
#define BRAKE_MIN_PWM               0 + RC_DEADZONE
#define BRAKE_MAX_PWM               1024 - RC_DEADZONE

// RC stick DEADZONEs are optionally used to adjust the ergonomics of RC control
// 0.0 values will disable them
#define RC_DEADZONE                 50 

// PWM input thresholds on the RC 3-way switch, these will map to gear positions
#define GEAR_PARK_PWM               300
#define GEAR_DRIVE_PWM              600
#define GEAR_REVERSE_PWM            900

// PWM input thresholds on the ignition and start switches, relays will be activated if the thresholds are reached
#define IGNITION_PWM                512           
#define START_PWM                   512

/**********************************************************************************/

// ^^ for each analog input pin map to 10 bit int with read PWM
// this will represent command position

// map command range to adc range recieved from sensor 
// desired position --> map function
// compare with current position (from analog input pins) and perform PID -> set velocity
// map(pulseIn(STEERING_PWM_PIN), )



// If a command from the RC or AI has not been recieved within WATCHDOG_TIMEOUT ms, will be switched to HALT state.
#define WATCHDOG_TIMEOUT 250        // milliseconds

SerialCommand sc;

class Linda{
    public:
        Linda() {
            // Initialise pins
            pinMode(RC_ENGINE_START_PWM_PIN, INPUT);
            pinMode(RC_IGNITION_PWM_PIN, INPUT);
            pinMode(RC_FAILSAFE_PIN, INPUT);
            pinMode(THROTTLE_PWM_PIN, INPUT);
            pinMode(STEERING_PWM_PIN, INPUT);
            pinMode(RC_GEAR_SWITCH_PIN, INPUT);

            // Initialise class member variables
            pinMode(FAILSAFE_LED_PIN, OUTPUT);
            pinMode(ENGINE_START_RELAY_PIN, OUTPUT);
            digitalWrite(ENGINE_START_RELAY_PIN, LOW);

            pinMode(IGNITION_RELAY_PIN, OUTPUT);
            digitalWrite(IGNITION_RELAY_PIN, LOW);
        }

        void init() {
            // initialise roboclaw controllers
            Serial.begin(57600);
            Serial1.begin(9600);
            Serial2.begin(9600);
            RoboClaw _roboclaw1(&Serial1, 9600);            
            RoboClaw _roboclaw2(&Serial2, 9600);
            _roboclaw1.begin(38400);
            _roboclaw2.begin(38400);

            MotorController brake_motor;
            MotorController gear_motor;
            MotorController steer_motor;

            roboclaw1 = &_roboclaw1;
            roboclaw2 = &_roboclaw2;
        }

        void startEngine() {
            // Engine AUTOSTART functionallity
            // Used in AI mode ONLY
            // This will attempt to start the engine, with multiple attempts on failure to do so
        }

        void stopEngine() {
            // Will stop the engine

        }

        bool is_engine_running() {
            // return flag value for now, we have no way of determining this YET
            // Should hook up some kind of sensor and return the digital reading
            return engine_currently_running;
        }

        void process_command(int cmd_x_velocity = 0, int cmd_b_velocity = 0, int cmd_theta = 0, int cmd_gamma = 0) {
            // This is the main function for the RC car control
            // It decides what action to do based on the current state and command input
            // RUNS REPEATEDLY, IT MUST BE CALLED FROM THE MAIN LOOP

            // Note: if in RC_TELEOP_STATE, commanded velocities will be ignored, PWM values will be read instead

            lastCommandTimestamp = millis();
            Serial.println("Processing command");

            // Will be changed into the HALT state if it is not safe to drive.
            checkFailsafes();

            int neutral_switch_pos = int(analogRead(NEUTRAL_CONTROL_SWITCH_PIN));
            int autonomous_switch_pos = int(analogRead(AUTONOMOUS_CONTROL_SWITCH_PIN));

            if (neutral_switch_pos > 750) {
                if (currentStateID != NEUTRAL_SWITCH_STATE) {
                    set_current_state_ID(NEUTRAL_SWITCH_STATE);
                }
                //return;
            } else if (autonomous_switch_pos > 750) {
                if(currentStateID != AUTONOMOUS_SWITCH_STATE) {
                    set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
                }    
                //return;
            }


            if(currentStateID != RC_TELEOP_STATE)
                set_current_state_ID(RC_TELEOP_STATE);
                //return;
            }

            // FSM1 : Engine States
            switch (current_engine_state) {
                case HALT_STATE:
                    if (is_engine_running() == true) {
                        
                    } else {
                    
                    }
                    // engine is off
                    x_velocity = 0.0;
                    theta = cmd_theta;

                    // Once we have slowed to a HALT, lets stop the engine
                    if (abs(x_velocity_sensed) <= 0.1) {
                        stopEngine();
                    }

                    // Check the Control State Switch on dash of car
                    // Neutral - Puts the car gear into neutral
                    // RC - Allows the car to be driven by Remote Controller
                    // Autonomous - car is drive by Nvidia Jetson

                    set_current_state_ID(RC_TELEOP_STATE);
                    break;

                case IGNITION_STATE:
                    // turn on ignition relay

                case ENGINE_START_STATE:
                    // turn on the start relay for 2 seconds
                case ENGINE_RUNNING_STATE:
                    // engine is running and receptive to control
            }
                
            // FSM2 : Control States
            switch (current_control_state) {
                case RC_TELEOP_STATE:
                    // recieving signals from RC

                case AI_READY_STATE:
                    // signals governed by AI

            }

            // FSM3 : Switch States
            switch (current_switch_state) {
                case NEUTRAL_SWITCH_STATE:
                    // switch to put car in neutral

                case AUTONOMOUS_SWITCH_STATE:
                    // switch to turn on or off autonomous mode
            } 

            // State Machine
            switch (currentStateID) {
                case HALT_STATE:
                    // We are in HALT_STATE
                    x_velocity = 0.0;
                    theta = cmd_theta;

                    // Once we have slowed to a HALT, lets stop the engine
                    if (abs(x_velocity_sensed) <= 0.1) {
                        stopEngine();
                    }

                    // Check the Control State Switch on dash of car
                    // Neutral - Puts the car gear into neutral
                    // RC - Allows the car to be driven by Remote Controller
                    // Autonomous - car is drive by Nvidia Jetson

                    if (neutral_switch_pos > 750) {
                        set_current_state_ID(NEUTRAL_SWITCH_STATE);
                        return;
                    } else if (autonomous_switch_pos > 750) {
                        set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
                        return;
                    } else {
                        set_current_state_ID(RC_TELEOP_STATE);
                        return;
                    }

                    break;

                case RC_TELEOP_STATE:
                    // The car is running and     

                {
                    command_position = map(pulseIn(), STEERING_FULL_LEFT_PWM, STEERING_FULL_RIGHT_PWM, STEERING_FULL_LEFT_ADC, STEERING_FULL_RIGHT_ADC)
                    desired_position = analogRead(STEERING_ACTUATOR_POSITION_SENSOR_PIN)
                    break;
                }
                case IGNITION_STATE:
                    // We don't do anything repetedly in ignition state
                    // (only once: on state change)
                    break;

                case ENGINE_START_STATE:
                    // We don't do anything repetedly in ignition state
                    // (only once: on state change)
                    break;

                case AUTONOMOUS_SWITCH_STATE:
                    Serial.println("Autonomous State Selected");

                    // Output the steering angle to the console
                    Serial.print("#");
                    Serial.print(double(analogRead(STEERING_ACTUATOR_POSITION_SENSOR_PIN)));
                    Serial.println("!");

                    //check_ignition_starter();

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

        int get_current_gear_position() {
            
        }

        bool set_engine_state_ID(int newStateID) {
            // This function gets called when a change state is requested
            // Returns true on a successful transition

            // Code blocks within this switch statement are ONLY CALLED ON STATE change
            switch (newStateID) {
                case IGNITION_STATE:
                    // Only allowed to transistion from HALT STATE to IGNITION STATE
                    // FIXME: add state to MotorController class so that we can request the current and last commanded position
                    if (currentStateID == HALT_STATE) {
                        // Ensure that we are in park before engaging ignition
                        if (abs(gear_motor->GetCurrentPosition() - PARK_GEAR_POSITION) > GEAR_FEEDBACK_TOLERENCE) {
                            Serial.println("Ignition command received, not in park.");

                            //Put the car into park
                            // current_gear_position = PARK_GEAR_POSITION;
                            /* DISABLE
                            gear_motor->SetTargetPosition(current_gear_position);
                            */

                            return false;
                        } else {
                            // Once the car is in park, we can start the ignition
                            Serial.println("Car in park, turning on ignition");
                            digitalWrite(IGNITION_RELAY_PIN, HIGH);
                            main_relay_on = 1;
                            return true;
                        }
                    }
                    break;
                    
                case ENGINE_START_STATE:
                    // Only transistion to ENGINE_START_STATE if currently in ignition state
                    if (currentStateID == IGNITION_STATE) {
                        startEngine();
                    } 
                    break;
                
                case HALT_STATE:
                    // Do nothing on transition into HALT
                    break;
                
                case RC_TELEOP_STATE:
                    // Do nothing on transition into RC_TELEOP
                    break;
                
                case AI_READY_STATE:
                    // Do nothing on transition into AI
                    break;
                
                case NEUTRAL_SWITCH_STATE:
                    // Do nothing on transition into NEUTRAL_SWITCH_STATE
                    break;
                
                case AUTONOMOUS_SWITCH_STATE:
                    // Do nothing on transition into AUTONOMOUS_SWITCH_STATE
                    break;
            }

            Serial.print("Changing state to: ");
            Serial.println(newStateID);

            // Change to desired state
            currentStateID = newStateID;
            return true;
        }

        bool set_control_state_ID(int newStateID) {
            
        }

        int getcurrentStateID() {
            // Return the ID of the state that we are currently in
            return currentStateID;
        }

        float read_pwm_value(int pwm_pin) {
            // Read a value from a PWM input
            float pwm_value = pulseIn(pwm_pin, HIGH);
            return pwm_value;
        }

        float convert_jetson_serial(int jetson_min, int jetson_max) {

        }

        bool checkFailsafes() {
            // This function will check all failsafes
            // If it is not safe to drive: the car will be switched to HALT_STATE
            // Pin 13 will be ON when it is safe to drive, otherwise OFF.

            // The failsafes include: a watchdog timer (i.e. an automatic shutdown if a command hasn't been recieved within 250ms)
            // Also included is PWM switch from the RC reciever.

            // Note that: the RC PWM switch failsafe should be connected in series with the emergency stop switch at the rear of the car

            Serial.println("Checking failsafes!");
            bool watchdogValid = ((millis() - lastCommandTimestamp) < WATCHDOG_TIMEOUT);
            bool rcFailsafeValid = read_pwm_value(RC_FAILSAFE_PIN) >= 0.5;

            Serial.print("Dutycycle for failsafe=");
            Serial.print(read_pwm_value(RC_FAILSAFE_PIN));

            Serial.print(", watchdog_valid=");
            Serial.println(watchdogValid);

            bool safeToDrive = (watchdogValid && rcFailsafeValid);

            if (!safeToDrive) {
                set_current_state_ID(HALT_STATE);
            }

            digitalWrite(FAILSAFE_LED_PIN, safeToDrive);
            return safeToDrive;
        }

        void send_throttle_command(int throttle_command) {
            // Send analalogue signal to the throttle servo
            analogWrite(THROTTLE_SERVO_PIN, throttle_command)
        }

        void check_ignition_starter() {
            double ignition_val = read_pwm_value(RC_IGNITION_PWM_PIN);
            // Ignition and Starter Motor Control

            double starter_val  = read_pwm_value(RC_ENGINE_START_PWM_PIN);

            if (ignition_val > RC_DUTY_THRESH_IGNITION) {
                // 
            } else {
                Serial.println("STOPPING ENGINE!!!!!");
                stopEngine();
                return;
            }
        }

    private:
        int currentStateID;
        long lastCommandTimestamp;
        double theta;
        double x_velocity;
        double x_velocity_sensed;
        int current_gear_position;
        bool ai_enabled;
        bool main_relay_on;
        bool engine_currently_running;
        Servo throttle_servo;

        MotorController brake_motor;
        MotorController gear_motor;
        MotorController steer_motor;

        RoboClaw* roboclaw1;
        RoboClaw* roboclaw2;

};
