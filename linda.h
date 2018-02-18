#include <Servo.h>
#include "motor_controller.h"
//#include "serial_command.h"
#include "pwm_command.h"

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
#define RC_PIN_1                      22          // RC PIN 1     // this should be a button 
#define RC_PIN_2                      24          // RC PIN 2     // this should be a button
#define RC_PIN_3                      26          // RC PIN 3
#define RC_PIN_4                      28          // RC PIN 4
#define RC_PIN_5                      30          // RC PIN 5
#define RC_PIN_6                      32          // RC PIN 6
#define RC_PIN_7                      34          // RC PIN 7
#define RC_PIN_8                      36          // RC PIN 8




// Digital output pins
#define ENGINE_START_RELAY_PIN               8          // ENGINE START RELAY OUTPUT
#define IGNITION_RELAY_PIN                   7          // IGNITION RELAY OUTPUT
#define FAILSAFE_LED_PIN                    13          // OUTPUT TO LED ON THE ARDUINO BOARD
#define THROTTLE_SERVO_PIN                   3          // THROTTLE SERVO MOTOR SIGNAL (OUTPUT)

#define BRAKE_PWM_OUTPUT                     9
#define GEAR_PWM_OUTPUT                     10
#define STEERING_PWM_OUTPUT                 11

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
#define BRAKE_SENSITIVITY        0.9     // almost fully compressed brakes
#define THROTTLE_SENSITIVITY     0.5     // half of max throttle
#define STEERING_SENSITIVITY     0.2     // 0.2 of steering range

// How much delta_v (from t-1 to t) will trigger the brake engage
// MUST BE NEGATIVE! 
#define AUTO_BRAKE_ENGAGE_THRESH      -5

// Max power applies a constraint to the driver output speed.
// Important note: set these low for testing so you don't destroy anything
#define GEAR_SERVO_MIN_POWER        0
#define GEAR_SERVO_MAX_POWER        90

#define BRAKE_SERVO_MIN_POWER       0
#define BRAKE_SERVO_MAX_POWER       90

#define STEERING_SERVO_MIN_POWER    0
#define STEERING_SERVO_MAX_POWER    90

// PID values for each motor driver
// Important note: These values are optional

// Velocity PID coefficients.
#define BRAKE_Kp            -0.357    # MAX
#define BRAKE_Ki            0
#define BRAKE_Kd            0

#define GEAR_Kp             0.136    # MAX
#define GEARK_Ki            0
#define GEAR_Kd             0

#define STEERING_Kp         0.456    # MAX
#define STEERING_Ki         0
#define STEERING_Kd         0

// Gear positions define where the gear actuator has to travel to engage a specified gear
#define GEAR_PARK_ADC                 759
#define GEAR_REVERSE_ADC              680
#define GEAR_NEUTRAL_ADC              616
#define GEAR_DRIVE_ADC                562    

// How close should the analog feedback reading be to the actual position, as confirmation that we are actually in the specified gear
// An absolute difference threshold
#define GEAR_FEEDBACK_TOLERENCE       10

// ALLOWABLE RANGE ON INPUTS FROM ADC:
// Define the allowable range of motion for the brake actuator
#define BRAKE_MIN_ADC                 635     // brake not depressed
#define BRAKE_MAX_ADC                 383     // maximum brake depression

// Define the allowable range of motion for the throttle servo actuator
#define THROTTLE_MIN_ADC              0       // throttle not depressed 
#define THROTTLE_MAX_ADC              30      // maximum throttle depression

// Define the allowable range of motion for the steering actuator
#define STEERING_FULL_LEFT_ADC        164     // full left lock
#define STEERING_CENTRE_ADC           511     // steering in centre
#define STEERING_FULL_RIGHT_ADC       823     // full right lock


// Define the limits on Steering PWM input from the RC Reciever
// In RC Mode: these values will get mapped to STEERING_FULL_LEFT and STEERING_FULL_RIGHT respectively
 #define STEERING_FULL_LEFT_PWM      1852
 #define STEERING_FULL_RIGHT_PWM     1009
     
 #define THROTTLE_MIN_PWM            1487
 #define THROTTLE_MAX_PWM            1911
 
 #define BRAKE_MIN_PWM               1487
 #define BRAKE_MAX_PWM               1068

 // RC stick DEADZONEs are optionally used to adjust the ergonomics of RC control
// 0.0 values will disable them
#define RC_DEADZONE                     0 //Don't worry about DEADZONEs, set to zero to ignore!

#define GEAR_REVERSE_PWM         1065
#define GEAR_NEUTRAL_PWM         1345
#define GEAR_DRIVE_PWM           1617

#define IGNITION_PWM                1500  // >           
#define START_PWM                   1500  // >



// Define the limits on Throttle PWM input from the RC Reciever
// In RC Mode: these values will get mapped to THROTTLE_SERVO_ZERO_POSITION and THROTTLE_SERVO_FULL_POSITION respectively


//#define STEERING_FULL_LEFT_SERIAL     -45   //+ RC_DEADZONE
//#define STEERING_FULL_RIGHT_SERIAL     45   //- RC_DEADZONE
//
//#define THROTTLE_MIN_SERIAL          -100     // Reverse full force
//#define THROTTLE_MIDDLE_SERIAL          0     // Toggle unpressed
//#define THROTTLE_MAX_SERIAL           100     // Forward full force
//
//// PWM input thresholds on the RC 3-way switch, these will map to gear positions
//#define GEAR_PARK_SERIAL                0
//#define GEAR_REVERSE_SERIAL            -1
//#define GEAR_DRIVE_SERIAL               1
//
//// PWM input thresholds on the ignition and start switches, relays will be activated if the thresholds are reached
//#define IGNITION_SERIAL                1           
//#define START_SERIAL                   1

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
            pinMode(RC_PIN_1, INPUT);
            pinMode(RC_PIN_2, INPUT);
            pinMode(RC_PIN_6, INPUT);
            pinMode(RC_PIN_7, INPUT);
            pinMode(RC_PIN_8, INPUT);

            // Initialise class member variables
            pinMode(FAILSAFE_LED_PIN, OUTPUT);
            pinMode(ENGINE_START_RELAY_PIN, OUTPUT);
            digitalWrite(ENGINE_START_RELAY_PIN, LOW);

            pinMode(IGNITION_RELAY_PIN, OUTPUT);
            digitalWrite(IGNITION_RELAY_PIN, LOW);
        }

        void Init() {
            
            // initialise servos controllers
            Serial.begin(9600);

            Servo _brake_servo;
            Servo _gear_servo;
            Servo _steering_servo;

            _brake_servo.attach(BRAKE_PWM_OUTPUT);
            _gear_servo.attach(GEAR_PWM_OUTPUT);
            _steering_servo.attach(STEERING_PWM_OUTPUT);

            brake_servo_ = &_brake_servo;
            gear_servo_  = &_gear_servo;
            steering_servo_ = &_steering_servo;

            // Initialise the servo motor
            throttle_servo_.attach(THROTTLE_SERVO_PIN);
            delay(5);
            throttle_servo_.write(0);

            brake_motor_ = new MotorController( brake_servo_, BRAKE_ACTUATOR_POSITION_SENSOR_PIN, 
                                                BRAKE_MIN_ADC, BRAKE_MAX_ADC,
                                                BRAKE_SERVO_MIN_POWER, BRAKE_SERVO_MAX_POWER,
                                                BRAKE_SENSITIVITY,
                                                0.5, 0.0, 0.0);

            gear_motor_ = new MotorController(  gear_servo_, GEAR_ACTUATOR_POSITION_SENSOR_PIN, 
                                                GEAR_PARK_ADC, GEAR_DRIVE_ADC,
                                                GEAR_SERVO_MIN_POWER, GEAR_SERVO_MAX_POWER,
                                                1,
                                                0.5, 0.0, 0.0);

            steer_motor_ = new MotorController( steering_servo_, STEERING_ACTUATOR_POSITION_SENSOR_PIN, 
                                                STEERING_FULL_LEFT_ADC, STEERING_FULL_RIGHT_ADC,
                                                STEERING_SERVO_MIN_POWER, STEERING_SERVO_MAX_POWER,
                                                STEERING_SENSITIVITY,
                                                0.5, 0.0, 0.0);

            current_engine_state_     = OFF_STATE;
            current_control_state_    = RC_TELEOP_STATE;
            current_switch_state_     = NEUTRAL_SWITCH_STATE;
            current_gear_pos_         = GEAR_NEUTRAL_ADC;

            Serial.println("\nFIRE NUGGET INITIALISED");
            Serial.print("Engine State: ");
            Serial.print(current_engine_state_);
            Serial.print("\nControl State: ");
            Serial.print(current_control_state_);
            Serial.print("\nCurrent Gear: ");
            Serial.print(current_gear_pos_);
            Serial.print("\n");
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

        void process_command(PWMCommand* sc) {
            // This is the main function for the RC car control
            // It decides what action to do based on the current state and command input
            // RUNS REPEATEDLY, IT MUST BE CALLED FROM THE MAIN LOOP

            // Note: if in RC_TELEOP_STATE, commanded velocities will be ignored, PWM values will be read instead

            last_command_timestamp_ = millis();
            Serial.print("\n");
//            Serial.println("Processing command");
            Serial.print("Engine State: ");
            Serial.print(current_engine_state_);
            Serial.print(", Control State: ");
            Serial.print(current_control_state_);
            Serial.print(", Current Gear: ");
            Serial.print(current_gear_pos_);
            Serial.println("");
            // Will be changed into the HALT state if it is not safe to drive.
//            checkFailsafes(sc);

            // FSM1 : Engine States
            switch (current_engine_state_) {
                case OFF_STATE:
                    // engine is off
                    if (sc->message_ignition > IGNITION_PWM) {               // ingition signal is recieved
                        set_engine_state(IGNITION_STATE, sc);
                    }
                    break;

                case IGNITION_STATE:
                    // ignition relay on
                    if (sc->message_engine_start > START_PWM) {              // start signal is recieved
                        set_engine_state(ENGINE_START_STATE, sc);
                    } else if (sc->message_ignition < IGNITION_PWM) {
                        set_engine_state(OFF_STATE, sc);
                    }
                    break;

                case ENGINE_START_STATE:
                    // this will only run once
                    if (sc->message_ignition < IGNITION_PWM) {
                        set_engine_state(OFF_STATE, sc);
                    } else {
                        set_engine_state(ENGINE_RUNNING_STATE, sc);
                    }
                    break;

                case ENGINE_RUNNING_STATE:
                    // engine is running and receptive to control
                    if (sc->message_ignition < IGNITION_PWM) {            // off signal is recieved
                        set_engine_state(OFF_STATE, sc);
                    } else if (sc->message_gear != current_gear_pos_) {                                  // gear change is requested
                        set_engine_state(GEAR_CHANGE_STATE, sc);
                    }
                    break;

                case GEAR_CHANGE_STATE:
                    // gear has been changed
                    set_engine_state(ENGINE_RUNNING_STATE, sc);
                    break;
            }
                
            // FSM2 : Control States
            switch (current_control_state_) {
                case RC_TELEOP_STATE:
                    // recieving signals from RC and send to motor controller PID and throttle serve
                    
                    // Receive serial command from XBox Controller and parse
                    if ( sc->message_type == 1 ) {
//                        Serial.println("Recieving");
                        // if car is off and ignition is pressed, change to ignition

                        // if car is in ignition and 
                        
                        
                        // convert velocity to throttle / brake
//                        if (sc->message_velocity > 10) {                      // anything forward
//                            unmapped_brake_       = 0;                        // brakes off
//                            unmapped_throttle_    = sc->message_velocity;
//                        } else if (sc->message_velocity < -10) {              // hit the brakes
//                            unmapped_brake_       = sc->message_velocity;
//                            unmapped_throttle_    = 0;
//                        } else {
//                            unmapped_brake_       = 0;
//                            unmapped_throttle_    = 0;  
//                        }

                        
                    } else if ( sc->message_type == 2 ) {
                        Serial.println("Jetson Command received in RC Mode");
                        return;
                    } else {
                        Serial.println("No command from Serial received");
                        return;
                    }

                    // BRAKE
//                    Serial.println("BRAKE VALUE:");
                    brake_command_pos_ = map(unmapped_brake_, BRAKE_MIN_PWM, BRAKE_MAX_PWM, BRAKE_MIN_ADC, BRAKE_MAX_ADC);
                    brake_motor_->SetTargetPosition(brake_command_pos_);

                    // STEERING
//                    Serial.println("STEERING VALUE:");
                    steering_command_pos_ = map(sc->message_steering, STEERING_FULL_LEFT_PWM, STEERING_FULL_RIGHT_PWM, STEERING_FULL_LEFT_ADC, STEERING_FULL_RIGHT_ADC);
                    steer_motor_->SetTargetPosition(steering_command_pos_);

                    // THROTTLE
//                    Serial.println("THROTTLE VALUE:");
                    throttle_command_pos_ = map(unmapped_throttle_, THROTTLE_MIN_PWM, THROTTLE_MAX_PWM, THROTTLE_MIN_ADC, THROTTLE_MAX_ADC);
                    throttle_servo_.write(int(throttle_command_pos_));

//                case AI_READY_STATE:
//                    // signals governed by AI
//                    // CURRENTLY AI_READY_STATE ONLY SUPPORTS FORWARD GEAR (i.e. no reverse)!!!!
//
//                    throttle_command_pos_ = AUTO_THROTTLE_SENSITIVITY * abs(cmd_x_velocity);
//                    steering_command_pos_ = AUTO_STEERING_SENSITIVITY * cmd_theta;
//                    brake_command_pos_ = 0.0;
//
//                    // When going forward: we want to brake if the current velocity command is AUTO_BRAKE_ENGAGE_THRESH less than the previous velocity command
//                    // Thus, we find the delta between the current command velocity and the previous command velocity
//                    // TODO: Replace ai_previous_velocity_ with actual feedback measurements from the GPS
//                    int delta_v = cmd_x_velocity - ai_previous_velocity_;
//
//                    // Check if we want to change gear
//                    if (sgn(cmd_x_velocity) != sgn(ai_previous_velocity_)) {
//                        // If so, which gear do we want?
//                        switch (sgn(cmd_x_velocity)) {
//                            case 1:
//                                gear_command_pos_ = GEAR_DRIVE_ADC;
//                                break;
//                            case 0:
//                                gear_command_pos_ = GEAR_PARK_ADC;
//                                break;
//                            case -1:
//                                gear_command_pos_ = GEAR_REVERSE_ADC;
//                                break;
//                        }
//                        set_engine_state(GEAR_CHANGE_STATE, sc);
//                    }
//
//                    if (delta_v < AUTO_BRAKE_ENGAGE_THRESH) {
//                        // Don't apply throttle when applying brakes
//                        brake_command_pos_ = AUTO_BRAKE_SENSITIVITY * delta_v;
//                        throttle_command_pos_ = 0.0;
//                    }
//
//                    // Store previous velocity state
//                    ai_previous_velocity_ = cmd_x_velocity;
//
//                    brake_motor_->SetTargetPosition(brake_command_pos_);
//                    steer_motor_->SetTargetPosition(steering_command_pos_);
//                    throttle_servo_.write(int(throttle_command_pos_));
            }

            // FSM3 : Switch States
            switch (current_switch_state_) {
                case NEUTRAL_SWITCH_STATE:
                    // switch to put car in neutral in ON
                    break;
                case AUTONOMOUS_SWITCH_STATE:
//                    Serial.println("Autonomous State Selected");
//
//                    // Output the steering angle to the console
//                    Serial.print("#");
//                    Serial.print(double(analogRead(STEERING_ACTUATOR_POSITION_SENSOR_PIN)));
//                    Serial.println("!");
//
//                    // Receive serial command from Jetson and parse
//                    //if(sc->message_time - timeDiff > 500)
//                    if ( sc->message_type != -1 ) {
//                        // to-do: parse the message from the serial
//
//                    } else {
//                        Serial.println("No command from Jetson received");
//                        return;
//                    }
//
//                    // Send command to the steering controller
//                    // Send command to the brake motor controller
//                    // Send command to the throttle controller
//                    // Send command to the gear controller
//
                      break;
            } 
        }

        bool set_engine_state(int new_engine_state, PWMCommand* sc) {
            switch (new_engine_state) {
                case OFF_STATE:
                    Serial.println("------------------");
                    Serial.println("ENTERING OFF STATE");
                    Serial.println("------------------");
                    
                    if (current_engine_state_ == IGNITION_STATE) {
                        stopEngine();
                        delay(2000);
                    } else if (current_engine_state_ == ENGINE_START_STATE) {
                        stopEngine();
                    } else if (current_engine_state_ == ENGINE_RUNNING_STATE) {
                        // STEERING WHEEL FORWARD
                        
                        // FOOT OFF ACCELLERATOR
                        
                        // FOOT ON BRAKE (2S)
                        delay(2000);    // 2 seconds to change gear
                        
                        stopEngine();
                        
                        // GEAR TO NEUTRAL
                        gear_motor_->SetTargetPosition(gear_command_pos_);
                        current_gear_pos_ = sc->message_gear;                // assume gear changed
                        
                    } else if (current_engine_state_ == GEAR_CHANGE_STATE) {
                        // STEERING WHEEL FORWARD
                        // FOOT OFF ACCELLERATOR
                        // FOOT ON BRAKE (2S)
                        stopEngine();
                        // GEAR TO NEUTRAL
                    }
                    break;
                
                case IGNITION_STATE:
                    if (current_engine_state_ == OFF_STATE) {
                        // Ensure that we are in NEUTRAL before engaging ignition
                        // CHECKS
                        // -- car in park
                        if (current_gear_pos_ != GEAR_NEUTRAL_ADC) {
                            Serial.println("Ignition command received, not in neutral.");
                            return false;
                        } else {
                            // Once the car is in park, we can start the ignition
                            Serial.println("------------------");
                            Serial.println("IGNITION ENGAGED");
                            Serial.println("------------------");
                            digitalWrite(IGNITION_RELAY_PIN, HIGH);
                            main_relay_on_ = 1;
                            break;
                        }
                    }
                    break;
                
                case ENGINE_START_STATE:
                    // may want some additional logic here
                    Serial.println("starting engine...");
                    startEngine();  // assume successful
                    Serial.println("------------------");
                    Serial.println("ENGINE STARTED");
                    Serial.println("------------------");
                    break;

                case ENGINE_RUNNING_STATE:

                    break;

                case GEAR_CHANGE_STATE:

                    // set throttle to zero
                    throttle_servo_.write(0);
                    delay(300);
                    // engage full brake
                    brake_motor_->SetTargetPosition(BRAKE_MAX_PWM);
                    // wait for the car to stop
                    delay(3500);
                    Serial.println("Changing Gear to: " + String(gear_command_pos_));

                    // change gear
                    if (sc->message_gear > (GEAR_DRIVE_PWM - 50)) {
                        gear_command_pos_ = GEAR_DRIVE_ADC;
                    } else if (sc->message_gear < (GEAR_REVERSE_PWM + 50)) {
                        gear_command_pos_ = GEAR_REVERSE_ADC;
                    } else {
                        gear_command_pos_ = GEAR_NEUTRAL_ADC;
                    }
   
                    gear_motor_->SetTargetPosition(gear_command_pos_);
                    current_gear_pos_ = sc->message_gear;                // assume gear changed
                    delay(2000);    // 2 seconds to change gear
                 
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

        bool checkFailsafes(PWMCommand* sc) {
            // This function will check all failsafes
            // If it is not safe to drive: the car will be switched to HALT_STATE
            // Pin 13 will be ON when it is safe to drive, otherwise OFF.

            // The failsafes include: a watchdog timer (i.e. an automatic shutdown if a command hasn't been recieved within 250ms)
            // Also included is PWM switch from the RC reciever.

            // Note that: the RC PWM switch failsafe should be connected in series with the emergency stop switch at the rear of the car

            Serial.println("Checking failsafes!");
            bool watchdogValid = ((millis() - last_command_timestamp_) < WATCHDOG_TIMEOUT);
            bool rcFailsafeValid = sc->message_failsafe >= 1500;

            Serial.print("Dutycycle for failsafe=");
            Serial.print(sc->message_failsafe);

            Serial.print(", watchdog_valid=");
            Serial.println(watchdogValid);

            bool safeToDrive = (watchdogValid && rcFailsafeValid);

            if (!safeToDrive and !(current_engine_state_ == OFF_STATE)) {
                set_engine_state(OFF_STATE, sc);
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
        
        Servo throttle_servo_;
        MotorController *brake_motor_;
        MotorController *gear_motor_;
        MotorController *steer_motor_;

        Servo* brake_servo_;
        Servo* gear_servo_;
        Servo* steering_servo_;

};
