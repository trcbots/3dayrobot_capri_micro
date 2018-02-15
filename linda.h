#include <Servo.h>
#include "motor_controller.h"
#include "serial_command.h"

// These are the names of the states that the car can be in
#define HALT_STATE              0
#define COAST_STATE             1
#define IGNITION_STATE          2
#define ENGINE_START_STATE      3
#define RC_TELEOP_STATE         4
#define AI_READY_STATE          5
#define NEUTRAL_SWITCH_STATE    6
#define AUTONOMOUS_SWITCH_STATE 7


/************************ ARDUINO PIN DEFINITIONS ********************************/
// PWM input pins from RC Reciever

// Digital output pins

// Analog input pins


// Motor driver Pins (UART Serial)


/*********************************************************************************/

/************************ DRIVE CONTROL DEFINEs **********************************/
// These parameters adjust how the car will behave.

// They will need to be changed according to the particular vehicle.
// However, most values provided should be fairly suitable for  configurations.

// Sensitivity values define how responsive the actuators are to a given input


// Max power applies a constraint to the driver output speed.
// Important note: set these low for testing so you don't destroy anything


// PID values for each motor driver
// Important note: These values are optional

// Gear positions define where the gear actuator has to travel to engage a specified gear

// How close should the analog feedback reading be to the actual position, as confirmation that we are actually in the specified gear
// An absolute difference threshold

// Define the allowable range of motion for the brake actuator

// Define the allowable range of motion for the throttle servo actuator

// Define the allowable range of motion for the steering actuator

// Define the limits on Steering PWM input from the RC Reciever
// In RC Mode: these values will get mapped to STEERING_FULL_LEFT and STEERING_FULL_RIGHT respectively

// Define the limits on Throttle PWM input from the RC Reciever
// In RC Mode: these values will get mapped to THROTTLE_SERVO_ZERO_POSITION and THROTTLE_SERVO_FULL_POSITION respectively

// Define the limits on Brake PWM input from the RC Receiver
// In RC Mode: these values will get mapped to BRAKE_SERVO_ZERO_POSITION and BRAKE_SERVO_FULL_POSITION respectively

// RC stick DEADZONEs are optionally used to adjust the ergonomics of RC control
// 0.0 values will disable them

// PWM input thresholds on the RC 3-way switch, these will map to gear positions

// PWM input thresholds on the ignition and start switches, relays will be activated if the thresholds are reached


/**********************************************************************************/


// If a command from the RC or AI has not been recieved within WATCHDOG_TIMEOUT ms, will be switched to HALT state.
#define WATCHDOG_TIMEOUT 250

SerialCommand sc;

class Linda
{
  public:
    Linda()
    {
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

    void Init() {

    }

    void startEngine()
    {
        // Engine AUTOSTART functionallity
        // Used in AI mode ONLY
        // This will attempt to start the engine, with multiple attempts on failure to do so


    }

    void stopEngine()
    {
        // Will stop the engine

    }

    bool is_engine_running()
    {
        // return flag value for now, we have no way of determining this YET
        // Should hook up some kind of sensor and return the digital reading

        return engine_currently_running;
    }

    void process_command(int cmd_x_velocity = 0, int cmd_b_velocity = 0, int cmd_theta = 0, int cmd_gamma = 0)
    {
        // This is the main function for the RC car control
        // It decides what action to do based on the current state and command input
        // RUNS REPEATEDLY, IT MUST BE CALLED FROM THE MAIN LOOP

        // Note: if in RC_TELEOP_STATE, commanded velocities will be ignored, PWM values will be read instead

        lastCommandTimestamp = millis();
        Serial.println("Processing command");

        // Will be changed into the HALT state if it is not safe to drive.
        //checkFailsafes();

        int neutral_switch_pos = int(analogRead(NEUTRAL_CONTROL_SWITCH_PIN));
        int autonomous_switch_pos = int(analogRead(AUTONOMOUS_CONTROL_SWITCH_PIN));

        if(neutral_switch_pos > 750) {
          if(currentStateID != NEUTRAL_SWITCH_STATE)
            set_current_state_ID(NEUTRAL_SWITCH_STATE);
            //return;
        } else if(autonomous_switch_pos > 750) {
          if(currentStateID != AUTONOMOUS_SWITCH_STATE)
            set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
            //return;
        } else {
          if(currentStateID != RC_TELEOP_STATE)
            set_current_state_ID(RC_TELEOP_STATE);
            //return;
        }

        // State Machine
        switch (currentStateID)
        {
        case HALT_STATE:
            // We are in HALT_STATE

            x_velocity = 0.0;
            theta = cmd_theta;

            // Once we have slowed to a HALT, lets stop the engine
            if (abs(x_velocity_sensed) <= 0.1)
            {
              stopEngine();
            }

            // Check the Control State Switch on dash of car
            // Neutral - Puts the car gear into neutral
            // RC - Allows the car to be driven by Remote Controller
            // Autonomous - car is drive by Nvidia Jetson

            if(neutral_switch_pos > 750) {
              set_current_state_ID(NEUTRAL_SWITCH_STATE);
              return;
            } else if(autonomous_switch_pos > 750) {
              set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
              return;
            } else {
              set_current_state_ID(RC_TELEOP_STATE);
              // set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
              return;
            }

            break;

        case RC_TELEOP_STATE:
        {

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

    int get_current_gear_position()
    {

    }

    bool set_current_state_ID(int newStateID)
    {
        // This function gets called when a change state is requested
        // Returns true on a successful transition

        // Code blocks within this switch statement are ONLY CALLED ON STATE change
        switch (newStateID)
        {
        case IGNITION_STATE:

            // Only allowed to transistion from HALT STATE to IGNITION STATE
            // FIXME: add state to MotorController class so that we can request the current and last commanded position
            if (currentStateID == HALT_STATE)
            {
                // Ensure that we are in park before engaging ignition
                if (abs(gear_motor->GetCurrentPosition() - PARK_GEAR_POSITION) > GEAR_FEEDBACK_TOLERENCE)
                {
                    Serial.println("Ignition command received, not in park.");

                    //Put the car into park
                    // current_gear_position = PARK_GEAR_POSITION;
                    /* DISABLE
                    gear_motor->SetTargetPosition(current_gear_position);
                    */

                    return false;
                }
                else
                {
                    // Once the car is in park, we can start the ignition
                    Serial.println("Car in park, turning on ignition");
                    digitalWrite(IGNITION_RELAY_PIN, HIGH);
                    main_relay_on = 1;
                    return true;
                }
                break;
            }
        case ENGINE_START_STATE:

            // Only transistion to ENGINE_START_STATE if currently in ignition state
            if (currentStateID == IGNITION_STATE)
            {
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

        currentStateID = newStateID;
        return true;
    }

    int getcurrentStateID()
    {
        // Return the ID of the state that we are currently in
        return currentStateID;
    }

    float read_pwm_value(int pwm_pin)
    {
        // Read a value from a PWM input

    }

    float convert_jetson_serial(int jetson_min, int jetson_max)
    {

    }

    bool checkFailsafes()
    {
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

        if (!safeToDrive)
        {
            set_current_state_ID(HALT_STATE);
        }

        digitalWrite(FAILSAFE_LED_PIN, safeToDrive);

        return safeToDrive;
    }

    void send_throttle_command(int throttle_command)
    {
        // Send command to the throttle servo

    }

    void check_ignition_starter()
    {
      double ignition_val = read_pwm_value(RC_IGNITION_PWM_PIN);
      // Ignition and Starter Motor Control

      double starter_val  = read_pwm_value(RC_ENGINE_START_PWM_PIN);

      if (ignition_val > RC_DUTY_THRESH_IGNITION) {

      }
      else
      {
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

    SabertoothSimplified* sabertooth_60A;
    SabertoothSimplified* sabertooth_32A;

    MotorController* brake_motor;
    MotorController* gear_motor;
    MotorController* steer_motor;
};
