//Includes required to use Roboclaw library
#include "RoboClaw.h"

#define address     0x80

// Motor IDs
#define BRAKE       1
#define GEAR        2
#define STEERING    3



class MotorController {
    public:
    
        MotorController(int _motor_id, RoboClaw* _motor_interface, int _feedback_pin, 
                        int _motor_min_pos, int _motor_max_pos, int _motor_max_power, 
                        bool _motor_is_moving, bool _interface_initialised, 
                        double _Kp, double _Ki, double _Kd, int _qpps);

        void SetTargetPosition(double target_pos);
        double get_current_pos();
        boolean is_motor_moving();

    private:
        int motor_id;  
        RoboClaw* motor_interface;
        int feedback_pin;
        int motor_min_pos;
        int motor_max_pos;
        int motor_max_power;
        double Kp;
        double Kd;
        double Ki;
        int qpps;
        bool motor_is_moving;
        bool interface_initialised;
};

// Initialise motor contoller
MotorController::MotorController(int _motor_id, RoboClaw* _motor_interface, int _feedback_pin, 
                                 int _motor_min_pos, int _motor_max_pos, int _motor_max_power, 
                                 bool _motor_is_moving, bool _interface_initialised, 
                                 double _Kp, double _Ki, double _Kd, int _qpps) {

    // init the motor controller here
    this->motor_id                  = _motor_id;                // ids   = [BRAKE, GEAR, STEERING]
    this->motor_interface           = _motor_interface;
    this->feedback_pin              = _feedback_pin;
    this->motor_min_pos             = _motor_min_pos;
    this->motor_max_pos             = _motor_max_pos;
    this->motor_max_power           = _motor_max_power;
    this->Kp                        = _Kp;
    this->Ki                        = _Ki;
    this->Kd                        = _Kd;
    this->qpps                      = _qpps;
    this->motor_is_moving           = false;
    this->interface_initialised     = _interface_initialised;   // need to set this from linda.h

    // check if interface has been initialised
    if (interface_initialised == false) {
        // Set PID Coefficients
        switch(motor_id) {
            case BRAKE:         // roboclaw1_m1
                motor_interface->SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
            case GEAR:          // roboclaw1_m2
                motor_interface->SetM2VelocityPID(address,Kd,Kp,Ki,qpps);
            case STEERING:      // roboclaw2_m1
                motor_interface->SetM2VelocityPID(address,Kd,Kp,Ki,qpps);
        }
        interface_initialised = true;
    } else {
        // don't initialise
    }
}

void MotorController::SetTargetPosition(double target_pos) {
    // Implementation of a PID controller
    // TODO add make P and D terms work properly

    if (target_pos < motor_min_pos) {
        target_pos = motor_min_pos;
    } else if (target_pos > motor_max_pos) {
        target_pos = motor_max_pos;
    }

    double current_pos = get_current_pos();
    // Serial.print(current_pos);

    double pTerm = current_pos - target_pos;
    double iTerm = 0.0;
    double dTerm = 0.0;
    double output = int(Kp * pTerm + Ki * iTerm + Kd * dTerm);

    if ( output < -1 * motor_max_power ) {
        output = -1 * motor_max_power;
    } else if ( output > motor_max_power ) {
        output = motor_max_power;
    }

    // Serial.println("");
    // Serial.print(my_name);
    // Serial.print(", motor ID: ");
    // Serial.print(motor_id);
    // Serial.print(", output=");
    // Serial.print(output);
    // Serial.print(", target_pos=");
    // Serial.print(target_pos);

    // update whether motor is moving
    if (abs(output) > 10) {
        motor_is_moving = true;
        // set speed of motor
        switch(motor_id) {
            case BRAKE:             // roboclaw1_m1
                motor_interface->SpeedM1(address, output);
            case GEAR:              // roboclaw1_m2
                motor_interface->SpeedM2(address, output);
            case STEERING:          // roboclaw2_m1
                motor_interface->SpeedM1(address, output);
        }
    } else {
        motor_is_moving = false;
    }
}

boolean MotorController::is_motor_moving() {
    // Returns true if a motion command is currently in operation
    return motor_is_moving;
}

double MotorController::get_current_pos() {
    // Returns true if a motion command is currently in operation
    return double(analogRead(feedback_pin));
}
