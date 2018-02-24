//Includes required to use Roboclaw library

#define address     0x80

// Motor IDs
#define BRAKE       1
#define GEAR        2
#define STEERING    3



class MotorController {
    public:
    
        MotorController(Servo* _motor_interface, int _feedback_pin, 
                        int _motor_min_pos, int _motor_max_pos, 
                        int _motor_min_power, int _motor_max_power,
                        float _motor_sensitivity,
                        double _Kp, double _Ki, double _Kd);

        void SetTargetPosition(double target_pos);
        double get_current_pos();

    private: 
        Servo* motor_interface;
        int feedback_pin;
        int motor_min_pos;
        int motor_max_pos;
        int motor_min_power;
        int motor_max_power;
        float motor_sensitivity;
        double Kp;
        double Ki;
        double Kd;

        bool motor_is_moving;
};

// Initialise motor contoller
MotorController::MotorController(Servo* _motor_interface, int _feedback_pin, 
                                 int _motor_min_pos, int _motor_max_pos, 
                                 int _motor_min_power, int _motor_max_power,
                                 float _motor_sensitivity,
                                 double _Kp, double _Ki, double _Kd) {

    // init the motor controller here
    this->motor_interface           = _motor_interface;
    this->feedback_pin              = _feedback_pin;
    this->motor_min_pos             = _motor_min_pos;
    this->motor_max_pos             = _motor_max_pos;
    this->motor_min_power           = _motor_min_power;
    this->motor_max_power           = _motor_max_power;
    this->motor_sensitivity         = _motor_sensitivity;
    this->Kp                        = _Kp;
    this->Ki                        = _Ki;
    this->Kd                        = _Kd;

    this->motor_is_moving           = false;

//    motor_interface->write(_motor_min_pos);
}

void MotorController::SetTargetPosition(double target_pos) {
    if (target_pos < motor_min_pos) {
        target_pos = motor_min_pos;
    } else if (target_pos > motor_max_pos) {
        target_pos = motor_max_pos;
    }

    double current_pos = get_current_pos();
    // Serial.print(current_pos);
    
    Serial.print("current pos: ");  
    Serial.print(current_pos);
    Serial.print(" - target pos: "); 
    Serial.print(target_pos);
    Serial.print(" = ");

    double pTerm = current_pos - target_pos;
    double output = int(Kp * pTerm);

    if ( output < -1 * motor_max_power ) {
        output = -1 * motor_max_power;
    } else if ( output > motor_max_power ) {
        output = motor_max_power;
    }

    double bias_output = output + 90;

    double sensitivity_bias_output = int(motor_sensitivity * bias_output);

    // update whether motor is moving
    if (abs(output) > 10) {
        motor_is_moving = true;
        motor_interface->write(sensitivity_bias_output);
    } else {
        motor_is_moving = false;
    }
}

//boolean MotorController::is_motor_moving() {
//    // Returns true if a motion command is currently in operation
//    return motor_is_moving;
//}

double MotorController::get_current_pos() {
    // Returns true if a motion command is currently in operation
    return double(analogRead(feedback_pin));
}
