//Includes required to use Roboclaw library

#define address     0x80

// Motor IDs
#define BRAKE       1
#define GEAR        2
#define STEERING    3

#define STEERING_PWM_OUTPUT                 11



class SteeringController {
    public:
        SteeringController(Servo* _motor_interface, int _feedback_pin, 
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
SteeringController::SteeringController(Servo* _motor_interface, int _feedback_pin, 
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

void SteeringController::SetTargetPosition(double target_pos) {

    double current_pos = get_current_pos();

    double pTerm = current_pos - target_pos;
    Serial.print("pterm: ");
    Serial.println(pTerm);

    double output = 90;
    if (pTerm < -15) {
        output = map(pTerm, -15, -215, 90, 60);
    } else if (pTerm > 15) {
        output = map(pTerm, 15, 215, 90, 120);
    } else {
        output = 90;
    }

    Serial.print("real out to actuator: ");
          Serial.println(output);
          
    motor_interface->write(output);
}

double SteeringController::get_current_pos() {
    // Returns true if a motion command is currently in operation
    return double(analogRead(feedback_pin));
}
