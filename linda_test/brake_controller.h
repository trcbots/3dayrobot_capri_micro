//Includes required to use Roboclaw library

class BrakeController {
    public:
        BrakeController(Servo* _motor_interface, int _feedback_pin,
                        int _motor_min_pos, int _motor_max_pos,
                        int _motor_min_power, int _motor_max_power,
                        float _motor_sensitivity,
                        double _Kp, double _Ki, double _Kd);

        void SetTargetPosition(double target_pos);
        double GetCurrentPosition();
        boolean isMotorMoving();

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
        double residual_error_;
        bool motor_is_moving;
};

// Initialise motor contoller
BrakeController::BrakeController(Servo* _motor_interface, int _feedback_pin,
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
    this->residual_error_           = 0.0;
    this->motor_is_moving           = false;

//    motor_interface->write(_motor_min_pos);
}



void BrakeController::SetTargetPosition(double target_pos)
{
    if (target_pos < motor_min_pos) {
        target_pos = motor_min_pos;
    } else if (target_pos > motor_max_pos) {
        target_pos = motor_max_pos;
    }
    double current_pos = double(analogRead(feedback_pin));
    // Serial.print(", current_pos=");
    // Serial.print(current_pos);

    // Serial.print(", target_pos=");
    // Serial.print(target_pos);

    double pTerm = Kp * (current_pos - target_pos);
    double iTerm = 0.0;
    double dTerm = 0.0;

    // double output = int(Kp * pTerm + Ki * iTerm + Kd * dTerm);

    // if ( output < -1 * motor_max_power ) {
    //   output = -1 * motor_max_power;
    // } else if ( output > motor_max_power ) {
    //   output = motor_max_power;
    // }
    //
    // if (abs(output) > 10)
    // {
    //     motor_is_moving = true;
    //     motor_interface->write(output);
    // }
    // else
    // {
    //     motor_is_moving = false;
    // }



    double output = 90;
    motor_is_moving = true;
    if (pTerm < -5) {
      output = map(pTerm, -5, -215, 85, 10);
    } else if (pTerm > 5) {
      output = map(pTerm, 5, 215, 95, 170);
    } else {
        output = 90;
        motor_is_moving = false;
    }
    motor_interface->write(output);
    Serial.print("brake pTerm:");
    Serial.println(pTerm);


}

double BrakeController::GetCurrentPosition()
{
    Serial.print("potpos = ");
    Serial.println(analogRead(feedback_pin));
    return double(analogRead(feedback_pin));
}

boolean BrakeController::isMotorMoving()
{
    return motor_is_moving;
}
