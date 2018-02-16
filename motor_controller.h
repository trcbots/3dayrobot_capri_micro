//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

class MotorController
{
    public:
      MotorController(String _my_name, RoboClaw* _motor_interface,
          int _motor_id, int _feedback_pin, int _motor_min_pos, int _motor_max_pos, int _motor_max_power,
          double _Kp = 0.5, double _Ki = 0.0, double _Kd = 0.0);

      void SetTargetPosition(double target_pos);

      // FIXME: TEST THIS!!!
      double GetCurrentPosition();
      
      bool isMotorMoving();

      // TODO: Add the option for a callback when the target position is reached???

    private:
      String my_name;
      RoboClaw* motor_interface;
      int motor_id;
      int feedback_pin;
      double Kp;
      double Kd;
      double Ki;
      int motor_min_pos;
      int motor_max_pos;
      int motor_max_power;
      bool motor_is_moving;
};

MotorController::MotorController(String _my_name, RoboClaw* _motor_interface,
    int _motor_id, int _feedback_pin, int _motor_min_pos, int _motor_max_pos, int _motor_max_power,
    double _Kp = 0.5, double _Ki = 0.0, double _Kd = 0.0)
{
    // init the motor controller here
    this->my_name         = _my_name;
    this->motor_id        = _motor_id;
    this->motor_interface = _motor_interface;
    this->feedback_pin    = _feedback_pin;
    this->motor_min_pos   = _motor_min_pos;
    this->motor_max_pos   = _motor_max_pos;
    this->motor_max_power = _motor_max_power;
    this->Kp              = _Kp;
    // this->Ki              = _Ki;
    // this->Kd              = _Kd;
    this->motor_is_moving = false;
}

double MotorController::GetCurrentPosition()
{
    Serial.print("potpos = ");
    Serial.println(analogRead(feedback_pin));
    return double(analogRead(feedback_pin));
}

void MotorController::SetTargetPosition(double target_pos)
{
    
    // Implementation of a P(ID) controller
    double current_pos = GetCurrentPosition();

    if (current_pos > target_pos) {
        
    } else if (current_pos < target_pos) {

    }

    this->motor_min_pos   = _motor_min_pos;
    this->motor_max_pos   = _motor_max_pos;
    this->motor_max_power = _motor_max_power;
    this->Kp              = _Kp;

   
}
