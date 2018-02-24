#include <Servo.h>

class SteeringController {
	public:
		SteeringController(uint8_t debug);

		void setup();
		void loop(uint8_t debug);
		void setTargetPosition(uint16_t target_position);

	private:

		void _updateCurrentPosition();

		uint8_t debug:


		const uint8_t MIN_ROTATION = 1;
		const uint8_t MAX_ROTATION = 50;
		const uint8_t FEEDBACK_PIN = A3;
		const uint16_t ACCELERATOR_TOL = 5;

		uint16_t nextMillis;

		Servo throttle;
}

SteeringController::SteeringController() {
	this->debug = debug;
}

void SteeringController::setup() {
	if (debug) {
	    Serial.print(CLASS_NAME);
	    Serial.println(": initialised");
  	}
}
 
void SteeringController::setTargetPosition(uint16_t target_position) {
	if (target_position <= MAX_ROTATION && target_position >= MIN_ROTATION) {
		servo.write(target_position);	
	}
}

// loop is expected to be called from the main loop with a value passed for how frequently it must execute in the timer wheel
void SteeringController::loop(uint8_t rate)
{
  
}
