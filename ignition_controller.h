class IgnitionController {
    public:
        IgnitionController(uint8_t debug);

        void setup();
        void ignition();
        void start();
        // void run();
        void stop();
        void loop(uint8_t rate);

        uint16_t  getCurrentStatus();

    private:
        void      _checkStarterMotorStatus();

        uint8_t   debug;
        uint8_t   current_status;               // CAR_STOPPED vs CAR_STARTED
        uint16_t  nextMillis;

        const uint8_t IGNITION_PIN  = 7; // Digital pin connected to the first relay on the ignition
        const uint8_t ENGINE_START_PIN    = 8; // Digital pin connected to the first relay on the ignition
        const uint8_t CAR_STOPPED         = 0;
        const uint8_t CAR_STARTED         = 1;
        const char*   CLASS_NAME          = "IgnitionController";
};

// Initialise the IgnitionController
// pass true for debug to get Serial replies
IgnitionController::IgnitionController(uint8_t debug) {
    this->debug = debug;

    pinMode(IGNITION_PIN, OUTPUT);
    pinMode(ENGINE_START_PIN, OUTPUT);

    digitalWrite(IGNITION_PIN, LOW);
    digitalWrite(ENGINE_START_PIN, LOW);
}

void IgnitionController::setup() {
    if (debug) {
        Serial.print(CLASS_NAME);
        Serial.println(": initialised");
    }
}

// Return whether car has started
uint16_t IgnitionController::getCurrentStatus() {
    return current_status;
}

void IgnitionController::_checkStarterMotorStatus() {
    // insert code to check that the starter moter is running and update current_status
    if(true) { 
        current_status = CAR_STARTED;
    } else {
        current_status = CAR_STOPPED;
    }
}

// Turn relays IGNITION_PIN then ENGINE_START_PIN
void IgnitionController::ignition() {
    if (current_status != CAR_STARTED) {
        digitalWrite(IGNITION_PIN, HIGH);
        // digitalWrite(ENGINE_START_PIN, HIGH);
        digitalWrite(13, HIGH);
        delay(500);
        // digitalWrite(IGNITION_PIN, LOW);
    }
}

// Turn relays IGNITION_PIN then ENGINE_START_PIN
void IgnitionController::start() {
    if (current_status != CAR_STARTED) {
        digitalWrite(IGNITION_PIN, HIGH);
        digitalWrite(ENGINE_START_PIN, HIGH);
        digitalWrite(13, HIGH);
        delay(2000);
        digitalWrite(ENGINE_START_PIN, LOW);
        current_status = CAR_STARTED;
    }
}

void IgnitionController::stop() {
    if (current_status != CAR_STOPPED) {
        digitalWrite(IGNITION_PIN, LOW);
        digitalWrite(ENGINE_START_PIN, LOW);
        digitalWrite(13, LOW);
    }
}


// void IgnitionController::run() {
//     if (current_status )
//         digitalWrite(IGNITION_PIN, LOW);
//         digitalWrite(ENGINE_START_PIN, HIGH);
// }
// loop is expected to be called from the main loop with a value passed for how frequently it must execute in the timer wheel
void IgnitionController::loop(uint8_t rate) {
    if (millis() >= nextMillis) {
        nextMillis = millis() + rate;
        // Execute code
        if (current_status == CAR_STARTED) {
            digitalWrite(IGNITION_PIN, LOW);
            digitalWrite(IGNITION_RUN_PIN, LOW);
        }
    }
}