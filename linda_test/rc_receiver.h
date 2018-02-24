class RCReceiver {
    public:
        RCReceiver(uint8_t debug);
        void readRCData();

        uint8_t     getExpectedAutonomyStatus();
        uint8_t     getExpectedIgnitionStatus();
        uint8_t     getExpectedStartStatus();
        uint16_t    getExpectedSteeringPosition();
        uint16_t    getExpectedVelocity();
        uint16_t    getExpectedGearPosition();

    private:
        void setup();
        
        const uint8_t RC_PIN_1 = 22;          // RC PIN 1
        const uint8_t RC_PIN_2 = 24;          // RC PIN 2 
        const uint8_t RC_PIN_3 = 26;          // RC PIN 3
        const uint8_t RC_PIN_4 = 28;          // RC PIN 4
        const uint8_t RC_PIN_5 = 30;          // RC PIN 5
        const uint8_t RC_PIN_6 = 32;          // RC PIN 6
        const uint8_t RC_PIN_7 = 34;          // RC PIN 7
        const uint8_t RC_PIN_8 = 36;          // RC PIN 8

        const char* CLASS_NAME      = "RCReceiver";

        uint8_t  autonomy_status            = 0;
        uint8_t  ignition_status            = 0;
        uint8_t  start_status               = 0;
        uint16_t steering_position;
        uint16_t velocity;
        uint16_t gear_position;
        uint8_t  debug;
};

// Initialise the RCReceiver
// pass true for debug to get Serial replies
RCReceiver::RCReceiver(uint8_t debug) {
    this->debug = debug;
}

void RCReceiver::setup() {
    if (debug) {
        Serial.print(CLASS_NAME);
        Serial.println(": initialised");
    }

    // Initialise pins
    pinMode(RC_PIN_1, INPUT);
    pinMode(RC_PIN_2, INPUT);
    pinMode(RC_PIN_6, INPUT);
    pinMode(RC_PIN_7, INPUT);
    pinMode(RC_PIN_8, INPUT);

    // Initialise pins
    pinMode(RC_PIN_3, INPUT);
    pinMode(RC_PIN_4, INPUT);
    pinMode(RC_PIN_5, INPUT);

    // Populate private variables with inputs from RC
    autonomy_status         = pulseIn(RC_PIN_5, HIGH);
    ignition_status         = pulseIn(RC_PIN_7, HIGH);
    start_status            = pulseIn(RC_PIN_8, HIGH);
    steering_position       = pulseIn(RC_PIN_1, HIGH);
    velocity                = pulseIn(RC_PIN_2, HIGH);
    gear_position           = pulseIn(RC_PIN_6, HIGH);

}

void RCReceiver::readRCData() {
    // Populate private variables with inputs from RC
    autonomy_status         = pulseIn(RC_PIN_5, HIGH);
    ignition_status         = pulseIn(RC_PIN_7, HIGH);
    start_status            = pulseIn(RC_PIN_8, HIGH);
    steering_position       = pulseIn(RC_PIN_1, HIGH);
    velocity                = pulseIn(RC_PIN_2, HIGH);
    gear_position           = pulseIn(RC_PIN_6, HIGH);
}

uint8_t  RCReceiver::getExpectedAutonomyStatus() {
    return autonomy_status;
}

uint8_t  RCReceiver::getExpectedIgnitionStatus() {
    return ignition_status;
}

uint8_t  RCReceiver::getExpectedStartStatus() {
    return start_status;
}

uint16_t RCReceiver::getExpectedSteeringPosition() {
    return steering_position;
}

uint16_t RCReceiver::getExpectedVelocity() {
    return velocity;
}

uint16_t  RCReceiver::getExpectedGearPosition() {
    return gear_position;
}
