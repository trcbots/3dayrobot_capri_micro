class PWMCommand {
    public:
        PWMCommand();
        int message_type;                   // [XBOX = 1, JETSON = 2]  
        int message_ignition;               // Igniton
        int message_engine_start;           // Start
        int message_steering;               // Steering
        int message_velocity;               // Velocity
        int message_gear;                   // Gear
        int message_failsafe;               // Failsafe
};

PWMCommand::PWMCommand() {
}
