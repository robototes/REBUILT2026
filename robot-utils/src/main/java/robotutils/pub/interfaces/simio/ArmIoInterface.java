package robotutils.pub.interfaces.simio;


/**
 * IO interface for the intake arm mechanism.
 */
public interface ArmIoInterface {
    /** Mutable container for arm telemetry. */
    class DeviceOutputs {
        /** Arm position in subsystem units. */
        public double position;
        /** Arm velocity in subsystem units per second. */
        public double velocity;
        /** Motor output current in amps. */
        public double currentAmps;
    }

    /** Immediately start moving arm [-1.0, 1.0]. */
    void moveArmWithSpeed(double speed);

    /** Command closed-loop position in subsystem units. */
    void setPosition(double position);

    /** Reset the arm encoder position to 0. */
    void resetEncoderValue();

    /** Stop arm motion immediately. */
    void stop();

    /**
     * Enable or disable hardware soft limits.
     * Must be disabled during homing (encoder not yet zeroed) and enabled after homing completes.
     */
    void setSoftLimitsEnabled(boolean enabled);

    /** Read latest arm telemetry. */
    void updateOutputs(DeviceOutputs outputs);
}
