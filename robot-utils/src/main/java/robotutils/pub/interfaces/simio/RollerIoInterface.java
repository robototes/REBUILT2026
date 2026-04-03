package robotutils.pub.interfaces.simio;

/**
 * IO interface for a single motor Roller mechanism.
 *
 * <p>Abstracts hardware access so the subsystem works identically
 * with real hardware and with a physics simulation.
 */
public interface RollerIoInterface {

    /** Mutable container for roller sensor readings. */
    class DeviceOutputs {
        /** Motor velocity in RPM. */
        public double m_velocityRpm;
        /** Motor output current in amps. */
        public double m_currentAmps;
    }

    /** Set the motor speed as a percentage (-1.0 to 1.0). */
    void setSpeed(double speed);

    /** Stop the motor immediately. */
    void stop();

    /** Read latest sensor data into the given outputs container. */
    void updateOutputs(DeviceOutputs outputs);
}
