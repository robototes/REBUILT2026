package robotutils.pub.interfaces.simio;


/**
 * IO interface for a single-motor elevator (linear lift) mechanism.
 *
 * <p>Abstracts hardware access so subsystems work identically with real
 * hardware and with a physics simulation.
 */
public interface ElevatorIoInterface {

    /** Mutable container for elevator sensor readings. */
    class DeviceOutputs {
        /** Carriage position in meters. */
        public double m_positionMeters;
        /** Motor output current in amps. */
        public double m_currentAmps;
    }

    /** Set the motor speed as a percentage (-1.0 to 1.0). */
    void setSpeed(double speed);

    /** Stop the motor immediately. */
    void stop();

    /** Read latest sensor data into the given outputs container. */
    void updateOutputs(DeviceOutputs outputs);

    /** Reset the encoder position to zero. */
    void resetEncoder();
}
