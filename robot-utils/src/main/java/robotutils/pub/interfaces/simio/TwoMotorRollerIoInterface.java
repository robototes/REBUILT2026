package robotutils.pub.interfaces.simio;


/**
 * IO interface for a two-motor Roller mechanism (leader + opposed follower).
 *
 * <p>Abstracts hardware access so the subsystem works identically
 * with real hardware and with a physics simulation.
 */
public interface TwoMotorRollerIoInterface {

    /** Mutable container for two-motor roller sensor readings. */
    class DeviceOutputs {
        /** Flywheel velocity in RPM (as seen from the leader motor). */
        public double m_velocityRpm;
        /** Leader motor output current in amps. */
        public double m_leaderCurrentAmps;
        /** Follower motor output current in amps. */
        public double m_followerCurrentAmps;
    }

    /** Set the motor speed as a percentage (-1.0 to 1.0). */
    void setSpeed(double speed);

    /** Stop both motors immediately. */
    void stop();

    /** Read latest sensor data into the given outputs container. */
    void updateOutputs(DeviceOutputs outputs);
}
