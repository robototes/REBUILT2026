package robotutils.sim.rollerssim;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import robotutils.pub.interfaces.simio.TwoMotorRollerIoInterface;


/**
 * Simulated implementation of {@link TwoMotorRollerIoInterface} using a
 * {@link FlywheelSim} driven by two motors (leader + opposed follower).
 *
 * <p>Physics are modeled as a single flywheel driven by two motors.
 * Each motor's current draw is assumed to be half the total load.
 * A {@link SimDevice} exposes velocity and per-motor current in the
 * WPILib sim GUI for debugging.
 */
public class TwoMotorRollerIoSim implements TwoMotorRollerIoInterface {
    /** Two-motor model (e.g. two NEO Vortex). */
    private static final DCMotor kMotor = DCMotor.getNeoVortex(2);
    private final FlywheelSim m_flyWheelSim;

    // Sim device entries visible in the WPILib sim GUI
    private final SimDevice m_simDevice;
    private final SimDouble m_simVelocity;
    private final SimDouble m_simLeaderCurrent;
    private final SimDouble m_simFollowerCurrent;

    /**
     * Constructs the simulated two-motor roller IO.
     *
     * @param deviceName      Name shown in the sim GUI
     * @param momentOfInertia MOI of the flywheel in kg·m²
     * @param gearRatio       Gear ratio (motor rotations per flywheel rotation)
     */
    public TwoMotorRollerIoSim(
        String deviceName,
        double momentOfInertia,
        double gearRatio) {

        m_flyWheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(kMotor, momentOfInertia, gearRatio),
            kMotor
        );

        m_simDevice = SimDevice.create(deviceName);
        m_simVelocity = m_simDevice.createDouble(
            "Velocity RPM", Direction.kOutput, 0.0);
        m_simLeaderCurrent = m_simDevice.createDouble(
            "Leader Current Amps", Direction.kOutput, 0.0);
        m_simFollowerCurrent = m_simDevice.createDouble(
            "Follower Current Amps", Direction.kOutput, 0.0);
    }

    @Override
    public void setSpeed(double speed) {
        m_flyWheelSim.setInputVoltage(speed * 12.0);
    }

    @Override
    public void stop() {
        m_flyWheelSim.setInputVoltage(0.0);
    }

    @Override
    public void updateOutputs(DeviceOutputs outputs) {
        m_flyWheelSim.update(0.02);

        outputs.m_velocityRpm = m_flyWheelSim.getAngularVelocityRPM();

        // Total current is shared equally between the two motors
        double totalCurrent = m_flyWheelSim.getCurrentDrawAmps();
        outputs.m_leaderCurrentAmps = totalCurrent / 2.0;
        outputs.m_followerCurrentAmps = totalCurrent / 2.0;

        m_simVelocity.set(outputs.m_velocityRpm);
        m_simLeaderCurrent.set(outputs.m_leaderCurrentAmps);
        m_simFollowerCurrent.set(outputs.m_followerCurrentAmps);
    }
}
