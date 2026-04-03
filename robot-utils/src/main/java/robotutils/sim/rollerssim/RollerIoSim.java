package robotutils.sim.rollerssim;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import robotutils.pub.interfaces.simio.RollerIoInterface;


/**
 * Simulated implementation of {@link RollerIoInterface} using a {@link FlywheelSim}.
 *
 * <p>Physics are modeled as a single-NEO flywheel. A {@link SimDevice}
 * exposes velocity and current in the WPILib sim GUI for debugging.
 */
public class RollerIoSim implements RollerIoInterface {
    /** NEO motor model (single motor). */
    private static final DCMotor kMotor = DCMotor.getNeoVortex(1);
    private final FlywheelSim m_flyWheelSim;

    // Sim device entries visible in the WPILib sim GUI
    private final SimDevice m_simDevice;
    private final SimDouble m_simVelocity;
    private final SimDouble m_simCurrent;

    /** Constructs the simulated intake IO. */
    public RollerIoSim(String deviceName, double momentOfInertia, double gearRatio) {
        m_flyWheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                kMotor,
                momentOfInertia,
                gearRatio),
            kMotor
        );

        // Register a SimDevice so values appear in the sim GUI
        m_simDevice = SimDevice.create(deviceName);
        m_simVelocity = m_simDevice.createDouble(
            "Velocity RPM", Direction.kOutput, 0.0);
        m_simCurrent = m_simDevice.createDouble(
            "Current Amps", Direction.kOutput, 0.0);
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
        // Step the physics simulation forward by one robot loop period
        m_flyWheelSim.update(0.02);

        outputs.m_velocityRpm = m_flyWheelSim.getAngularVelocityRPM();
        outputs.m_currentAmps = m_flyWheelSim.getCurrentDrawAmps();

        // Push values into the sim GUI
        m_simVelocity.set(outputs.m_velocityRpm);
        m_simCurrent.set(outputs.m_currentAmps);
    }
}
