package frc.robot.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.sim.visionproducers.VisionSimFactory;
import frc.robot.sim.visionproducers.VisionSimInterface;
import java.util.function.Consumer;


/**
 * Unified simulation wrapper combining ground truth physics, vision simulation,
 * and joystick orientation handling.
 *
 * <p>This class encapsulates all simulation-specific logic, keeping Robot and
 * RobotContainer free from simulation implementation details.
 *
 * <p>Note that we encapsulated this into a composeable class rather than having
 * the sim class subclass RobotContainer, since its cleaner this way to know
 * that the simulation code is only running under simulation conditions.
 */
public class SimWrapper {
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_drivetrain;
    private final GroundTruthSimInterface m_groundTruthSim;
    private final VisionSimInterface m_visionSim;
    private final ShowVisionOnField m_showVisionOnField;

    /**
     * Creates a new SimWrapper.
     *
     * @param drivetrain The swerve drivetrain
     * @param poseResetConsumer Consumer called when ground truth resets the pose
     *     (typically drivetrain::resetPose)
     */
    public SimWrapper(
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
            Consumer<Pose2d> poseResetConsumer) {

        if (!Robot.isSimulation()) {
            throw new IllegalStateException("SimWrapper should only be instantiated in simulation");
        }
        if (drivetrain == null) {
            throw new IllegalArgumentException("SwerveDrivetrain cannot be null");
        }
        if (poseResetConsumer == null) {
            throw new IllegalArgumentException("Pose reset consumer cannot be null");
        }

        m_drivetrain = drivetrain;

        // Create ground truth simulation
        m_groundTruthSim = GroundTruthSimFactory.create(drivetrain, poseResetConsumer);

        // Create vision simulation
        m_visionSim = VisionSimFactory.create();
        if (m_visionSim == null) {
            throw new IllegalStateException("VisionSimInterface creation failed");
        }

        // Create field visualization helper
        Field2d debugField = m_visionSim.getSimDebugField();
        m_showVisionOnField = new ShowVisionOnField(null, debugField);
    }

    /**
     * Must be called by Robot::robotPeriodic.
     * Updates vision simulation (processes camera results and updates pose estimator).
     */
    public void robotPeriodic() {
        m_visionSim.periodic();
    }

    /**
     * Must be called by Robot::simulationPeriodic.
     * Updates physics simulation and vision based on ground truth pose.
     */
    public void simulationPeriodic() {
        var driveState = m_drivetrain.getState();

        // Update ground truth physics simulation
        m_groundTruthSim.simulationPeriodic();

        // Update vision simulation with ground truth pose (not odometry)
        // This ensures cameras see AprilTags based on actual robot position
        Pose2d groundTruthPose = m_groundTruthSim.getGroundTruthPose();
        m_visionSim.simulationPeriodic(groundTruthPose);

        // Debug field visualization
        m_showVisionOnField.showEstimatedPoseAndWheels(
            ShowVisionOnField.FieldType.SIMULATION_FIELD, driveState);
        m_showVisionOnField.showGroundTruthPoseOnField(
            ShowVisionOnField.FieldType.SIMULATION_FIELD, groundTruthPose);
    }

    /**
     * Resets the simulated robot to a new pose.
     * Updates both ground truth physics and vision simulation.
     *
     * @param pose The new pose
     */
    public void resetSimPose(Pose2d pose) {
        m_groundTruthSim.resetGroundTruthPoseForSim(pose);
        m_visionSim.resetSimPose(pose);
    }

    /**
     * Transforms joystick inputs based on the operator's forward direction.
     * Static method that can be called without a SimWrapper instance.
     */
    public static JoystickInputsRecord transformJoystickOrientation(
            double degreesFieldForward,
            double driveX,
            double driveY,
            double rotateX) {
        return SimJoystickOrientation.simTransformJoystickOrientation(
            degreesFieldForward, driveX, driveY, rotateX);
    }

    /**
     * Proxy call to ground truth sim to inject odometry drift.
     */
    public void injectDrift(double translationOffsetMeters, double rotationOffsetDegrees) {
        m_groundTruthSim.injectDrift(translationOffsetMeters, rotationOffsetDegrees);
    }

    /**
     * Proxy call to ground truth sim to cycle reset position.
     *
     * @param blueAlliancePose The pose to reset to if on blue alliance
     */
    public void cycleResetPosition(Pose2d blueAlliancePose) {
        m_groundTruthSim.cycleResetPosition(blueAlliancePose);
    }

    /**
     * Get the simulation debug Field2d for visualization.
     *
     * @return The VisionSystemSim's debug field, or null if not in simulation
     */
    public Field2d getSimDebugField() {
        return m_visionSim.getSimDebugField();
    }
}
