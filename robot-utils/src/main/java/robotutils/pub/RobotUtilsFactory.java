package robotutils.pub;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import robotutils.dashboard.DashboardManager;
import robotutils.drivesmooth.DriveSmooth;
import robotutils.faultydrivemanager.FaultyDriveManager;
import robotutils.groundtruthsim.GroundTruthSim;
import robotutils.groundtruthsim.GroundTruthSimDashboardProvider;
import robotutils.groundtruthsim.GroundTruthSimDashboardSettings;
import robotutils.joystickinput.JoystickInput;
import robotutils.perrobotconfig.PerRobotConfig;
import robotutils.perrobotconfig.PerRobotConfigDashboardProvider;
import robotutils.perrobotconfig.PerRobotConfigDashboardSettings;
import robotutils.pub.interfaces.CameraInfoList;
import robotutils.pub.interfaces.DriveSmoothInterface;
import robotutils.pub.interfaces.FaultyDriveManagerInterface;
import robotutils.pub.interfaces.GroundTruthSimInterface;
import robotutils.pub.interfaces.JoystickInputInterface;
import robotutils.pub.interfaces.MacKey;
import robotutils.pub.interfaces.PerRobotConfigInterface;
import robotutils.pub.interfaces.SimLimelightProducerInterface;
import robotutils.pub.interfaces.dashboard.DashboardManagerInterface;
import robotutils.pub.interfaces.dashboard.DashboardConstants;
import robotutils.pub.interfaces.dashboard.DashboardProviderInterface;
import robotutils.pub.interfaces.simio.ArmIoInterface;
import robotutils.pub.interfaces.simio.ElevatorIoInterface;
import robotutils.pub.interfaces.simio.RollerIoInterface;
import robotutils.pub.interfaces.simio.TwoMotorRollerIoInterface;
import robotutils.sim.armsim.ArmIoSim;
import robotutils.sim.elevatorssim.ElevatorIoSim;
import robotutils.sim.rollerssim.RollerIoSim;
import robotutils.sim.rollerssim.TwoMotorRollerIoSim;
import robotutils.simlimelightproducer.SimLimelightProducer;


/** Factory for robot utility objects. */
public class RobotUtilsFactory {

    /** Creates a dashboard manager. */
    public DashboardManagerInterface createDashboardManager() {
        return new DashboardManager();
    }

    /** Creates a default drive smoothing pipeline. */
    public DriveSmoothInterface createDriveSmooth() {
        return new DriveSmooth();
    }

    /**
     * Creates a drive smoothing pipeline with explicit configuration.
     *
     * @param translationSlewRate max translation change per second
     * @param rotationSlewRate max rotation change per second
     * @param joystickDeadband deadband threshold in [0, 1)
     * @param translationExponent response exponent for translation
     * @param rotationExponent response exponent for rotation
     * @return configured drive smoothing pipeline
     */
    public DriveSmoothInterface createDriveSmooth(
        double translationSlewRate,
        double rotationSlewRate,
        double joystickDeadband,
        double translationExponent,
        double rotationExponent) {

        return new DriveSmooth(
            translationSlewRate,
            rotationSlewRate,
            joystickDeadband,
            translationExponent,
            rotationExponent);
    }

    /** Create using default params. */
    public DriveSmoothInterface createDefaultDriveSmooth() {
        return createDriveSmooth();
    }

    /**
     * Creates a joystick input processor.
     *
     * @param driveSmooth smoothing pipeline (deadband, curve, slew)
     * @param rawxSupplier supplier for forward/back axis
     * @param rawySupplier supplier for strafe axis
     * @param rawRotateSupplier supplier for rotation axis
     * @param finePositioningEnabledSupplier returns {@code true} when fine-positioning is enabled
     * @param teleoperatedSpeed maximum linear velocity in m/s
     * @param maxAngularRate maximum angular velocity in rad/s
     * @param isSimulation {@code true} when running in simulation
     * @param operatorForwardDegreesSupplier supplier for operator forward direction in degrees
     * @return configured joystick input processor
     */
    public JoystickInputInterface createJoystickInput(
        DriveSmoothInterface driveSmooth,
        DoubleSupplier rawxSupplier,
        DoubleSupplier rawySupplier,
        DoubleSupplier rawRotateSupplier,
        BooleanSupplier finePositioningEnabledSupplier,
        double teleoperatedSpeed,
        double maxAngularRate,
        boolean isSimulation,
        DoubleSupplier operatorForwardDegreesSupplier) {

        return new JoystickInput(
            driveSmooth,
            rawxSupplier,
            rawySupplier,
            rawRotateSupplier,
            finePositioningEnabledSupplier,
            teleoperatedSpeed,
            maxAngularRate,
            isSimulation,
            operatorForwardDegreesSupplier);
    }

    /**
     * Creates a per-robot configuration selector.
     *
     * @param optionalDashboardManager optional dashboard manager for reporting per-robot config settings
     * @param macToRobotNameDict maps MAC key suffixes to robot names
     * @param robotNameToConfigNameDict maps robot names to config names
     * @param configNameToConfigObjDict maps config names to config objects
     * @param defaultConfigName config name used when no MAC address matches
     * @param simulationConfigName config name used in simulation
     * @return configured per-robot config selector
     */
    public <T> PerRobotConfigInterface<T> createPerRobotConfig(
        Optional<DashboardManagerInterface> optionalDashboardManager,
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName) {

        Optional<DashboardProviderInterface<PerRobotConfigDashboardSettings>> optionalDashboardProvider;
        if (optionalDashboardManager.isPresent()) {
            PerRobotConfigDashboardProvider provider = new PerRobotConfigDashboardProvider();
            provider.init();
            optionalDashboardManager.get().registerProvider(DashboardConstants.kPerRobotConfigProviderName, provider);

            optionalDashboardProvider = Optional.of(provider);
        }
        else {
            optionalDashboardProvider = Optional.empty();
        }

        return new PerRobotConfig<T>(
            optionalDashboardProvider,
            macToRobotNameDict,
            robotNameToConfigNameDict,
            configNameToConfigObjDict,
            defaultConfigName,
            simulationConfigName);
    }

    /**
     * Creates a simulated arm IO implementation.
     *
     * @param deviceName simulation device name
     * @param minArmAngleDegrees minimum arm angle in degrees
     * @param maxArmAngleDegrees maximum arm angle in degrees
     * @param momentOfInertia arm moment of inertia
     * @param armLengthMeters arm length in meters
     * @param gearRatio arm gear ratio
     * @return configured arm IO simulation
     */
    public ArmIoInterface createArmIoSim(
        String deviceName,
        double minArmAngleDegrees,
        double maxArmAngleDegrees,
        double momentOfInertia,
        double armLengthMeters,
        double gearRatio) {

        return new ArmIoSim(
            deviceName,
            minArmAngleDegrees,
            maxArmAngleDegrees,
            momentOfInertia,
            armLengthMeters,
            gearRatio);
    }

    /**
     * Creates a simulated elevator IO implementation.
     *
     * @param deviceName simulation device name
     * @param gearRatio elevator gear ratio
     * @param carriageMassKg carriage mass in kilograms
     * @param drumRadiusMeters drum radius in meters
     * @param minHeightMeters minimum height in meters
     * @param maxHeightMeters maximum height in meters
     * @return configured elevator IO simulation
     */
    public ElevatorIoInterface createElevatorIoSim(
        String deviceName,
        double gearRatio,
        double carriageMassKg,
        double drumRadiusMeters,
        double minHeightMeters,
        double maxHeightMeters) {

        return new ElevatorIoSim(
            deviceName,
            gearRatio,
            carriageMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters);
    }

    /**
     * Creates a simulated single-motor roller IO implementation.
     *
     * @param deviceName simulation device name
     * @param momentOfInertia roller moment of inertia
     * @param gearRatio roller gear ratio
     * @return configured roller IO simulation
     */
    public RollerIoInterface createRollerIoSim(
        String deviceName,
        double momentOfInertia,
        double gearRatio) {

        return new RollerIoSim(
            deviceName,
            momentOfInertia,
            gearRatio);
    }

    /**
     * Creates a simulated two-motor roller IO implementation.
     *
     * @param deviceName simulation device name
     * @param momentOfInertia roller moment of inertia
     * @param gearRatio roller gear ratio
     * @return configured two-motor roller IO simulation
     */
    public TwoMotorRollerIoInterface createTwoMotorRollerIoSim(
        String deviceName,
        double momentOfInertia,
        double gearRatio) {

        return new TwoMotorRollerIoSim(
            deviceName,
            momentOfInertia,
            gearRatio);
    }

    /**
     * Creates a simulated Limelight producer when running in simulation.
     *
     * @param cameras configured camera list
     * @return a vision simulation instance, or null when not in simulation
     */
    public SimLimelightProducerInterface createSimLimelightProducer(CameraInfoList cameras) {
        if (RobotBase.isSimulation()) {
            return new SimLimelightProducer(cameras);
        }
        return null;
    }

    /**
     * Creates a FaultyDriveManager for injecting simulated hardware faults.
     *
     * @param groundTruthSim The ground truth sim to apply drivetrain faults to
     * @param simLimelightProducer The sim limelight producer to apply camera faults to
     * @return configured FaultyDriveManager
     */
    public FaultyDriveManagerInterface createFaultyDriveManager(
            GroundTruthSimInterface groundTruthSim,
            SimLimelightProducerInterface simLimelightProducer) {
        return new FaultyDriveManager(groundTruthSim, simLimelightProducer);
    }

    /**
     * Creates a GroundTruthSimInterface instance if running in simulation mode.
     *
     * @param optionalDashboardManager optional dashboard manager for reporting ground truth pose
     * @param drivetrain The swerve drivetrain to track and manipulate
     * @param poseResetConsumer Consumer to be called when pose is reset
     * @return A GroundTruthSimInterface instance, or null if not in simulation
     */
    public GroundTruthSimInterface createGroundTruthSim(
            Optional<DashboardManagerInterface> optionalDashboardManager,
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
            Consumer<Pose2d> poseResetConsumer) {
        if (RobotBase.isSimulation()) {
            Optional<DashboardProviderInterface<GroundTruthSimDashboardSettings>> optionalDashboardProvider;
            if (optionalDashboardManager.isPresent()) {
                GroundTruthSimDashboardProvider provider = new GroundTruthSimDashboardProvider();
                provider.init();
                optionalDashboardManager.get().registerProvider(DashboardConstants.kGroundTruthProviderName, provider);
                optionalDashboardProvider = Optional.of(provider);
            }
            else {
                optionalDashboardProvider = Optional.empty();
            }
            return new GroundTruthSim(drivetrain, poseResetConsumer, optionalDashboardProvider);
        }
        return null;
    }
}
