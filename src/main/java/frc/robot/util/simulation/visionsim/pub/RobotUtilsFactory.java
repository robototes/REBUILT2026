package frc.robot.util.simulation.visionsim.pub;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.simulation.visionsim.faultydrivemanager.FaultyDriveManager;
import frc.robot.util.simulation.visionsim.groundtruthsim.GroundTruthSim;
import frc.robot.util.simulation.visionsim.joystickinput.JoystickInput;
import frc.robot.util.simulation.visionsim.pub.interfaces.CameraInfoList;
import frc.robot.util.simulation.visionsim.pub.interfaces.FaultyDriveManagerInterface;
import frc.robot.util.simulation.visionsim.pub.interfaces.GroundTruthSimInterface;
import frc.robot.util.simulation.visionsim.pub.interfaces.JoystickInputInterface;
import frc.robot.util.simulation.visionsim.pub.interfaces.SimLimelightProducerInterface;
import frc.robot.util.simulation.visionsim.simlimelightproducer.SimLimelightProducer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/** Factory for robot utility objects. */
public class RobotUtilsFactory {

  /**
   * Creates a joystick input processor.
   *
   * @param rawxSupplier supplier for forward/back axis
   * @param rawySupplier supplier for strafe axis
   * @param rawRotateSupplier supplier for rotation axis
   * @param isSimulation {@code true} when running in simulation
   * @param operatorForwardDegreesSupplier supplier for operator forward direction in degrees
   * @return configured joystick input processor
   */
  public JoystickInputInterface createJoystickInput(
      DoubleSupplier rawxSupplier,
      DoubleSupplier rawySupplier,
      DoubleSupplier rawRotateSupplier,
      boolean isSimulation,
      DoubleSupplier operatorForwardDegreesSupplier) {

    return new JoystickInput(
        rawxSupplier,
        rawySupplier,
        rawRotateSupplier,
        isSimulation,
        operatorForwardDegreesSupplier);
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
      GroundTruthSimInterface groundTruthSim, SimLimelightProducerInterface simLimelightProducer) {
    return new FaultyDriveManager(groundTruthSim, simLimelightProducer);
  }

  /**
   * Creates a GroundTruthSimInterface instance if running in simulation mode.
   *
   * @param drivetrain The swerve drivetrain to track and manipulate
   * @param poseResetConsumer Consumer to be called when pose is reset
   * @return A GroundTruthSimInterface instance, or null if not in simulation
   */
  public GroundTruthSimInterface createGroundTruthSim(
      SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain, Consumer<Pose2d> poseResetConsumer) {
    if (RobotBase.isSimulation()) {
      return new GroundTruthSim(drivetrain, poseResetConsumer);
    }
    return null;
  }
}
