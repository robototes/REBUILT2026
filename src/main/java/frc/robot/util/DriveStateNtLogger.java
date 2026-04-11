package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class DriveStateNtLogger {
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final DriveStateSignalLogger telem;
  private final double MaxSpeed;

  /* Robot swerve drive state */
  private final NetworkTable driveStateTable = inst.getTable("DriveState");

  private final DoubleArrayPublisher drivePose =
      driveStateTable.getDoubleArrayTopic("Pose").publish();
  private final DoubleArrayPublisher driveSpeeds =
      driveStateTable.getDoubleArrayTopic("Speeds").publish();
  private final DoubleArrayPublisher driveModuleStates =
      driveStateTable.getDoubleArrayTopic("ModuleStates").publish();
  private final DoubleArrayPublisher driveModuleTargets =
      driveStateTable.getDoubleArrayTopic("ModuleTargets").publish();
  private final DoubleArrayPublisher driveModulePositions =
      driveStateTable.getDoubleArrayTopic("ModulePositions").publish();
  private final DoublePublisher driveTimestamp =
      driveStateTable.getDoubleTopic("Timestamp").publish();
  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] m_moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] m_moduleSpeeds =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] m_moduleDirections =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      };

  public DriveStateNtLogger(DriveStateSignalLogger telemetry, double MaxSpeed) {
    this.MaxSpeed = MaxSpeed;
    this.telem = telemetry;
    /* Telemeterize the module states to a Mechanism2d */
    for (int i = 0; i < 4; ++i) {
      SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
    }
    // Legacy double[] publisher has been removed entirely
  }

  // should be updated periodically in robot.periodic()
  public void update() {
    SwerveDriveState state = telem.returnDriveState();
    if (state == null) return;

    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
    }

    drivePose.set(
        new double[] {state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getRadians()});

    driveSpeeds.set(
        new double[] {
          state.Speeds.vxMetersPerSecond,
          state.Speeds.vyMetersPerSecond,
          state.Speeds.omegaRadiansPerSecond
        });

    double[] moduleStates = new double[8];
    double[] moduleTargets = new double[8];
    double[] modulePositions = new double[8];
    for (int i = 0; i < 4; ++i) {
      moduleStates[i * 2] = state.ModuleStates[i].angle.getRadians();
      moduleStates[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
      moduleTargets[i * 2] = state.ModuleTargets[i].angle.getRadians();
      moduleTargets[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
      modulePositions[i * 2] = state.ModulePositions[i].angle.getRadians();
      modulePositions[i * 2 + 1] = state.ModulePositions[i].distanceMeters;
    }
    driveModuleStates.set(moduleStates);
    driveModuleTargets.set(moduleTargets);
    driveModulePositions.set(modulePositions);

    driveTimestamp.set(state.Timestamp);
    driveOdometryFrequency.set(state.OdometryPeriod == 0 ? 0 : 1.0 / state.OdometryPeriod);
  }
}
