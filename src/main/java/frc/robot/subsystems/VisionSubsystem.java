package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.Hardware;
import frc.robot.Subsystems;
import frc.robot.util.LLCamera;

public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names

  private static final String LIMELIGHT_B = Hardware.LIMELIGHT_B;
  private static double limelightbTX;
  private final Subsystems subsystems = new Subsystems();
  // Deviations
  private static final Vector<N3> STANDARD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));
  private static final Vector<N3> DISTANCE_SC_STANDARD_DEVS =
      VecBuilder.fill(1, 1, Units.degreesToRadians(50));
  private final SwerveRequest.FieldCentricFacingAngle autoOrient =
      new SwerveRequest.FieldCentricFacingAngle();

  // AprilTag field layout for 2025
  private static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private final Controls controls = new Controls(subsystems);
  double lastBestDistance = 0;

  private final DrivebaseWrapper drivebaseWrapper;

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private final GenericSubscriber disableVision;
  private final LLCamera BCamera = new LLCamera(LIMELIGHT_B);

  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryLeft =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dLeft", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryRight =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dRight", Pose3d.struct)
          .publish();
}
