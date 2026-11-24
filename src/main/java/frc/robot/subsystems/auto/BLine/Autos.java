package frc.robot.subsystems.auto.BLine;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedAutoChooser;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Autos {

  private Subsystems systems;

  private final LoggedAutoChooser autoChooser = new LoggedAutoChooser("Auto Chooser");
  private final LoggedNetworkNumber autoDelay = new LoggedNetworkNumber("Auto Delay", 0.0);
  private final LoggedNetworkBoolean autoMirror =
      new LoggedNetworkBoolean("Mirrored Auto(Left Auto)", false);
  private FollowPath.Builder pathBuilder;
  private FollowPath.Builder noResetPathBuilder;
  private SwerveRequest.PointWheelsAt pointWheelsAt = new PointWheelsAt();
  private final NetworkTableEntry autoDelayEntry =
      NetworkTableInstance.getDefault().getTable("Autos").getEntry("Auto Delay");

  private boolean pathsInitialized = false;
  private boolean autosRegistered = false;

  private final List<Path> paths = new ArrayList<>();
  public final Map<String, List<Path>> pathsMap = new LinkedHashMap<>();

  // ============= BLINE PATHS =============

  private final Path Path1 = new Path("FirstNeutralTrench");
  private final Path Path2 = new Path("SecondNeutralTrench");
  private final Path Path3 = new Path("default");
  private final Field2d field = new Field2d();

  public Supplier<Pose2d> TRENCH_START_LEFT;

  // ============= CONFIGURATION =============

  private FollowPath.Builder createPathBuilder(Subsystems s, FollowPath.Builder builder) {

    builder =
        new FollowPath.Builder(
                s.drivebaseSubsystem,
                () -> s.drivebaseSubsystem.getState().Pose,
                () -> s.drivebaseSubsystem.getState().Speeds,
                (speeds) ->
                    s.drivebaseSubsystem.setControl(
                        new SwerveRequest.ApplyRobotSpeeds()
                            .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0))
            .withDefaultShouldFlip()
            .withShouldMirror(() -> autoMirror.get());
    registerAutos();
    return builder;
  }

  public void init(Subsystems s) {

    systems = s;

    initializePaths();
    FollowPath.registerEventTrigger("intake", AutosCommands.intakeCommand());
    FollowPath.registerEventTrigger("shoot", AutosCommands.launcherCommand(5, s));
  }

  public void configure(Subsystems s) {
    pathBuilder =
        createPathBuilder(s, pathBuilder)
            .withPoseReset(pose -> s.drivebaseSubsystem.resetPose(pose));
    noResetPathBuilder = createPathBuilder(s, noResetPathBuilder).withPoseReset(pose -> {});
  }

  public void unitTestInit() {
    initializePaths();
  }

  // ============= PATH INITIALIZATION =============

  public void initializePaths() {

    if (pathsInitialized) return;

    pathsMap.put("SingleNeutralTrench", List.of(Path1));
    pathsMap.put("DoubleNeutralTrench", List.of(Path1, Path2));
    pathsMap.put("Default Auto", List.of(Path3));

    pathsMap.values().forEach(pathList -> pathList.forEach(p -> paths.add(p)));
    TRENCH_START_LEFT = AllianceFlipUtil.flipped(Path1.getStartPose());

    pathsInitialized = true;
  }

  public List<Path> getPaths() {
    return paths;
  }

  public Path getPath(int index) {
    return paths.get(index);
  }

  public Map<String, List<Path>> getMap() {
    return pathsMap;
  }

  public Path getPathsToDraw() {
    Path ultraPath = new Path();
    for (Path path : getPaths()) {
      for (PathElement element : path.getPathElements()) {
        ultraPath.addPathElement(element);
      }
    }
    return ultraPath;
  }

  public Path getSelectedPath() {
    String selectedAuto = autoChooser.getSelected();
    List<Path> selectedPaths = pathsMap.get(selectedAuto);

    if (selectedPaths == null || selectedPaths.isEmpty()) {
      throw new IllegalStateException(
          "No BLine path for selected auto '"
              + selectedAuto
              + "'. Expected one of: "
              + pathsMap.keySet());
    }

    return selectedPaths.get(0);
  }

  public List<String> getBLinePathsNames() {
    return new ArrayList<>(pathsMap.keySet());
  }

  // ============= AUTO REGISTRATION =============

  private void registerAutos() {

    if (autosRegistered) return;
    autoChooser.addCmd(
        "Default Auto",
        () ->
            Commands.sequence(
                Commands.waitSeconds(getDelay()),
                buildPath(Path3, true),
                AutosCommands.launcherCommand(systems)));

    autoChooser.addCmd(
        "SingleNeutralTrench",
        () ->
            Commands.sequence(
                Commands.waitSeconds(getDelay()),
                buildPath(Path1, true),
                AutosCommands.launcherCommand(systems)));

    autoChooser.addCmd(
        "DoubleNeutralTrench",
        () ->
            Commands.sequence(
                Commands.waitSeconds(getDelay()),
                buildPath(Path1, true),
                AutosCommands.launcherCommand(5, systems),
                buildPath(Path2, false),
                AutosCommands.launcherCommand(systems)));

    autosRegistered = true;
  }

  // ============= AUTO SELECTION =============

  public Command selectedCommand() {
    return autoChooser.selectedCommand();
  }

  /*
   * Return the selected start in blue field coordinates with the mirror setting applied.
   * GroundTruthSim applies the alliance flip when cycling reset positions.
   */
  public Pose2d getSelectedAutoStartingPose() {
    return getSelectedAutoStartingPose(false);
  }

  /*
   * Match B-line's flip-then-mirror order for the actual autonomous start.
   * Use this pose before the path command performs its own pose reset.
   */
  public Pose2d getSelectedAutoStartingPoseForAlliance() {
    return getSelectedAutoStartingPose(true);
  }

  /*
   * Apply B-line's flip-then-mirror transforms to a copy of the path.
   * Leave the selected source path unchanged for future runs.
   */
  private Pose2d getSelectedAutoStartingPose(boolean flipForAlliance) {
    Path startPath = getSelectedPath().copy();
    if (flipForAlliance && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      startPath.flip();
    }
    if (isMirrored()) {
      startPath.mirror();
    }
    return startPath.getStartPose();
  }

  public Autos get() {
    return this;
  }

  public List<Path> getSelectedPaths() {
    String selectedAuto = autoChooser.getSelected();
    return pathsMap.getOrDefault(selectedAuto, List.of(Path3));
  }

  // ============= AUTO BUILDER HELPERS =============s

  private Command buildPath(Path path, boolean resetPose) {
    if (systems == null || pathBuilder == null || noResetPathBuilder == null) {
      return Commands.none();
    }

    return (resetPose ? pathBuilder : noResetPathBuilder).build(path);
  }

  public boolean isMirrored() {
    return autoMirror.getAsBoolean();
  }

  public double getDelay() {
    double delay = autoDelayEntry.getDouble(0.0);
    return delay;
  }
}
