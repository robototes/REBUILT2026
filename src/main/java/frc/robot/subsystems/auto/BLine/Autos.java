package frc.robot.subsystems.auto.BLine;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Autos {

  private Subsystems systems;

  private final LoggedAutoChooser autoChooser = new LoggedAutoChooser("Auto Chooser");
  private final LoggedNetworkNumber autoDelay = new LoggedNetworkNumber("Auto Delay", 0.0);
  private FollowPath.Builder pathBuilder;
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

  public void configure(Subsystems s) {

    systems = s;
    autoDelayEntry.setDefaultDouble(0.0);
    initializePaths();

    pathBuilder =
        new FollowPath.Builder(
                s.drivebaseSubsystem,
                () -> s.drivebaseSubsystem.getState().Pose,
                () -> s.drivebaseSubsystem.getState().Speeds,
                speeds ->
                    s.drivebaseSubsystem.setControl(
                        new SwerveRequest.ApplyRobotSpeeds()
                            .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0))
            .withDefaultShouldFlip()
            .withPoseReset(pose -> s.drivebaseSubsystem.resetPose(pose));

    registerAutos();
  }

  public void init(Subsystems s) {

    systems = s;

    initializePaths();
    FollowPath.registerEventTrigger("intake", AutosCommands.intakeCommand());
    FollowPath.registerEventTrigger("shoot", AutosCommands.launcherCommand(5, s));
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
            auto(
                c ->
                    c.addCommands(
                        Commands.waitSeconds(getDelay()),
                        pathBuilder.build(Path3),
                        AutosCommands.launcherCommand(systems))));

    autoChooser.addCmd(
        "SingleNeutralTrench",
        () ->
            auto(
                c ->
                    c.addCommands(
                        Commands.waitSeconds(getDelay()),
                        pathBuilder.build(Path1),
                        AutosCommands.launcherCommand(systems))));

    autoChooser.addCmd(
        "DoubleNeutralTrench",
        () ->
            auto(
                c ->
                    c.addCommands(
                        Commands.waitSeconds(getDelay()),
                        pathBuilder.build(Path1),
                        AutosCommands.launcherCommand(5, systems),
                        pathBuilder.withPoseReset(pose -> {}).build(Path2),
                        AutosCommands.launcherCommand(systems))));

    autosRegistered = true;
  }

  // ============= AUTO SELECTION =============

  public Command selectedCommand() {
    return autoChooser.selectedCommand();
  }

  public Pose2d getSelectedAutoStartingPose() {

    return getPath(0).getStartPose();
  }

  public Autos get() {
    return this;
  }

  public List<Path> getSelectedPaths() {
    String selectedAuto = autoChooser.getSelected();
    return pathsMap.getOrDefault(selectedAuto, List.of(Path3));
  }

  // ============= AUTO BUILDER HELPERS =============s

  private Command auto(Pose2d startPose, Consumer<Command> builder) {

    SequentialCommandGroup group = new SequentialCommandGroup();

    // group.addCommands(
    // Commands.runOnce(
    //    () -> systems.drivebaseSubsystem.resetPose(startPose), systems.drivebaseSubsystem));

    builder.accept(
        Commands.runOnce(
            () ->
                systems.drivebaseSubsystem.resetRotation(
                    getSelectedPath().getInitialModuleDirection())));

    return group;
  }

  private Command auto(Consumer<SequentialCommandGroup> builder) {

    SequentialCommandGroup group = new SequentialCommandGroup();
    group.addCommands(
        Commands.runOnce(
            () ->
                systems.drivebaseSubsystem.resetRotation(
                    getSelectedPath().getInitialModuleDirection())));
    builder.accept(group);

    return group;
  }

  public double getDelay() {
    double delay = autoDelayEntry.getDouble(0.0);
    return delay;
  }
}
