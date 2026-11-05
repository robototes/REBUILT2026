package frc.robot.subsystems.auto.BLine;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Autos {

  private Subsystems systems;

  private final AutoChooser autoChooser = new AutoChooser();

  private FollowPath.Builder pathBuilder;

  private boolean pathsInitialized = false;
  private boolean autosRegistered = false;

  private final List<Path> paths = new ArrayList<>();
  private final Map<String, List<Path>> pathsMap = new LinkedHashMap<>();

  // ============= BLINE PATHS =============

  private final Path Path1 = new Path("FirstNeutralTrench");
  private final Path Path2 = new Path("SecondNeutralTrench");

  public Supplier<Pose2d> TRENCH_START_LEFT;

  // ============= CONFIGURATION =============

  public void configure(Subsystems s) {

    systems = s;

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
            .withDefaultShouldFlip();

    registerAutos();
  }

  public void init(Subsystems s) {
    systems = s;
    initializePaths();
    FollowPath.registerEventTrigger("intake", AutosCommands.intakeCommand());
    FollowPath.registerEventTrigger("shoot", AutosCommands.launcherCommand(5, s));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void unitTestInit() {
    initializePaths();
  }

  // ============= PATH INITIALIZATION =============

  public void initializePaths() {

    if (pathsInitialized) return;

    pathsMap.put("SingleNeutralTrench", List.of(Path1));
    pathsMap.put("bumptotrench", List.of(Path1, Path2));

    pathsMap.values().forEach(pathList -> pathList.forEach(p -> paths.add(p)));
    TRENCH_START_LEFT = AllianceFlipUtil.flipped(Path1.getStartPose());

    pathsInitialized = true;
  }

  public List<Path> getPaths() {
    return paths;
  }

  public Map<String, List<Path>> getMap() {
    return pathsMap;
  }

  public List<String> getBLinePathsNames() {
    return new ArrayList<>(pathsMap.keySet());
  }

  // ============= AUTO REGISTRATION =============

  private void registerAutos() {

    if (autosRegistered) return;

    autoChooser.addCmd(
        "SingleNeutralTrench",
        () -> auto(TRENCH_START_LEFT.get(), c -> c.addCommands(pathBuilder.build(Path1), AutosCommands.launcherCommand(systems))));

    autoChooser.addCmd(
        "DoubleNeutralTrench",
        () ->
            auto(
                AllianceFlipUtil.flipped(Path2.getStartPose()).get(),
                c -> c.addCommands(pathBuilder.build(Path1), AutosCommands.launcherCommand(5,systems), pathBuilder.build(Path2), AutosCommands.launcherCommand(systems))));

    autosRegistered = true;
  }

  // ============= AUTO SELECTION =============

  public Command selectedCommand() {
    Command selected = autoChooser.selectedCommand();
    return selected != null ? selected : Commands.none();
  }

  // ============= AUTO BUILDER HELPERS =============

  private Command auto(Pose2d startPose, Consumer<SequentialCommandGroup> builder) {

    SequentialCommandGroup group = new SequentialCommandGroup();

    group.addCommands(
        Commands.runOnce(
            () -> systems.drivebaseSubsystem.resetPose(startPose), systems.drivebaseSubsystem));

    builder.accept(group);

    return group;
  }

  private Command auto(Consumer<SequentialCommandGroup> builder) {

    SequentialCommandGroup group = new SequentialCommandGroup();

    builder.accept(group);

    return group;
  }
}
