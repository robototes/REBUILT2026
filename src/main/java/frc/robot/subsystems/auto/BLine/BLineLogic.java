package frc.robot.subsystems.auto.BLine;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.Misc.DynamicSendableChooser;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.util.simulation.RobotSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

public class BLineLogic {

  private static Subsystems s;
  public static Field2d field = new Field2d();
  public static Field2d fieldPoseStart = new Field2d();

  private static final Pose2d RIGHT_TRENCH_POSE =
      new Pose2d(4.013, 0.473, Rotation2d.fromDegrees(-90));
  private static final Pose2d LEFT_TRENCH_POSE =
      new Pose2d(4.013, 7.597, Rotation2d.fromDegrees(90));

  public enum StartPosition {
    TRENCH("Trench", new Pose2d(4.013, 0.473, Rotation2d.fromDegrees(-90))),
    CENTER("Center", new Pose2d(3.600, 4.035, Rotation2d.fromDegrees(0))),
    MISC("Misc", new Pose2d());

    public final String title;
    public final Pose2d startPose;

    StartPosition(String title, Pose2d startPose) {
      this.title = title;
      this.startPose = startPose;
    }
  }

  public enum TrenchSide {
    RIGHT("Right"),
    LEFT("Left");

    public final String title;

    TrenchSide(String title) {
      this.title = title;
    }
  }

  private static final List<BLinePath> autos = new ArrayList<>();
  private static final SendableChooser<StartPosition> startPositionChooser =
      new SendableChooser<>();
  private static final SendableChooser<TrenchSide> trenchSideChooser = new SendableChooser<>();
  private static final DynamicSendableChooser<String> autoChooser = new DynamicSendableChooser<>();
  private static final SendableChooser<Integer> gameObjects = new SendableChooser<>();

  private static final NetworkTableEntry autoDelayEntry =
      NetworkTableInstance.getDefault().getTable("Autos").getEntry("Auto Delay");

  public static final String keys = "RB=Right Bump, LB=Left Bump, LT=Left Trench, RT=Right Trench";

  private static BLinePath defaultPath;
  private static List<BLinePath> rebuiltPaths = List.of();
  private static Map<Integer, List<BLinePath>> commandsMap = Map.of();
  private static final Map<String, BLinePath> namesToAuto = new HashMap<>();

  private static boolean pathsInitialized = false;
  private static Command bLineLaunching;
  private static Command bLineSimLaunching;

  public static FollowPath.Builder pathBuilder;
  private static FollowPath.Builder continuingPathBuilder;

  // ========================= MIRRORING =========================

  public static boolean isMirrored() {
    return startPositionChooser.getSelected() == StartPosition.TRENCH
        && trenchSideChooser.getSelected() == TrenchSide.LEFT;
  }

  // ========================= INIT =========================

  public static void init(Subsystems subsystems) {
    s = subsystems;
    registerCommands();

    if (pathsInitialized) return;

    initializePaths();
    pathsInitialized = true;
  }

  public static void unitTestInit() {
    s = null; // Explicitly set to null for unit tests
    initializePaths();
  }

  private static void initializePaths() {
    defaultPath = new BLinePath("default", "Center", "default");

    rebuiltPaths =
        List.of(
            defaultPath,
            new BLinePath("TrenchNeutral", "RT", "FirstNeutralTrench"),
            new BLinePath("DoubleTrenchNeutral", "RT", "FirstNeutralTrench", "SecondNeutralTrench"),
            new BLinePath("BumpNeutral", "RT", "FirstNeutralBump"),
            new BLinePath("DoubleBumpNeutral", "RT", "FirstNeutralBump", "SecondNeutralBump"),
            new BLinePath("BumpNeutralDepot", "RT", "FirstNeutralBump"));

    autos.clear();
    autos.addAll(rebuiltPaths);
    commandsMap = Map.of(0, rebuiltPaths);

    namesToAuto.clear();
    for (List<BLinePath> list : commandsMap.values()) {
      for (BLinePath auto : list) {
        handleStartingPoses(auto);
        namesToAuto.put(auto.getDisplayName(), auto);
      }
    }
  }

  public static void handleStartingPoses(BLinePath path) {
    switch (path.getStartingPosName()) {
      case "RT":
      case "LT":
        path.setStartPose2d(StartPosition.TRENCH);
        break;
      case "Center":
        path.setStartPose2d(StartPosition.CENTER);
        break;
      default:
        path.setStartPose2d(StartPosition.MISC);
        break;
    }
  }

  public static void configure(Subsystems s) {
    pathBuilder = createPathBuilder(s).withPoseReset(pose -> s.drivebaseSubsystem.resetPose(pose));
    continuingPathBuilder = createPathBuilder(s);
  }

  private static FollowPath.Builder createPathBuilder(Subsystems s) {
    return new FollowPath.Builder(
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
        .withShouldMirror(BLineLogic::isMirrored);
  }

  // ========================= LOGGING =========================

  public static void initSmartDashboard() {
    startPositionChooser.setDefaultOption(StartPosition.MISC.title, StartPosition.MISC);
    for (StartPosition pos : StartPosition.values()) {
      startPositionChooser.addOption(pos.title, pos);
    }

    trenchSideChooser.setDefaultOption(TrenchSide.RIGHT.title, TrenchSide.RIGHT);
    trenchSideChooser.addOption(TrenchSide.LEFT.title, TrenchSide.LEFT);

    gameObjects.setDefaultOption("0", 0);
    filterAutos(0);

    SmartDashboard.putData("Selected auto", field);
    SmartDashboard.putData("Start pose", fieldPoseStart);
    SmartDashboard.putData("Starting Position", startPositionChooser);
    SmartDashboard.putData("Trench Side", trenchSideChooser);
    SmartDashboard.putData("Auto Mode", gameObjects);
    SmartDashboard.putData("Available Auto Variants", autoChooser);
    SmartDashboard.putString("Auto Key", keys);

    autoDelayEntry.setDouble(0.0);

    startPositionChooser.onChange(
        v -> {
          filterAutos(gameObjects.getSelected());
          updateInitialHeading();
          updateFieldDisplay();
        });

    trenchSideChooser.onChange(
        v -> {
          updateInitialHeading();
          updateFieldDisplay();
        });

    autoChooser.onChange(
        v -> {
          updateInitialHeading();
          updateFieldDisplay();
        });

    updateFieldDisplay();
  }

  public static void updateFieldDisplay() {
    fieldPoseStart.setRobotPose(getSelectedAutoStartingPose());
  }

  static Pose2d getTrenchPose() {
    return isMirrored() ? LEFT_TRENCH_POSE : RIGHT_TRENCH_POSE;
  }

  public static void filterAutos(int numGameObjects) {
    autoChooser.clearOptions();

    StartPosition selected = startPositionChooser.getSelected();
    if (selected == null) selected = StartPosition.MISC;

    for (BLinePath auto : autos) {
      if (selected == StartPosition.MISC) {
        autoChooser.addOption(auto.getDisplayName(), auto.getDisplayName());
        continue;
      }

      if (auto.getStartPositionType() == selected) {
        autoChooser.addOption(auto.getDisplayName(), auto.getDisplayName());
      }
    }
  }

  // ========================= SELECTION METHODS =========================

  public static String getSelectedAutoName() {
    if (autoChooser.getSelected() == null) {
      return "Default";
    }
    return autoChooser.getSelected();
  }

  public static BLinePath getSelectedAutoPath() {
    String selectedName = autoChooser.getSelected();
    if (selectedName == null) return defaultPath;
    return namesToAuto.getOrDefault(selectedName, defaultPath);
  }

  public static Pose2d getSelectedAutoStartingPose() {
    BLinePath selected = getSelectedAutoPath();
    if (selected == null) return Pose2d.kZero;

    if (startPositionChooser.getSelected() == StartPosition.TRENCH) {
      return isMirrored() ? LEFT_TRENCH_POSE : RIGHT_TRENCH_POSE;
    }

    return selected.getStartPose2d();
  }

  // ========================= AUTO EXECUTION =========================
  public static List<Path> getPathsToBuild() {
    BLinePath selected = getSelectedAutoPath();
    if (selected == null) {
      return List.of();
    }
    return selected.getAllPaths();
  }

  public static Command getSelectedAuto() {
    if (s == null) {
      // Unit test mode - return empty command
      return Commands.none();
    }

    double delay = autoDelayEntry.getDouble(0.0);
    BLinePath selected = getSelectedAutoPath();

    if (selected == null) {
      return Commands.none();
    }

    s.drivebaseSubsystem.resetRotation(selected.getPath().getInitialModuleDirection());

    List<Command> commands = new ArrayList<>();
    commands.add(Commands.waitSeconds(delay));

    List<Path> paths = getPathsToBuild();
    for (int i = 0; i < paths.size(); i++) {
      boolean resetPose = (i == 0);
      commands.add(buildPath(paths.get(i), resetPose));
    }

    return Commands.sequence(commands.toArray(new Command[0]));
  }

  public static List<BLinePath> getBLinePaths() {
    return rebuiltPaths;
  }

  public static List<String> getBLinePathsNames() {
    List<String> pathsNames = new ArrayList<>();
    for (BLinePath path : getBLinePaths()) {
      pathsNames.addAll(path.getDisplayingNames());
    }
    return pathsNames;
  }

  public static Command buildSingleNeutralTrenchAuto() {
    return Commands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralTrench"), true),
        launcherCommand());
  }

  public static Command buildDoubleNeutralTrenchAuto() {
    return BLineCommands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralTrench"), true),
        launcherCommand(4.5),
        buildPath(new Path("SecondNeutralTrench"), false),
        launcherCommand());
  }

  public static Command buildSingleNeutralBumpAuto() {
    return Commands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralBump"), true),
        launcherCommand());
  }

  public static Command buildDoubleNeutralBumpAuto() {
    return BLineCommands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralBump"), true),
        buildPath(new Path("BumpToTrench"), false),
        buildPath(new Path("SecondNeutralBump"), false),
        launcherCommand());
  }

  public static Command buildSingleNeutralBumpDepotAuto() {
    return BLineCommands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralBump"), true),
        buildPath(new Path("BumpDepot"), false),
        launcherCommand());
  }

  public static Command buildDefaultAuto() {
    return BLineCommands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("Default"), true),
        launcherCommand());
  }

  private static Command buildPath(Path path, boolean resetPose) {
    if (s == null || pathBuilder == null || continuingPathBuilder == null) {
      return Commands.none();
    }
    return (resetPose ? pathBuilder : continuingPathBuilder).build(path);
  }

  public static Command handleAutos() {
    switch (getSelectedAutoName()) {
      case "TrenchNeutral":
        return buildSingleNeutralTrenchAuto();
      case "DoubleTrenchNeutral":
        return buildDoubleNeutralTrenchAuto();
      case "BumpNeutral":
        return buildSingleNeutralBumpAuto();
      case "DoubleBumpNeutral":
        return buildDoubleNeutralBumpAuto();
      case "BumpNeutralDepot":
        return buildSingleNeutralBumpDepotAuto();
      default:
        return buildDefaultAuto();
    }
  }

  private static void updateInitialHeading() {
    BLinePath selected = getSelectedAutoPath();
    if (selected == null || selected.getPath() == null) {
      SmartDashboard.putNumber("Initial Heading(Deg)", 0.0);
      return;
    }
    Pose2d start = selected.getPath().getStartPose();
    SmartDashboard.putNumber("Initial Heading(Deg)", Math.round(start.getRotation().getDegrees()));
  }

  // ========================= COMMANDS =========================

  public static Command intakeCommand() {
    return Commands.runOnce(() -> Controls.intakeMode = IntakeMode.INTAKE)
        .withName("Auto Intake Command");
  }

  public static Command launcherCommand(double timeout) {
    if (s == null || Robot.isSimulation()) return RobotSim.launch(s, timeout);
    return Commands.parallel(
            Commands.runOnce(() -> s.flywheels.resetFuelCheck()),
            s.launcherSubsystem.launcherAimCommand(),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
        .withTimeout(timeout)
        .andThen(s.launcherSubsystem.rawStowCommand())
        .withName("Auto Launcher Command");
  }

  public static Command launcherCommand() {
    if (s == null) return Commands.none();
    return Commands.parallel(
            Commands.runOnce(() -> s.flywheels.resetFuelCheck()),
            s.launcherSubsystem.launcherAimCommand(),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
        .withName("Auto Launcher Command");
  }

  public static Command stowCommand() {
    if (s == null) return Commands.none();
    return s.launcherSubsystem.rawStowCommand();
  }

  public static Command climbCommand() {
    return Commands.none().withName("Auto Climb Command");
  }

  public static void cancelCommand() {
    if (s == null) return;
    if (Robot.isSimulation()) {
      CommandScheduler.getInstance().cancel(bLineSimLaunching);
    } else {
      CommandScheduler.getInstance().cancel(bLineLaunching);
    }
  }

  private static void registerCommands() {
    if (s == null) return; // Skip registration during unit tests

    AtomicBoolean launchAllowed = new AtomicBoolean(true);

    if (s.launcherSubsystem != null && s.indexerSubsystem != null) {
      if (Robot.isSimulation()) {
        bLineSimLaunching = RobotSim.launch(s, 30);
        FollowPath.registerEventTrigger(
            "launch",
            Commands.runOnce(() -> launchAllowed.set(true))
                .andThen(bLineSimLaunching.onlyWhile(launchAllowed::get))
                .andThen(Commands.print("LAUNCH FINISHED")));
      } else {
        bLineLaunching = launcherCommand();
        FollowPath.registerEventTrigger("launch", bLineLaunching);
      }
    }

    FollowPath.registerEventTrigger("intake", intakeCommand());
    FollowPath.registerEventTrigger("climb", climbCommand());
    FollowPath.registerEventTrigger(
        "cancel", Commands.runOnce(() -> launchAllowed.set(false)).andThen(stowCommand()));
  }
}
