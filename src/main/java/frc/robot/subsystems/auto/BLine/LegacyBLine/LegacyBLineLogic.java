package frc.robot.subsystems.auto.BLine.LegacyBLine;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import frc.robot.subsystems.auto.BLine.BLineLogic;
import frc.robot.subsystems.auto.BLine.BLineLogic.Position;
import frc.robot.subsystems.auto.BLine.BLineTriggers;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.util.simulation.RobotSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class LegacyBLineLogic {

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

  private static final List<LegacyBLinePath> autos = new ArrayList<>();
  private static final LoggedDashboardChooser<Position> startPositionChooser =
      new LoggedDashboardChooser<>("BLine/Start Position");
  private static final LoggedDashboardChooser<TrenchSide> trenchSideChooser =
      new LoggedDashboardChooser<>("BLine2/Trench Side");
  private static final LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  private static final LoggedDashboardChooser<Integer> gameObjects =
      new LoggedDashboardChooser<>("BLine/Game Objects");
  private static final LoggedNetworkNumber initialHeading =
      new LoggedNetworkNumber("BLine/Initial Heading (Deg)", 0);
  private static final NetworkTableEntry autoDelayEntry =
      NetworkTableInstance.getDefault().getTable("BLine Delays").getEntry("Auto Delay");

  public static final String keys = "RB=Right Bump, LB=Left Bump, LT=Left Trench, RT=Right Trench";
  private static final LoggedNetworkString loggedKeys =
      new LoggedNetworkString("BLine/Auto Key", keys);
  private static LegacyBLinePath defaultPath;
  private static List<LegacyBLinePath> rebuiltPaths = List.of();
  private static Map<Integer, List<LegacyBLinePath>> commandsMap = Map.of();
  private static final Map<String, LegacyBLinePath> namesToAuto = new HashMap<>();

  private static boolean pathsInitialized = false;
  private static Command bLineLaunching;
  private static Command bLineSimLaunching;

  public static FollowPath.Builder pathBuilder;
  private static FollowPath.Builder continuingPathBuilder;

  // ========================= MIRRORING =========================

  public static boolean isMirrored() {
    return startPositionChooser.getSendableChooser().getSelected() == Position.TRENCH.title
        && trenchSideChooser.getSendableChooser().getSelected() == TrenchSide.LEFT.title;
  }

  // ========================= INIT =========================

  public static void init(Subsystems subsystems) {
    s = subsystems;
    registerCommands(s);

    if (pathsInitialized) return;

    initializePaths();
    pathsInitialized = true;
  }

  public static void unitTestInit() {
    s = null; // Explicitly set to null for unit tests
    initializePaths();
  }

  private static void initializePaths() {
    defaultPath = new LegacyBLinePath("default", "Center", "default");

    rebuiltPaths =
        List.of(
            defaultPath,
            new LegacyBLinePath("TrenchNeutral", "RT", "FirstNeutralTrench"),
            new LegacyBLinePath(
                "DoubleTrenchNeutral", "RT", "FirstNeutralTrench", "SecondNeutralTrench"),
            new LegacyBLinePath("Depot", "RT", "TrenchDepot"),
            new LegacyBLinePath("TrenchNeutralDepot", "RT", "FirstNeutralTrench", "TrenchDepot"));
    autos.clear();
    autos.addAll(rebuiltPaths);
    commandsMap = Map.of(0, rebuiltPaths);

    namesToAuto.clear();
    for (List<LegacyBLinePath> list : commandsMap.values()) {
      for (LegacyBLinePath auto : list) {
        handleStartingPoses(auto);
        namesToAuto.put(auto.getDisplayName(), auto);
      }
    }
  }

  public static void handleStartingPoses(LegacyBLinePath path) {
    switch (path.getStartingPosName()) {
      case "RT":
      case "LT":
        path.setStartPose2d(Position.TRENCH);
        break;
      case "Center":
        path.setStartPose2d(Position.CENTER);
        break;
      default:
        path.setStartPose2d(Position.MISC);
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

  public static void initAdvantageKit() {
    startPositionChooser.addDefaultOption(Position.MISC.title, Position.MISC);
    for (Position pos : Position.values()) {
      startPositionChooser.addOption(pos.title, pos);
    }

    trenchSideChooser.addDefaultOption(TrenchSide.RIGHT.title, TrenchSide.RIGHT);
    trenchSideChooser.addOption(TrenchSide.LEFT.title, TrenchSide.LEFT);

    gameObjects.addDefaultOption("0", 0);
    filterAutos(0);

    SmartDashboard.putData("BLine/Selected auto", field);
    SmartDashboard.putData("BLine/Start pose", fieldPoseStart);

    autoDelayEntry.setDouble(0.0);
    startPositionChooser.onChange(
        v -> {
          filterAutos(Integer.valueOf(gameObjects.getSendableChooser().getSelected()));
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
    // autoChooser.clearOptions();
    Position selected;
    String selectedString = startPositionChooser.getSendableChooser().getSelected();
    switch (selectedString) {
      case "Trench":
        selected = Position.TRENCH;
        break;
      case "Center":
        selected = Position.CENTER;
        break;
      default:
        selected = Position.MISC;
        break;
    }

    if (selected == null) selected = Position.MISC;

    for (LegacyBLinePath auto : autos) {
      if (selected == Position.MISC) {
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
    if (autoChooser.getSendableChooser().getSelected() == null) {
      return "Default";
    }
    return autoChooser.getSendableChooser().getSelected();
  }

  public static LegacyBLinePath getSelectedAutoPath() {
    String selectedName = autoChooser.getSendableChooser().getSelected();
    if (selectedName == null) return defaultPath;
    return namesToAuto.getOrDefault(selectedName, defaultPath);
  }

  public static Pose2d getSelectedAutoStartingPose() {
    LegacyBLinePath selected = getSelectedAutoPath();
    if (selected == null) return Pose2d.kZero;

    if (startPositionChooser.getSendableChooser().getSelected() == Position.TRENCH.title) {
      return isMirrored() ? LEFT_TRENCH_POSE : RIGHT_TRENCH_POSE;
    }

    return selected.getStartPose2d();
  }

  // ========================= AUTO EXECUTION =========================
  public static List<Path> getPathsToBuild() {
    LegacyBLinePath selected = getSelectedAutoPath();
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
    LegacyBLinePath selected = getSelectedAutoPath();

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

  public static List<LegacyBLinePath> getBLinePaths() {
    return rebuiltPaths;
  }

  public static List<String> getBLinePathsNames() {
    List<String> pathsNames = new ArrayList<>();
    for (LegacyBLinePath path : getBLinePaths()) {
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

  public static Command buildSingleDepotAuto() {
    return Commands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("TrenchDepot"), true),
        launcherCommand());
  }



  public static Command buildSingleNeutralTrenchDepotAuto() {
    return BLineCommands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralTrench"), true),
        launcherCommand(4.5),
        buildPath(new Path("TrenchDepot"), false),
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
      case "Depot":
        return buildSingleDepotAuto();
      case "TrenchNeutralDepot":
        return buildSingleNeutralTrenchDepotAuto();
      default:
        return buildDefaultAuto();
    }
  }

  private static void updateInitialHeading() {
    LegacyBLinePath selected = getSelectedAutoPath();
    if (selected == null || selected.getPath() == null) {
      initialHeading.setDefault(0.0);
      return;
    }
    Pose2d start = selected.getPath().getStartPose();
    initialHeading.set(Math.round(start.getRotation().getDegrees()));
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

    if(Robot.isSimulation()) {
      return RobotSim.launch(s, 5);
    }
    if (s == null) return Commands.none();
    return Commands.parallel(
            Commands.runOnce(() -> s.flywheels.resetFuelCheck()),
            s.launcherSubsystem.launcherAimCommand(),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
        .withName("Auto Launcher Command UNLIMITED");
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

  private static void registerCommands(Subsystems s) {
    BLineTriggers.registerTriggers(s);
  }
}
