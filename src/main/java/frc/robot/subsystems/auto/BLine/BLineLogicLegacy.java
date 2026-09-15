package frc.robot.subsystems.auto.BLine;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.util.simulation.RobotSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class BLineLogicLegacy {

  private static Subsystems s;

  public static Field2d field = new Field2d();
  public static Field2d fieldPoseStart = new Field2d();

  private static final Pose2d RIGHT_TRENCH_POSE =
      new Pose2d(4.013, 0.473, Rotation2d.fromDegrees(-90));

  private static final Pose2d LEFT_TRENCH_POSE =
      new Pose2d(4.013, 7.597, Rotation2d.fromDegrees(90));

  private static final String REMOVE_OPTION = "REMOVE";

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

  private static final LoggedDashboardChooser<StartPosition> startPositionChooser =
      new LoggedDashboardChooser<>("Start Position");

  private static final LoggedDashboardChooser<TrenchSide> trenchSideChooser =
      new LoggedDashboardChooser<>("Trench Side");

  private static LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("Available Auto Variants");

  private static final LoggedDashboardChooser<Integer> gameObjects =
      new LoggedDashboardChooser<>("Game Objects");

  private static final LoggedNetworkNumber initialHeading =
      new LoggedNetworkNumber("Initial Heading(Deg)");

  private static final NetworkTableEntry autoDelayEntry =
      NetworkTableInstance.getDefault().getTable("Autos").getEntry("Auto Delay");

  public static final String keys = "RB=Right Bump, LB=Left Bump, LT=Left Trench, RT=Right Trench";

  private static final LoggedNetworkString autoKeys = new LoggedNetworkString("Auto Key");

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

    String startPosition = startPositionChooser.getSendableChooser().getSelected();

    String trenchSide = trenchSideChooser.getSendableChooser().getSelected();

    return StartPosition.TRENCH.title.equals(startPosition)
        && TrenchSide.LEFT.title.equals(trenchSide);
  }

  static Pose2d getTrenchPose() {
    return isMirrored() ? LEFT_TRENCH_POSE : RIGHT_TRENCH_POSE;
  }

  // ========================= INIT =========================

  public static void init(Subsystems subsystems) {

    s = subsystems;

    if (!pathsInitialized) {
      initializePaths();
      pathsInitialized = true;
    }

    registerCommands();
  }

  public static void unitTestInit() {

    s = null;

    if (!pathsInitialized) {
      initializePaths();
      pathsInitialized = true;
    }
  }

  private static void initializePaths() {

    defaultPath = new BLinePath("default", "Center", "default");

    rebuiltPaths =
        List.of(
            defaultPath,
            new BLinePath("TrenchNeutral", "RT", "FirstNeutralTrench"),
            new BLinePath(
                "DoubleTrenchNeutral", "RT", "FirstNeutralTrench", "SecondNeutralTrench"));

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

  // ========================= START POSITIONS =========================

  public static void handleStartingPoses(BLinePath path) {

    switch (path.getStartingPosName()) {
      case "RT":
      case "LT":
        path.setStartPose2d(StartPosition.TRENCH.startPose);
        break;

      case "Center":
        path.setStartPose2d(StartPosition.CENTER.startPose);
        break;

      default:
        path.setStartPose2d(StartPosition.MISC.startPose);
        break;
    }
  }

  // ========================= PATH FOLLOWING =========================

  public static void configure(Subsystems subsystems) {

    pathBuilder =
        createPathBuilder(subsystems)
            .withPoseReset(pose -> subsystems.drivebaseSubsystem.resetPose(pose));

    continuingPathBuilder = createPathBuilder(subsystems);
  }

  private static FollowPath.Builder createPathBuilder(Subsystems subsystems) {

    return new FollowPath.Builder(
            subsystems.drivebaseSubsystem,
            () -> subsystems.drivebaseSubsystem.getState().Pose,
            () -> subsystems.drivebaseSubsystem.getState().Speeds,
            speeds ->
                subsystems.drivebaseSubsystem.setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0))
        .withDefaultShouldFlip()
        .withShouldMirror(BLineLogicLegacy::isMirrored);
  }

  // ========================= LOGGING =========================

  public static void initAdvantageKit() {

    // Make sure paths exist before using defaultPath.
    if (!pathsInitialized) {
      initializePaths();
      pathsInitialized = true;
    }

    /*
     * Start position
     *
     * MISC is the default, so don't add it twice.
     */
    startPositionChooser.addDefaultOption(StartPosition.MISC.title, StartPosition.MISC);

    for (StartPosition position : StartPosition.values()) {

      if (position != StartPosition.MISC) {
        startPositionChooser.addOption(position.title, position);
      }
    }

    /*
     * Trench side
     */
    trenchSideChooser.addDefaultOption(TrenchSide.RIGHT.title, TrenchSide.RIGHT);

    trenchSideChooser.addOption(TrenchSide.LEFT.title, TrenchSide.LEFT);

    /*
     * Game objects
     */
    gameObjects.addDefaultOption("0", 0);

    /*
     * Auto chooser
     *
     * IMPORTANT:
     * This must happen after initializePaths().
     */
    autoChooser = new LoggedDashboardChooser<>("Available Auto Variants");

    autoChooser.addDefaultOption(defaultPath.getDisplayName(), defaultPath.getDisplayName());

    filterAutos(0);

    autoKeys.set(keys);

    autoDelayEntry.setDouble(0.0);

    startPositionChooser.onChange(
        value -> {
          filterAutos(Integer.valueOf(gameObjects.getSendableChooser().getSelected()));

          updateInitialHeading();
          updateFieldDisplay();
        });

    trenchSideChooser.onChange(
        value -> {
          updateInitialHeading();
          updateFieldDisplay();
        });

    autoChooser.onChange(
        value -> {
          updateInitialHeading();
          updateFieldDisplay();
        });

    updateFieldDisplay();
  }

  public static void updateFieldDisplay() {

    fieldPoseStart.setRobotPose(getSelectedAutoStartingPose());
  }

  // ========================= AUTO FILTERING =========================

  public static void filterAutos(int numGameObjects) {

    String selectedStartPosition = startPositionChooser.getSendableChooser().getSelected();

    if (selectedStartPosition == null) {
      selectedStartPosition = StartPosition.MISC.title;
    }

    /*
     * Clear the previous options.
     *
     * Without this, every call to filterAutos()
     * keeps adding more options.
     */
    autoChooser = new LoggedDashboardChooser<>("Available Auto Variants");

    /*
     * Re-add the default option.
     *
     * The default auto is always available.
     */
    autoChooser.addDefaultOption(defaultPath.getDisplayName(), defaultPath.getDisplayName());

    /*
     * MISC means "show everything".
     */
    if (StartPosition.MISC.title.equals(selectedStartPosition)) {

      for (BLinePath auto : autos) {

        if (auto == defaultPath) {
          continue;
        }

        autoChooser.addOption(auto.getDisplayName(), auto.getDisplayName());
      }

      return;
    }

    /*
     * Otherwise only show autos whose starting
     * position matches the selected position.
     */
    for (BLinePath auto : autos) {

      if (auto == defaultPath) {
        continue;
      }

      if (auto.getStartPositionType() != null
          && auto.getStartPositionType().title.equals(selectedStartPosition)) {

        autoChooser.addOption(auto.getDisplayName(), auto.getDisplayName());
      }
    }
  }

  // ========================= SELECTION =========================

  public static String getSelectedAutoName() {

    String selected = autoChooser.getSendableChooser().getSelected();

    if (selected == null) {
      return defaultPath.getDisplayName();
    }

    return selected;
  }

  public static BLinePath getSelectedAutoPath() {

    String selectedName = autoChooser.getSendableChooser().getSelected();

    if (selectedName == null) {
      return defaultPath;
    }

    return namesToAuto.getOrDefault(selectedName, defaultPath);
  }

  public static Pose2d getSelectedAutoStartingPose() {

    BLinePath selected = getSelectedAutoPath();

    if (selected == null) {
      return Pose2d.kZero;
    }

    String selectedStartPosition = startPositionChooser.getSendableChooser().getSelected();

    if (StartPosition.TRENCH.title.equals(selectedStartPosition)) {

      return isMirrored() ? LEFT_TRENCH_POSE : RIGHT_TRENCH_POSE;
    }

    return selected.getStartPose2d();
  }

  // ========================= PATHS =========================

  public static List<Path> getPathsToBuild() {

    BLinePath selected = getSelectedAutoPath();

    if (selected == null) {
      return List.of();
    }

    return selected.getAllPaths();
  }

  public static List<BLinePath> getBLinePaths() {
    return rebuiltPaths;
  }

  public static List<String> getBLinePathsNames() {

    List<String> pathNames = new ArrayList<>();

    for (BLinePath path : getBLinePaths()) {
      pathNames.addAll(path.getDisplayingNames());
    }

    return pathNames;
  }

  // ========================= AUTO EXECUTION =========================

  public static Command getSelectedAuto() {

    if (s == null) {
      return Commands.none();
    }

    BLinePath selected = getSelectedAutoPath();

    if (selected == null) {
      return Commands.none();
    }

    double delay = autoDelayEntry.getDouble(0.0);

    s.drivebaseSubsystem.resetRotation(selected.getPath().getInitialModuleDirection());

    List<Command> commands = new ArrayList<>();

    commands.add(Commands.waitSeconds(delay));

    List<Path> paths = selected.getAllPaths();

    for (int i = 0; i < paths.size(); i++) {

      boolean resetPose = i == 0;

      commands.add(buildPath(paths.get(i), resetPose));
    }

    return Commands.sequence(commands.toArray(new Command[0]));
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

      default:
        return buildDefaultAuto();
    }
  }

  // ========================= LOGGING / DISPLAY =========================

  private static void updateInitialHeading() {

    BLinePath selected = getSelectedAutoPath();

    if (selected == null || selected.getPath() == null) {

      initialHeading.set(0.0);
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

    if (s == null || Robot.isSimulation()) {

      return RobotSim.launch(s, timeout);
    }

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

    if (s == null) {
      return Commands.none();
    }

    return Commands.parallel(
            Commands.runOnce(() -> s.flywheels.resetFuelCheck()),
            s.launcherSubsystem.launcherAimCommand(),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
        .withName("Auto Launcher Command");
  }

  public static Command stowCommand() {

    if (s == null) {
      return Commands.none();
    }

    return s.launcherSubsystem.rawStowCommand();
  }

  public static Command climbCommand() {

    return Commands.none().withName("Auto Climb Command");
  }

  public static void cancelCommand() {

    if (s == null) {
      return;
    }

    if (Robot.isSimulation()) {

      CommandScheduler.getInstance().cancel(bLineSimLaunching);

    } else {

      CommandScheduler.getInstance().cancel(bLineLaunching);
    }
  }

  // ========================= EVENT REGISTRATION =========================

  private static void registerCommands() {

    if (s == null) {
      return;
    }

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
