package frc.robot.subsystems.auto.BLine;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.Misc.DynamicSendableChooser;
import frc.robot.subsystems.auto.Misc.StuckOnBallRecovery;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.util.simulation.RobotSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

public class BLineLogic {

  private static Subsystems s;

  public static Field2d field = new Field2d();
  public static Field2d fieldPoseStart = new Field2d();

  private static Trigger beachedTrigger;

  private static Boolean enableAutoUnbeach = true;
  private static Boolean enableLaunchOnTheMove = true;

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

  private static final SendableChooser<TrenchSide> trenchSideChooser = new SendableChooser<>();
  private static final List<DynamicSendableChooser<String>> pathChoosers = new ArrayList<>();
  private static final Map<BLinePath, List<BLinePath>> rebuiltPaths = new HashMap<>();
  private static final List<BLinePath> autos = new ArrayList<>();
  private static final Map<String, BLinePath> namesToAuto = new HashMap<>();
  private static BLinePath defaultPath;
  private static boolean pathsInitialized = false;
  private static Command bLineLaunching;
  private static Command bLineSimLaunching;
  private static StructPublisher<Pose2d> recoveryPose;
  public static FollowPath.Builder pathBuilder;
  private static FollowPath.Builder continuingPathBuilder;
  private static Path currentPath;
  private static FollowPath follow;
  private static int savedPathIndex = -1;

  private static final List<NetworkTableEntry> autoDelayEntries = new ArrayList<>();

  public static final String keys = "RB=Right Bump, LB=Left Bump, LT=Left Trench, RT=Right Trench";

  public static boolean isMirrored() {

    return getSelectedAutoPath().getStartPositionType() == StartPosition.TRENCH
        && trenchSideChooser.getSelected() == TrenchSide.LEFT;
  }

  static Pose2d getTrenchPose() {

    return isMirrored() ? LEFT_TRENCH_POSE : RIGHT_TRENCH_POSE;
  }

  public static void init(Subsystems subsystems) {

    s = subsystems;

    recoveryPose =
        NetworkTableInstance.getDefault()
            .getTable("Autos")
            .getStructTopic("RecoveryPose", Pose2d.struct)
            .publish();

    registerTriggersAndCommands();

    if (pathsInitialized) {
      return;
    }

    initializePaths();

    pathsInitialized = true;
  }

  public static void unitTestInit() {

    s = null;

    initializePaths();
  }

  private static void initializePaths() {

    defaultPath = new BLinePath("Default", "default");

    rebuiltPaths.clear();
    autos.clear();
    namesToAuto.clear();
    pathChoosers.clear();

    rebuiltPaths.put(
        BLinePaths.FirstNeutralTrench,
        List.of(
            BLinePaths.FirstNeutralBump,
            BLinePaths.SecondNeutralBump,
            BLinePaths.SecondNeutralTrench));

    rebuiltPaths.put(
        BLinePaths.FirstNeutralBump, List.of(BLinePaths.BumpToTrench, BLinePaths.BumpDepot));

    rebuiltPaths.put(
        BLinePaths.SecondNeutralBump, List.of(BLinePaths.BumpToTrench, BLinePaths.BumpDepot));

    rebuiltPaths.put(
        BLinePaths.BumpToTrench,
        List.of(BLinePaths.FirstNeutralTrench, BLinePaths.SecondNeutralTrench));

    rebuiltPaths.put(
        BLinePaths.BumpDepot,
        List.of(BLinePaths.FirstNeutralTrench, BLinePaths.SecondNeutralTrench));

    rebuiltPaths.put(
        BLinePaths.SecondNeutralTrench,
        List.of(BLinePaths.FirstNeutralBump, BLinePaths.SecondNeutralBump));

    for (BLinePath path : rebuiltPaths.keySet()) {

      if (!autos.contains(path)) {
        autos.add(path);
      }

      namesToAuto.put(path.getDisplayName(), path);

      handleStartingPoses(path);
    }

    for (List<BLinePath> paths : rebuiltPaths.values()) {

      for (BLinePath path : paths) {

        if (!autos.contains(path)) {
          autos.add(path);
        }

        namesToAuto.put(path.getDisplayName(), path);

        handleStartingPoses(path);
      }
    }

    createPathChoosers();
  }

  private static void createPathChoosers() {
    final int MAX_STEPS = 3;

    for (int i = 0; i < MAX_STEPS; i++) {

      DynamicSendableChooser<String> chooser = new DynamicSendableChooser<>();

      pathChoosers.add(chooser);

      SmartDashboard.putData("BLine/Path Step " + (i + 1), chooser);
    }

    populateFirstChooser();
    for (DynamicSendableChooser<String> chooser : pathChoosers) {

      chooser.onChange(value -> updatePathChoosers());
    }
  }

  private static void populateFirstChooser() {

    DynamicSendableChooser<String> chooser = pathChoosers.get(0);

    chooser.clearOptions();

    for (BLinePath path : autos) {

      if (path != BLinePaths.DepotToTrench) {

        chooser.addOption(path.getDisplayName(), path.getDisplayName());
      }
    }
  }

  public static void updatePathChoosers() {

    for (int step = 0; step < pathChoosers.size() - 1; step++) {

      BLinePath currentPath = getSelectedPath(step);

      if (currentPath == null) {

        clearChoosersAfter(step);

        break;
      }

      List<BLinePath> nextPaths = rebuiltPaths.get(currentPath);

      if (nextPaths == null || nextPaths.isEmpty()) {

        clearChoosersAfter(step);

        break;
      }

      DynamicSendableChooser<String> nextChooser = pathChoosers.get(step + 1);

      nextChooser.clearOptions();

      for (BLinePath nextPath : nextPaths) {

        nextChooser.addOption(nextPath.getDisplayName(), nextPath.getDisplayName());
      }
    }
  }

  private static void clearChoosersAfter(int step) {

    for (int i = step + 1; i < pathChoosers.size(); i++) {

      pathChoosers.get(i).clearOptions();
    }
  }

  private static BLinePath getSelectedPath(int step) {

    if (step < 0 || step >= pathChoosers.size()) {

      return null;
    }

    String selectedName = pathChoosers.get(step).getSelected();

    if (selectedName == null) {
      return null;
    }

    return namesToAuto.get(selectedName);
  }

  public static List<BLinePath> getSelectedPathSequence() {

    List<BLinePath> selectedPaths = new ArrayList<>();

    for (int step = 0; step < pathChoosers.size(); step++) {

      BLinePath selected = getSelectedPath(step);

      if (selected == null) {
        break;
      }

      selectedPaths.add(selected);

      List<BLinePath> nextPaths = rebuiltPaths.get(selected);

      if (nextPaths == null || nextPaths.isEmpty()) {

        break;
      }
    }

    return selectedPaths;
  }

  public static void filterAutos(int numGameObjects) {

    updatePathChoosers();
  }

  public static void initSmartDashboard() {
    autoDelayEntries.clear();

    for (int i = 0; i < pathChoosers.size(); i++) {
      String key = "BLine/Step " + (i + 1) + " Delay";

      SmartDashboard.putNumber(key, 0.0);

      autoDelayEntries.add(
          NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry(key));
    }
    trenchSideChooser.setDefaultOption(TrenchSide.RIGHT.title, TrenchSide.RIGHT);

    trenchSideChooser.addOption(TrenchSide.LEFT.title, TrenchSide.LEFT);

    SmartDashboard.putData("Trench Side", trenchSideChooser);

    SmartDashboard.putData("Selected Auto", field);

    SmartDashboard.putString("Auto Key", keys);

    SmartDashboard.putBoolean("Enable SOTM?", enableLaunchOnTheMove);
    SmartDashboard.putData("Start Pose", fieldPoseStart);

    SmartDashboard.putBoolean("Enable Auto Unbeach?", enableAutoUnbeach);

    trenchSideChooser.onChange(
        value -> {
          updateInitialHeading();
          updateFieldDisplay();
        });

    updatePathChoosers();
    updateFieldDisplay();
  }

  public static void updateFieldDisplay() {
    fieldPoseStart.setRobotPose(getSelectedAutoStartingPose());
  }

  public static String getSelectedAutoName() {

    List<BLinePath> sequence = getSelectedPathSequence();

    if (sequence.isEmpty()) {
      return "Default";
    }

    StringBuilder name = new StringBuilder();

    for (BLinePath path : sequence) {
      if (name.length() > 0) {
        name.append(" -> ");
      }

      name.append(path.getDisplayName());
    }

    return name.toString();
  }

  public static BLinePath getSelectedAutoPath() {

    BLinePath selected = getSelectedPath(0);

    if (selected == null) {
      return defaultPath;
    }

    return selected;
  }

  public static Pose2d getSelectedAutoStartingPose() {

    BLinePath selected = getSelectedAutoPath();

    if (selected == null) {
      return Pose2d.kZero;
    }

    return selected.getStartPose2d();
  }

  public static List<Path> getPathsToBuild() {

    List<Path> paths = new ArrayList<>();

    List<BLinePath> sequence = getSelectedPathSequence();

    for (BLinePath bLinePath : sequence) {

      paths.addAll(bLinePath.getAllPaths());
    }

    return paths;
  }

  public static List<BLinePath> getBLinePaths() {

    return autos;
  }

  public static List<String> getBLinePathsNames() {

    List<String> names = new ArrayList<>();

    for (BLinePath path : autos) {

      names.add(path.getDisplayName());
    }

    return names;
  }

  public static void handleStartingPoses(BLinePath path) {
    List<Pose2d> posses = new ArrayList<>();
    for (StartPosition s : StartPosition.values()) {
      posses.add(s.startPose);
    }

    String startingPosition = null;

    if (posses != null) {
      if (path.getStartPose2d().nearest(posses).equals(StartPosition.CENTER.startPose)) {
        startingPosition = "Center";
      } else if (path.getStartPose2d().nearest(posses).equals(StartPosition.TRENCH.startPose)) {
        startingPosition = "RT";
      }
    }

    if (startingPosition == null) {
      return;
    }

    switch (startingPosition) {
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

  private static void updateInitialHeading() {

    BLinePath selected = getSelectedAutoPath();

    if (selected == null || selected.getPath() == null) {

      SmartDashboard.putNumber("Initial Heading(Deg)", 0.0);

      return;
    }

    Pose2d start = selected.getStartPose2d();

    SmartDashboard.putNumber("Initial Heading(Deg)", Math.round(start.getRotation().getDegrees()));
  }

  public static Command getSelectedAuto() {
    if (s == null) {
      return Commands.none();
    }

    List<BLinePath> sequence = getSelectedPathSequence();

    if (sequence.isEmpty()) {
      return Commands.none();
    }

    List<Command> commands = new ArrayList<>();

    boolean firstPath = true;

    for (int step = 0; step < sequence.size(); step++) {

      // Delay before this chooser step
      double delay = autoDelayEntries.get(step).getDouble(0.0);
      commands.add(Commands.waitSeconds(delay));

      // Build every Path belonging to this BLinePath
      for (Path path : sequence.get(step).getAllPaths()) {
        commands.add(buildPath(path, firstPath, true));
        firstPath = false;
      }
    }

    // Set the initial rotation before starting the first path
    List<Path> firstStepPaths = sequence.get(0).getAllPaths();
    if (!firstStepPaths.isEmpty()) {
      s.drivebaseSubsystem.resetRotation(firstStepPaths.get(0).getInitialModuleDirection());
    }

    return Commands.sequence(commands.toArray(new Command[0]));
  }

  private static Command buildPath(Path path, boolean resetPose, boolean saveForRecovery) {

    if (s == null || pathBuilder == null || continuingPathBuilder == null) {
      return Commands.none();
    }

    FollowPath command = (resetPose ? pathBuilder : continuingPathBuilder).build(path);

    if (saveForRecovery) {
      currentPath = path;
      follow = command;
    }

    return command;
  }

  public static Command handleAutos() {

    return getSelectedAuto();
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

    if (s.launcherSubsystem != null && s.flywheels != null) {

      return Commands.parallel(
              Commands.runOnce(() -> s.flywheels.resetFuelCheck()),
              s.launcherSubsystem.launcherAimCommand(),
              Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                  .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
          .withName("Auto Launcher Command");
    }

    return Commands.none();
  }

  public static Command stowCommand() {

    return s.launcherSubsystem != null ? s.launcherSubsystem.rawStowCommand() : Commands.none();
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

  public static void updateRecoveryPose() {

    recoveryPose.set(
        StuckOnBallRecovery.getRecoveryPose(
            s.drivebaseSubsystem.getState().Pose,
            Rotation2d.fromDegrees(s.drivebaseSubsystem.getPigeon2().getPitch().getValueAsDouble()),
            Rotation2d.fromDegrees(
                s.drivebaseSubsystem.getPigeon2().getRoll().getValueAsDouble())));
  }

  public static Command recoverCommand() {

    return Commands.sequence(
            buildPath(
                StuckOnBallRecovery.getRecoverySegment(
                    () -> s.drivebaseSubsystem.getState().Pose,
                    () ->
                        Rotation2d.fromDegrees(
                            s.drivebaseSubsystem.getPigeon2().getPitch().getValueAsDouble()),
                    () ->
                        Rotation2d.fromDegrees(
                            s.drivebaseSubsystem.getPigeon2().getRoll().getValueAsDouble())),
                false,
                false))
        .until(() -> !s.drivebaseSubsystem.isBeached(5));
  }

  private static Command resume() {

    if (currentPath == null || savedPathIndex < 0) {
      return Commands.none();
    }

    int i = savedPathIndex;

    var flat = currentPath.getPathElementsWithConstraintsNoWaypoints();

    List<Path.PathElement> remaining = new ArrayList<>();

    remaining.add(
        new Path.TranslationTarget(s.drivebaseSubsystem.getState().Pose.getTranslation()));

    for (int j = i; j < flat.size(); j++) {
      remaining.add(flat.get(j).getFirst().copy());
    }

    Path remainder = new Path(remaining, currentPath.getPathConstraints());

    return buildPath(remainder, false, false);
  }

  private static void registerTriggersAndCommands() {

    beachedTrigger =
        new Trigger(
            () -> {
              enableAutoUnbeach =
                  SmartDashboard.getBoolean("Enable Auto Unbeach?", enableAutoUnbeach);

              return enableAutoUnbeach
                  && RobotState.isAutonomous()
                  && s.drivebaseSubsystem.isBeached(5);
            });

    beachedTrigger.onTrue(
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (follow != null) {
                    savedPathIndex = follow.getCurrentTranslationElementIndex();
                  }
                }),
            recoverCommand(),
            resume()));

    AtomicBoolean launchAllowed = new AtomicBoolean(true);

    if (s.launcherSubsystem != null && s.indexerSubsystem != null) {

      if (Robot.isSimulation()) {

        bLineSimLaunching = RobotSim.launch(s, 30);

        FollowPath.registerEventTrigger(
            "launch",
            Commands.defer(
                () -> {
                  enableLaunchOnTheMove =
                      SmartDashboard.getBoolean("Enable SOTM?", enableLaunchOnTheMove);

                  return enableLaunchOnTheMove
                      ? Commands.runOnce(() -> launchAllowed.set(true))
                          .andThen(bLineSimLaunching.onlyWhile(launchAllowed::get))
                          .andThen(Commands.print("LAUNCH FINISHED"))
                      : Commands.none();
                },
                Set.of()));
      } else {

        bLineLaunching = launcherCommand();

        FollowPath.registerEventTrigger(
            "launch",
            Commands.defer(
                () -> {
                  enableLaunchOnTheMove =
                      SmartDashboard.getBoolean("Enable SOTM?", enableLaunchOnTheMove);

                  return enableLaunchOnTheMove ? bLineLaunching : Commands.none();
                },
                Set.of()));
      }
    }

    FollowPath.registerEventTrigger("intake", intakeCommand());

    FollowPath.registerEventTrigger("climb", climbCommand());

    FollowPath.registerEventTrigger(
        "cancel", Commands.runOnce(() -> launchAllowed.set(false)).andThen(stowCommand()));
  }
}
