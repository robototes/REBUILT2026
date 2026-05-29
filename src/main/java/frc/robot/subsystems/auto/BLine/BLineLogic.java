package frc.robot.subsystems.auto.BLine;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.Misc.DynamicSendableChooser;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.util.simulation.RobotSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class BLineLogic {

  private static Subsystems s;

  public enum StartPosition {
    LEFT_TRENCH("Left Trench", new Pose2d(4.013, 7.597, Rotation2d.fromDegrees(90))),
    CENTER("Center", new Pose2d(3.600, 4.035, Rotation2d.fromDegrees(0))),
    RIGHT_TRENCH("Right Trench", new Pose2d(4.013, 0.473, Rotation2d.fromDegrees(-90))),
    MISC("Misc", null);

    public final String title;
    public final Pose2d startPose;

    StartPosition(String title, Pose2d startPose) {
      this.title = title;
      this.startPose = startPose;
    }
  }

  private static final List<BLinePath> autos = new ArrayList<>();

  private static final SendableChooser<StartPosition> startPositionChooser =
      new SendableChooser<>();

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

  public static FollowPath.Builder pathBuilder;
  private static Command bLineLaunching;

  // ========================= INIT =========================

  public static void init(Subsystems subsystems) {

    s = subsystems;

    registerCommands();

    if (pathsInitialized) return;

    defaultPath = new BLinePath("test1", "test1");

    rebuiltPaths =
        List.of(
            defaultPath,
            new BLinePath("RTNEUTRAL", "RTNEUTRAL"),
            new BLinePath("Sample 2", "sample2"),
            new BLinePath("Sample 5", "Sample 5"));

    autos.clear();
    autos.addAll(rebuiltPaths);

    commandsMap = Map.of(0, rebuiltPaths);

    namesToAuto.clear();
    for (List<BLinePath> list : commandsMap.values()) {
      for (BLinePath auto : list) {
        namesToAuto.put(auto.getDisplayName(), auto);
      }
    }

    pathsInitialized = true;
  }

  public static void configure(Subsystems s) {

    pathBuilder =
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
            .withPoseReset(pose -> s.drivebaseSubsystem.resetPose(pose));
  }

  // ========================= LOGGING =========================

  public static void initSmartDashboard() {

    startPositionChooser.setDefaultOption(StartPosition.MISC.title, StartPosition.MISC);

    for (StartPosition pos : StartPosition.values()) {
      startPositionChooser.addOption(pos.title, pos);
    }

    gameObjects.setDefaultOption("0", 0);

    filterAutos(0);

    SmartDashboard.putData("Starting Position", startPositionChooser);
    SmartDashboard.putData("Auto Mode", gameObjects);
    SmartDashboard.putData("Available Auto Variants", autoChooser);
    SmartDashboard.putString("Auto Key", keys);

    autoDelayEntry.setDouble(0.0);

    startPositionChooser.onChange(
        v -> {
          filterAutos(gameObjects.getSelected());
          updateInitialHeading();
        });

    autoChooser.onChange(v -> updateInitialHeading());
  }

  // ========================= AUTOS FILTERING =========================

  public static void filterAutos(int numGameObjects) {

    autoChooser.clearOptions();

    StartPosition selected = startPositionChooser.getSelected();

    // fallback safety
    if (selected == null) {
      selected = StartPosition.MISC;
    }

    for (BLinePath auto : autos) {

      if (selected == StartPosition.MISC) {
        autoChooser.addOption(auto.getDisplayName(), auto.getDisplayName());
      }

      Pose2d autoPose = auto.getStartPose2d();

      if (autoPose != null && selected.startPose != null && autoPose.equals(selected.startPose)) {

        autoChooser.addOption(auto.getDisplayName(), auto.getDisplayName());
      }
    }
  }

  // ========================= SELECTION METHODS =========================

  public static String getSelectedAutoName() {
    return autoChooser.getSelected();
  }

  public static BLinePath getSelectedAutoPath() {

    String selectedName = autoChooser.getSelected();

    if (selectedName == null) {
      return defaultPath;
    }

    return namesToAuto.getOrDefault(selectedName, defaultPath);
  }

  public static Pose2d getSelectedAutoStartingPose() {

    BLinePath selected = getSelectedAutoPath();

    if (selected == null) return Pose2d.kZero;

    return selected.getStartPose2d();
  }

  // ========================= AUTO EXECUTION =========================

  public static Command getSelectedAuto() {

    double delay = autoDelayEntry.getDouble(0.0);

    BLinePath path = getSelectedAutoPath();
    s.drivebaseSubsystem.resetRotation(path.getPath().getInitialModuleDirection());
    return Commands.waitSeconds(delay).andThen(pathBuilder.build(path.getPath()));
  }

  public static Command buildRTNeutralAuto() {
    return Commands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildAndSetHeading(new Path("RTNEUTRAL")),
        launcherCommand(5));
  }

  public static Command buildRTNeutralTestingAuto() {
    return Commands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildAndSetHeading(new Path("RTNEUTRAL")),
        launcherCommand(5),
        buildAndSetHeading(new Path("RTRBNEUTRAL")));
  }

  private static Command buildAndSetHeading(Path path) {

    return Commands.runOnce(
            () -> s.drivebaseSubsystem.resetRotation(path.getInitialModuleDirection()))
        .andThen(pathBuilder.build(path));
  }

  private static void updateInitialHeading() {

    BLinePath selected = getSelectedAutoPath();

    if (selected == null || selected.getPath() == null) {
      SmartDashboard.putNumber("Initial Heading", 0.0);
      return;
    }

    Pose2d start = selected.getPath().getStartPose();

    SmartDashboard.putNumber("Initial Heading", start.getRotation().getDegrees());
  }

  public static Command intakeCommand() {
    return Commands.runOnce(() -> Controls.intakeMode = IntakeMode.INTAKE)
        .withName("Auto Intake Command");
  }

  public static Command launcherCommand(double timeout) {
    if(Robot.isSimulation()) {
      return RobotSim.launch(s,timeout);
    }
    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  s.flywheels.resetFuelCheck();
                }),
            s.launcherSubsystem.launcherAimCommand(),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
        // .until(() -> s.flywheels.isOutOfFuel())
        .withTimeout(timeout)
        .andThen(s.launcherSubsystem.rawStowCommand())
        .withName("Auto Launcher Command");
  }

  public static Command launcherCommand() {

    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  s.flywheels.resetFuelCheck();
                }),
            s.launcherSubsystem.launcherAimCommand(),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(s.indexerSubsystem.runIndexer(() -> s.flywheels.getTargetSpeed())))
        // .until(() -> s.flywheels.isOutOfFuel())

        .withName("Auto Launcher Command");
  }

  public static Command stowCommand() {
    return s.launcherSubsystem.rawStowCommand();
  }

  public static Command climbCommand() {
    return Commands.none().withName("Auto Climb Command");
  }

  public static void cancelCommand() {
    CommandScheduler.getInstance().cancel(bLineLaunching);
  }

  private static void registerCommands() {

    if (s.launcherSubsystem != null && s.indexerSubsystem != null) {

      if (Robot.isSimulation()) {
        bLineLaunching = RobotSim.launch(s, 1);
        FollowPath.registerEventTrigger(
            "launch", bLineLaunching.andThen(Commands.print("LAUNCH FINISHED")));
      } else {
        bLineLaunching = launcherCommand();
        FollowPath.registerEventTrigger("launch", bLineLaunching);
      }
    }

    FollowPath.registerEventTrigger("intake", intakeCommand());
    FollowPath.registerEventTrigger("climb", climbCommand());
  }
}
