package frc.robot.subsystems.auto;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.util.simulation.FuelSim;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.parser.ParseException;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

public class AutoLogic {

  private static Subsystems s;

  /* ---------------- Start positions ---------------- */

  public enum StartPosition {
    LEFT_TRENCH(
        "Left Trench", new Pose2d(4.013, 7.597, new Rotation2d(Units.degreesToRadians(90)))),
    CENTER("Center", new Pose2d(3.600, 4.035, new Rotation2d(Units.degreesToRadians(0)))),
    RIGHT_TRENCH(
        "Right Trench", new Pose2d(4.013, 0.473, new Rotation2d(Units.degreesToRadians(-90)))),
    MISC("Misc", null);

    final String title;
    final Pose2d startPose;

    StartPosition(String title, Pose2d startPose) {
      this.title = title;
      this.startPose = startPose;
    }
  }

  /* ---------------- Paths ---------------- */

  private static AutoPath defaultPath;

  private static final List<String> REBUILT_AUTO_NAMES =
      List.of(
          "C-Outpost-Depot",
          "LeftTrench-Depot",
          "LT-Neutral-Depot",
          "LT-Neutral",
          "LT-DoubleNeutral",
          "RightTrench-Outpost",
          "RT-Neutral-Outpost",
          "Rotate-RT-Neutral",
          "RT-Neutral",
          "RT-DoubleNeutral",
          "RT-BLOCK",
          "LT-BLOCK");

  private static List<AutoPath> rebuiltPaths = List.of();

  private static Map<Integer, List<AutoPath>> commandsMap = Map.of();

  private static final Map<String, AutoPath> namesToAuto = new HashMap<>();
  private static boolean pathsInitialized = false;

  /* ---------------- Choosers ---------------- */

  private static final Selectable<StartPosition> startPositionChooser = new Selectable<>();

  private static final Selectable<String> availableAutos = new Selectable<>();

  private static final Selectable<Integer> gameObjects = new Selectable<>();

  private static final NetworkTableEntry autoDelayEntry =
      NetworkTableInstance.getDefault().getTable("Autos").getEntry("Auto Delay");

  public static final String keys = "RB=Right Bump, LB=Left Bump, LT=Left Trench, RT=Right Trench";

  public static List<AutoPath> getAutos() {
    if (rebuiltPaths != null) {
      return rebuiltPaths;
    }

    return List.of();
  }

  public static List<String> getConfiguredAutoNames() {
    return REBUILT_AUTO_NAMES;
  }

  /* ---------------- Init ---------------- */
  public static void init(Subsystems subsystems) {
    s = subsystems;
  }

  // We always need to register commands BEFORE we initialize paths.  This is
  // because paths may reference commands or triggers that need to be registered first.
  // This helper-method insures caller initializes these in the correct order.
  public static void initCommandsAndPaths(boolean testMode) {
    if (!testMode) {
      registerCommands();
    }

    initPaths();
  }

  private static void initPaths() {
    List<AutoPath> physicalRebuiltPaths;

    if (pathsInitialized) {
      return;
    }

    defaultPath = new AutoPath("Default", "Default");

    physicalRebuiltPaths =
      REBUILT_AUTO_NAMES.stream().map(name -> new AutoPath(name, name)).toList();

    rebuiltPaths = physicalRebuiltPaths;

    commandsMap = Map.of(0, rebuiltPaths);
    namesToAuto.clear();
    for (List<AutoPath> autos : commandsMap.values()) {
      for (AutoPath auto : autos) {
        namesToAuto.put(auto.getDisplayName(), auto);
      }
    }

    pathsInitialized = true;
  }

  private static void requirePathsInitialized() {
    if (!pathsInitialized) {
      throw new IllegalStateException(
          "Auto paths are not initialized. Call AutoLogic.initCommandsAndPaths().");
    }
  }

  public static void initSmartDashBoard() {
    requirePathsInitialized();

    startPositionChooser.addDefault(StartPosition.MISC.title, StartPosition.MISC);

    for (StartPosition pos : StartPosition.values()) {
      startPositionChooser.add(pos.title, pos);
    }

    gameObjects.addDefault("0", 0);
    for (int i = 1; i < commandsMap.size(); i++) {
      gameObjects.add(String.valueOf(i), i);
    }

    autoDelayEntry.setDouble(0.0);

    Tunables.publish("Starting Position", startPositionChooser);
    Tunables.publish("Auto Mode", gameObjects);
    Tunables.publish("Available Auto Variants", availableAutos);
    Telemetry.log("Auto Key", keys);

    startPositionChooser.onChange(v -> filterAutos(gameObjects.getSelected()));
    gameObjects.onChange(v -> filterAutos(gameObjects.getSelected()));

    filterAutos(gameObjects.getSelected());
  }

  /* ---------------- Filtering ---------------- */

  public static void filterAutos(int numGameObjects) {
    requirePathsInitialized();

    availableAutos.clear();
    availableAutos.addDefault(defaultPath.getDisplayName(), defaultPath.getDisplayName());

    List<AutoPath> autoList = commandsMap.get(numGameObjects);
    if (autoList == null) return;

    for (AutoPath auto : autoList) {
      if (auto.getStartPose().equals(startPositionChooser.getSelected())) {
        availableAutos.add(auto.getDisplayName(), auto.getDisplayName());
      }
    }
  }

  /* ---------------- Getters ---------------- */

  public static String getSelectedAutoName() {
    return availableAutos.getSelected();
  }

  public static boolean chooserHasAutoSelected() {
    return availableAutos.getSelected() != null;
  }

  public static Pose2d getSelectedAutoStartingPose() {
    requirePathsInitialized();

    String selectedAutoName = getSelectedAutoName();
    AutoPath selectedPath = namesToAuto.get(selectedAutoName);

    if (selectedPath != null && selectedPath.getStartPose2d() != null) {
      return selectedPath.getStartPose2d();
    }

    if (defaultPath.getDisplayName().equals(selectedAutoName)
        && defaultPath.getStartPose2d() != null) {
      return defaultPath.getStartPose2d();
    }

    return Pose2d.ZERO;
  }

  public static Command getSelectedAuto() {
    requirePathsInitialized();

    double delay = autoDelayEntry.getDouble(0.0);

    AutoPath path = namesToAuto.get(getSelectedAutoName());
    if (path == null) {
      path = defaultPath;
    }

    String autoName = path.getAutoName();

    return Commands.waitSeconds(delay).andThen(AutoBuilder.buildAuto(autoName)).withName(autoName);
  }

  /* ---------------- PathPlanner ---------------- */

  public static Command getAutoCommand(String pathName)
      throws FileVersionException, IOException, ParseException {

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }

  private static void registerCommands() {
    if (s.launcherSubsystem != null && s.indexerSubsystem != null) {
      if (Robot.isSimulation()) {
        NamedCommands.registerCommand(
            "launch", launcherSimCommand().andThen(Commands.print("launch")));
      } else {
        NamedCommands.registerCommand(
            "launch", launcherCommand().andThen(Commands.print("launch")));
      }
    }
    if (s.indexerSubsystem != null) {
      NamedCommands.registerCommand("intake", intakeCommand());
    }
    NamedCommands.registerCommand("climb", climbCommand());
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
        .withTimeout(4.5)
        .andThen(s.launcherSubsystem.rawStowCommand())
        .withName("Auto Launcher Command");
  }

  public static Command launcherSimCommand() {
    return Commands.sequence(
            AutoDriveRotate.autoRotate(
                s.drivebaseSubsystem, () -> 0, () -> 0, () -> 0), // SIM PURPOSES ONLY
            Commands.run(
                    () ->
                        FuelSim.getInstance()
                            .launchFuel(
                                MetersPerSecond.of(6),
                                Radians.of(s.hood.getHoodPosition()),
                                Radians.of(s.turretSubsystem.getTurretPosition() + Math.PI),
                                Meters.of(1.45)))
                .withTimeout(3))
        .withName("Auto Launcher Sim Command");
  }

  public static Command intakeCommand() {
    return Commands.runOnce(() -> Controls.intakeMode = IntakeMode.INTAKE)
        .withName("Auto Intake Command");
  }

  public static Command climbCommand() {
    return Commands.none().withName("Auto Climb Command");
  }
}
