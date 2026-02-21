package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.util.simulation.FuelSim;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.parser.ParseException;

public class AutoLogic {

  private static Subsystems s;

  /* ---------------- Start positions ---------------- */

  public enum StartPosition {
    LEFT_TRENCH(
        "Left Trench", new Pose2d(4.354, 7.465, new Rotation2d(Units.degreesToRadians(90)))),
    LEFT_BUMP("Left Bump", new Pose2d(3.664, 5.411, new Rotation2d(Units.degreesToRadians(90)))),
    CENTER("Center", new Pose2d(3.62, 4.008, new Rotation2d(Units.degreesToRadians(90)))),
    RIGHT_BUMP("Right Bump", new Pose2d(3.638, 2.322, new Rotation2d(Units.degreesToRadians(-90)))),
    RIGHT_TRENCH(
        "Right Trench", new Pose2d(4.291, 0.55, new Rotation2d(Units.degreesToRadians(-90)))),
    MISC("Misc", null);

    final String title;
    final Pose2d startPose;

    StartPosition(String title, Pose2d startPose) {
      this.title = title;
      this.startPose = startPose;
    }
  }

  /* ---------------- Paths ---------------- */

  private static final AutoPath defaultPath = new AutoPath("Default", "Default");

  private static final List<AutoPath> rebuiltPaths =
      List.of(
          new AutoPath("C-Climb", "C-Climb"),
          new AutoPath("C-Depot-Climb", "C-Depot-Climb"),
          new AutoPath("Default", "Default"),
          new AutoPath("LB-Depot-Climb", "LB-Depot-Climb"),
          new AutoPath("LT-NeutralLeft-Shallow-Climb", "LT-NeutralLeft-Shallow-Climb"),
          new AutoPath("LT-NeutralLeft-Shallow-Middle", "LT-NeutralLeft-Shallow-Middle"),
          new AutoPath(
              "LT-NeutralLeft-Shallow-Middle-Climb", "LT-NeutralLeft-Shallow-Middle-Climb"),
          new AutoPath("LT-Depot-Climb", "LT-Depot-Climb"),
          new AutoPath("RB-Outpost-Climb", "RB-Outpost-Climb"),
          new AutoPath("RT-NeutralRight-Shallow-Climb", "RT-NeutralRight-Shallow-Climb"),
          new AutoPath("RT-NeutralRight-Shallow-Middle", "RT-NeutralRight-Shallow-Middle"),
          new AutoPath(
              "RT-NeutralRight-Shallow-Middle-Climb", "RT-NeutralRight-Shallow-Middle-Climb"),
          new AutoPath("RT-Outpost-Climb", "RT-Outpost-Climb"));

  private static final Map<Integer, List<AutoPath>> commandsMap = Map.of(0, rebuiltPaths);

  private static final Map<String, AutoPath> namesToAuto = new HashMap<>();

  static {
    for (List<AutoPath> autos : commandsMap.values()) {
      for (AutoPath auto : autos) {
        namesToAuto.put(auto.getDisplayName(), auto);
      }
    }
  }

  /* ---------------- Choosers ---------------- */

  private static final SendableChooser<StartPosition> startPositionChooser =
      new SendableChooser<>();

  private static final DynamicSendableChooser<String> availableAutos =
      new DynamicSendableChooser<>();

  private static final SendableChooser<Integer> gameObjects = new SendableChooser<>();

  private static final NetworkTableEntry autoDelayEntry =
      NetworkTableInstance.getDefault().getTable("Autos").getEntry("Auto Delay");

  public static final String keys = "RB=Right Bump, LB=Left Bump, LT=Left Trench, RT=Right Trench";

  public static List<AutoPath> getAutos() {
    if (rebuiltPaths != null) {
      return rebuiltPaths;
    }
    System.out.println("fail");
    return List.of();
  }

  /* ---------------- Init ---------------- */
  public static void init(Subsystems subsystems) {
    s = subsystems;
  }

  public static void initSmartDashBoard() {

    startPositionChooser.setDefaultOption(StartPosition.MISC.title, StartPosition.MISC);

    for (StartPosition pos : StartPosition.values()) {
      startPositionChooser.addOption(pos.title, pos);
    }

    gameObjects.setDefaultOption("0", 0);
    for (int i = 1; i < commandsMap.size(); i++) {
      gameObjects.addOption(String.valueOf(i), i);
    }

    autoDelayEntry.setDouble(0.0);

    SmartDashboard.putData("Starting Position", startPositionChooser);
    SmartDashboard.putData("Auto Mode", gameObjects);
    SmartDashboard.putData("Available Auto Variants", availableAutos);
    SmartDashboard.putString("Auto Key", keys);

    startPositionChooser.onChange(v -> filterAutos(gameObjects.getSelected()));
    gameObjects.onChange(v -> filterAutos(gameObjects.getSelected()));

    filterAutos(gameObjects.getSelected());
  }

  /* ---------------- Filtering ---------------- */

  public static void filterAutos(int numGameObjects) {

    availableAutos.clearOptions();
    availableAutos.setDefaultOption(defaultPath.getDisplayName(), defaultPath.getDisplayName());

    List<AutoPath> autoList = commandsMap.get(numGameObjects);
    if (autoList == null) return;

    for (AutoPath auto : autoList) {
      if (auto.getStartPose().equals(startPositionChooser.getSelected())) {
        availableAutos.addOption(auto.getDisplayName(), auto.getDisplayName());
      }
    }
  }

  /* ---------------- Getters ---------------- */

  public static String getSelectedAutoName() {
    return availableAutos.getSelectedName();
  }

  public static boolean chooserHasAutoSelected() {
    return availableAutos.getSelected() != null;
  }

  public static Command getSelectedAuto() {
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

  public static void registerCommands() {

    NamedCommands.registerCommand("launch", launcherCommand());
    NamedCommands.registerCommand("aim", aimCommand());
    NamedCommands.registerCommand("intake", intakeCommand());
    NamedCommands.registerCommand("climb", climbCommand());
    NamedCommands.registerCommand("rollers", rollerCommand());
  }

  public static final Command empty() {
    return Commands.none();
  }

  public static Command aimCommand() {
    return s.turretSubsystem.rotateToHub();
   // return empty();
  }
public static Command rollerCommand() {
  return empty();
 // return s.intakeRollers.runSingleRoller();
}
  public static Command launcherCommand() {

   /*  return Commands.parallel(
            s.launcherSubsystem.launcherAimCommand(s.drivebaseSubsystem),
            Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                .andThen(
                    Commands.parallel(
                      s.spindexerSubsystem.startMotor(), s.feederSubsystem.startMotor())))
        .withTimeout(4.5); */
        return empty();
  }

  public static Command launcherSimCommand() {

    return Commands.sequence(
        AutoDriveRotate.autoRotate(s.drivebaseSubsystem, () -> 0, () -> 0), // SIM PURPOSES ONLY
        Commands.run(
                () ->
                    FuelSim.getInstance()
                        .launchFuel(
                            MetersPerSecond.of(6),
                            Radians.of(s.hood.getHoodPosition()),
                            Radians.of(s.turretSubsystem.getTurretPosition() + Math.PI),
                            Meters.of(1.45)))
            .withTimeout(3));
  }

  public static Command intakeCommand() {
return empty();
    //return s.intakeSubsystem.deployPivot();
  }

  public static Command climbCommand() {
    return Commands.none();
    // return s.climbSubsystem.autoAlignRoutine(ClimbState.Climbing);
  }
}
