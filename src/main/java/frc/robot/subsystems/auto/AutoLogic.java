package frc.robot.subsystems.auto;

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
import frc.robot.subsystems.AutoRotate;
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
        "Left Trench", new Pose2d(4.014, 7.382, new Rotation2d(Units.degreesToRadians(90)))),
    LEFT_BUMP("Left Bump", new Pose2d(3.664, 5.411, new Rotation2d(Units.degreesToRadians(90)))),
    CENTER("Center", new Pose2d(3.595, 4.008, new Rotation2d(Units.degreesToRadians(180)))),
    RIGHT_BUMP("Right Bump", new Pose2d(3.638, 2.322, new Rotation2d(Units.degreesToRadians(-90)))),
    RIGHT_TRENCH(
        "Right Trench", new Pose2d(3.641, 0.576, new Rotation2d(Units.degreesToRadians(-90)))),
    MISC("Misc", null);

    final String title;
    final Pose2d startPose;

    StartPosition(String title, Pose2d startPose) {
      this.title = title;
      this.startPose = startPose;
    }
  }

  /* ---------------- Paths ---------------- */

  private static final AutoPath defaultPath = new AutoPath("Drive", "Drive");

  private static final List<AutoPath> rebuiltPaths =
      List.of(
          new AutoPath("LB-NeutralLeft-Climb", "LB-NeutralLeft-Climb"),
          new AutoPath("C-Climb", "C-Climb"),
          new AutoPath("C-Depot-Climb", "C-Depot-Climb"),
          new AutoPath("C-Outpost-Climb", "C-Outpost-Climb"),
          new AutoPath("RT-Outpost-Climb", "RT-Outpost-Climb"),
           new AutoPath("RB-Outpost-Climb", "RB-Outpost-Climb"),
          new AutoPath("Drive", "Drive"),
          new AutoPath("LB-NeutralLeft-LB-NeutralLeft-LB", "LB-NeutralLeft-LB-NeutralLeft-LB"),
          new AutoPath("LB-Depot-Climb", "LB-Depot-Climb"),
          new AutoPath("LT-Depot-Climb", "LT-Depot-Climb"),
          new AutoPath("LT-NeutralLeft-Climb", "LT-NeutralLeft-Climb"),
          new AutoPath("LT-NeutralLeft-LB-NeutralLeft-LB", "LT-NeutralLeft-LB-NeutralLeft-LB"),
          new AutoPath("RB-NeutralRight-Climb", "RB-NeutralRight-Climb"),
          new AutoPath("RB-NeutralRight-NeutralRight-RB", "RB-NeutralRight-NeutralRight-RB"),
          new AutoPath("RT-NeutralRight-Climb", "RT-NeutralRight-Climb"),
          new AutoPath("RT-NeutralRight-RB-NeutralRight", "RT-NeutralRight-RB-NeutralRight")
         );
  private static final List<AutoPath> choreoPaths =
      List.of(new AutoPath("LB-Depot-Climb(CHOREO)", "LB-Depot-Climb(CHOREO)"));

  private static final Map<Integer, List<AutoPath>> commandsMap =
      Map.of(0, rebuiltPaths, 1, choreoPaths);

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
    NamedCommands.registerCommand("intake", intakeCommand());
    NamedCommands.registerCommand("climb", climbCommand());
  }

  public static Command launcherCommand() {
    return Commands.sequence(
        Commands.parallel(
            s.Flywheels.setVelocityCommand(100),
            s.Hood.hoodPositionCommand(0.5),
            AutoRotate.autoRotate(s.drivebaseSubsystem, () -> 0, () -> 0)),
        s.Index.setPowerCommand(0.3).withTimeout(2));
  }

  public static Command intakeCommand() {
    return s.Intake.setPowerCommand(1);
  }

  public static Command climbCommand() {
    return Commands.none();
  }
}
