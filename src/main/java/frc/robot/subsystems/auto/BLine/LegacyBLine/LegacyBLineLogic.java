package frc.robot.subsystems.auto.BLine.LegacyBLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.AutosCommands;
import frc.robot.subsystems.auto.BLine.BLineBuilder;
import frc.robot.subsystems.auto.BLine.BLineSystem;
import frc.robot.subsystems.auto.BLine.BLineTriggers;
import frc.robot.subsystems.auto.BLine.Positions;
import frc.robot.subsystems.auto.BLine.Positions.Position;
import frc.robot.subsystems.auto.BLine.TrenchSides.TrenchSide;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class LegacyBLineLogic implements BLineSystem {

  private static Subsystems s;
  public static Field2d field = new Field2d();
  public static Field2d fieldPoseStart = new Field2d();

  private static final List<LegacyBLinePath> autos = new ArrayList<>();
  private static final LoggedDashboardChooser<Position> startPositionChooser =
      new LoggedDashboardChooser<Position>("BLine/Start Position");
  private static final LoggedDashboardChooser<TrenchSide> trenchSideChooser =
      new LoggedDashboardChooser<TrenchSide>("BLine2/Trench Side");
  private static LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("BLine/Auto Chooser");
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

  // ========================= INIT =========================

  public void init(Subsystems subsystems) {
    s = subsystems;
    BLineTriggers.registerTriggers(s);

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

  // ========================= LOGGING =========================

  public void initAdvantageKit() {
    startPositionChooser.addDefaultOption(Position.MISC.title, Position.MISC);

    for (Position pos : Position.values()) {
      if (pos != Position.MISC) {
        startPositionChooser.addOption(pos.title, pos);
      }
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

  public void updateFieldDisplay() {
    fieldPoseStart.setRobotPose(getSelectedAutoStartingPose());
  }

  static Pose2d getTrenchPose() {
    boolean mirrored =
        getSelectedAutoPath().getStartPositionType() == Position.TRENCH
            && trenchSideChooser.getSendableChooser().getSelected() == TrenchSide.LEFT.title;

    return mirrored ? Positions.LEFT_TRENCH_POSE : Positions.RIGHT_TRENCH_POSE;
  }

  public static void filterAutos(int numGameObjects) {
    Position selected = startPositionChooser.get();

    if (selected == null) {
      selected = Position.MISC;
    }

    // Rebuild the options to avoid retaining stale selections.
    autoChooser = new LoggedDashboardChooser<>("BLine/Auto Chooser");

    for (LegacyBLinePath auto : autos) {
      if (selected == Position.MISC || auto.getStartPositionType() == selected) {
        autoChooser.addOption(auto.getDisplayName(), auto.getDisplayName());
      }
    }
  }

  // ========================= SELECTION METHODS =========================

  public String getSelectedAutoName() {
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

  public Pose2d getSelectedAutoStartingPose() {
    LegacyBLinePath selected = getSelectedAutoPath();

    if (selected == null) {
      return Pose2d.kZero;
    }

    if (startPositionChooser.get() == Position.TRENCH) {
      return getTrenchPose();
    }

    return selected.getStartPose2d();
  }

  // ========================= AUTO EXECUTION =========================
  public List<Path> getPathsToBuild() {
    LegacyBLinePath selected = getSelectedAutoPath();
    if (selected == null) {
      return List.of();
    }
    return selected.getAllPaths();
  }

  public Command getSelectedAuto() {
    if (s == null) {
      // Unit test mode - return empty command
      return Commands.none();
    }

    double delay = autoDelayEntry.getDouble(0.0);
    LegacyBLinePath selected = getSelectedAutoPath();

    if (selected == null) {
      return Commands.none();
    }

    List<Command> commands = new ArrayList<>();
    commands.add(
        Commands.runOnce(
            () ->
                s.drivebaseSubsystem.resetRotation(
                    selected.getPath().getInitialModuleDirection())));
    commands.add(Commands.waitSeconds(delay));

    List<Path> paths = getPathsToBuild();
    for (int i = 0; i < paths.size(); i++) {
      boolean resetPose = (i == 0);
      commands.add(buildPath(paths.get(i), resetPose));
    }

    return Commands.sequence(commands.toArray(new Command[0]));
  }

  public List<LegacyBLinePath> getBLinePaths() {
    return rebuiltPaths;
  }

  public List<String> getBLinePathsNames() {
    List<String> pathsNames = new ArrayList<>();
    for (LegacyBLinePath path : getBLinePaths()) {
      pathsNames.addAll(path.getDisplayingNames());
    }
    return pathsNames;
  }

  public Command buildSingleNeutralTrenchAuto() {
    return Commands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralTrench"), true),
        AutosCommands.launcherCommand(s));
  }

  public Command buildDoubleNeutralTrenchAuto() {
    return BLineCommands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralTrench"), true),
        AutosCommands.launcherCommand(4.5, s),
        buildPath(new Path("SecondNeutralTrench"), false),
        AutosCommands.launcherCommand(s));
  }

  public Command buildSingleDepotAuto() {
    return Commands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("TrenchDepot"), true),
        AutosCommands.launcherCommand(s));
  }

  public Command buildSingleNeutralTrenchDepotAuto() {
    return BLineCommands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("FirstNeutralTrench"), true),
        AutosCommands.launcherCommand(4.5, s),
        buildPath(new Path("TrenchDepot"), false),
        AutosCommands.launcherCommand(s));
  }

  public Command buildDefaultAuto() {
    return BLineCommands.sequence(
        Commands.waitSeconds(autoDelayEntry.getDouble(0.0)),
        buildPath(new Path("Default"), true),
        AutosCommands.launcherCommand(s));
  }

  private Command buildPath(Path path, boolean resetPose) {
    if (s == null
        || BLineBuilder.pathBuilder == null
        || BLineBuilder.continuingPathBuilder == null) {
      return Commands.none();
    }
    return (resetPose ? BLineBuilder.pathBuilder : BLineBuilder.continuingPathBuilder).build(path);
  }

  public Command handleAutos() {
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

  private void updateInitialHeading() {
    LegacyBLinePath selected = getSelectedAutoPath();
    if (selected == null || selected.getPath() == null) {
      initialHeading.setDefault(0.0);
      return;
    }
    Pose2d start = selected.getPath().getStartPose();
    initialHeading.set(Math.round(start.getRotation().getDegrees()));
  }
}
