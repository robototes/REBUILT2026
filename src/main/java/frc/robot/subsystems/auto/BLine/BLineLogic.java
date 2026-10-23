package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.Positions.Position;
import frc.robot.subsystems.auto.BLine.TrenchSides.TrenchSide;
import frc.robot.util.Elastic;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class BLineLogic implements BLineSystem {

  private static Subsystems s;

  public static Field2d field = new Field2d();

  private static final String REMOVE_OPTION = "REMOVE";
  private static final LoggedDashboardChooser<TrenchSide> trenchSideChooser =
      new LoggedDashboardChooser<TrenchSide>("BLine/Trench Side");
  private static final List<SendableChooser<String>> pathChoosers = new ArrayList<>();
  private static final Map<BLinePath, List<BLinePath>> rebuiltPaths = new HashMap<>();
  private static final List<BLinePath> autos = new ArrayList<>();
  private static final Map<String, BLinePath> namesToAuto = new HashMap<>();
  private static boolean pathsInitialized = false;
  public static Command bLineLaunching;
  public static Path currentPath;
  public static FollowPath follow;
  public static int savedPathIndex = -1;
  private static boolean unlimitedAlreadySelected = false;
  private static LoggedNetworkNumber initialHeading =
      new LoggedNetworkNumber("Initial Heading(Deg)", 0.0);
  private static LoggedNetworkNumber autoDelay = new LoggedNetworkNumber("BLine Delay", 0.0);

  private static Elastic.Notification autoTimingWarning =
      new Elastic.Notification(
          Elastic.NotificationLevel.WARNING,
          "LAUNCH COMMAND WARNING",
          "The Launch command will be called and will not time out.");
  private static final List<NetworkTableEntry> autoDelayEntries = new ArrayList<>();

  public static final String keys = "RB=Right Bump, LB=Left Bump, LT=Left Trench, RT=Right Trench";
  private static LoggedNetworkString key = new LoggedNetworkString("Auto Keys", keys);

  public static boolean isMirrored() {
    return getSelectedAutoPath().getStartPositionType() == Position.TRENCH
        && trenchSideChooser.get() == TrenchSide.LEFT;
  }

  static Pose2d getTrenchPose() {

    return isMirrored() ? Positions.LEFT_TRENCH_POSE : Positions.RIGHT_TRENCH_POSE;
  }

  public void init(Subsystems subsystems) {

    s = subsystems;

    BLineTriggers.registerTriggers(s);

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

    rebuiltPaths.clear();
    autos.clear();
    namesToAuto.clear();
    pathChoosers.clear();
    for (BLinePath path : BLinePaths.paths) {
      handleStartingPoses(path);
    }

    for (BLinePath path : BLinePaths.paths) {
      rebuiltPaths.put(path, BLinePaths.findAdjecentPaths(path));
    }

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

    for (int i = 0; i < Robot.MAX_STEPS; i++) {

      LoggedDashboardChooser<String> chooser =
          new LoggedDashboardChooser<>("BLine/Path Step" + (i + 1));

      pathChoosers.add(i, chooser.getSendableChooser());
    }

    populateFirstChooser();
    for (int i = 0; i < pathChoosers.size(); i++) {
      int ind = i;
      pathChoosers
          .get(i)
          .onChange(
              value -> {
                BLinePath selected = getSelectedPath(ind);

                if (selected != null && selected.getShootMode() == BLinePath.ShootMode.UNLIMITED) {
                  Elastic.sendNotification(autoTimingWarning);
                }

                updatePathChoosers();
              });
    }
  }

  private static void populateFirstChooser() {

    SendableChooser<String> chooser = pathChoosers.get(0);

    // chooser.clearOptions();

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

      // Check whether an UNLIMITED path has already been selected
      unlimitedAlreadySelected = false;

      for (int i = 0; i <= step; i++) {
        BLinePath selectedPath = getSelectedPath(i);

        if (selectedPath != null && selectedPath.getShootMode() == BLinePath.ShootMode.UNLIMITED) {
          unlimitedAlreadySelected = true;
          break;
        }
      }

      SendableChooser<String> nextChooser = pathChoosers.get(step + 1);
      // nextChooser.clearOptions();

      for (BLinePath nextPath : nextPaths) {

        // Don't allow more than one UNLIMITED shooting path
        if (nextPath.getShootMode() == BLinePath.ShootMode.UNLIMITED && unlimitedAlreadySelected) {

          Elastic.sendNotification(autoTimingWarning);
          continue;
        }

        nextChooser.addOption(nextPath.getDisplayName(), nextPath.getDisplayName());
      }

      // Always allow removing this step
      nextChooser.addOption("Remove", REMOVE_OPTION);
    }
  }

  private static void clearChoosersAfter(int step) {

    for (int i = step + 1; i < pathChoosers.size(); i++) {

      // pathChoosers.get(i).clearOptions();
    }
  }

  private static BLinePath getSelectedPath(int step) {
    if (step < 0 || step >= pathChoosers.size()) {

      return null;
    }

    String selectedName = pathChoosers.get(step).getSelected();

    if (selectedName == null || REMOVE_OPTION.equals(selectedName)) {
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

  public void initAdvantageKit() {
    autoDelayEntries.clear();

    for (int i = 0; i < pathChoosers.size(); i++) {
      String key = "BLine/Step " + (i + 1) + " Delay";

      autoDelayEntries.add(NetworkTableInstance.getDefault().getTable("BLine").getEntry(key));
      NetworkTableInstance.getDefault().getTable("BLine").getEntry(key).setDefaultDouble(0.0);
    }
    trenchSideChooser.addDefaultOption(TrenchSide.RIGHT.title, TrenchSide.RIGHT);

    trenchSideChooser.addOption(TrenchSide.LEFT.title, TrenchSide.LEFT);

    // SmartDashboard.putData("BLine/Selected Auto", field);

    trenchSideChooser.onChange(
        value -> {
          updateInitialHeading();
          // updateFieldDisplay();
        });

    updatePathChoosers();
    // updateFieldDisplay();
  }

  public void updateFieldDisplay() {
    // fieldPoseStart.setRobotPose(getSelectedAutoStartingPose());
  }

  public String getSelectedAutoName() {

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
      return BLinePaths.Default;
    }

    return selected;
  }

  public Pose2d getSelectedAutoStartingPose() {

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

  public List<BLinePath> getBLinePaths() {

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
    Pose2d pathPose = path.getPath().getStartPose();

    double positionTolerance = 0.1;
    double rotationTolerance = 5.0;

    for (Position position : Position.values()) {
      Pose2d startPose = position.pose;

      double distance = pathPose.getTranslation().getDistance(startPose.getTranslation());

      double rotationDifference =
          Math.abs(pathPose.getRotation().minus(startPose.getRotation()).getDegrees());

      if (rotationDifference > 180) {
        rotationDifference = 360 - rotationDifference;
      }

      if (distance <= positionTolerance && rotationDifference <= rotationTolerance) {

        path.setStartPose2d(position);
        return;
      }
    }

    path.setStartPose2d(Position.MISC);
  }

  private static void updateInitialHeading() {

    BLinePath selected = getSelectedAutoPath();

    if (selected == null || selected.getPath() == null) {
      initialHeading.set(0.0);

      return;
    }

    Pose2d start = selected.getStartPose2d();

    initialHeading.set(Math.round(start.getRotation().getDegrees()));
  }

  public Command getSelectedAuto() {
    if (s == null) return Commands.none();

    List<BLinePath> sequence = getSelectedPathSequence();

    if (sequence.isEmpty()) {
      return buildPath(BLinePaths.Default.getPath(), true, true);
    }

    List<Command> commands = new ArrayList<>();
    boolean firstPath = true;

    for (int step = 0; step < sequence.size(); step++) {

      //  before this BLinePath
      double delay = autoDelayEntries.get(step).getDouble(0.0);
      commands.add(Commands.waitSeconds(delay));

      // Run every Path contained in this BLinePath
      for (Path path : sequence.get(step).getAllPaths()) {
        if (sequence.get(0).equals(BLinePaths.Default)) {
          commands.add(buildPath(BLinePaths.Default.getPath(), firstPath, true));
          break;
        }
        commands.add(buildPath(path, firstPath, true));
        firstPath = false;
      }

      // Shoot based on this BLinePath's ShootMode
      BLinePath.ShootMode shootMode = sequence.get(step).getShootMode();

      if (shootMode == BLinePath.ShootMode.TIMED) {
        commands.add(AutosCommands.launcherCommand(5.0, s));

      } else if (shootMode == BLinePath.ShootMode.UNLIMITED) {
        commands.add(
            (AutosCommands.launcherCommand(12.0, s).until(() -> !RobotState.isAutonomous())));
      }
    }

    // Set the robot's initial rotation from the first path
    List<Path> firstStepPaths = sequence.get(0).getAllPaths();

    if (!firstStepPaths.isEmpty()) {
      s.drivebaseSubsystem.resetRotation(firstStepPaths.get(0).getInitialModuleDirection());
    }

    return Commands.sequence(commands.toArray(new Command[0]));
  }

  public static Command buildPath(Path path, boolean resetPose, boolean saveForRecovery) {

    if (s == null
        || BLineBuilder.pathBuilder == null
        || BLineBuilder.continuingPathBuilder == null) {
      return Commands.none();
    }

    FollowPath command =
        (resetPose ? BLineBuilder.pathBuilder : BLineBuilder.continuingPathBuilder).build(path);

    if (saveForRecovery) {
      currentPath = path;
      follow = command;
    }

    return command;
  }
}
