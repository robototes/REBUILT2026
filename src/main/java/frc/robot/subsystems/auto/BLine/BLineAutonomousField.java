package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.BLine.Path;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.ObjDoubleConsumer;
import java.util.function.Supplier;

public class BLineAutonomousField {

  private static final double DEFAULT_PLAYBACK_SPEED = 1.0;
  private static final double UPDATE_RATE = 0.02;
  private static final double ASSUMED_SPEED_MPS = 3.0;

  /* ---------------- NetworkTables init ---------------- */

  public static void initSmartDashBoard(
      Supplier<String> tabName,
      int columnIndex,
      int rowIndex,
      ObjDoubleConsumer<Runnable> addPeriodic) {

    NetworkTableEntry speedMultiplier =
        NetworkTableInstance.getDefault().getTable("Autos").getEntry("DisplaySpeed");

    speedMultiplier.setDouble(DEFAULT_PLAYBACK_SPEED);

    BLineAutonomousField autonomousField =
        new BLineAutonomousField(() -> speedMultiplier.getDouble(DEFAULT_PLAYBACK_SPEED));

    addPeriodic.accept(
        () -> {
          autonomousField.update(BLineLogic.getSelectedAutoName());
        },
        UPDATE_RATE);
  }

  /* ---------------- Path data ---------------- */

  private List<Path> paths = List.of();
  private int pathIndex = 0;
  private Pose2d startingPose = Pose2d.kZero;

  /* ---------------- Polylines ---------------- */

  private List<Translation2d> currentTranslations = List.of();
  private double currentPathLength = 1e-9;

  /* ---------------- Time / distance ---------------- */

  private final DoubleSupplier speedMultiplier;
  private double lastFPGATime;
  private double distanceAlongCurrentPath;

  private Optional<String> lastName = Optional.empty();

  /* ---------------- Constructors ---------------- */

  public BLineAutonomousField() {
    this(() -> 1.0);
  }

  public BLineAutonomousField(double speedMultiplier) {
    this(() -> speedMultiplier);
  }

  public BLineAutonomousField(DoubleSupplier speedMultiplier) {
    this.speedMultiplier = speedMultiplier;
  }

  /* ---------------- Geometry helpers ---------------- */
  private void updatePathDisplay() {

    List<BLinePath> sequence = BLineLogic.getSelectedPathSequence();

    for (int i = 0; i < sequence.size(); i++) {

      BLinePath bLinePath = sequence.get(i);

      for (int j = 0; j < bLinePath.getAllPaths().size(); j++) {

        Path path = bLinePath.getAllPaths().get(j);

        List<Translation2d> translations = path.getTranslations();

        List<Pose2d> poses = new java.util.ArrayList<>();

        for (int k = 0; k < translations.size(); k++) {

          Translation2d translation = translations.get(k);

          Rotation2d rotation = Rotation2d.kZero;

          if (k < translations.size() - 1) {
            Translation2d next = translations.get(k + 1);

            rotation =
                new Rotation2d(next.getX() - translation.getX(), next.getY() - translation.getY());
          } else if (k > 0) {
            Translation2d previous = translations.get(k - 1);

            rotation =
                new Rotation2d(
                    translation.getX() - previous.getX(), translation.getY() - previous.getY());
          }

          poses.add(new Pose2d(translation, rotation));
        }

        BLineLogic.field.getObject("Path " + i + "-" + j).setPoses(poses);
      }
    }
  }

  private static double arcLength(List<Translation2d> pts) {

    double total = 0;

    for (int i = 1; i < pts.size(); i++) {
      total += pts.get(i).getDistance(pts.get(i - 1));
    }

    return total;
  }

  private static Pose2d samplePolyline(List<Translation2d> pts, double dist) {

    if (pts.isEmpty()) {
      return Pose2d.kZero;
    }

    if (pts.size() == 1) {
      return new Pose2d(pts.get(0), Rotation2d.kZero);
    }

    double remaining = Math.max(0, dist);

    for (int i = 1; i < pts.size(); i++) {

      Translation2d a = pts.get(i - 1);
      Translation2d b = pts.get(i);

      double segLen = b.getDistance(a);
      boolean lastSeg = (i == pts.size() - 1);

      if (remaining <= segLen || lastSeg) {

        double t = (segLen > 1e-9) ? Math.min(remaining / segLen, 1.0) : 1.0;

        return new Pose2d(
            a.interpolate(b, t), new Rotation2d(b.getX() - a.getX(), b.getY() - a.getY()));
      }

      remaining -= segLen;
    }

    Translation2d last = pts.get(pts.size() - 1);
    Translation2d prev = pts.get(pts.size() - 2);

    return new Pose2d(last, new Rotation2d(last.getX() - prev.getX(), last.getY() - prev.getY()));
  }

  private void refreshCurrentPath() {

    if (paths.isEmpty()) {
      currentTranslations = List.of();
      currentPathLength = 0;
      return;
    }

    currentTranslations = paths.get(pathIndex).getTranslations();
    currentPathLength = Math.max(arcLength(currentTranslations), 0);
  }

  /* ---------------- Pose update ---------------- */

  public Pose2d getUpdatedPose(String autoName) {

    if (autoName == null) {
      return Pose2d.kZero;
    }

    double speed = speedMultiplier.getAsDouble();
    double fpgaTime = Timer.getFPGATimestamp();

    if (lastName.isEmpty() || !lastName.get().equals(autoName)) {

      lastName = Optional.of(autoName);

      List<BLinePath> sequence = BLineLogic.getSelectedPathSequence();

      paths = sequence.stream().flatMap(bLinePath -> bLinePath.getAllPaths().stream()).toList();

      startingPose = BLineLogic.getSelectedAutoStartingPose();

      pathIndex = 0;
      distanceAlongCurrentPath = 0;
      lastFPGATime = fpgaTime;

      refreshCurrentPath();
    }

    if (paths.isEmpty()) {
      return startingPose;
    }

    distanceAlongCurrentPath += (fpgaTime - lastFPGATime) * speed * ASSUMED_SPEED_MPS;

    lastFPGATime = fpgaTime;

    while (distanceAlongCurrentPath > currentPathLength) {

      if (currentPathLength <= 1e-9) {

        pathIndex++;

        if (pathIndex >= paths.size()) {
          pathIndex = 0;
        }

        distanceAlongCurrentPath = 0;
        refreshCurrentPath();
        continue;
      }

      distanceAlongCurrentPath -= currentPathLength;

      pathIndex++;

      if (pathIndex >= paths.size()) {
        pathIndex = 0;
      }

      refreshCurrentPath();
    }

    return samplePolyline(currentTranslations, distanceAlongCurrentPath);
  }

  /* ---------------- Periodic update ---------------- */

  public void update(String autoName) {
    

    if (DriverStation.isEnabled()) {
      lastName = Optional.empty();
      return;
    }

    if (autoName == null) {
      return;
    }

    BLineLogic.field.setRobotPose(getUpdatedPose(autoName));



    BLineLogic.fieldPoseStart.setRobotPose(startingPose);
  }
}
