package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.BLine.Path;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.ObjDoubleConsumer;
import java.util.function.Supplier;

public class BLineAutonomousField {
  private static final double DEFAULT_PLAYBACK_SPEED = 1.0;
  private static final double UPDATE_RATE = 0.05;

  private static final double ASSUMED_SPEED_MPS = 3.0;

  public static void initSmartDashBoard(
      Supplier<String> tabName,
      int columnIndex,
      int rowIndex,
      ObjDoubleConsumer<Runnable> addPeriodic) {

    NetworkTableEntry speedMultiplier =
        NetworkTableInstance.getDefault().getTable("Autos").getEntry("DisplaySpeed");

    speedMultiplier.setDouble(DEFAULT_PLAYBACK_SPEED);

    @SuppressWarnings("static-access")
    BLineAutonomousField autonomousField =
        new BLineAutonomousField(() -> speedMultiplier.getDouble(DEFAULT_PLAYBACK_SPEED));

    SmartDashboard.putData("Selected auto", autonomousField.getField());
    SmartDashboard.putData("Start pose", autonomousField.getStartPose());

    addPeriodic.accept(
        () -> {
          autonomousField.update(BLineLogic.getSelectedAutoName());
          BLineLogic.getSelectedAutoPath();
          SmartDashboard.putNumber(
              "Est. Time (s)",
              Math.round(BLineLogic.getSelectedAutoPath().autoTotalTime() * 100.0) / 100.0);
        },
        UPDATE_RATE);
  }

  private final Field2d field = new Field2d();
  private final Field2d fieldPoseStart = new Field2d();

  private BLineAutos autoData;
  private List<Path> paths;
  private int pathIndex = 0;

  private List<Translation2d> currentTranslations = List.of();
  private double currentPathLength = 1e-9;

  private final DoubleSupplier speedMultiplier;
  private double lastFPGATime;
  private double distanceAlongCurrentPath;

  private Optional<String> lastName = Optional.empty();

  public BLineAutonomousField() {
    this(() -> 1.0);
  }

  public BLineAutonomousField(double speedMultiplier) {
    this(() -> speedMultiplier);
  }

  public BLineAutonomousField(DoubleSupplier speedMultiplier) {
    this.speedMultiplier = speedMultiplier;
  }

  public Field2d getField() {
    return field;
  }

  public Field2d getStartPose() {
    return fieldPoseStart;
  }

  private static double arcLength(List<Translation2d> pts) {
    double total = 0;
    for (int i = 1; i < pts.size(); i++) {
      total += pts.get(i).getDistance(pts.get(i - 1));
    }
    return total;
  }

  private static Pose2d samplePolyline(List<Translation2d> pts, double dist) {
    if (pts.isEmpty()) return Pose2d.kZero;
    if (pts.size() == 1) return new Pose2d(pts.get(0), Rotation2d.kZero);

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
    if (paths == null || paths.isEmpty()) {
      currentTranslations = List.of();
      currentPathLength = 1e-9;
      return;
    }
    currentTranslations = paths.get(pathIndex).getTranslations();
    currentPathLength = Math.max(arcLength(currentTranslations), 1e-9);
  }

  public Pose2d getUpdatedPose(String autoName) {
    if (autoName == null) return Pose2d.kZero;

    double speed = speedMultiplier.getAsDouble();
    double fpgaTime = Timer.getFPGATimestamp();

    if (lastName.isEmpty() || !lastName.get().equals(autoName)) {
      lastName = Optional.of(autoName);
      autoData = new BLineAutos(autoName);
      paths = autoData.getPaths();
      pathIndex = 0;
      distanceAlongCurrentPath = 0;
      lastFPGATime = fpgaTime;
      refreshCurrentPath();
    }

    if (paths.isEmpty()) {
      return autoData.getStartingPose();
    }

    distanceAlongCurrentPath += (fpgaTime - lastFPGATime) * speed * ASSUMED_SPEED_MPS;
    lastFPGATime = fpgaTime;

    while (distanceAlongCurrentPath > currentPathLength) {
      distanceAlongCurrentPath -= currentPathLength;
      pathIndex++;
      if (pathIndex >= paths.size()) {
        pathIndex = 0;
      }
      refreshCurrentPath();
    }

    return samplePolyline(currentTranslations, distanceAlongCurrentPath);
  }

  public void update(String autoName) {
    if (DriverStation.isEnabled()) {
      lastName = Optional.empty();
      return;
    }
    if (autoName == null) return;

    field.setRobotPose(getUpdatedPose(autoName));
    if (autoData != null) {
      fieldPoseStart.setRobotPose(autoData.getStartingPose());
    }
  }
}
