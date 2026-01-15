package frc.robot.subsystems.auto;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.ObjDoubleConsumer;
import java.util.function.Supplier;

public class AutonomousField {
  private static final double DEFAULT_PLAYBACK_SPEED = 1.0;
  private static final double UPDATE_RATE = 0.05;

  /* ---------------- NetworkTables init ---------------- */

  public static void initSmartDashBoard(
      Supplier<String> tabName,
      int columnIndex,
      int rowIndex,
      ObjDoubleConsumer<Runnable> addPeriodic) {

    NetworkTableEntry speedMultiplier =
        NetworkTableInstance.getDefault()
            .getTable("Auto")
            .getEntry("DisplaySpeed");

    speedMultiplier.setDouble(DEFAULT_PLAYBACK_SPEED);

    AutonomousField autonomousField =
        new AutonomousField(() -> speedMultiplier.getDouble(DEFAULT_PLAYBACK_SPEED));

    SmartDashboard.putData("Selected auto", autonomousField.getField());
    SmartDashboard.putData("Start pose", autonomousField.getStartPose());

    addPeriodic.accept(
        () -> {
          autonomousField.update(AutoLogic.getSelectedAutoName());
          SmartDashboard.putNumber(
              "Est. Time (s)",
              Math.round(autonomousField.autoTotalTime() * 100.0) / 100.0
          );
        },
        UPDATE_RATE
    );
  }

  /* ---------------- Display ---------------- */

  private final Field2d field = new Field2d();
  private final Field2d fieldPoseStart = new Field2d();

  /* ---------------- Auto data ---------------- */

  private PathPlannerAutos autoData;
  private List<PathPlannerTrajectory> trajectories;
  private int trajectoryIndex = 0;

  /* ---------------- Time ---------------- */

  private final DoubleSupplier speedMultiplier;
  private double lastFPGATime;
  private double lastTrajectoryTimeOffset;

  /* ---------------- State tracking ---------------- */

  private Optional<String> lastName = Optional.empty();

  /* ---------------- Constructors ---------------- */

  public AutonomousField() {
    this(() -> 1.0);
  }

  public AutonomousField(double speedMultiplier) {
    this(() -> speedMultiplier);
  }

  public AutonomousField(DoubleSupplier speedMultiplier) {
    this.speedMultiplier = speedMultiplier;
  }

  /* ---------------- Accessors ---------------- */

  public Field2d getField() {
    return field;
  }

  public Field2d getStartPose() {
    return fieldPoseStart;
  }

  /* ---------------- Pose update ---------------- */

  public Pose2d getUpdatedPose(String autoName) {
    double speed = speedMultiplier.getAsDouble();
    double fpgaTime = Timer.getFPGATimestamp();

    if (lastName.isEmpty() || !lastName.get().equals(autoName)) {
      lastName = Optional.of(autoName);
      autoData = new PathPlannerAutos(autoName);
      trajectories = autoData.getTrajectories();
      trajectoryIndex = 0;
      lastFPGATime = fpgaTime;
      lastTrajectoryTimeOffset = 0;
    }

    if (trajectories.isEmpty()) {
      if (autoData.getStartingPose() != null) {
        return autoData.getStartingPose();
      }
      return Pose2d.kZero;
    }

    lastTrajectoryTimeOffset += (fpgaTime - lastFPGATime) * speed;
    lastFPGATime = fpgaTime;

    while (lastTrajectoryTimeOffset >
        trajectories.get(trajectoryIndex).getTotalTimeSeconds()) {

      lastTrajectoryTimeOffset -=
          trajectories.get(trajectoryIndex).getTotalTimeSeconds();

      trajectoryIndex++;
      if (trajectoryIndex >= trajectories.size()) {
        trajectoryIndex = 0;
      }
    }

    return trajectories
        .get(trajectoryIndex)
        .sample(lastTrajectoryTimeOffset)
        .pose;
  }

  /* ---------------- Periodic update ---------------- */

  public void update(String autoName) {
    if (DriverStation.isEnabled()) {
      lastName = Optional.empty();
      return;
    }

    field.setRobotPose(getUpdatedPose(autoName));
    fieldPoseStart.setRobotPose(autoData.getStartingPose());
  }

  /* ---------------- Timing ---------------- */

  public double autoTotalTime() {
    if (autoData == null) {
      return 0;
    }
    return autoData.getRunTime();
  }
}
