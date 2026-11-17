package frc.robot.subsystems.auto.BLine;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.BLine.BLineField;
import frc.robot.lib.BLine.Path;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class BLinesField {
  private static final double DEFAULT_PLAYBACK_SPEED = 1.0;


  /* ---------------- NetworkTables init ---------------- */

  public static Runnable initSmartDashBoard(
      Supplier<String> tabName, int columnIndex, int rowIndex,Autos autos) {

    NetworkTableEntry speedMultiplier =
        NetworkTableInstance.getDefault().getTable("Autos").getEntry("DisplaySpeed");

    speedMultiplier.setDouble(DEFAULT_PLAYBACK_SPEED);

    BLinesField autonomousField =
        new BLinesField(() -> speedMultiplier.getDouble(DEFAULT_PLAYBACK_SPEED));

    SmartDashboard.putData("Selected auto", autonomousField.getField());
    SmartDashboard.putData("Start pose", autonomousField.getStartPose());

    return () -> {
      autonomousField.update(autos.getSelectedPath());

    };
  }

  /* ---------------- Display ---------------- */

  private final Field2d field = new Field2d();
  private final Field2d fieldPoseStart = new Field2d();

  /* ---------------- Auto data ---------------- */


  private int trajectoryIndex = 0;

  /* ---------------- Time ---------------- */

  private final DoubleSupplier speedMultiplier;
  private double lastFPGATime;
  private double lastTrajectoryTimeOffset;

  /* ---------------- State tracking ---------------- */

  private Optional<String> lastName = Optional.empty();

  /* ---------------- Constructors ---------------- */

  public BLinesField() {
    this(() -> 1.0);
  }

  public BLinesField(double speedMultiplier) {
    this(() -> speedMultiplier);
  }

  public BLinesField(DoubleSupplier speedMultiplier) {
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



  /* ---------------- Periodic update ---------------- */

  public void update(Path autoPath) {
    if (DriverStation.isEnabled()) {
        return;
    }

    if (autoPath == null) {
        System.out.println("Selected path is NULL");
        return;
    }

    if (!autoPath.isValid()) {
        System.out.println("Selected path is INVALID");
        return;
    }

    System.out.println("Drawing path: " + autoPath);

    BLineField.drawPath(field, autoPath);
    fieldPoseStart.setRobotPose(autoPath.getStartPose());
}

  /* ---------------- Timing ---------------- */


}
