package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.CompTunerConstants;

import java.util.function.BooleanSupplier;

public class Controls {
  private static final int SOLO_CONTROLLER_PORT = 0;
  private static final int DRIVER_CONTROLLER_PORT = 1;
  private static final int OPERATOR_CONTROLLER_PORT = 2;
  private static final int ARM_PIVOT_SPINNY_CLAW_CONTROLLER_PORT = 3;
  private static final int ELEVATOR_CONTROLLER_PORT = 4;
  private static final int CLIMB_TEST_CONTROLLER_PORT = 5;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final CommandXboxController soloController;

  private final Subsystems s;

  /* ---------------- Swerve constants ---------------- */

  public static final double MaxSpeed =
      CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public static double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0001)
          .withRotationalDeadband(0.0001)
          .withDriveRequestType(DriveRequestType.Velocity);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final BooleanSupplier driveSlowMode;

  /* ---------------- NetworkTables ---------------- */

  private final NetworkTableEntry swerveCoastEntry =
      NetworkTableInstance.getDefault()
          .getTable("Controls")
          .getEntry("Swerve Coast Mode");

  public Controls(Subsystems s) {
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  
    soloController = new CommandXboxController(SOLO_CONTROLLER_PORT);

    this.s = s;

    driveSlowMode = driverController.start();

    // default value
    swerveCoastEntry.setBoolean(false);
    SmartDashboard.putBoolean("Swerve Coast Mode", false);

    configureDrivebaseBindings();
    configureAutoAlignBindings();
  }

  private Trigger connected(CommandXboxController controller) {
    return new Trigger(controller::isConnected);
  }

  private double getDriveX() {
    double input = MathUtil.applyDeadband(-driverController.getLeftY(), 0.1);
    double scale = driveSlowMode.getAsBoolean() ? 0.5 : 1.0;
    return input * MaxSpeed * scale;
  }

  private double getDriveY() {
    double input = MathUtil.applyDeadband(-driverController.getLeftX(), 0.1);
    double scale = driveSlowMode.getAsBoolean() ? 0.5 : 1.0;
    return input * MaxSpeed * scale;
  }

  private double getDriveRotate() {
    double input = MathUtil.applyDeadband(-driverController.getRightX(), 0.1);
    double scale = driveSlowMode.getAsBoolean() ? 0.5 : 1.0;
    return input * MaxSpeed * scale;
  }

  /* ---------------- Drive bindings ---------------- */

  private void configureDrivebaseBindings() {
    if (s.drivebaseSubsystem == null) {
      return;
    }

    s.drivebaseSubsystem.registerTelemetry(logger::telemeterize);

    // Coast mode toggle via NetworkTables / SmartDashboard
    new Trigger(() -> swerveCoastEntry.getBoolean(false))
        .whileTrue(s.drivebaseSubsystem.coastMotors());
  }

  private void configureAutoAlignBindings() {
    if (s.drivebaseSubsystem == null) {
      return;
    }
  }

  /* ---------------- Rumble ---------------- */

  private Command rumble(CommandXboxController controller, double vibration, Time duration) {
    return Commands.startEnd(
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, vibration),
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(duration)
        .withName("Rumble Port " + controller.getHID().getPort());
  }

  public void vibrateDriveController(double vibration) {
    if (!DriverStation.isAutonomous()) {
      driverController.getHID().setRumble(RumbleType.kBothRumble, vibration);
    }
  }

  public void vibrateCoDriveController(double vibration) {
    if (!DriverStation.isAutonomous()) {
      operatorController.getHID().setRumble(RumbleType.kBothRumble, vibration);
    }
  }

  /* ---------------- Solo drive helpers ---------------- */

  private double getJoystickInput(double input) {
    if (soloController.leftStick().getAsBoolean()
        || soloController.rightStick().getAsBoolean()) {
      return 0;
    }
    return MathUtil.applyDeadband(input, 0.1);
  }

  private double getSoloDriveX() {
    return getJoystickInput(-soloController.getLeftY()) * MaxSpeed;
  }

  private double getSoloDriveY() {
    return getJoystickInput(-soloController.getLeftX()) * MaxSpeed;
  }

  private double getSoloDriveRotate() {
    return getJoystickInput(-soloController.getRightX()) * MaxSpeed;
  }
}
