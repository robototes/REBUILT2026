package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.auto.AutoDriveRotate;
import frc.robot.subsystems.auto.FuelAutoAlign;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Controls {
  // The robot's subsystems and commands are defined here...
  private final Subsystems s;

  // Controller Ports
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int FEEDER_TEST_CONTROLLER_PORT = 1;
  private static final int SPINDEXER_TEST_CONTROLLER_PORT = 2;
  private static final int INTAKE_TEST_CONTROLLER_PORT = 3;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private final CommandXboxController feederTestController =
      new CommandXboxController(FEEDER_TEST_CONTROLLER_PORT);

  private final CommandXboxController spindexerTestController =
      new CommandXboxController(SPINDEXER_TEST_CONTROLLER_PORT);
  private final CommandXboxController intakeTestController =
      new CommandXboxController(INTAKE_TEST_CONTROLLER_PORT);

  public static final double MaxSpeed = CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // kSpeedAt12Volts desired top speed
  public static double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final double driveInputScale = 1;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0001)
          .withRotationalDeadband(0.0001)
          .withDriveRequestType(DriveRequestType.Velocity);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Controls(Subsystems subsystems) {
    // Configure the trigger bindings
    s = subsystems;
    configureDrivebaseBindings();
    if (s.detectionSubsystem != null) {
      configureAutoAlignBindings();
    }
    configureSpindexerBindings();
    configureFeederBindings();
    configureIntakeBindings();
  }

  private void configureSpindexerBindings() {
    // start serializer motor
    spindexerTestController.a().onTrue(s.spindexerSubsystem.startMotor());

    // stop serializer motor
    spindexerTestController.b().onTrue(s.spindexerSubsystem.stopMotor());
  }

  public Command setRumble(RumbleType type, double value) {
    return Commands.runOnce(
        () -> {
          driverController.setRumble(type, value);
        });
  }

  private void configureFeederBindings() {
    // TODO: wait for sensor to reach threshold, and trigger rumble

    // start feeder motor
    feederTestController.a().onTrue(s.feederSubsystem.startMotor());

    // stop feeder motor
    feederTestController.b().onTrue(s.feederSubsystem.stopMotor());
  }

  private Command rumble(CommandXboxController controller, double vibration, Time duration) {
    return Commands.startEnd(
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, vibration),
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(duration)
        .withName("Rumble Port " + controller.getHID().getPort());
  }

  // takes the X value from the joystick, and applies a deadband and input scaling
  private double getDriveX() {
    // Joystick +Y is back
    // Robot +X is forward
    double input = MathUtil.applyDeadband(-driverController.getLeftY(), 0.1);
    return input * MaxSpeed * driveInputScale;
  }

  // takes the Y value from the joystick, and applies a deadband and input scaling
  private double getDriveY() {
    // Joystick +X is right
    // Robot +Y is left
    double input = MathUtil.applyDeadband(-driverController.getLeftX(), 0.1);
    return input * MaxSpeed * driveInputScale;
  }

  // takes the rotation value from the joystick, and applies a deadband and input scaling
  private double getDriveRotate() {
    // Joystick +X is right
    // Robot +angle is CCW (left)
    double input = MathUtil.applyDeadband(-driverController.getRightX(), 0.1);
    return input * MaxSpeed * driveInputScale;
  }

  private void configureDrivebaseBindings() {
    if (s.drivebaseSubsystem == null) {
      // Stop running this method
      return;
    }

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    // the driving command for just driving around
    s.drivebaseSubsystem.setDefaultCommand(
        // s.drivebaseSubsystem will execute this command periodically

        // applying the request to drive with the inputs
        s.drivebaseSubsystem
            .applyRequest(
                () ->
                    drive
                        .withVelocityX(getDriveX())
                        .withVelocityY(getDriveY())
                        .withRotationalRate(getDriveRotate()))
            .withName("Drive"));

    // driverController.a().whileTrue(s.drivebaseSubsystem.sysIdDynamic(Direction.kForward));
    // driverController.b().whileTrue(s.drivebaseSubsystem.sysIdDynamic(Direction.kReverse));
    // driverController.y().whileTrue(s.drivebaseSubsystem.sysIdQuasistatic(Direction.kForward));
    // driverController.x().whileTrue(s.drivebaseSubsystem.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on back button press
    driverController
        .back()
        .onTrue(
            s.drivebaseSubsystem
                .runOnce(() -> s.drivebaseSubsystem.seedFieldCentric())
                .alongWith(rumble(driverController, 0.5, Seconds.of(0.3)))
                .withName("Reset gyro"));

    // logging the telemetry
    s.drivebaseSubsystem.registerTelemetry(logger::telemeterize);

    // Auto rotate
    driverController
        .rightTrigger()
        .whileTrue(
            AutoDriveRotate.autoRotate(
                    s.drivebaseSubsystem, () -> this.getDriveX(), () -> this.getDriveY())
                .withName("Drivebase rotation towards the hub"));
  }

  private void configureAutoAlignBindings() {
    driverController.rightBumper().whileTrue(FuelAutoAlign.autoAlign(this, s));
  }

  private void configureIntakeBindings() {
    if (s.intakeSubsystem == null) {
      System.out.println("line 191 Controls.java, intakeSubsystem is disabled");
    }
    driverController.b().whileTrue(s.intakeSubsystem.runIntake(1));
    intakeTestController.a().whileTrue(s.intakeSubsystem.temporaryRunIntake(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }

  public void vibrateDriveController(double vibration) {
    if (!DriverStation.isAutonomous()) {
      driverController.getHID().setRumble(RumbleType.kBothRumble, vibration);
    }
  }

  public Command rumbleDriveController(double vibration, double seconds) {
    return Commands.startEnd(
            () -> vibrateDriveController(vibration), // start
            () -> vibrateDriveController(0.0) // end
            )
        .withTimeout(seconds);
  }
}
