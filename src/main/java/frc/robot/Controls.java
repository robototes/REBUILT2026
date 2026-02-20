package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.auto.AutoAim;
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
  // private static final int FEEDER_TEST_CONTROLLER_PORT = 1;
  // private static final int SPINDEXER_TEST_CONTROLLER_PORT = 2;
  private static final int INDEXING_TEST_CONTROLLER_PORT = 3; // for now
  private static final int LAUNCHER_TUNING_CONTROLLER_PORT = 4; // for now
  private static final int LED_CONTROLLER_PORT = 5; // keep this one at 5 plz
  // private static final int TURRET_TEST_CONTROLLER_PORT = 3; // not sure what to do about this
  // because Driver Station allows only 6 controllers.

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private final CommandXboxController ledTestController =
      new CommandXboxController(LED_CONTROLLER_PORT);

  // private final CommandXboxController feederTestController =
  // new CommandXboxController(FEEDER_TEST_CONTROLLER_PORT);

  // private final CommandXboxController spindexerTestController =
  // new CommandXboxController(SPINDEXER_TEST_CONTROLLER_PORT);

  private final CommandXboxController indexingTestController =
      new CommandXboxController(INDEXING_TEST_CONTROLLER_PORT);

  private final CommandXboxController launcherTuningController =
      new CommandXboxController(LAUNCHER_TUNING_CONTROLLER_PORT);

  // private final CommandXboxController turretTestController =
  // new CommandXboxController(TURRET_TEST_CONTROLLER_PORT);

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
    configureLEDBindings();
    configureLauncherBindings();
    configureIndexingBindings();
    configureAutoAlignBindings();
    configureVisionBindings();
    // configureTurretBindings();
  }

  public Command setRumble(RumbleType type, double value) {
    return Commands.runOnce(
        () -> {
          driverController.setRumble(type, value);
        });
  }

  private void configureIndexingBindings() {
    // TODO: wait for sensor to reach threshold, and trigger rumble

    // start feeder motor
    indexingTestController.a().onTrue(s.feederSubsystem.startMotor());

    // stop feeder motor
    indexingTestController.b().onTrue(s.feederSubsystem.stopMotor());

    // start spindexer motor
    indexingTestController.x().onTrue(s.spindexerSubsystem.startMotor());

    // stop spindexer motor
    indexingTestController.y().onTrue(s.spindexerSubsystem.stopMotor());

    // run both while left trigger is held
    indexingTestController
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  s.feederSubsystem.startMotor();
                  s.spindexerSubsystem.startMotor();
                },
                () -> {
                  s.feederSubsystem.stopMotor();
                  s.spindexerSubsystem.stopMotor();
                },
                s.feederSubsystem,
                s.spindexerSubsystem));
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
  }

  private void configureAutoAlignBindings() {
    if (s.detectionSubsystem == null) {
      DataLogManager.log("Game piece detection is disabled");
      return;
    }
    driverController.rightBumper().whileTrue(FuelAutoAlign.autoAlign(this, s));
  }

  private void configureLEDBindings() {
    LEDSubsystem LEDs = s.ledSubsystem;
    final RGBWColor[] colors =
        new RGBWColor[] {
          LEDSubsystem.DEFAULT_COLOR,
          LEDSubsystem.INTAKE_COLOR,
          LEDSubsystem.OUTTAKE_COLOR,
          LEDSubsystem.CLIMB_COLOR,
          LEDSubsystem.OFF_COLOR
        };
    if (s.ledSubsystem == null) {
      // Stop running this method
      return;
    }
    ledTestController
        .a()
        .onTrue(
            LEDs.setLEDsCommand(LEDSubsystem.OFF_COLOR, LEDSubsystem.DEFAULT_BRIGHTNESS)
                .andThen(
                    LEDs.setLEDsCommand(LEDSubsystem.INTAKE_COLOR, LEDSubsystem.DEFAULT_BRIGHTNESS))
                .withName("Set intake color")); // blue

    ledTestController
        .b()
        .onTrue(
            LEDs.setLEDsCommand(LEDSubsystem.OFF_COLOR, LEDSubsystem.DEFAULT_BRIGHTNESS)
                .andThen(
                    LEDs.setLEDsCommand(LEDSubsystem.CLIMB_COLOR, LEDSubsystem.DEFAULT_BRIGHTNESS))
                .withName("Set climb color")); // cyan

    ledTestController
        .x()
        .onTrue(
            LEDs.setLEDsCommand(LEDSubsystem.OFF_COLOR, LEDSubsystem.DEFAULT_BRIGHTNESS)
                .andThen(
                    LEDs.setLEDsCommand(
                        LEDSubsystem.OUTTAKE_COLOR, LEDSubsystem.DEFAULT_BRIGHTNESS))
                .withName("Set outtake color")); // green

    ledTestController
        .y()
        .onTrue(
            LEDs.setLEDsCommand(LEDSubsystem.OFF_COLOR, LEDSubsystem.DEFAULT_BRIGHTNESS)
                .andThen(
                    LEDs.setLEDsCommand(
                        LEDSubsystem.DEFAULT_COLOR, LEDSubsystem.DEFAULT_BRIGHTNESS))
                .withName("Set default color")); // red

    ledTestController
        .leftBumper()
        .whileTrue(
            LEDs.alternateColors(LEDSubsystem.CLIMB_COLOR, LEDSubsystem.OUTTAKE_COLOR, 0.5)
                .withName("Alternate climb and outtake colors with 0.5s delay"));

    ledTestController
        .rightBumper()
        .whileTrue(
            LEDs.cycleColors(colors, 0.5).withName("Cycle through all colors with 0.5s delay"));
  }

  private void configureLauncherBindings() {
    if (s.flywheels == null || s.hood == null) {
      // Stop running this method
      DataLogManager.log("Flywheels and/or Hood are disabled");
      return;
    }

    driverController
        .rightTrigger()
        .whileTrue(
            Commands.sequence(
                    AutoAim.autoAim(s.drivebaseSubsystem, s.hood, s.flywheels),
                    Commands.parallel(
                        s.spindexerSubsystem.startMotor(), s.feederSubsystem.startMotor()))
                .withName("Autorotate, Autoaim done, feeder and spindexer started"))
        .toggleOnFalse(
            Commands.parallel(
                s.hood.hoodPositionCommand(0.0), s.flywheels.setVelocityCommand(0.0)));
    driverController
        .y()
        .onTrue(Commands.parallel(s.hood.zeroHoodCommand(), s.turretSubsystem.zeroTurret()));
    if (s.flywheels.TUNER_CONTROLLED) {
      launcherTuningController
          .leftBumper()
          .onTrue(s.flywheels.suppliedSetVelocityCommand(() -> s.flywheels.targetVelocity.get()));
    }
    if (s.hood.TUNER_CONTROLLED) {
      launcherTuningController
          .rightBumper()
          .onTrue(s.hood.suppliedHoodPositionCommand(() -> s.hood.targetPosition.get()));
    }
    launcherTuningController.start().onTrue(s.hood.autoZeroCommand());
    launcherTuningController.a().onTrue(s.hood.hoodPositionCommand(0.5));
    launcherTuningController.b().onTrue(s.hood.hoodPositionCommand(1));

    launcherTuningController.x().onTrue(s.flywheels.setVelocityCommand(50));
    launcherTuningController.y().onTrue(s.flywheels.setVelocityCommand(60));
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

  private void configureVisionBindings() {
    if (s.visionSubsystem != null && s.drivebaseSubsystem != null) {
      driverController
          .leftBumper()
          .onTrue(
              s.drivebaseSubsystem
                  .runOnce(
                      () -> {
                        Pose2d referenceVisionPose = s.visionSubsystem.getLastVisionPose2d();
                        if (referenceVisionPose != null) {
                          s.drivebaseSubsystem.resetPose(referenceVisionPose);
                        }
                      })
                  .withName("Now Drive Pose is Vision Pose"));
    }
  }

  // private void configureTurretBindings() {
  //   if (s.turretSubsystem == null) {
  //     return;
  //   }
  //   // use static position constants from TurretSubsystem
  //   turretTestController
  //       .povUp()
  //       .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.FRONT_POSITION));
  //   turretTestController
  //       .povLeft()
  //       .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.LEFT_POSITION));
  //   turretTestController
  //       .povRight()
  //       .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.RIGHT_POSITION));
  //   turretTestController
  //       .povDown()
  //       .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.BACK_POSITION));
  //   turretTestController.y().onTrue(s.turretSubsystem.zeroTurret());
  //   turretTestController
  //       .rightStick()
  //       .whileTrue(
  //           s.turretSubsystem.manualMovingVoltage(
  //               () ->
  //                   Volts.of(
  //                       TurretSubsystem.TURRET_MANUAL_SPEED *
  // turretTestController.getRightY())));
  //   turretTestController
  //       .leftStick()
  //       .whileTrue(
  //           s.turretSubsystem.pointFacingJoystick(
  //               () -> turretTestController.getLeftX(), () -> turretTestController.getLeftY()));
  //   turretTestController
  //       .leftTrigger()
  //       .whileTrue(Commands.run(() -> System.out.println("Stick Pressed")));
  //   turretTestController.rightTrigger().whileTrue(s.turretSubsystem.rotateToHub());
  //   turretTestController
  //       .rightBumper()
  //       .onTrue(
  //           s.drivebaseSubsystem.runOnce(
  //               () -> s.drivebaseSubsystem.resetPose(new Pose2d(13, 4, Rotation2d.kZero))));
  //   driverController.povUp().whileTrue(s.turretSubsystem.rotateToHub());
  //   driverController.povDown().onTrue(s.turretSubsystem.zeroTurret());
  // }
}
