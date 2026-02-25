package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Controllers.ControllerConstants.INTAKE_TEST_CONTROLLER_ENABLED;
import static frc.robot.Controllers.ControllerConstants.LAUNCHER_TUNING_CONTROLLER_ENABLED;
import static frc.robot.Controllers.ControllerConstants.VISION_TEST_CONTROLLER_ENABLED;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.auto.FuelAutoAlign;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.launcher.TurretSubsystem;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.robotType.RobotTypesEnum;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Controls {
  // The robot's subsystems and commands are defined here...
  private final Subsystems s;
  private final Controllers c;

  // Controller Ports
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int INDEXING_TEST_CONTROLLER_PORT = 1;
  public static final int LAUNCHER_TUNING_CONTROLLER_PORT = 2;
  public static final int TURRET_TEST_CONTROLLER_PORT = 3;
  public static final int INTAKE_TEST_CONTROLLER_PORT = 4;
  public static final int VISION_TEST_CONTROLLER_PORT = 5;

  // Check Controllers.java for this stuff

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  // private final CommandXboxController indexingTestController =
  //     new CommandXboxController(INDEXING_TEST_CONTROLLER_PORT);

  // private final CommandXboxController launcherTuningController =
  //     new CommandXboxController(LAUNCHER_TUNING_CONTROLLER_PORT);

  // private final CommandXboxController intakeTestController =
  //     new CommandXboxController(INTAKE_TEST_CONTROLLER_PORT);

  // private final CommandXboxController turretTestController =
  //     new CommandXboxController(TURRET_TEST_CONTROLLER_PORT);

  // private final CommandXboxController visionTestController =
  //     new CommandXboxController(VISION_TEST_CONTROLLER_PORT);

  public static final double MaxSpeed =
      (RobotType.type == RobotTypesEnum.ALPHA)
          ? AlphaTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
          : CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
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
  public Controls(Subsystems subsystems, Controllers controllers) {
    // Configure the trigger bindings
    s = subsystems;
    c = controllers;
    configureDrivebaseBindings();
    configureLauncherBindings();
    configureIndexingBindings();
    configureIntakeBindings();
    configureAutoAlignBindings();
    configureVisionBindings();
    configureTurretBindings();

    DriverStation.silenceJoystickConnectionWarning(true); // this doesn't work during competitions
  }

  public Command setRumble(RumbleType type, double value) {
    return Commands.runOnce(
        () -> {
          c.driverControllerTest.setRumble(type, value);
        });
  }

  private Trigger connected(CommandXboxController controller) {
    return new Trigger(() -> controller.isConnected());
  }

  private void configureIndexingBindings() {
    if (s.feederSubsystem == null || s.spindexerSubsystem == null) {
      DataLogManager.log("Feeder and/or Spindexer subsystem is disabled, indexer bindings skipped");
      return;
    }
    // TODO: wait for sensor to reach threshold, and trigger rumble

    // run feeder motor
    indexingTestController.a().whileTrue(s.feederSubsystem.startMotor());

    // run spindexer motor
    indexingTestController.x().whileTrue(s.spindexerSubsystem.startMotor());

    // run both while left trigger is held
    c.indexingTestController.leftTrigger()
        .whileTrue(
            Commands.parallel(s.feederSubsystem.startMotor(), s.spindexerSubsystem.startMotor()));
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
    double input = MathUtil.applyDeadband(-c.driverControllerTest.getLeftY(), 0.1);
    return input * MaxSpeed * driveInputScale;
  }

  // takes the Y value from the joystick, and applies a deadband and input scaling
  private double getDriveY() {
    // Joystick +X is right
    // Robot +Y is left
    double input = MathUtil.applyDeadband(-c.driverControllerTest.getLeftX(), 0.1);
    return input * MaxSpeed * driveInputScale;
  }

  // takes the rotation value from the joystick, and applies a deadband and input scaling
  private double getDriveRotate() {
    // Joystick +X is right
    // Robot +angle is CCW (left)
    double input = MathUtil.applyDeadband(-c.driverControllerTest.getRightX(), 0.1);
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
    c.driverControllerTest
        .back()
        .onTrue(
            s.drivebaseSubsystem
                .runOnce(() -> s.drivebaseSubsystem.seedFieldCentric())
                .alongWith(rumble(c.driverControllerTest, 0.5, Seconds.of(0.3)))
                .withName("Reset gyro"));

    // logging the telemetry
    s.drivebaseSubsystem.registerTelemetry(logger::telemeterize);
  }

  private void configureAutoAlignBindings() {
    if (s.detectionSubsystem == null) {
      DataLogManager.log("Game piece detection is disabled");
      return;
    }
    c.visionTestController.rightBumper().whileTrue(FuelAutoAlign.autoAlign(this, s));
  }

  private void configureLauncherBindings() {
    if (s.flywheels == null || s.hood == null) {
      // Stop running this method
      DataLogManager.log("Flywheels and/or Hood are disabled");
      return;
    }

    c.driverControllerTest
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                    s.launcherSubsystem.launcherAimCommand(s.drivebaseSubsystem),
                    Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                        .andThen(s.indexerSubsystem.runIndexer()))
                .withName("Aim turret then feeder and spindexer started"));
    c.driverControllerTest
        .y()
        .onTrue(s.launcherSubsystem.zeroSubsystemCommand().ignoringDisable(true));

    if (s.flywheels.TUNER_CONTROLLED) {
      c.launcherTestController
          .leftBumper()
          .onTrue(s.flywheels.suppliedSetVelocityCommand(() -> s.flywheels.targetVelocity.get()));
    }
    if (s.hood.TUNER_CONTROLLED) {
      c.launcherTestController
          .rightBumper()
          .onTrue(s.hood.suppliedHoodPositionCommand(() -> s.hood.targetPosition.get()));
    }
    if (!LAUNCHER_TUNING_CONTROLLER_ENABLED) {
      DataLogManager.log("Controllers.java: launcher test controller doesn't exist");
      return;
    } else {
    c.launcherTestController.start().onTrue(s.hood.autoZeroCommand());
    c.launcherTestController.a().onTrue(s.hood.hoodPositionCommand(0.5));
    c.launcherTestController.b().onTrue(s.hood.hoodPositionCommand(1));

    c.launcherTestController.x().onTrue(s.flywheels.setVelocityCommand(50));
    c.launcherTestController.y().onTrue(s.flywheels.setVelocityCommand(60));
    }
  }

  private void configureIntakeBindings() {
    if (s.intakeRollers == null || s.intakePivot == null) {
      DataLogManager.log("Controls.java: intakeRollers or intakeArm is disabled, bindings skipped");
      return;
    }

    s.intakePivot.setDefaultCommand(s.intakePivot.setPivotPosition(IntakePivot.DEPLOYED_POS));

    c.driverControllerTest.leftTrigger().whileTrue(s.intakeSubsystem.smartIntake());
    c.driverControllerTest.povUp().onTrue(s.intakeSubsystem.deployPivot());
    c.driverControllerTest.povDown().onTrue(s.intakeSubsystem.retractPivot());

    if (!INTAKE_TEST_CONTROLLER_ENABLED) {
      DataLogManager.log("Controllers.java: intake test controller doesn't exist");
      return;
    } else {
    c.intakeController.a().whileTrue(s.intakeRollers.runRollers());
    c.intakeController.x().onTrue(s.intakePivot.setPivotPosition(IntakePivot.DEPLOYED_POS));
    c.intakeController.y().onTrue(s.intakePivot.setPivotPosition(IntakePivot.RETRACTED_POS));
    }
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
      c.driverControllerTest.getHID().setRumble(RumbleType.kBothRumble, vibration);
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
    if (!VISION_TEST_CONTROLLER_ENABLED) {
      DataLogManager.log("Controllers.java: vision test controller is disabled");
    } else {
    if (s.visionSubsystem != null && s.drivebaseSubsystem != null) {
      c.visionTestController
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
  }

  private void configureTurretBindings() {
    if (s.turretSubsystem == null) {
      DataLogManager.log("Turret Subsystem is disabled, skipping bindings");
      return;
    }
    // use static position constants from TurretSubsystem
    c.turretTestController
        .povUp()
        .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.FRONT_POSITION));
    c.turretTestController
        .povLeft()
        .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.LEFT_POSITION));
    c.turretTestController
        .povRight()
        .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.RIGHT_POSITION));
    c.turretTestController
        .povDown()
        .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.BACK_POSITION));
    c.turretTestController.y().onTrue(s.turretSubsystem.zeroTurret());
    c.turretTestController
        .rightStick()
        .whileTrue(
            s.turretSubsystem.manualMovingVoltage(
                () ->
                    Volts.of(
                        TurretSubsystem.TURRET_MANUAL_SPEED * c.turretTestController.getRightY())));
    c.turretTestController
        .leftStick()
        .whileTrue(
            s.turretSubsystem.pointFacingJoystick(
                () -> c.turretTestController.getLeftX(), () -> c.turretTestController.getLeftY()));

    c.turretTestController.rightTrigger().whileTrue(s.turretSubsystem.rotateToHub());
    c.turretTestController.rightBumper()
        .onTrue(
            s.drivebaseSubsystem.runOnce(
                () -> s.drivebaseSubsystem.resetPose(new Pose2d(13, 4, Rotation2d.kZero))));
    c.driverControllerTest
        .rightTrigger()
        .whileTrue(
            s.turretSubsystem.pointFacingJoystick(
                () -> c.driverControllerTest.getLeftX(), () -> c.driverControllerTest.getLeftY()));
  }
}
