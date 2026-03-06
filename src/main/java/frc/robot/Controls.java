package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.sensors.LEDSubsystem;
import frc.robot.sensors.LEDSubsystem.LEDMode;
import frc.robot.subsystems.auto.FuelAutoAlign;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.subsystems.launcher.TurretSubsystem;
import frc.robot.util.AllianceUtils;
import frc.robot.util.HubShiftUtil;
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

  // Controller Ports
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int INDEXING_TEST_CONTROLLER_PORT = 1;
  private static final int LAUNCHER_TUNING_CONTROLLER_PORT = 2;
  private static final int TURRET_TEST_CONTROLLER_PORT = 3;
  private static final int INTAKE_TEST_CONTROLLER_PORT = 4;
  private static final int VISION_TEST_CONTROLLER_PORT = 5;

  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private final CommandXboxController indexingTestController =
      new CommandXboxController(INDEXING_TEST_CONTROLLER_PORT);

  private final CommandXboxController launcherTuningController =
      new CommandXboxController(LAUNCHER_TUNING_CONTROLLER_PORT);

  private final CommandXboxController intakeTestController =
      new CommandXboxController(INTAKE_TEST_CONTROLLER_PORT);

  private final CommandXboxController turretTestController =
      new CommandXboxController(TURRET_TEST_CONTROLLER_PORT);

  private final CommandXboxController visionTestController =
      new CommandXboxController(VISION_TEST_CONTROLLER_PORT);

  AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  Transform2d robotOffsetFromTag =
      new Transform2d(
          new Translation2d(Units.inchesToMeters(30 / 2), 0), Rotation2d.fromDegrees(180));
  Pose2d redHub = aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(robotOffsetFromTag);
  Pose2d blueHub = aprilTagFieldLayout.getTagPose(26).get().toPose2d().plus(robotOffsetFromTag);

  private LEDMode ledsMode = LEDMode.DEFAULT;
  private IntakeMode intakeMode = IntakeMode.RETRACTED;

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
  public Controls(Subsystems subsystems) {
    // Configure the trigger bindings
    s = subsystems;
    configureDrivebaseBindings();
    configureLauncherBindings();
    configureIndexingBindings();
    configureIntakeBindings();
    configureAutoAlignBindings();
    configureVisionBindings();
    configureTurretBindings();
    configureLedBindings();
  }

  private Trigger connected(CommandXboxController controller) {
    return new Trigger(() -> controller.isConnected());
  }

  public Command setRumble(RumbleType type, double value) {
    return Commands.runOnce(
        () -> {
          driverController.setRumble(type, value);
        });
  }

  private void configureIndexingBindings() {
    if (s.feederSubsystem == null || s.spindexerSubsystem == null) {
      DataLogManager.log("Feeder and/or Spindexer subsystem is disabled, indexer bindings skipped");
      return;
    }
    // TODO: wait for sensor to reach threshold, and trigger rumble

    // run feeder motor
    connected(indexingTestController)
        .and(indexingTestController.a())
        .whileTrue(s.feederSubsystem.startMotor());

    // run spindexer motor
    connected(indexingTestController)
        .and(indexingTestController.x())
        .whileTrue(s.spindexerSubsystem.startMotor());

    // run both while left trigger is held
    connected(indexingTestController)
        .and(indexingTestController.leftTrigger())
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

    // reset pose incase vision is bugging
    driverController
        .rightBumper()
        .onTrue(
            s.drivebaseSubsystem.runOnce(
                () -> s.drivebaseSubsystem.resetPose(AllianceUtils.isRed() ? redHub : blueHub)));
  }

  private void configureAutoAlignBindings() {
    if (s.detectionSubsystem == null) {
      DataLogManager.log("Game piece detection is disabled");
      return;
    }
    connected(visionTestController)
        .and(visionTestController.rightBumper())
        .whileTrue(FuelAutoAlign.autoAlign(this, s));
  }

  private void configureLauncherBindings() {
    if (s.flywheels == null || s.hood == null || s.ledSubsystem == null) {
      // Stop running this method
      DataLogManager.log("Flywheels and/or Hood and/or LEDs are disabled");
      return;
    }

    driverController
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                s.launcherSubsystem.launcherAimCommand(s.drivebaseSubsystem),
                Commands.runOnce(() -> ledsMode = LEDMode.LAUNCHING),
                Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                    .andThen(
                        Commands.parallel(
                            s.indexerSubsystem.runIndexer(),
                            Commands.runOnce(() -> ledsMode = LEDMode.LAUNCH),
                            Commands.waitSeconds(1)
                                .andThen(
                                    Commands.runOnce(
                                        () ->
                                            intakeMode =
                                                driverController.leftTrigger().getAsBoolean()
                                                    ? IntakeMode.INTAKE
                                                    : IntakeMode.LAUNCH))))))
        .onFalse(
            s.launcherSubsystem
                .rawStowCommand()
                .alongWith(
                    Commands.runOnce(
                        () -> {
                          ledsMode = LEDMode.DEFAULT;
                          intakeMode =
                              driverController.leftTrigger().getAsBoolean()
                                  ? IntakeMode.INTAKE
                                  : IntakeMode.DEPLOYED;
                        })));
    driverController
        .start()
        .onTrue(
            Commands.parallel(
                    s.launcherSubsystem.zeroSubsystemCommand(),
                    s.intakePivot.zeroPivot(),
                    s.turretSubsystem.zeroTurret(),
                    s.ledSubsystem.flashCommand(LEDSubsystem.LAUNCH_COLOR, 3, 0.2))
                .ignoringDisable(true));

    if (s.flywheels.TUNER_CONTROLLED) {
      connected(launcherTuningController)
          .and(launcherTuningController.leftBumper())
          .onTrue(s.flywheels.suppliedSetVelocityCommand(() -> s.flywheels.targetVelocity.get()));
      launcherTuningController.a().whileTrue(Commands.parallel(s.indexerSubsystem.runIndexer()));
    }
    if (s.hood.TUNER_CONTROLLED) {
      connected(launcherTuningController)
          .and(launcherTuningController.rightBumper())
          .onTrue(s.hood.suppliedHoodPositionCommand(() -> s.hood.targetPosition.get()));
    }

    connected(launcherTuningController)
        .and(launcherTuningController.start())
        .onTrue(s.hood.autoZeroCommand());
    connected(launcherTuningController)
        .and(launcherTuningController.a())
        .onTrue(s.hood.hoodPositionCommand(0.5));
    connected(launcherTuningController)
        .and(launcherTuningController.b())
        .onTrue(s.hood.hoodPositionCommand(1));

    connected(launcherTuningController)
        .and(launcherTuningController.x())
        .onTrue(s.flywheels.setVelocityCommand(50));
    connected(launcherTuningController)
        .and(launcherTuningController.y())
        .onTrue(s.flywheels.setVelocityCommand(60));
  }

  private void configureIntakeBindings() {
    if (s.intakeRollers == null || s.intakePivot == null || s.ledSubsystem == null) {
      DataLogManager.log(
          "Controls.java: intakeRollers or intakeArm or LEDs is disabled, bindings skipped");
      return;
    }

    s.intakeSubsystem.setDefaultCommand(
        Commands.run(
                () -> {
                  switch (intakeMode) {
                    case DEPLOYED -> s.intakeSubsystem.deployPivotVoid();
                    case RETRACTED -> s.intakeSubsystem.retractPivotVoid();
                    case SPIN -> s.intakeSubsystem.runRollersVoid();
                    case LAUNCH -> s.intakeSubsystem.intakeWhileLaunchVoid();
                    case INTAKE -> s.intakeSubsystem.smartIntakeVoid();
                  }
                },
                s.intakeSubsystem)
            .withName("Intake Default Command"));

    driverController
        .leftTrigger()
        .whileTrue(
            Commands.runOnce(
                () -> {
                  intakeMode = IntakeMode.INTAKE;
                  ledsMode = LEDMode.INTAKE;
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  intakeMode =
                      driverController.rightTrigger().getAsBoolean()
                          ? IntakeMode.LAUNCH
                          : IntakeMode.DEPLOYED;
                  ledsMode = LEDMode.DEFAULT;
                }));
    driverController.povUp().onTrue(Commands.runOnce(() -> intakeMode = IntakeMode.DEPLOYED));
    driverController.povDown().onTrue(Commands.runOnce(() -> intakeMode = IntakeMode.RETRACTED));

    connected(intakeTestController)
        .and(intakeTestController.a())
        .whileTrue(s.intakeRollers.runRollers(IntakeRollers.INTAKE_VOLTAGE));
    connected(intakeTestController)
        .and(intakeTestController.x())
        .onTrue(s.intakePivot.setPivotPosition(IntakePivot.DEPLOYED_POS));
    connected(intakeTestController)
        .and(intakeTestController.y())
        .onTrue(s.intakePivot.setPivotPosition(IntakePivot.RETRACTED_POS));
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
      connected(visionTestController)
          .and(visionTestController.leftBumper())
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

  private void configureTurretBindings() {
    if (s.turretSubsystem == null) {
      return;
    }

    s.turretSubsystem.setDefaultCommand(s.turretSubsystem.rotateToTarget());
    // use static position constants from TurretSubsystem
    connected(turretTestController)
        .and(turretTestController.povUp())
        .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.FRONT_POSITION));
    connected(turretTestController)
        .and(turretTestController.povLeft())
        .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.LEFT_POSITION));
    connected(turretTestController)
        .and(turretTestController.povRight())
        .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.RIGHT_POSITION));
    connected(turretTestController)
        .and(turretTestController.povDown())
        .onTrue(s.turretSubsystem.setTurretPosition(TurretSubsystem.BACK_POSITION));
    connected(turretTestController)
        .and(turretTestController.y())
        .onTrue(s.turretSubsystem.zeroTurret());
    connected(turretTestController)
        .and(turretTestController.rightStick())
        .whileTrue(
            s.turretSubsystem.manualMovingVoltage(
                () ->
                    Volts.of(
                        TurretSubsystem.TURRET_MANUAL_SPEED * turretTestController.getRightY())));
    connected(turretTestController)
        .and(turretTestController.leftStick())
        .whileTrue(
            s.turretSubsystem.pointFacingJoystick(
                () -> turretTestController.getLeftX(), () -> turretTestController.getLeftY()));
    connected(turretTestController)
        .and(turretTestController.rightTrigger())
        .whileTrue(s.turretSubsystem.rotateToTarget());
    connected(turretTestController)
        .and(turretTestController.rightBumper())
        .onTrue(
            s.drivebaseSubsystem.runOnce(
                () -> s.drivebaseSubsystem.resetPose(AllianceUtils.isRed() ? redHub : blueHub)));
    driverController
        .rightStick()
        .whileTrue(
            s.turretSubsystem.pointFacingJoystick(
                () -> driverController.getRightX(), () -> driverController.getRightY()));
  }

  public void configureLedBindings() {
    if (s.ledSubsystem == null) {
      return;
    }
    s.ledSubsystem.setDefaultCommand(
        Commands.run(() -> s.ledSubsystem.setMode(ledsMode), s.ledSubsystem)
            .withName("LED Default Command"));

    Trigger shiftWarning =
        new Trigger(
            () -> {
              if (RobotState.isAutonomous()) return false;

              var shiftInfo = HubShiftUtil.getOfficialShiftInfo();

              return shiftInfo.remainingTime() <= 4.0;
            });

    shiftWarning.onTrue(s.ledSubsystem.flashCommand(LEDSubsystem.DEFAULT_COLOR, 5, 0.1));
  }
}
