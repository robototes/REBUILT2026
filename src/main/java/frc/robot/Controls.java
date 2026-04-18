package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
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
import frc.robot.sim.SimWrapper;
import frc.robot.subsystems.auto.AutoDriveRotate;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;
import frc.robot.subsystems.launcher.TurretSubsystem;
import frc.robot.util.AllianceUtils;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.robotType.RobotTypesEnum;
import frc.robot.util.tuning.WheelRadiusCharacterization;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Controls {
  // The robot's subsystems and commands are defined here...
  private final Subsystems s;
  private final SimWrapper m_simWrapper;

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
  // Robot with bumpers is 36.875 inches by 30.750 inches
  Transform2d robotOffsetFromTag =
      new Transform2d(
          new Translation2d(Units.inchesToMeters(30.750 / 2), 0), Rotation2d.fromDegrees(180));
  Pose2d redHub = aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(robotOffsetFromTag);
  Pose2d blueHub = aprilTagFieldLayout.getTagPose(26).get().toPose2d().plus(robotOffsetFromTag);

  private LEDMode ledsMode = LEDMode.DEFAULT;
  public static IntakeMode intakeMode = IntakeMode.RETRACTED;

  public static boolean turretKillActive = false;

  public static final double MaxSpeed =
      (RobotType.TYPE == RobotTypesEnum.ALPHA)
          ? AlphaTunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
          : CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // kSpeedAt12Volts desired top speed
  public static double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private static final double DRIVE_INPUT_SCALE = 1;
  private static final double JOYSTICK_DEADBAND = 0.1;
  private static final double SWERVE_DEADBAND = 0.001;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SWERVE_DEADBAND)
          .withRotationalDeadband(SWERVE_DEADBAND)
          .withDriveRequestType(DriveRequestType.Velocity);

  private Trigger readyToShoot = new Trigger(() -> false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Controls(Subsystems subsystems, SimWrapper simWrapper) {
    // Configure the trigger bindings
    s = subsystems;
    m_simWrapper = simWrapper;
    configureDrivebaseBindings();
    configureLauncherBindings();
    configureIndexingBindings();
    configureIntakeBindings();
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
            })
        .withName("Set Rumble");
  }

  private void configureIndexingBindings() {
    if (s.feederSubsystem == null || s.spindexerSubsystem == null) {
      DataLogManager.log("Feeder and/or Spindexer subsystem is disabled, indexer bindings skipped");
      return;
    }
    connected(indexingTestController)
        .and(indexingTestController.leftTrigger())
        .whileTrue(s.indexerSubsystem.runIndexer());
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
    double input = MathUtil.applyDeadband(-driverController.getLeftY(), JOYSTICK_DEADBAND);
    return input * MaxSpeed * DRIVE_INPUT_SCALE;
  }

  // takes the Y value from the joystick, and applies a deadband and input scaling
  private double getDriveY() {
    // Joystick +X is right
    // Robot +Y is left
    double input = MathUtil.applyDeadband(-driverController.getLeftX(), JOYSTICK_DEADBAND);
    return input * MaxSpeed * DRIVE_INPUT_SCALE;
  }

  // takes the rotation value from the joystick, and applies a deadband and input scaling
  private double getDriveRotate() {
    // Joystick +X is right
    // Robot +angle is CCW (left)
    double input = MathUtil.applyDeadband(-driverController.getRightX(), JOYSTICK_DEADBAND);
    return input * MaxSpeed * DRIVE_INPUT_SCALE;
  }

  private void configureDrivebaseBindings() {
    if (s.drivebaseSubsystem == null) {
      // Stop running this method
      return;
    }

    // readyToShoot = GetTargetFromPose.autoShoot(s.drivebaseSubsystem);

    connected(launcherTuningController)
        .and(launcherTuningController.y())
        .whileTrue(
            WheelRadiusCharacterization.wheelRadiusCharacterizationCommand(s.drivebaseSubsystem));
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
    // driverController
    //     .back()
    //     .onTrue(
    //         s.drivebaseSubsystem
    //             .runOnce(() -> s.drivebaseSubsystem.seedFieldCentric())
    //             .alongWith(rumble(driverController, 0.5, Seconds.of(0.3)))
    //             .withName("Reset gyro"));

    driverController
        .a()
        .whileTrue(Commands.run(() -> s.drivebaseSubsystem.setControl(new SwerveDriveBrake())));

    // $VISIONSIM - Bumper buttons
    if (Robot.isSimulation()) {
      // In simulation, inject drift with POV-right to test vision correction
      driverController
          .povRight()
          .onTrue(
              s.drivebaseSubsystem
                  .runOnce(() -> m_simWrapper.injectDrift(0.5, 15.0))
                  .withName("Inject Drift"));

      // POV-left resets robot to the starting pose of the selected auto
      driverController
          .povLeft()
          .onTrue(
              s.drivebaseSubsystem
                  .runOnce(() -> m_simWrapper.cycleResetPosition(Pose2d.kZero))
                  .withName("Reset to Start Pose"));
    }

    // reset pose incase vision is bugging
    driverController
        .back()
        .onTrue(
            s.drivebaseSubsystem
                .runOnce(
                    () -> s.drivebaseSubsystem.resetPose(AllianceUtils.isRed() ? redHub : blueHub))
                .withName("Reset to Hub"));
  }

  private void configureLauncherBindings() {
    if (s.flywheels == null || s.hood == null || s.ledSubsystem == null) {
      // Stop running this method
      DataLogManager.log("Flywheels and/or Hood and/or LEDs are disabled");
      return;
    }

    driverController
        .rightTrigger()
        .or(readyToShoot)
        .and(driverController.rightBumper().negate())
        .whileTrue(
            Commands.parallel(
                    Commands.either(
                        AutoDriveRotate.autoRotate(
                            s.drivebaseSubsystem,
                            () -> getDriveX(),
                            () -> getDriveY(),
                            () -> Units.rotationsToDegrees(s.turretSubsystem.getTurretPosition())),
                        s.drivebaseSubsystem.applyRequest(
                            () ->
                                drive
                                    .withVelocityX(getDriveX())
                                    .withVelocityY(getDriveY())
                                    .withRotationalRate(getDriveRotate())),
                        () -> turretKillActive),
                    s.launcherSubsystem.launcherAimCommand(),
                    Commands.runOnce(() -> ledsMode = LEDMode.LAUNCHING),
                    Commands.waitUntil(() -> s.launcherSubsystem.isAtTarget())
                        .andThen(
                            Commands.parallel(
                                    s.indexerSubsystem.runIndexer(
                                        () -> s.flywheels.getTargetSpeed()),
                                    Commands.runOnce(() -> ledsMode = LEDMode.LAUNCH),
                                    Commands.waitSeconds(1)
                                        .andThen(Commands.runOnce(() -> updateIntakeMode())))
                                .onlyWhile(() -> s.launcherSubsystem.isAtTarget())
                                .andThen(
                                    Commands.runOnce(
                                        () -> {
                                          updateIntakeMode();
                                          ledsMode = LEDMode.LAUNCHING;
                                        })))
                        .repeatedly())
                .withName("Launching Command"))
        .onFalse(
            s.launcherSubsystem
                .rawStowCommand()
                .alongWith(
                    Commands.runOnce(
                        () -> {
                          updateIntakeMode();
                        }))
                .withName("Launching Finished"));

    driverController
        .start()
        .onTrue(
            Commands.parallel(
                    Commands.either(
                        Commands.parallel(
                            s.hood.autoZeroCommand(), s.intakePivot.autoZeroCommand()),
                        Commands.parallel(
                            s.launcherSubsystem.zeroSubsystemCommand(),
                            s.intakePivot.zeroPivot(),
                            s.turretSubsystem.zeroTurret()),
                        () -> DriverStation.isEnabled()),
                    s.ledSubsystem.flashCommand(LEDSubsystem.LAUNCH_COLOR, 3, 0.2))
                .ignoringDisable(true)
                .withName("Zero Subsystems"));

    driverController
        .x()
        .onTrue(
            Commands.runOnce(() -> HubShiftUtil.setAllianceWinOverride(() -> Optional.of(false)))
                .withName("Enable Alliance Win Override")
                .ignoringDisable(true));

    driverController
        .b()
        .onTrue(
            Commands.runOnce(() -> HubShiftUtil.setAllianceWinOverride(() -> Optional.of(true)))
                .withName("Disable Alliance Win Override")
                .ignoringDisable(true));

    connected(launcherTuningController)
        .and(launcherTuningController.start())
        .onTrue(s.hood.autoZeroCommand());
    connected(launcherTuningController)
        .and(launcherTuningController.x())
        .onTrue(s.flywheels.setVelocityCommand(50));
    connected(launcherTuningController)
        .and(launcherTuningController.y())
        .onTrue(s.flywheels.setVelocityCommand(60));
  }

  private void updateIntakeMode() {
    if (driverController.leftTrigger().getAsBoolean()) {
      intakeMode = IntakeMode.INTAKE;
      ledsMode = LEDMode.INTAKE;
    } else if ((driverController.rightTrigger().getAsBoolean() || readyToShoot.getAsBoolean())
        && s.launcherSubsystem.isAtTarget()) {
      intakeMode = IntakeMode.LAUNCH;
      s.intakePivot.restartTimer();
    } else {
      intakeMode = IntakeMode.DEPLOYED;
      ledsMode = LEDMode.DEFAULT;
    }
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
                    case DEPLOYED -> s.intakeSubsystem.deployPivot();
                    case RETRACTED -> s.intakeSubsystem.retractPivot();
                    case SPIN -> s.intakeSubsystem.runRollers();
                    case LAUNCH -> s.intakeSubsystem.intakeWhileLaunch();
                    case INTAKE ->
                        s.intakeSubsystem.smartIntake(() -> s.drivebaseSubsystem.getState().Speeds);
                    case EXTAKE -> s.intakeSubsystem.extakeIntake();
                  }
                },
                s.intakeSubsystem)
            .withName("Intake Default Command"));
    driverController
        .leftTrigger()
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      updateIntakeMode();
                    })
                .withName("Intaking"))
        .onFalse(Commands.runOnce(() -> updateIntakeMode()).withName("Intaking Finished"));
    driverController
        .povUp()
        .onTrue(Commands.runOnce(() -> intakeMode = IntakeMode.DEPLOYED).withName("Deploy Intake"));
    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(() -> intakeMode = IntakeMode.RETRACTED).withName("Retract Intake"));
    driverController
        .leftBumper()
        .whileTrue(Commands.runOnce(() -> intakeMode = IntakeMode.EXTAKE).withName("Extaking"))
        .onFalse(
            Commands.runOnce(() -> intakeMode = IntakeMode.DEPLOYED).withName("Extaking Finished"));

    connected(intakeTestController)
        .and(intakeTestController.a())
        .whileTrue(Commands.run(() -> s.intakeSubsystem.runRollers()));
    connected(intakeTestController)
        .and(intakeTestController.x())
        .onTrue(Commands.runOnce(() -> intakeMode = IntakeMode.DEPLOYED));
    connected(intakeTestController)
        .and(intakeTestController.y())
        .onTrue(Commands.runOnce(() -> intakeMode = IntakeMode.RETRACTED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none().withName("Empty Autonomous Command");
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
        .withTimeout(seconds)
        .withName("Rumble Drive Controller");
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

    s.turretSubsystem.setDefaultCommand(
        s.turretSubsystem.rotateToTargetWithCalc().withName("Turret Default Command"));
    new Trigger(() -> s.turretSubsystem.atLimitSwitch())
        .onTrue(
            Commands.runOnce(() -> s.turretSubsystem.zeroTurret())
                .withName("Zero Turret on Limit Switch"));

    driverController
        .y()
        .toggleOnTrue(
            s.turretSubsystem
                .run(
                    () ->
                        s.turretSubsystem.setTurretRawPosition(
                            s.turretSubsystem.getTurretPosition()))
                .withName("Turret Kill — Hold Position"))
        .onTrue(
            Commands.runOnce(() -> turretKillActive = !turretKillActive)
                .withName("Toggle Turret Kill"));
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
        .whileTrue(s.turretSubsystem.rotateToTargetWithCalc());
    connected(turretTestController)
        .and(turretTestController.rightBumper())
        .onTrue(
            s.drivebaseSubsystem
                .runOnce(
                    () -> s.drivebaseSubsystem.resetPose(AllianceUtils.isRed() ? redHub : blueHub))
                .withName("Reset to Hub"));
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
        Commands.run(
                () -> {
                  if (RobotState.isTeleop()) {
                    s.ledSubsystem.setMode(ledsMode);
                  }
                },
                s.ledSubsystem)
            .withName("LED Default Command"));

    readyToShoot.onFalse(s.ledSubsystem.flashCommand(LEDSubsystem.CLIMB_COLOR, 30, 0.1));
  }
}
