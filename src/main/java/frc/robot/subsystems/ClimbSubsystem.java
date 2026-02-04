package frc.robot.subsystems;

/*
public class ClimbSubsystem extends SubsystemBase {
  // --- HARDWARE VARIABLES --- //
  private final TalonFX m_climb_motor;
  private final double GEAR_RATIO = 9;
  private final double ROBOT_LENGTH_METERS = 2; // Meters

  // IS IT CLOSE? TUNABLES
  private final double MIN_X = 0.4572; // Meters
  private final double MIN_Y = 0.1524; // Meters
  private final double MIN_ANGLE = 0.261799; // Radians

  // STATES
  public enum ClimbState {
    notClimbing,
    Climbing
  }

  // Was told to make an "isClimbing" variable, so i made this. Static makes the most sense right?
  private static ClimbState currentState = ClimbState.notClimbing;

  // POSES
  private final CommandSwerveDrivetrain driveTrain;
  private final Pose2d frontBumper;

  private final Pose2d TowerPose =
      (AllianceUtils.isBlue())
          ? AllianceUtils.FIELD_LAYOUT
              .getTagPose(31)
              .get()
              .toPose2d()
              .transformBy(new Transform2d(0, -0.6985, Rotation2d.kZero))
          : AllianceUtils.FIELD_LAYOUT
              .getTagPose(15)
              .get()
              .toPose2d()
              .transformBy(new Transform2d(0, -0.6985, Rotation2d.kZero));

  public ClimbSubsystem(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
    m_climb_motor = new TalonFX(Hardware.CLIMB_MOTOR_ID);

    configureMotors();
  }

  // Configure motors
  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot1 = configs.Slot0;

    slot1.kS = 0.25;
    slot1.kV = 1;
    slot1.kA = 1;
    slot1.kP = 3;
    slot1.kI = 0;
    slot1.kD = 0.3;

    CurrentLimitsConfigs currentLimits = configs.CurrentLimits;
    currentLimits.StatorCurrentLimit = 20;
    currentLimits.SupplyCurrentLimit = 10;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimitEnable = true;

    var MotionMagicConfig = configs.MotionMagic;
    MotionMagicConfig.MotionMagicCruiseVelocity = 1.5;
    MotionMagicConfig.MotionMagicAcceleration = 160;
    MotionMagicConfig.MotionMagicJerk = 20;

    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_climb_motor.getConfigurator().apply(configs);
  }

  // command that if scheduled would automatically align with the tower
  public Command autoAlignRoutine(ClimbState state) {
    return Commands.run(
            () -> {
              if (currentState != ClimbState.Climbing && isClose()) {
                CommandScheduler.getInstance().schedule(new ClimbAutoAlign());
              }
            },
            ClimbSubsystem.this)
        .andThen(() -> {});
  }

  private boolean isClose() {
    Transform2d delta = TowerPose.minus(frontBumper);

    double x = delta.getX(); // forward
    double y = delta.getY(); // sideways

    double angle = Math.abs(Math.atan2(y, x));

    return x < MIN_X && Math.abs(y) < MIN_Y && angle < MIN_ANGLE;
  }

  private static class ClimbAutoAlign extends Command {
    // Poses

    // PID CONTROLLERS
    private final PIDController pidX = new PIDController(0, 0, 4);
    private final PIDController pidY = new PIDController(0, 0, 4);
    private final PIDController pidRotate = new PIDController(8, 0, 0);

    private final SwerveRequest.FieldCentric m_driveRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public ClimbAutoAlign() {
      // Any rotational calculations are automatically clampd to the nearest target
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      setName("Climb auto align"); // Sets name of command
    }

    @Override
    public void initialize() {
      frontBumper =
          driveTrain
              .getStateCopy()
              .Pose
              .transformBy(
                  new Transform2d(new Translation2d(ROBOT_LENGTH_METERS / 2, 0), Rotation2d.kZero));
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {

      return false;
    }

    @Override
    public void end(boolean interrupted) {
      if (interrupted) {}
    }
  }

  public Command terminate(Command object) {
    return new InstantCommand(
        () -> {
          CommandScheduler.getInstance().cancel(object);
        });
  }
}

*/
