package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.generated.CompTunerConstants;

public class IntakePivot extends SubsystemBase {
  // motor
  private final TalonFX pivotMotor;
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

  // positions
  public static final double PIVOT_DEPLOYED_POS = 12; // (120/360) * pivot gear ratio
  public static final double PIVOT_RETRACTED_POS = 0;
  public static final double POS_TOLERANCE = Units.degreesToRotations(10); // for tolerance check
  private double targetPos;
  private boolean pivotIsRetracted = true; // for toggle feature

  // networktables and sim
  private DoubleTopic pivotTopic;
  private DoublePublisher pivotPublisher;
  private PivotSim pivotSim;

  public IntakePivot() {
    pivotMotor = new TalonFX(Hardware.INTAKE_PIVOT_MOTOR_ID, CompTunerConstants.kCANBus);
    TalonFXConfigs();
    networktables();

    // sim creator
    if (RobotBase.isSimulation()) {
      pivotSim = new PivotSim(pivotMotor);
    }
  }

  // configs
  private void TalonFXConfigs() {
    // define config variables
    var talonFXConfigs = new TalonFXConfiguration();
    var simConfigs = talonFXConfigs.Slot0;
    var irlConfigs = talonFXConfigs.Slot0;
    var motionMagic = talonFXConfigs.MotionMagic; // motion magic settings
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // KEEP TS IN COAST

    // PIDS

    // simConfigs.kV = 5.0;
    simConfigs.kA = 0.0;
    simConfigs.kP = 2.0;
    simConfigs.kI = 0.0;
    simConfigs.kD = 0.0;
    simConfigs.kG = 0.0;

    // note: these are PIDS from last year's robot
    // TODO: tune with physical robot
    irlConfigs.kP = 45;
    irlConfigs.kI = 0.0;
    irlConfigs.kD = 0.0;
    irlConfigs.kA = 0.0;
    irlConfigs.kV = 0.0;
    irlConfigs.kS = 0.155;
    irlConfigs.kG = 0.0;

    // motion magic settings
    motionMagic.MotionMagicAcceleration = 25;
    motionMagic.MotionMagicJerk = 0;

    // apply configs
    talonFXConfigs.Slot0 =
        (Robot.isSimulation()) ? simConfigs : irlConfigs; // interchanging between sim and irl
    pivotMotor.getConfigurator().apply(talonFXConfigs);
  }

  // configure networktables
  private void networktables() {
    var nt = NetworkTableInstance.getDefault();
    this.pivotTopic = nt.getDoubleTopic("intake/pivotPosition");
    this.pivotPublisher = pivotTopic.publish();

    pivotPublisher.set(0); // default value
  }

  // pivot position for networktables
  public double getPivotPos() {
    var curPos = pivotMotor.getPosition();
    return curPos.getValueAsDouble();
  }

  private Command setTargetPos(double pos) {
    return Commands.runOnce(
        () -> {
          pivotMotor.setControl(pivotRequest.withPosition(pos));
          targetPos = pos;
        });
  }

  // main command to go to a position
  public Command goToPos(double position) {
    return setTargetPos(position).andThen(Commands.waitUntil(atPosition(position)));
  }

  // check position with tolerance
  public Trigger atPosition(double position) {
    return new Trigger(() -> Math.abs(getPivotPos() - position) < POS_TOLERANCE);
  }

  // stop command just in case
  public Command stop() {
    return Commands.runOnce(
        () -> {
          pivotMotor.stopMotor();
        });
  }

  // check if pivot is retracted
  public boolean isDeployed() {
    return Math.abs(getPivotPos() - PIVOT_DEPLOYED_POS) < POS_TOLERANCE;
  }

  @Override
  // update networktables
  public void periodic() {
    pivotPublisher.set(getPivotPos());
  }

  // update sim
  public void simulationPeriodic() {
    if (pivotSim != null) {
      pivotSim.updateArm();
    }
  }
}
