package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.util.robotType.RobotType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeRollers extends SubsystemBase {
  // motors
  private final TalonFX leftRoller;
  private final TalonFX rightRoller;
  private RollerSim rollerSim;

  public final double TARGET_RPS = 68;
  public final double AGITATE_RPS = TARGET_RPS / 2;
  private final LoggedNetworkBoolean TUNABLE_ENABLE =
      new LoggedNetworkBoolean("Tuning/TuneIntakeRollers", false);
  private final LoggedNetworkNumber NT_TARGET_RPS =
      new LoggedNetworkNumber("Tuning/intake/TargetVelocityRPS", TARGET_RPS);
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

  // status signals
  private final StatusSignal<AngularVelocity> rollerLeftVelocity;
  private final StatusSignal<Current> rollerLeftStatorCurrent;
  private final StatusSignal<Current> rollerLeftSupplyCurrent;
  private final StatusSignal<AngularVelocity> rollerRightVelocity;
  private final StatusSignal<Current> rollerRightStatorCurrent;
  private final StatusSignal<Current> rollerRightSupplyCurrent;

  public IntakeRollers() {
    // define motors and configs
    leftRoller =
        new TalonFX(
            Hardware.INTAKE_MOTOR_ONE_ID,
            (RobotType.isAlpha() ? AlphaTunerConstants.kCANBus : CANBus.roboRIO()));
    rightRoller = new TalonFX(Hardware.INTAKE_MOTOR_TWO_ID);
    motorConfigs();
    leftRoller.clearStickyFaults();
    rightRoller.clearStickyFaults();

    // sim creator
    if (RobotBase.isSimulation()) {
      rollerSim = new RollerSim(leftRoller, rightRoller);
    }

    rollerLeftVelocity = leftRoller.getVelocity();
    rollerRightVelocity = rightRoller.getVelocity();
    rollerLeftStatorCurrent = leftRoller.getStatorCurrent();
    rollerLeftSupplyCurrent = leftRoller.getSupplyCurrent();
    rollerRightStatorCurrent = rightRoller.getStatorCurrent();
    rollerRightSupplyCurrent = rightRoller.getSupplyCurrent();
  }

  // roller configs
  private void motorConfigs() {
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // KEEP TS AT COAST
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonFXConfigs.Slot0.kP = RobotType.isAlpha() ? 5.0 : 4.0;
    talonFXConfigs.Slot0.kS = RobotType.isAlpha() ? 5.0 : 1.0;
    talonFXConfigs.Slot0.kA = 0.2;

    // configurator
    leftRoller.getConfigurator().apply(talonFXConfigs);
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightRoller.getConfigurator().apply(talonFXConfigs);
  }

  public void runRollers(double velocity) {
    if (TUNABLE_ENABLE.get() && velocity == TARGET_RPS) {
      leftRoller.setControl(velocityRequest.withVelocity(NT_TARGET_RPS.get()));
      rightRoller.setControl(velocityRequest.withVelocity(NT_TARGET_RPS.get()));
    } else {
      leftRoller.setControl(velocityRequest.withVelocity(velocity));
      rightRoller.setControl(velocityRequest.withVelocity(velocity));
    }
  }

  public void stopMotor() {
    leftRoller.stopMotor();
    rightRoller.stopMotor();
  }

  @Override
  // update networktables
  public void periodic() {
    StatusSignal.refreshAll(
        rollerLeftVelocity,
        rollerRightVelocity,
        rollerLeftStatorCurrent,
        rollerLeftSupplyCurrent,
        rollerRightStatorCurrent,
        rollerRightSupplyCurrent);
    Logger.recordOutput(
        "Intake/Rollers/Left/leftRollerSpeed", rollerLeftVelocity.getValueAsDouble());
    Logger.recordOutput(
        "Intake/Rollers/Right/rightRollerSpeed", rollerRightVelocity.getValueAsDouble());
    Logger.recordOutput(
        "Intake/Rollers/Left/leftRollerStator", rollerLeftStatorCurrent.getValueAsDouble());
    Logger.recordOutput(
        "Intake/Rollers/Right/rightRollerStator", rollerRightStatorCurrent.getValueAsDouble());
    Logger.recordOutput(
        "Intake/Rollers/Left/leftRollerSupply", rollerLeftSupplyCurrent.getValueAsDouble());
    Logger.recordOutput(
        "Intake/Rollers/Right/rightRollerSupply", rollerRightSupplyCurrent.getValueAsDouble());
  }

  // update sim
  public void simulationPeriodic() {
    if (rollerSim != null) {
      rollerSim.updateRollers();
    }
  }
}
