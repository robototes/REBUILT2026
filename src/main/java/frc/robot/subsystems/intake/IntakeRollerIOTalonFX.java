package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class IntakeRollerIOTalonFX implements IntakeIO {

  private TalonFX leftRoller;
  private TalonFX rightRoller;

  private final StatusSignal<AngularVelocity> rollerLeftVelocity;
  private final StatusSignal<Current> rollerLeftStatorCurrent;
  private final StatusSignal<Current> rollerLeftSupplyCurrent;
  private final StatusSignal<AngularVelocity> rollerRightVelocity;
  private final StatusSignal<Current> rollerRightStatorCurrent;
  private final StatusSignal<Current> rollerRightSupplyCurrent;
  private final StatusSignal<Temperature> rollerLeftTemp;
  private final StatusSignal<Temperature> rollerRightTemp;

  public IntakeRollerIOTalonFX(TalonFX rollerMotor1, TalonFX rollerMotor2) {
    leftRoller = rollerMotor1;
    rightRoller = rollerMotor2;
    rollerLeftVelocity = leftRoller.getVelocity();
    rollerRightVelocity = rightRoller.getVelocity();
    rollerLeftStatorCurrent = leftRoller.getStatorCurrent();
    rollerLeftSupplyCurrent = leftRoller.getSupplyCurrent();
    rollerRightStatorCurrent = rightRoller.getStatorCurrent();
    rollerRightSupplyCurrent = rightRoller.getSupplyCurrent();
    rollerLeftTemp = leftRoller.getDeviceTemp();
    rollerRightTemp = rightRoller.getDeviceTemp();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    StatusSignal.refreshAll(
        rollerLeftStatorCurrent,
        rollerLeftSupplyCurrent,
        rollerLeftTemp,
        rollerLeftVelocity,
        rollerRightStatorCurrent,
        rollerRightSupplyCurrent,
        rollerRightTemp,
        rollerRightVelocity);
    inputs.currentVelocityMotor1 = rollerLeftVelocity.getValueAsDouble();
    inputs.currentVelocityMotor2 = rollerRightVelocity.getValueAsDouble();
    inputs.statorCurrentRoller1 = rollerLeftStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentRoller1 = rollerLeftSupplyCurrent.getValueAsDouble();
    inputs.statorCurrentRoller2 = rollerRightStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentRoller2 = rollerRightSupplyCurrent.getValueAsDouble();
    inputs.isConnected1 = leftRoller.isConnected();
    inputs.isConnected2 = rightRoller.isConnected();
    inputs.motorTemp1 = rollerLeftTemp.getValueAsDouble();
    inputs.motorTemp2 = rollerRightTemp.getValueAsDouble();
  }
}
