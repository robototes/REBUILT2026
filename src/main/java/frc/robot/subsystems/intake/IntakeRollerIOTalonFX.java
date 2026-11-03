package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeRollerIOTalonFX implements IntakeIO {

  private TalonFX rollerMotor1;
  private TalonFX rollerMotor2;

  public IntakeRollerIOTalonFX(TalonFX rollerMotor1a, TalonFX rollerMotor2a) {
    rollerMotor1 = rollerMotor1a;
    rollerMotor2 = rollerMotor2a;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.currentVelocityMotor1 = rollerMotor1.getVelocity().getValueAsDouble();
    inputs.currentVelocityMotor2 = rollerMotor2.getVelocity().getValueAsDouble();
    inputs.statorCurrentRoller1 = rollerMotor1.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentRoller1 = rollerMotor1.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentRoller2 = rollerMotor2.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentRoller2 = rollerMotor2.getSupplyCurrent().getValueAsDouble();
    inputs.isConnected1 = rollerMotor1.isConnected();
    inputs.isConnected2 = rollerMotor2.isConnected();
    inputs.motorTemp1 = rollerMotor1.getDeviceTemp().getValueAsDouble();
    inputs.motorTemp2 = rollerMotor2.getDeviceTemp().getValueAsDouble();
  }
}
