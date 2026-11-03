package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
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
    StatusSignal.refreshAll(
        inputs.currentVelocityMotor1,
        inputs.currentVelocityMotor2,
        inputs.statorCurrentRoller1,
        inputs.supplyCurrentRoller1,
        inputs.statorCurrentRoller2,
        inputs.supplyCurrentRoller2,
        inputs.isConnected1,
        inputs.isConnected2,
        inputs.motorTemp1,
        inputs.motorTemp2);
  }
}
