package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeRollerIOTalonFX implements IntakeIO {

    private TalonFX rollerMotor1;
    private TalonFX rollerMotor2;
    private TalonFX rollerStator1;
    private TalonFX rollerSupply1;
    private TalonFX rollerStator2;
    private TalonFX rollerSupply2;


    public IntakeRollerIOTalonFX(TalonFX rollerMotor1, TalonFX rollerMotor2, TalonFX rollerStator1, TalonFX rollerSupply1, TalonFX rollerStator2, TalonFX rollerSupply2) {
        rollerMotor1 = rollerMotor1;
        rollerMotor2 = rollerMotor2;
        rollerStator1 = rollerStator1;
        rollerSupply1 = rollerSupply1;
        rollerStator2 = rollerStator2;
        rollerSupply2 = rollerSupply2;
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.currentVelocityMotor1 = rollerMotor1.getPosition().getValueAsDouble();
        inputs.currentVelocityMotor2 = rollerMotor2.getPosition().getValueAsDouble();
        inputs.statorCurrentRoller1 = rollerStator1.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrentRoller1 = rollerSupply1.getStatorCurrent().getValueAsDouble();
        inputs.statorCurrentRoller2 = rollerStator2.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrentRoller2 = rollerSupply2.getStatorCurrent().getValueAsDouble();
    }
}
