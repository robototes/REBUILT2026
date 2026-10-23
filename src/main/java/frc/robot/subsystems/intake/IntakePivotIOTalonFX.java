package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakePivotIOTalonFX implements IntakeIO {
    private TalonFX pivotMotor;
    private TalonFX pivotStator;
    private TalonFX pivotSupply;
    public IntakePivotIOTalonFX(TalonFX pivotMotor, TalonFX pivotStator, TalonFX pivotSupply) {
        pivotMotor = pivotMotor;
        pivotStator = pivotStator;
        pivotSupply = pivotSupply;
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.currentPos = pivotMotor.getPosition().getValueAsDouble();
        inputs.statorCurrentPivot = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrentPivot = pivotMotor.getSupplyCurrent().getValueAsDouble();
    }
}
