package frc.robot.subsystems.flywheel_IOs;

import com.ctre.phoenix6.hardware.TalonFX;

public class FlywheelOneIOTalonFX implements FlywheelOneIO{
    private TalonFX flywheelOne;

    public FlywheelOneIOTalonFX(TalonFX flywheel1) {
        flywheelOne = flywheel1;
    }
    @Override
    public void updateInputs(FlywheelOneIOInputs inputs) {
        inputs.velocity = flywheelOne.getVelocity().getValueAsDouble();
        inputs.statorCurrent = flywheelOne.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = flywheelOne.getSupplyCurrent().getValueAsDouble();
    }
}
