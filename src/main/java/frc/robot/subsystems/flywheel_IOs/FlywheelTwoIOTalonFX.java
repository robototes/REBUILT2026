package frc.robot.subsystems.flywheel_IOs;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.flywheel_IOs.FlywheelOneIO.FlywheelOneIOInputs;

public class FlywheelTwoIOTalonFX implements FlywheelTwoIO{
private TalonFX flywheelTwo;

    public FlywheelTwoIOTalonFX(TalonFX flywheel2) {
        flywheelTwo = flywheel2;
    }
    @Override
    public void updateInputs(FlywheelTwoIOInputs inputs) {
        inputs.velocity = flywheelTwo.getVelocity().getValueAsDouble();
        inputs.statorCurrent = flywheelTwo.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = flywheelTwo.getSupplyCurrent().getValueAsDouble();
    }
}
