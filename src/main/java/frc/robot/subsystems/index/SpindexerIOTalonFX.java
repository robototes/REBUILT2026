package frc.robot.subsystems.index;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import java.util.Objects;

public class SpindexerIOTalonFX implements SpindexerIO {

  private final TalonFX spindexerMotor;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;

  public SpindexerIOTalonFX(TalonFX motor) {
    spindexerMotor = Objects.requireNonNull(motor, "motor must not be null");
    velocitySignal = spindexerMotor.getVelocity();
    statorCurrentSignal = spindexerMotor.getStatorCurrent();
    supplyCurrentSignal = spindexerMotor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    Objects.requireNonNull(inputs, "inputs must not be null");
    StatusCode status =
        BaseStatusSignal.refreshAll(velocitySignal, statorCurrentSignal, supplyCurrentSignal);
    if (!status.isOK()) {
      // Refresh failed (e.g. CAN bus disconnect); keep last known-good values
      return;
    }
    inputs.velocityRPS = velocitySignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
  }
}
