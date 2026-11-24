package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.util.StatusSignalRefresher;

public class IntakeRollerIOTalonFX implements IntakeIO {

  private final TalonFX leftRoller;
  private final TalonFX rightRoller;

  private final StatusSignal<AngularVelocity> rollerLeftVelocity;
  private final StatusSignal<Current> rollerLeftStatorCurrent;
  private final StatusSignal<Current> rollerLeftSupplyCurrent;
  private final StatusSignal<AngularVelocity> rollerRightVelocity;
  private final StatusSignal<Current> rollerRightStatorCurrent;
  private final StatusSignal<Current> rollerRightSupplyCurrent;
  private final StatusSignal<Temperature> rollerLeftTemp;
  private final StatusSignal<Temperature> rollerRightTemp;

  private final BaseStatusSignal[] leftSignals;
  private final BaseStatusSignal[] rightSignals;

  public IntakeRollerIOTalonFX(TalonFX rollerMotor1, TalonFX rollerMotor2) {
    leftRoller = rollerMotor1;
    rightRoller = rollerMotor2;

    rollerLeftVelocity = leftRoller.getVelocity();
    rollerLeftStatorCurrent = leftRoller.getStatorCurrent();
    rollerLeftSupplyCurrent = leftRoller.getSupplyCurrent();
    rollerLeftTemp = leftRoller.getDeviceTemp();

    rollerRightVelocity = rightRoller.getVelocity();
    rollerRightStatorCurrent = rightRoller.getStatorCurrent();
    rollerRightSupplyCurrent = rightRoller.getSupplyCurrent();
    rollerRightTemp = rightRoller.getDeviceTemp();

    leftSignals =
        new BaseStatusSignal[] {
          rollerLeftVelocity, rollerLeftStatorCurrent, rollerLeftSupplyCurrent
        };
    rightSignals =
        new BaseStatusSignal[] {
          rollerRightVelocity, rollerRightStatorCurrent, rollerRightSupplyCurrent
        };

    BaseStatusSignal.setUpdateFrequencyForAll(50, leftSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, rightSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(4, rollerLeftTemp, rollerRightTemp);


    leftRoller.optimizeBusUtilization();
    rightRoller.optimizeBusUtilization();

    var refresher = StatusSignalRefresher.getInstance();
    refresher.addStatusSignals(leftSignals);
    refresher.addStatusSignals(rightSignals);
    refresher.addStatusSignals(12, rollerLeftTemp, rollerRightTemp); // ~4 Hz
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.currentVelocityMotor1 = rollerLeftVelocity.getValueAsDouble();
    inputs.currentVelocityMotor2 = rollerRightVelocity.getValueAsDouble();
    inputs.statorCurrentRoller1 = rollerLeftStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentRoller1 = rollerLeftSupplyCurrent.getValueAsDouble();
    inputs.statorCurrentRoller2 = rollerRightStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentRoller2 = rollerRightSupplyCurrent.getValueAsDouble();
    inputs.isConnected1 = BaseStatusSignal.isAllGood(leftSignals);
    inputs.isConnected2 = BaseStatusSignal.isAllGood(rightSignals);
    inputs.motorTemp1 = rollerLeftTemp.getValueAsDouble();
    inputs.motorTemp2 = rollerRightTemp.getValueAsDouble();
  }
}
