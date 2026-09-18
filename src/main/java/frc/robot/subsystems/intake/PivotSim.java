package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import frc.robot.util.simulation.RobotSim;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.telemetry.Telemetry;

public class PivotSim extends SubsystemBase {
  private final SingleJointedArmSim intakeArmSim;
  private final TalonFXSimState intakeArmSimState;

  private final MechanismRoot2d root;
  private final MechanismLigament2d arm;

  // physical specs
  private static final double PIVOT_GEAR_RATIO = 36.0;
  private static final double ARM_LENGTH_METERS = Units.inchesToMeters(11.598);
  private static final double ARM_START_POS = 45; // starting position

  public PivotSim(TalonFX pivotMotor) {
    if (!RobotBase.isSimulation()) {
      throw new IllegalStateException("This is not sim what are you doing");
    }
    Mechanism2d mech = new Mechanism2d(40, 40);

    // define the simulator
    intakeArmSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            PIVOT_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, 2),
            ARM_LENGTH_METERS, // length
            Units.degreesToRadians(-120), // minimum arm angle
            Units.degreesToRadians(120), // maximum arm angle
            false,
            Units.degreesToRadians(ARM_START_POS)); // starting arm angle

    // update sim state and some configs
    intakeArmSimState = pivotMotor.getSimState();
    intakeArmSimState.setMotorType(MotorType.KrakenX44);
    intakeArmSimState.Orientation = ChassisReference.Clockwise_Positive;

    root = mech.getRoot("Shoulder", 20, 20);
    arm = root.append(new MechanismLigament2d("arm", ARM_LENGTH_METERS, 0));

    Telemetry.log("Pivot", mech);
  }

  // update sim
  public void updateArm() {
    intakeArmSim.setInput(intakeArmSimState.getMotorVoltage());
    intakeArmSim.update(RobotSim.UPDATE_S);

    double angleRads = intakeArmSim.getAngle();
    double motorRotations = Units.radiansToRotations(angleRads) * PIVOT_GEAR_RATIO;

    intakeArmSimState.setRawRotorPosition(motorRotations);
    intakeArmSimState.setRotorVelocity(
        Units.radiansToRotations(intakeArmSim.getVelocity()) * PIVOT_GEAR_RATIO);

    // update sim
    arm.setAngle(Units.radiansToDegrees(angleRads) + ARM_START_POS);
  }
}
