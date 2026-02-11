package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSim extends SubsystemBase {
  private static final double updateSim = 0.02;
  private final FlywheelSim leftRollerSim;
  private final FlywheelSim rightRollerSim;
  private final SingleJointedArmSim pivotSimV2;

  private final TalonFXSimState leftRollerSimState;
  private final TalonFXSimState rightRollerSimState;
  private final TalonFXSimState pivotSimState;

  private final MechanismRoot2d pivotRoot;
  private final MechanismLigament2d pivotArm;

  private static final double PIVOT_GEAR_RATIO = 36.0;
  private static final double ROLLERS_GEAR_RATIO = 1.0;
  private static final double ARM_LENGTH_INCHES = 11.598;
  private static final double ARM_START_POS = 45;

  LinearSystem rollerSystem =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1), 20, ROLLERS_GEAR_RATIO); // idk how to calculate MOI

  public IntakeSim(TalonFX leftRollers, TalonFX rightRollers, TalonFX pivotMotor) {

    if (!RobotBase.isSimulation()) {
      throw new IllegalStateException("This is not sim what are you doing");
    }

    Mechanism2d mech = new Mechanism2d(40, 40);

    leftRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));
    rightRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));

    leftRollerSimState = leftRollers.getSimState();
    leftRollerSimState.setMotorType(MotorType.KrakenX60);
    rightRollerSimState = rightRollers.getSimState();
    rightRollerSimState.setMotorType(MotorType.KrakenX60);

    pivotSimV2 =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            PIVOT_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH_INCHES, 2),
            ARM_LENGTH_INCHES, // length
            Units.degreesToRadians(-120), // minimum arm angle
            Units.degreesToRadians(120), // maximum arm angle
            false,
            Units.degreesToRadians(ARM_START_POS)); // starting arm angle

    pivotSimState = pivotMotor.getSimState();
    pivotSimState.setMotorType(MotorType.KrakenX44);
    pivotSimState.Orientation = ChassisReference.Clockwise_Positive;

    pivotRoot = mech.getRoot("Shoulder", 20, 20);
    pivotArm = pivotRoot.append(new MechanismLigament2d("arm", ARM_LENGTH_INCHES, 90));

    SmartDashboard.putData("Pivot", mech);
  }

  public void update() {
    double leftRPM = leftRollerSim.getAngularVelocityRPM();
    double leftRPS = leftRPM / 60.0;
    double rightRPM = rightRollerSim.getAngularVelocityRPM();
    double rightRPS = rightRPM / 60.0;
    // rollers
    leftRollerSim.setInput(leftRollerSimState.getMotorVoltage());
    leftRollerSim.update(updateSim);
    leftRollerSimState.setRotorVelocity(leftRPS * ROLLERS_GEAR_RATIO);

    rightRollerSim.setInput(rightRollerSimState.getMotorVoltage());
    rightRollerSim.update(updateSim);
    rightRollerSimState.setRotorVelocity(rightRPS * ROLLERS_GEAR_RATIO);
    // pivot arm
    pivotSimV2.setInput(pivotSimState.getMotorVoltage());
    pivotSimV2.update(updateSim);

    double angleRads = pivotSimV2.getAngleRads();
    double motorRotations = Units.radiansToRotations(angleRads) * PIVOT_GEAR_RATIO;

    pivotSimState.setRawRotorPosition(motorRotations);
    pivotSimState.setRotorVelocity(
        Units.radiansToRotations(pivotSimV2.getVelocityRadPerSec()) * PIVOT_GEAR_RATIO);

    // update sim
    pivotArm.setAngle(Units.radiansToDegrees(angleRads) + ARM_START_POS);
  }
}
