package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class HoodSim {

  private final TalonFXSimState simState;
  private final SingleJointedArmSim armSim;

  private final Mechanism2d mechanism;
  private final MechanismLigament2d hoodLigament;

  private static final double GEAR_RATIO = 23.2727;
  private static final double ARM_LENGTH_METERS = Units.inchesToMeters(7);

  public HoodSim(TalonFX hoodMotor) {
    if (!RobotBase.isSimulation()) {
      throw new IllegalStateException("HoodSim created outside of simulation");
    }

    simState = hoodMotor.getSimState();
    simState.setMotorType(MotorType.KrakenX44);
    simState.Orientation = ChassisReference.Clockwise_Positive;

    armSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44(1),
            GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, 1),
            ARM_LENGTH_METERS,
            Units.degreesToRadians(-540),
            Units.degreesToRadians(540),
            false,
            Units.degreesToRadians(0));

    mechanism = new Mechanism2d(60, 60);
    MechanismRoot2d root = mechanism.getRoot("hoodRoot", 30, 10);

    hoodLigament =
        root.append(new MechanismLigament2d("hood", 20, 0, 6, new Color8Bit(Color.kAqua)));

    SmartDashboard.putData("Hood Mechanism", mechanism);
  }

  public void update() {
    // Run physics
    armSim.setInput(simState.getMotorVoltage());
    armSim.update(0.02);

    // Convert arm into motor units
    double armAngleRad = armSim.getAngleRads();
    double motorRotations = Units.radiansToRotations(armAngleRad) * GEAR_RATIO;

    simState.setRawRotorPosition(motorRotations);
    simState.setRotorVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec()) * GEAR_RATIO);

    // Update visualization/sim
    hoodLigament.setAngle(Units.radiansToDegrees(armAngleRad));
  }
}
