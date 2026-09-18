package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.framework.RobotBase;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import frc.robot.util.simulation.RobotSim;

public class HoodSim {

  private final TalonFXSimState simState;
  private final SingleJointedArmSim armSim;

  private final Mechanism2d mechanism;
  private final MechanismLigament2d hoodLigament;

  private static final double GEAR_RATIO = 23.2727;
  private static final double ARM_LENGTH_METERS = Units.inchesToMeters(7);
  private static final double STARTING_ANGLE_OFFSET = 12;

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
            Units.degreesToRadians(STARTING_ANGLE_OFFSET));

    mechanism = new Mechanism2d(60, 60);
    MechanismRoot2d root = mechanism.getRoot("hoodRoot", 30, 10);

    hoodLigament =
        root.append(new MechanismLigament2d("hood", 20, 0, 6, new Color8Bit(Color.AQUA)));

    SmartDashboard.putData("Hood Mechanism", mechanism);
  }

  public void update() {
    // Run physics
    armSim.setInput(simState.getMotorVoltage());
    armSim.update(RobotSim.UPDATE_S);

    // Convert arm into motor units
    double armAngleRad = armSim.getAngleRads();
    double motorRotations = Units.radiansToRotations(armAngleRad) * GEAR_RATIO;

    simState.setRawRotorPosition(motorRotations);
    simState.setRotorVelocity(Units.radiansToRotations(armSim.getVelocity()) * GEAR_RATIO);

    // Update visualization/sim
    hoodLigament.setAngle(Units.radiansToDegrees(armAngleRad) + STARTING_ANGLE_OFFSET);
  }
}
