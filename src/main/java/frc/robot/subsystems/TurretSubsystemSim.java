package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystemSim {
  private final TalonFX motor;
  private final TalonFXSimState simState;

  private final double gearRatio; // motor rotations per mechanism rotation
  private final double minMechRad;
  private final double maxMechRad;

  private final FlywheelSim plant;

  // mechanism-frame state
  private double mechPosRad = 0.0;

  // Mechanism2d
  private final Mechanism2d mech2d;
  private final MechanismLigament2d turretLigament;

  private static final double DEFAULT_MOI = 0.0003; // kg*m^2
  private static final double LIGAMENT_LENGTH_M = 0.35;

  public TurretSubsystemSim(
      TalonFX motor, double gearRatio, double turretMinRad, double turretMaxRad) {
    this(motor, gearRatio, turretMinRad, turretMaxRad, DEFAULT_MOI);
  }

  public TurretSubsystemSim(
      TalonFX motor, double gearRatio, double turretMinRad, double turretMaxRad, double moiKgM2) {
    this.motor = motor;
    this.simState = motor.getSimState();

    this.gearRatio = gearRatio;
    this.minMechRad = Math.min(turretMinRad, turretMaxRad);
    this.maxMechRad = Math.max(turretMinRad, turretMaxRad);

    LinearSystem<N1, N1, N1> sys =
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), moiKgM2, gearRatio);

    this.plant = new FlywheelSim(sys, DCMotor.getKrakenX60(1));

    mech2d = new Mechanism2d(1.0, 1.0);
    MechanismRoot2d root = mech2d.getRoot("TurretRoot", 0.5, 0.5);
    turretLigament = new MechanismLigament2d("Turret", LIGAMENT_LENGTH_M, 0.0);
    root.append(turretLigament);
    SmartDashboard.putData("Turret/Mech2d", mech2d);

    // START MID-RANGE (not at a stop)
    mechPosRad = 0.5 * (minMechRad + maxMechRad);

    pushStateToPhoenix(mechPosRad, 0.0);
    updateMech2d(mechPosRad);
  }

  public void update(double dtSeconds) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    double motorVolts = simState.getMotorVoltage();

    plant.setInputVoltage(motorVolts);
    plant.update(dtSeconds);

    double mechVelRadPerSec = plant.getAngularVelocityRadPerSec();
    mechPosRad += mechVelRadPerSec * dtSeconds;

    // hard stops (position clamp only)
    if (mechPosRad < minMechRad) {
      mechPosRad = minMechRad;
      if (mechVelRadPerSec < 0.0) mechVelRadPerSec = 0.0;
    } else if (mechPosRad > maxMechRad) {
      mechPosRad = maxMechRad;
      if (mechVelRadPerSec > 0.0) mechVelRadPerSec = 0.0;
    }

    pushStateToPhoenix(mechPosRad, mechVelRadPerSec);
    updateMech2d(mechPosRad);
  }

  private void pushStateToPhoenix(double mechPosRad, double mechVelRadPerSec) {
    double mechRot = Units.radiansToRotations(mechPosRad);
    double rotorRot = mechRot * gearRatio;

    double mechRps = Units.radiansToRotations(mechVelRadPerSec);
    double rotorRps = mechRps * gearRatio;

    simState.setRawRotorPosition(rotorRot);
    simState.setRotorVelocity(rotorRps);
  }

  private void updateMech2d(double mechPosRad) {
    turretLigament.setAngle(Units.radiansToDegrees(mechPosRad));
  }
}
