// TurretSubsystemSim.java
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turret mechanism simulation model + Mechanism2d visualization for AdvantageScope.
 *
 * <p>Publishes a Mechanism2d to NetworkTables via SmartDashboard, so AdvantageScope can show it.
 */
public final class TurretSubsystemSim {
  private final TalonFXSimState simState;

  private final DCMotor motorModel;
  private final LinearSystem<N2, N1, N2> plant;
  private final DCMotorSim motorSim;

  private final double gearRatio; // rotorRotations / mechanismRotations
  private final double minRad; // mechanism radians
  private final double maxRad; // mechanism radians

  // ---- Mechanism2d (for AdvantageScope) ----
  private final Mechanism2d mech2d;
  private final MechanismLigament2d turretArm;

  /**
   * @param motor TalonFX being simulated
   * @param gearRatio rotor rotations per mechanism rotation
   * @param minRad mechanism minimum radians
   * @param maxRad mechanism maximum radians
   */
  public TurretSubsystemSim(TalonFX motor, double gearRatio, double minRad, double maxRad) {
    this.simState = motor.getSimState();

    this.gearRatio = gearRatio;
    this.minRad = minRad;
    this.maxRad = maxRad;

    // Motor model + plant
    this.motorModel = DCMotor.getFalcon500(1);

    // Tune MOI (kg*m^2) to match turret response
    final double moiKgM2 = 0.0035;

    // Plant for rotational mechanism with gear reduction (motor-to-mechanism)
    this.plant = LinearSystemId.createDCMotorSystem(motorModel, moiKgM2, this.gearRatio);

    this.motorSim = new DCMotorSim(plant, motorModel);

    // Phoenix sim supply voltage
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // ---- Mechanism2d setup ----
    // A 1x1 “canvas” with root at center
    mech2d = new Mechanism2d(1.0, 1.0);
    MechanismRoot2d root = mech2d.getRoot("turretRoot", 0.5, 0.5);

    // Arm length is arbitrary for display. Angle will be set each update.
    turretArm = new MechanismLigament2d("turretArm", 0.45, 90.0);
    root.append(turretArm);

    // Publish so AdvantageScope can see it (NetworkTables)
    SmartDashboard.putData("Turret/Mech2d", mech2d);
  }

  /** Call from TurretSubsystem.simulationPeriodic() (typically every 20ms). */
  public void update(double dtSeconds) {
    double batt = RobotController.getBatteryVoltage();
    simState.setSupplyVoltage(batt);

    double motorVolts = MathUtil.clamp(simState.getMotorVoltage(), -12.0, 12.0);

    motorSim.setInputVoltage(motorVolts);
    motorSim.update(dtSeconds);

    // Mechanism pos/vel
    double mechPosRad = motorSim.getAngularPositionRad();
    double mechVelRadPerSec = motorSim.getAngularVelocityRadPerSec();

    // Hard stops
    double clampedPos = MathUtil.clamp(mechPosRad, minRad, maxRad);
    if (clampedPos != mechPosRad) {
      if ((clampedPos <= minRad && mechVelRadPerSec < 0.0)
          || (clampedPos >= maxRad && mechVelRadPerSec > 0.0)) {
        mechVelRadPerSec = 0.0;
      }
      mechPosRad = clampedPos;
    }

    // Feed Phoenix rotor sensors (rotations, rotations/sec)
    double rotorPosRot = (mechPosRad * gearRatio) / (2.0 * Math.PI);
    double rotorVelRps = (mechVelRadPerSec * gearRatio) / (2.0 * Math.PI);
    simState.setRawRotorPosition(rotorPosRot);
    simState.setRotorVelocity(rotorVelRps);

    // Battery sag
    double currentDrawA = motorSim.getCurrentDrawAmps();
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(currentDrawA));

    // ---- Mechanism2d update (degrees) ----
    // MechanismLigament2d uses degrees. Positive angles are CCW in the 2D view.
    turretArm.setAngle(Units.radiansToDegrees(mechPosRad));
  }

  // Optional getters if you want to publish numeric plots too
  public double getMechanismPositionRad() {
    return motorSim.getAngularPositionRad();
  }

  public double getMechanismVelocityRadPerSec() {
    return motorSim.getAngularVelocityRadPerSec();
  }
}
