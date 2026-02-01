package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class FlywheelsSim {

  private final TalonFXSimState simTop;
  private final TalonFXSimState simBottom;

  private final FlywheelSim flywheelSim;

  private final MechanismLigament2d wheelLigament;

  // Tune these to match reality
  private static final double GEAR_RATIO = 1.0;
  private static final double MOI = 0.002; // kg*m^2 (increase for heavier wheels)

  public FlywheelsSim(TalonFX topMotor, TalonFX bottomMotor) {

    if (!RobotBase.isSimulation()) {
      throw new IllegalStateException("FlywheelSim created outside simulation");
    }

    simTop = topMotor.getSimState();
    simBottom = bottomMotor.getSimState();

    // Physics model
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(2), MOI, GEAR_RATIO),
            DCMotor.getFalcon500(2), // two motors total
            1);

    // --- Mechanism2d ---
    Mechanism2d mech = new Mechanism2d(40, 40);
    MechanismRoot2d root = mech.getRoot("flywheelRoot", 20, 20);

    wheelLigament =
        root.append(new MechanismLigament2d("wheel", 2, 0, 10, new Color8Bit(Color.kCoral)));

    SmartDashboard.putData("Flywheel", mech);
  }

  public void update() {

    // Average voltage from both motors
    double topVoltage = simTop.getMotorVoltage();
    double bottomVoltage = simBottom.getMotorVoltage();

    // Bottom motor runs opposite direction
    double appliedVoltage = (topVoltage - bottomVoltage) / 2.0;

    // Apply to physics
    flywheelSim.setInputVoltage(appliedVoltage);
    flywheelSim.update(0.02);

    double rpm = flywheelSim.getAngularVelocityRPM();

    // Convert RPM → rotor RPS
    double rps = rpm / 60.0;

    // Feed velocity back to both motors
    simTop.setRotorVelocity(rps * GEAR_RATIO);
    simBottom.setRotorVelocity(-rps * GEAR_RATIO);

    // Spin the visualization
    wheelLigament.setAngle(wheelLigament.getAngle() + rpm * 6.0 * 0.02); // visual spin factor
  }
}
