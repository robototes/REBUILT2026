package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.simulation.RobotSim;

public class RollerSim extends SubsystemBase {
  // flywheel sim variables
  private final FlywheelSim leftRollerSim;
  private final FlywheelSim rightRollerSim;

  // sim STATE variables
  private final TalonFXSimState leftRollerSimState;
  private final TalonFXSimState rightRollerSimState;

  private final MechanismLigament2d roller;

  private final double ROLLERS_GEAR_RATIO = 1.0; // tune this

  // flywheel system variable
  LinearSystem rollerSystem =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(1), 0.002, ROLLERS_GEAR_RATIO); // idk how to calculate MOI

  public RollerSim(TalonFX leftRoller, TalonFX rightRoller) {
    if (!RobotBase.isSimulation()) {
      throw new IllegalStateException("This is not sim what are you doing");
    }
    // define the simulators
    leftRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));
    rightRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));

    // update sim state
    leftRollerSimState = leftRoller.getSimState();
    leftRollerSimState.setMotorType(MotorType.KrakenX60);
    rightRollerSimState = rightRoller.getSimState();
    rightRollerSimState.setMotorType(MotorType.KrakenX60);

    // visualization
    Mechanism2d mech = new Mechanism2d(40, 40);
    MechanismRoot2d root = mech.getRoot("root", 20, 20);

    roller = root.append(new MechanismLigament2d("roller", 2, 0));

    SmartDashboard.putData("Roller", mech);
  }

  // update sim
  public void updateRollers() {
    // set sim variables based on data
    leftRollerSim.setInput(leftRollerSimState.getMotorVoltage());
    leftRollerSim.update(RobotSim.UPDATE_S);
    rightRollerSim.setInput(rightRollerSimState.getMotorVoltage());
    rightRollerSim.update(RobotSim.UPDATE_S);

    double leftRPM = leftRollerSim.getAngularVelocityRPM();
    double leftRPS = leftRPM / 60.0;
    double rightRPM = rightRollerSim.getAngularVelocityRPM();
    double rightRPS = rightRPM / 60.0;

    leftRollerSimState.setRotorVelocity(leftRPS * ROLLERS_GEAR_RATIO);
    rightRollerSimState.setRotorVelocity(rightRPS * ROLLERS_GEAR_RATIO);
  }
}
