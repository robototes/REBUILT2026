// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TuningConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederSubsystem extends SubsystemBase {
  private final double ARMPIVOT_KP = 38.5;
  private final double ARMPIVOT_KI = 0;
  private final double ARMPIVOT_KD = 0;
  private final double ARMPIVOT_KS = 0;
  private final double ARMPIVOT_KV = 0;
  private final double ARMPIVOT_KG = 0;
  private final double ARMPIVOT_KA = 0;

  private final TalonFX feedMotor;

  //MechanismRoot2d motorRoot = Robot.getInstance().getRobotMechanism2d().getRoot("motorRoot", 0, 0.0);
  //MechanismLigament2d flywheelmotor1 = motorRoot.append(new MechanismLigament2d("flywheelMotor1", 0, 0));
  //MechanismLigament2d flywheelmotor2 = motorRoot.append(new MechanismLigament2d("flywheelMotor2", 0, 0));

  private final SingleJointedArmSim motorSim;

  public FeederSubsystem() {
    feedMotor = new TalonFX(Constants.HardwareConstants.feedMotorID);
    feederConfig();

    if (RobotBase.isSimulation()) {
      motorSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        1,
        SingleJointedArmSim.estimateMOI(0, 0),
        0, // Arm length (meters)
        -Math.PI, // Min angle (rad)
        Math.PI, // Max angle (rad)
        true, // Has gravity
        0.0 // Initial angle (rad)
      );
    } else {
      motorSim = null;
    }
  }

  public void feederConfig() {
    //DigitalInput m_sensor = new DigitalInput(HardwareConstants.digitalInputChannel);

    TalonFXConfigurator cfg = feedMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    // Inverting motor output direction
    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Setting the motor to brake when not moving
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 20; 
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 10; 
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // PID
    // set slot 0 gains
    talonFXConfiguration.Slot0.kS = ARMPIVOT_KS;
    talonFXConfiguration.Slot0.kV = ARMPIVOT_KV;
    talonFXConfiguration.Slot0.kA = ARMPIVOT_KA;
    talonFXConfiguration.Slot0.kP = ARMPIVOT_KP;
    talonFXConfiguration.Slot0.kI = ARMPIVOT_KI;
    talonFXConfiguration.Slot0.kD = ARMPIVOT_KD;
    talonFXConfiguration.Slot0.kG = ARMPIVOT_KG;
    talonFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    cfg.apply(talonFXConfiguration);
  }

  public void setSpeed(double speed) {
    feedMotor.set(speed);
  }

  public Command startMotor() {
    return runOnce(
        () -> {
          setSpeed(TuningConstants.feederSpeed);
        });
  }

  public Command stopMotor() {
    return runOnce(
        () -> {
          setSpeed(0);
        });
  }

  //remove in final, only here to stop Autos.java from throwing errors while also doing nothing
  public Command doNothing() {
    return runOnce(
        () -> {
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
