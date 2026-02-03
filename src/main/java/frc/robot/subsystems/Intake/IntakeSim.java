package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSim extends SubsystemBase {
    private static final double updateSim = 0.02;
    private final DoubleTopic leftRollerTopic;
    private final DoubleTopic rightRollerTopic;
    private final DoublePublisher leftRollerPub;
    private final DoublePublisher rightRollerPub;
    private final FlywheelSim leftRollerSim;
    private final FlywheelSim rightRollerSim;
    private final SingleJointedArmSim pivotSimV2;

      private final TalonFXSimState leftRollerSimState;
      private final TalonFXSimState rightRollerSimState;

    private final MechanismRoot2d pivotShoulder;
    private final MechanismLigament2d pivotArm;

    private static final double PIVOT_GEAR_RATIO = (54.0 / 12.0) * (54.0 / 18.0) * (48.0 / 18.0);
    private static final double ARM_LENGTH_METERS = Units.inchesToMeters(10.919);

    LinearSystem rollerSystem = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1);

    public IntakeSim(Mechanism2d mechanism2d, TalonFX leftRollers, TalonFX rightRollers) {
        pivotShoulder = mechanism2d.getRoot("Shoulder", 0.178 / 2.0, 0.2);
        pivotArm = pivotShoulder.append(new MechanismLigament2d("arm", ARM_LENGTH_METERS, 90));

        if (RobotBase.isSimulation()) {
        leftRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));
        rightRollerSim = new FlywheelSim(rollerSystem, DCMotor.getKrakenX60(1));

        leftRollerSimState = leftRollers.getSimState();
        leftRollerSimState.setMotorType(MotorType.KrakenX60);
        rightRollerSimState = rightRollers.getSimState();
        rightRollerSimState.setMotorType(MotorType.KrakenX60);


        pivotSimV2 =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              PIVOT_GEAR_RATIO,
              SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, 2),
              Units.inchesToMeters(ARM_LENGTH_METERS),
              Units.degreesToRadians(0), // minimum arm angle
              Units.degreesToRadians(120), // maximum arm angle
              false,
              Units.degreesToRadians(
                  120)); // starting arm angle, keep this the same value as max arm angle.
        } else {
            throw new IllegalStateException("this is not sim what are you doing");
        }

        var nt = NetworkTableInstance.getDefault();
        this.leftRollerTopic = nt.getDoubleTopic("left intake status/speed in RPM");
        this.leftRollerPub = leftRollerTopic.publish();

        this.rightRollerTopic = nt.getDoubleTopic("right intake status/speed in RPM");
        this.rightRollerPub = rightRollerTopic.publish();
    }

    public void update() {
        leftRollerSim.setInput(leftRollerSimState.getMotorVoltage());
    }
}
