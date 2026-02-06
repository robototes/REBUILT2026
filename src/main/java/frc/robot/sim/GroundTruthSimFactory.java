package frc.robot.sim;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;

/**
 * Factory for creating GroundTruthSimInterface instances.
 * Returns null when not in simulation mode.
 */
public class GroundTruthSimFactory {

    /**
     * Creates a GroundTruthSimInterface instance if running in simulation mode.
     *
     * @param drivetrain The swerve drivetrain to track and manipulate
     * @param poseResetConsumer Consumer to be called when pose is reset
     * @return A GroundTruthSimInterface instance, or null if not in simulation
     */
    public static GroundTruthSimInterface create(
            SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain,
            Consumer<Pose2d> poseResetConsumer) {
        if (Robot.isSimulation()) {
            return new GroundTruthSim(drivetrain, poseResetConsumer);
        }
        return null;
    }
}
