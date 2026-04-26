package frc.robot.subsystems.auto;
 import frc.robot.Subsystems;
 import frc.robot.lib.BLine.FollowPath;
 import frc.robot.lib.BLine.Path;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
public class BLineLogic {
private static Subsystems s;
  private  Consumer<Pose2d> chassisConsumer;

private  static   FollowPath.Builder pathBuilder;

     public static void init(Subsystems subsystems) {

    s = subsystems;
    s.drivebaseSubsystem.resetPose(getStartPose());
                                                    }


// 2. Create a reusable path builder
public static void configure(Subsystems s) {

    pathBuilder = new FollowPath.Builder(
        s.drivebaseSubsystem,

        // pose supplier
        () -> s.drivebaseSubsystem.getState().Pose,

        // robot-relative speeds supplier
        () -> s.drivebaseSubsystem.getState().Speeds,

        // robot-relative speeds consumer
        (speeds) -> {
            s.drivebaseSubsystem.setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
            );
        },

        // translation PID
        new PIDController(5.0, 0.0, 0.0),

        // rotation PID
        new PIDController(3.0, 0.0, 0.0),

        // cross-track PID
        new PIDController(2.0, 0.0, 0.0)
    )
    .withDefaultShouldFlip()
    .withPoseReset((pose) -> getStartPose());

        };
public static Command runAuto() {

Path myPath = new Path("RT-Neutral-ClimbD");  // loads deploy/autos/paths/myPathFile.json

return pathBuilder.build(myPath);

}
public static Pose2d getStartPose() {

Path myPath = new Path("RT-Neutral-ClimbD");  // loads deploy/autos/paths/myPathFile.json

return myPath.getStartPose();

}
public static FollowPath.Builder getBuilder() {

    return pathBuilder;
}



}


