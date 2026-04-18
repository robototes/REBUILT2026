package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.robotType.ConfigShift;
import frc.robot.util.robotType.RobotType;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AutoBuilderConfig {

  public static void buildAuto(CommandSwerveDrivetrain drivebase, boolean unitTest) {
    buildAuto(drivebase, null, unitTest);
  }

  public static void buildAuto(
      CommandSwerveDrivetrain drivebase, Subsystems subsystems, boolean unitTest) {

    try {
      AutoBuilder.configure(
          () -> {
            return drivebase.getState().Pose;
          }, // Robot pose supplier
          (pose) -> {
            if (subsystems != null) {
              subsystems.resetRobotPoseExceptGroundTruth(pose);
            } else {
              drivebase.resetPose(pose);
            }
          }, // Method to reset odometry (will be called if your auto has a starting
          // pose)
          () -> {
            return drivebase.getState().Speeds;
          },
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) ->
              drivebase.setControl(
                  new SwerveRequest.ApplyRobotSpeeds()
                      .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(
                          feedforwards
                              .robotRelativeForcesYNewtons())), // Method that will drive the robot
          // given ROBOT RELATIVE
          // ChassisSpeeds. Also optionally outputs individual module
          // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for holonomic drive trains
              new PIDConstants(5, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ),
          ConfigShift.configFromRobot(setRobot(unitTest)), // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            return AllianceUtils.isRed(); // Checking alliance is red
          },
          drivebase // Reference to this subsystem to set requirements
          );

    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  public static String setRobot(boolean unitTest) throws IOException, ParseException {
    if (unitTest) {
      return "sim";
    }

    return (RobotType.isAlpha() ? "alpha" : "comp");
  }
}
