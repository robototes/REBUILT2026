package frc.robot.util.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

/*SOURCE: https://github.com/githubAnish/robot-2026-rewrite/blob/main/src/main/java/org/frogforce503/lib/rebuilt/BumpPhysicsSim.java */
public class BumpPhysicsSim {
  private final double GRAVITY = 9.81;
  private final double BUMP_KICK_SCALAR =
      0.2; // Tune this (1.0 = Perfect rigid bounce (lots of air), 0.0 = Magnetically glued to the
  // ramp)

  private final double halfLength = RobotSim.SIM_ROBOT_LENGTH_M / 2.0;
  private final double halfWidth = RobotSim.SIM_ROBOT_WIDTH_M / 2.0;

  private double currentZ = 0.0;
  private double velocityZ = 0.0;

  public Pose3d updateSim(Pose2d robot2dPose, ChassisSpeeds fieldVelocity, double dt) {
    Rotation2d yaw = robot2dPose.getRotation();

    // 1. Sample 4 corners AND the center point
    Translation2d pos = robot2dPose.getTranslation();
    TerrainState center = getTerrainState(pos);
    TerrainState fl =
        getTerrainState(pos.plus(new Translation2d(halfLength, halfWidth).rotateBy(yaw)));
    TerrainState fr =
        getTerrainState(pos.plus(new Translation2d(halfLength, -halfWidth).rotateBy(yaw)));
    TerrainState bl =
        getTerrainState(pos.plus(new Translation2d(-halfLength, halfWidth).rotateBy(yaw)));
    TerrainState br =
        getTerrainState(pos.plus(new Translation2d(-halfLength, -halfWidth).rotateBy(yaw)));

    // 2. Natural Tilting (Pitch/Roll)
    double front_h = (fl.height() + fr.height()) / 2.0;
    double back_h = (bl.height() + br.height()) / 2.0;
    double left_h = (fl.height() + bl.height()) / 2.0;
    double right_h = (fr.height() + br.height()) / 2.0;

    double pitch = Math.atan2(back_h - front_h, halfLength * 2.0);
    double roll = Math.atan2(left_h - right_h, halfWidth * 2.0);

    // 3. Rigid Body Target Z
    double dz_pitch = Math.sin(pitch) * halfLength;
    double dz_roll = Math.sin(roll) * halfWidth;

    double fl_z_offset = -dz_pitch + dz_roll;
    double fr_z_offset = -dz_pitch - dz_roll;
    double bl_z_offset = dz_pitch + dz_roll;
    double br_z_offset = dz_pitch - dz_roll;

    double targetZ =
        Math.max(
            center.height(),
            Math.max(
                Math.max(fl.height() - fl_z_offset, fr.height() - fr_z_offset),
                Math.max(bl.height() - bl_z_offset, br.height() - br_z_offset)));

    // 4. Needed Vertical Velocity for "Kick" (Now with dampening!)
    double neededVelocityZ =
        (fieldVelocity.vxMetersPerSecond * center.slopeX())
            + (fieldVelocity.vyMetersPerSecond * center.slopeY());

    // Apply the dampener to simulate tire squish and energy loss
    neededVelocityZ *= BUMP_KICK_SCALAR;

    // 5. Ballistic Physics (Flight Logic)
    velocityZ -= GRAVITY * dt;
    currentZ += velocityZ * dt;

    // Ground Collision
    if (currentZ <= targetZ) {
      currentZ = targetZ;
      velocityZ = Math.max(velocityZ, neededVelocityZ);
    }

    return new Pose3d(
        robot2dPose.getX(),
        robot2dPose.getY(),
        currentZ,
        new Rotation3d(roll, pitch, yaw.getRadians()));
  }

  private TerrainState getTerrainState(Translation2d pos) {
    if (!FieldSimConstants.Bump.contains(pos)) {
      return new TerrainState(0, 0, 0);
    }

    // Constants from Field Manual
    final double bumpDepth = Units.inchesToMeters(44.4);
    final double tan15 = Math.tan(Units.degreesToRadians(15.0));

    double distFromInit;
    double slopeDir;

    if (pos.getX() < FieldSimConstants.fieldLength / 2.0) {
      distFromInit = pos.getX() - FieldSimConstants.Lines.blueInitLineX;
      slopeDir = 1.0;

    } else {

      distFromInit = FieldSimConstants.Lines.redInitLineX - pos.getX();
      slopeDir = -1.0;
    }

    double x = Math.max(0, Math.min(distFromInit, bumpDepth));

    if (x < bumpDepth / 2.0) {
      return new TerrainState(x * tan15, tan15 * slopeDir, 0);
    } else {
      return new TerrainState((bumpDepth - x) * tan15, -tan15 * slopeDir, 0);
    }
  }

  private record TerrainState(double height, double slopeX, double slopeY) {}
}
