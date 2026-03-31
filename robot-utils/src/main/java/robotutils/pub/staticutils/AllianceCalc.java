package robotutils.pub.staticutils;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/**
 * Static utility methods for alliance-aware field calculations.
 */
public final class AllianceCalc {
    /** Private constructor. */
    private AllianceCalc() {
        throw new UnsupportedOperationException("AllianceCalc is a static utility class");
    }

    /**
     * Returns true if the robot is currently configured as red alliance.
     */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /**
     * Flips a pose to the opposite alliance side of the field.
     *
     * @param pose The pose to flip (typically authored for blue alliance)
     * @return The flipped pose
     */
    public static Pose2d flipFieldPose(Pose2d pose) {
        return FlippingUtil.flipFieldPose(pose);
    }
}
