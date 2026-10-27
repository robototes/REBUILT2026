package frc.robot.util;

import java.util.Map;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AllianceFlipUtil {
    public static final double FIELD_WIDTH = FieldSize.FIELD_WIDTH_M;
    public static final double FIELD_LENGTH = FieldSize.FIELD_LENGTH_M;

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public static Translation2d flipTranslation(Translation2d t) {
        if (!isRedAlliance())
            return t;
        return new Translation2d(FIELD_LENGTH - t.getX(), FIELD_WIDTH - t.getY());
    }

    public static Rotation2d flipRotation(Rotation2d r) {
        if (!isRedAlliance())
            return r;
        return r.plus(Rotation2d.fromDegrees(180));
    }

    /** Flips a Pose2d to the red alliance */
    public static Pose2d flipPose(Pose2d p) {
        if (!isRedAlliance())
            return p;
        return new Pose2d(flipTranslation(p.getTranslation()), flipRotation(p.getRotation()));
    }

    public static Command command(Command blueCommand, Command redCommand, Command neitherCommand) {
        var options = Map.of(
                Optional.of(Alliance.Red), redCommand,
                Optional.of(Alliance.Blue), blueCommand,
                Optional.empty(), neitherCommand);

        return Commands.select(options, DriverStation::getAlliance);
    }

    public static Command command(Command blueCommand, Command redCommand) {
        var neitherCommand = Commands.none();
        neitherCommand.addRequirements(blueCommand.getRequirements());
        var options = Map.of(
                Optional.of(Alliance.Red), redCommand,
                Optional.of(Alliance.Blue), blueCommand,
                Optional.empty(), neitherCommand);

        return Commands.select(options, DriverStation::getAlliance);
    }

    public static <T> Supplier<T> constant(T blue, T red) {
        return () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? blue : red;
    }

    public static Supplier<Rotation2d> flipped(Rotation2d blue) {
        return constant(blue, ChoreoAllianceFlipUtil.flip(blue));
    }

    public static Supplier<Translation2d> flipped(Translation2d blue) {
        return constant(blue, ChoreoAllianceFlipUtil.flip(blue));
    }

    public static Supplier<Pose2d> flipped(Pose2d blue) {
        return constant(blue, ChoreoAllianceFlipUtil.flip(blue));
    }

    /** Flips an axis-aligned Rectangle2d to its red-alliance mirror. */
    public static Rectangle2d flipRectangle(Rectangle2d blue) {
        Translation2d center = blue.getCenter().getTranslation();
        double halfX = blue.getXWidth() / 2.0;
        double halfY = blue.getYWidth() / 2.0;
        Translation2d cornerA = new Translation2d(center.getX() - halfX, center.getY() - halfY);
        Translation2d cornerB = new Translation2d(center.getX() + halfX, center.getY() + halfY);
        return new Rectangle2d(ChoreoAllianceFlipUtil.flip(cornerA), ChoreoAllianceFlipUtil.flip(cornerB));
    }

    public static Supplier<Rectangle2d> flipped(Rectangle2d blue) {
        return constant(blue, flipRectangle(blue));
    }

    public static Command flippedCommand(Function<Pose2d, Command> supplier, Pose2d blue) {
        return command(supplier.apply(blue), supplier.apply(ChoreoAllianceFlipUtil.flip(blue)));
    }

    public static Command flippedCommand(BiFunction<Pose2d, Rotation2d, Command> supplier, Pose2d bluePose,
            Rotation2d blueRotation) {
        return command(supplier.apply(bluePose, blueRotation),
                supplier.apply(ChoreoAllianceFlipUtil.flip(bluePose), ChoreoAllianceFlipUtil.flip(blueRotation)));
    }
    // JS: implement more helpers as needed.
}
