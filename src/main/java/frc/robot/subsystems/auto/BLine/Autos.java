package frc.robot.subsystems.auto.BLine;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.util.AllianceFlipUtil;

public class Autos {

    private  Subsystems systems;
    private  AutoChooser autoChooser = new AutoChooser();
    private  Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private  FollowPath.Builder pathBuilder;
    private static List<Path> paths;
    //TODO add back in autonoumsfield
    //TODO add LoggedDashboardChooer for AdvantageKit logging

    // ============= BLINE PATHS =============
    /**
     * BLine trajectory paths for autonomous routines. Each Path references a
     * trajectory file (in lowercase)
     */

    private final Path Depot1Path = new Path("left-center-line-depot".toLowerCase());
    public final Supplier<Pose2d> TRENCH_START_LEFT = AllianceFlipUtil.flipped(Depot1Path.getStartPose());
  //BLine Paths


    // Constructor
    public Autos(){
       initializePaths();

    }

    public void configure(Subsystems s) {

        // ===== BLINE PATH FOLLOWING CONFIGURATION =====
        pathBuilder = new FollowPath.Builder(
            s.drivebaseSubsystem,
            () -> s.drivebaseSubsystem.getState().Pose,
            () -> s.drivebaseSubsystem.getState().Speeds,
            (speeds) ->
                s.drivebaseSubsystem.setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0))
             .withDefaultShouldFlip(); // Auto-flip trajectory when on red alliance

        registerAutos();
    }

    public void init(Subsystems s) {
        this.systems = s;
    }
    public void unitTestInit() {
        this.systems = null;
    }
    public void initializePaths() {
    Depot2Path = new Path("depot-2".toLowerCase());
    Depot3Path = new Path("depot-3".toLowerCase());
    LeftBump1Path = new Path("path-1".toLowerCase());
    LeftBump2Path = new Path("path-2".toLowerCase());
    paths.forEach(null);



    }

    public static List<Path> getPaths() {
        return paths;
    }


    // ============= AUTO REGISTRATION =============
    /** Registers all autonomous routines with the auto chooser. */
    private void registerAutos() {
        //Format as follows: autos.put(name, () -> auto.flipped(), c -> Commands... pathBuilder.build(path))


        // Register all autos with the chooser for driver station selection
        autos.forEach(autoChooser::addCmd);
    }

    /**
     * Runs a path command until it times out or the robot is close to a
     * wall/obstacle but only after a specified event marker has fired in the
     * trajectory.
     *
     * @param path           the BLine path command to follow
     * @param eventKey       the path event marker name that must fire before sensor
     *                       termination is active
     * @param timeoutSeconds maximum time to allow the path to run (safety limit)
     * @return a command that follows the path with sensor-based early termination
     */


    public Command selectedCommand() {
        return autoChooser.selectedCommand();
    }

    public String getSelectedPathName() {
        return selectedCommand().getName();
    }
    public AutoChooser getAutoChooser() {
        return autoChooser;
    }


    // ============= AUTO BUILDER HELPERS =============

    /**
     * Constructs an autonomous routine with a starting pose and a sequence of
     * commands.
     *
     * @param startPose the initial robot pose for odometry reset
     * @param builder   a {@link Consumer} that accepts a
     *                  {@link SequentialCommandGroup}
     *                  to which commands should be added in the desired order
     * @return a {@link Command} that resets odometry and executes the command
     *         sequence
     */
    private Command auto(Pose2d startPose, Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        // reset odometry
        group.addCommands(Commands.runOnce(() -> systems.drivebaseSubsystem.resetPose(startPose), systems.drivebaseSubsystem));
        builder.accept(group);
        return group;
    }

    /**
     * Constructs an autonomous routine with a sequence of commands (no pose reset).
     *
     * @param builder a {@link Consumer} that accepts a
     *                {@link SequentialCommandGroup}
     *                to which commands should be added in the desired order
     * @return a {@link Command} that executes the command sequence
     */
    private Command auto(Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        builder.accept(group);
        return group;
    }
}

