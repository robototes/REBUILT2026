package frc.robot.subsystems.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.parser.ParseException;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class PathPlannerAutos {
  private static List<PathPlannerTrajectory> pathsToTrajectories(
      List<PathPlannerPath> paths, Rotation2d startingRotation) throws IOException, ParseException {
    if (paths.isEmpty()) {
      return List.of();
    }
    RobotConfig robotConfig = RobotConfig.fromGUISettings();
    ChassisVelocities startingSpeeds = new ChassisVelocities();
    List<PathPlannerTrajectory> trajectories = new ArrayList<>(paths.size());
    for (var path : paths) {
      PathPlannerTrajectory trajectory =
          path.generateTrajectory(startingSpeeds, startingRotation, robotConfig);
      trajectories.add(trajectory);
      startingRotation = trajectory.getEndState().pose.getRotation();
      startingSpeeds = trajectory.getEndState().fieldSpeeds;
    }
    return List.copyOf(trajectories);
  }

  private final PathPlannerAuto auto;
  private final double autoDuration;
  private final Pose2d startingPose;
  private final List<PathPlannerTrajectory> trajectories;

  public PathPlannerAutos(String autoName) {
    auto = new PathPlannerAuto(autoName);

    Pose2d startingPose = auto.getStartingPose();
    if (startingPose == null) {
      startingPose = Pose2d.ZERO;
    }
    this.startingPose = startingPose;
    List<PathPlannerTrajectory> trajectories;
    try {
      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      if (startingPose != null) {
        trajectories = pathsToTrajectories(paths, startingPose.getRotation());
      } else {
        trajectories = pathsToTrajectories(paths, null);
      }
    } catch (IOException | ParseException e) {
      trajectories = List.of();
    }
    this.trajectories = trajectories;
    double totalTime = 0;
    for (PathPlannerTrajectory traj : this.trajectories) {
      totalTime += traj.getTotalTimeSeconds();
    }
    autoDuration = totalTime;
  }

  public List<PathPlannerTrajectory> getTrajectories() {
    return this.trajectories;
  }

  public Pose2d getStartingPose() {
    return this.startingPose;
  }

  public double getRunTime() {
    return autoDuration;
  }

  private static final Map<String, List<PathPlannerPath>> namedCommandPathsCache = new HashMap<>();

  public static void registerAuto(String autoName, List<PathPlannerPath> paths) {
    namedCommandPathsCache.put(autoName, paths);
  }
}
