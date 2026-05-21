// ========================= BLineLogic.java =========================

package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;

import frc.robot.lib.BLine.FollowPath;

import frc.robot.subsystems.intake.IntakeSubsystem.IntakeMode;

import frc.robot.util.simulation.RobotSim;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

public class BLineLogic {

  private static Subsystems s;

  // ========================= START POSITIONS =========================

  public enum StartPosition {

    LEFT_TRENCH(
        "Left Trench",
        new Pose2d(
            4.013,
            7.597,
            Rotation2d.fromDegrees(90))),

    CENTER(
        "Center",
        new Pose2d(
            3.600,
            4.035,
            Rotation2d.fromDegrees(0))),

    RIGHT_TRENCH(
        "Right Trench",
        new Pose2d(
            4.013,
            0.473,
            Rotation2d.fromDegrees(-90))),

    MISC("Misc", null);

    public final String title;
    public final Pose2d startPose;

    StartPosition(String title, Pose2d startPose) {
      this.title = title;
      this.startPose = startPose;
    }
  }

  // ========================= AUTOS =========================

  private static final List<BLinePath> autos = new ArrayList<>();

  // ========================= CHOOSERS =========================

  private static final SendableChooser<StartPosition> startPositionChooser =
      new SendableChooser<>();

  private static final SendableChooser<String> autoChooser =
      new SendableChooser<>();

  private static final SendableChooser<Integer> gameObjects =
      new SendableChooser<>();

  private static final NetworkTableEntry autoDelayEntry =
      NetworkTableInstance.getDefault()
          .getTable("Autos")
          .getEntry("Auto Delay");

  public static final String keys =
      "RB=Right Bump, LB=Left Bump, LT=Left Trench, RT=Right Trench";

  // ========================= PATH BUILDER =========================

  public static FollowPath.Builder pathBuilder;

  private static Command bLineLaunching;

  // ========================= INIT =========================

  public static void init(Subsystems subsystems) {

    s = subsystems;

    registerCommands();

    autos.clear();

    // ========================= DEFINE AUTOS HERE =========================

    autos.add(new BLinePath(
        "Sample 1",
        "sample1"));

    autos.add(new BLinePath(
        "Sample 2",
        "sample2"));

    autos.add(new BLinePath(
        "Sample 3",
        "sample3"));
  }

  // ========================= CONFIGURE =========================

  public static void configure(Subsystems s) {

    pathBuilder =
        new FollowPath.Builder(
                s.drivebaseSubsystem,

                () -> s.drivebaseSubsystem.getState().Pose,

                () -> s.drivebaseSubsystem.getState().Speeds,

                (speeds) ->
                    s.drivebaseSubsystem.setControl(
                        new SwerveRequest.ApplyRobotSpeeds()
                            .withSpeeds(
                                ChassisSpeeds.discretize(
                                    speeds,
                                    0.020))),

                new PIDController(3.0, 0.0, 0.0),

                new PIDController(5.0, 0.0, 0.0),

                new PIDController(2.0, 0.0, 0.0))

            .withDefaultShouldFlip()

            .withPoseReset(
                pose -> s.drivebaseSubsystem.resetPose(pose));
  }

  // ========================= SMARTDASHBOARD =========================

  public static void initSmartDashboard() {

    // starting positions

    startPositionChooser.setDefaultOption(
        StartPosition.MISC.title,
               StartPosition.MISC);

    for (StartPosition pos : StartPosition.values()) {

      startPositionChooser.addOption(
          pos.title,
          pos);
    }

    // game objects chooser

    gameObjects.setDefaultOption("0", 0);

    // auto chooser

    refreshAutoChooser();

    // NT / Elastic publishing

    SmartDashboard.putData(
        "Starting Position",
        startPositionChooser);

    SmartDashboard.putData(
        "Auto Mode",
        gameObjects);

    SmartDashboard.putData(
        "Available Auto Variants",
        autoChooser);

    SmartDashboard.putString(
        "Auto Key",
        keys);

    autoDelayEntry.setDouble(0.0);

    // update chooser when selection changes

    startPositionChooser.onChange(
        v -> refreshAutoChooser());

    // debug

    System.out.println("========== REGISTERED AUTOS ==========");

    for (BLinePath auto : autos) {

      System.out.println(
          auto.getDisplayName()
              + " | "
              + auto.getStartPose2d());
    }

    System.out.println("======================================");
  }

  // ========================= AUTO FILTERING =========================

  private static void refreshAutoChooser() {

    StartPosition selectedStart =
        startPositionChooser.getSelected();

    if (selectedStart == null) {
      selectedStart = StartPosition.MISC;
    }

    boolean first = true;

    for (BLinePath auto : autos) {

      boolean matches =
          selectedStart == StartPosition.MISC
              ||
              auto.getStartPose2d() == selectedStart.startPose;

      if (!matches) {
        continue;
      }

      if (first) {

        autoChooser.setDefaultOption(
            auto.getDisplayName(),
            auto.getDisplayName());

        first = false;

      } else {

        autoChooser.addOption(
            auto.getDisplayName(),
            auto.getDisplayName());
      }
    }

    // safety fallback

    if (first && !autos.isEmpty()) {

      autoChooser.setDefaultOption(
          autos.get(0).getDisplayName(),
          autos.get(0).getDisplayName());
    }
  }

  // ========================= GETTERS =========================

  public static String getSelectedAutoName() {

    return autoChooser.getSelected();
  }

  public static BLinePath getSelectedAutoPath() {

    String selectedName =
        autoChooser.getSelected();

    if (selectedName == null) {
      return null;
    }

    for (BLinePath auto : autos) {

      if (auto.getDisplayName().equals(selectedName)) {
        return auto;
      }
    }

    return null;
  }

  public static Pose2d getSelectedAutoStartingPose() {

    BLinePath selected =
        getSelectedAutoPath();

    if (selected == null) {
      return Pose2d.kZero;
    }

    return selected.getStartPose2d();
  }

  public static Command getSelectedAuto() {

    double delay =
        autoDelayEntry.getDouble(0.0);

    BLinePath selected =
        getSelectedAutoPath();

    if (selected == null) {

      System.out.println(
          "NO AUTO SELECTED");

      return Commands.none();
    }

    System.out.println(
        "RUNNING AUTO: "
            + selected.getDisplayName());

    return Commands.waitSeconds(delay)

        .andThen(
            AutoBuilder.buildAuto(
                selected.getAutoName()))

        .withName(
            selected.getDisplayName());
  }

  // ========================= PATHPLANNER DIRECT =========================



  // ========================= COMMANDS =========================

  public static Command intakeCommand() {

    return Commands.runOnce(
            () ->
                Controls.intakeMode =
                    IntakeMode.INTAKE)

        .withName("Auto Intake Command");
  }

  public static Command climbCommand() {

    return Commands.none()
        .withName("Auto Climb Command");
  }

  public static void cancelCommand() {

    CommandScheduler.getInstance()
        .cancel(bLineLaunching);
  }

  // ========================= EVENT TRIGGERS =========================

  private static void registerCommands() {

    if (s.launcherSubsystem != null
        && s.indexerSubsystem != null) {

      if (Robot.isSimulation()) {

        bLineLaunching =
            RobotSim.launch(s,1);

        FollowPath.registerEventTrigger(
            "launch",

            bLineLaunching.andThen(
                Commands.print(
                    "LAUNCH FINISHED")));

      } else {

        bLineLaunching =
            Commands.none();

        FollowPath.registerEventTrigger(
            "launch",
            bLineLaunching);
      }
    }

    FollowPath.registerEventTrigger(
        "intake",
        intakeCommand());

    FollowPath.registerEventTrigger(
        "climb",
        climbCommand());
  }
}

