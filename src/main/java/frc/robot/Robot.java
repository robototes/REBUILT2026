// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SubsystemConstants;
import frc.robot.sim.SimWrapper;
import frc.robot.subsystems.auto.AutoBuilderConfig;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.auto.AutonomousField;
import frc.robot.util.FuelSim;
import frc.robot.util.LimelightHelpers;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private final Controls controls;
  public final Subsystems subsystems;
  private final PowerDistribution PDH;
  private final int APRILTAG_PIPELINE = 0;
  private final int VIEWFINDER_PIPELINE = 1;
  private final int GAMEPIECE_PIPELINE = 2;
  private FuelSim fuelSimulation;
  private final Mechanism2d mechanismRobot;
  public final SimWrapper m_simWrapper;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  protected Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    mechanismRobot = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(24));
    SmartDashboard.putData("Mechanism2d", mechanismRobot);
    subsystems = new Subsystems(mechanismRobot);
    controls = new Controls(subsystems);

    // $VISIONSIM - Wrapper for sim features
    if (Robot.isSimulation()) {
      m_simWrapper = new SimWrapper(subsystems.drivebaseSubsystem, this::resetRobotPose);
    } else {
      m_simWrapper = null;
    }

    if (DRIVEBASE_ENABLED) {
      AutoBuilderConfig.buildAuto(subsystems.drivebaseSubsystem);
    }
    AutoLogic.init(subsystems);
    if (Robot.isSimulation()) {
      fuelSimulation = FuelSim.getInstance();
      fuelSimulation.spawnStartingFuel();

      fuelSimulation.registerRobot(
          0.8,
          0.8,
          0.7,
          () -> subsystems.drivebaseSubsystem.getState().Pose,
          () -> subsystems.drivebaseSubsystem.getState().Speeds);
      fuelSimulation.registerIntake(0.4, 0.8, 0.4, 0.8, () -> true);

      fuelSimulation.start();
    }
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> System.out.println("Command initialized: " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (command, interruptor) ->
                System.out.println(
                    "Command interrupted: "
                        + command.getName()
                        + "; Cause: "
                        + interruptor.map(cmd -> cmd.getName()).orElse("<none>")));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));

    SmartDashboard.putData(CommandScheduler.getInstance());

    if (SubsystemConstants.DRIVEBASE_ENABLED) {
      AutoLogic.registerCommands();
      AutonomousField.initSmartDashBoard(() -> "Field", 0, 0, this::addPeriodic);

      AutoLogic.initSmartDashBoard();
      CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    PDH = new PowerDistribution(Hardware.PDH_ID, PowerDistribution.ModuleType.kRev);
    LiveWindow.disableAllTelemetry();
    LiveWindow.enableTelemetry(PDH);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if (subsystems.visionSubsystem != null) {
      if (!subsystems.visionSubsystem.isViewFinder()) {
        subsystems.visionSubsystem.update();
      }
    }
    if (subsystems.detectionSubsystem != null) {
      if (!subsystems.detectionSubsystem.isViewFinder()) {
        subsystems.detectionSubsystem.update();
      }
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (subsystems.visionSubsystem != null) {
      // ViewFinder Pipeline Switch to reduce Limelight heat
      LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_C, VIEWFINDER_PIPELINE);
    }
    if (subsystems.detectionSubsystem != null) {
      subsystems.detectionSubsystem.fuelPose3d = null;
      // ViewFinder Pipeline Switch to reduce Limelight heat
      LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_A, VIEWFINDER_PIPELINE);
    }
  }

  @Override
  public void disabledExit() {
    if (subsystems.visionSubsystem != null) {
      LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_C, APRILTAG_PIPELINE);
    }
    if (subsystems.detectionSubsystem != null) {
      // ViewFinder Pipeline Switch to reduce Limelight heat
      LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_A, GAMEPIECE_PIPELINE);
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (AutoLogic.getSelectedAuto() != null) {
      if (Robot.isSimulation()) {

        fuelSimulation.clearFuel();
        fuelSimulation.spawnStartingFuel();
      }

      CommandScheduler.getInstance().schedule(AutoLogic.getSelectedAuto());
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    subsystems.hood.zero();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    FuelSim.getInstance().updateSim();
  }

  /** Only used in simulation to reset robot pose */
  private void resetRobotPose(Pose2d pose) {
    if (Robot.isSimulation()) {
      System.out.println("Robot pose reset to: " + pose);

      subsystems.drivebaseSubsystem.resetPose(pose);

      // $VISIONSIM - Clean reset
      m_simWrapper.resetSimPose(pose);
    }
  }
}
