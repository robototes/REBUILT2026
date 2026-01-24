// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  public final Controls controls;
  public final Subsystems subsystems;
  private final PowerDistribution PDH;
  private final int APRILTAG_PIPELINE = 0;
  private final int VIEWFINDER_PIPELINE = 1;
  private final int GAMEPIECE_PIPELINE = 2;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  protected Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    subsystems = new Subsystems();
    controls = new Controls(subsystems);

    PDH = new PowerDistribution(Hardware.PDH_ID, ModuleType.kRev);
    LiveWindow.disableAllTelemetry();
    LiveWindow.enableTelemetry(PDH);
  }

  private static Robot instance = null;

  public static Robot getInstance() {
    if (instance == null) instance = new Robot();
    return instance;
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
      // ViewFinder Pipeline Switch to reduce Limelight heat
      subsystems.visionSubsystem.update();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (subsystems.visionSubsystem != null) {
      // ViewFinder Pipeline Switch to reduce Limelight heat
      LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_B, VIEWFINDER_PIPELINE);
    }
    if (subsystems.detectionSubsystem != null) {
      // ViewFinder Pipeline Switch to reduce Limelight heat
      LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_A, VIEWFINDER_PIPELINE);
    }
  }

  @Override
  public void disabledExit() {
    if (subsystems.visionSubsystem != null) {
      LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_B, APRILTAG_PIPELINE);
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
  public void autonomousInit() {}

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
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
