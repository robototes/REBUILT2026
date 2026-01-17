// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SubsystemConstants;
import frc.robot.subsystems.auto.AutoBuilderConfig;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.auto.AutonomousField;
import frc.robot.util.LimelightHelpers;

public class Robot extends TimedRobot {

  private final Controls controls;
  public final Subsystems subsystems;
  private final PowerDistribution PDH;
  private final int APRILTAG_PIPELINE = 0;
  private final int VIEWFINDER_PIPELINE = 1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  protected Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    subsystems = new Subsystems();

    CanBridge.runTCP();

    controls = new Controls(subsystems);

    AutoBuilderConfig.buildAuto(subsystems.drivebaseSubsystem);

    if (RobotBase.isReal()) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog(), true);
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
    CommandScheduler.getInstance().run();
    if (subsystems.visionSubsystem != null) {
      // ViewFinder Pipeline Switch to reduce Limelight heat
      subsystems.visionSubsystem.update();
    }
  }

  @Override
  public void disabledInit() {
    if (subsystems.visionSubsystem != null) {
      // ViewFinder Pipeline Switch to reduce Limelight heat
      LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_B, VIEWFINDER_PIPELINE);
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    // on the end of diabling, make sure all of the motors are set to break and wont move upon
    // enabling
    if (subsystems.drivebaseSubsystem != null) {
      if (Robot.isSimulation()) {

      } else {
        if (subsystems.visionSubsystem != null) {
          LimelightHelpers.setPipelineIndex(Hardware.LIMELIGHT_B, APRILTAG_PIPELINE);
        }

        subsystems.drivebaseSubsystem.brakeMotors();
      }
    }
  }

  @Override
  public void autonomousInit() {

    if (SubsystemConstants.DRIVEBASE_ENABLED && AutoLogic.getSelectedAuto() != null) {
      CommandScheduler.getInstance().schedule(AutoLogic.getSelectedAuto());
    }
  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    ;
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
