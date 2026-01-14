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
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SubsystemConstants;
import frc.robot.subsystems.auto.AutoBuilderConfig;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.auto.AutonomousField;


public class Robot extends TimedRobot {
  /** Singleton Stuff */
  private static Robot instance = null;

  public static Robot getInstance() {
    if (instance == null) instance = new Robot();
    return instance;
  }

  
  public final Subsystems subsystems;

 private Controls controls;
  protected Robot() {
    // non public for singleton. Protected so test class can subclass

    instance = this;
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

      AutonomousField.initShuffleBoard("Field", 0, 0, this::addPeriodic);
      AutoLogic.initShuffleBoard();
      CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
      
    }
    
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    // on the end of diabling, make sure all of the motors are set to break and wont move upon
    // enabling
    if (subsystems.drivebaseSubsystem != null) {
      subsystems.drivebaseSubsystem.brakeMotors();
    }
 
  
  }

  @Override
  public void autonomousInit() {
    Shuffleboard.startRecording();
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
    Shuffleboard.startRecording();
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
