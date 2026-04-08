// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SubsystemConstants;
import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import frc.robot.sensors.LEDSubsystem;
import frc.robot.sim.ShowVisionOnField;
import frc.robot.sim.SimWrapper;
import frc.robot.subsystems.auto.AutoBuilderConfig;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.auto.AutonomousField;
import frc.robot.util.AllianceUtils;
import frc.robot.util.BuildInfo;
import frc.robot.util.DriveStateNtLogger;
import frc.robot.util.DriveStateSignalLogger;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.simulation.RobotSim;
import frc.robot.util.tuning.LauncherConstants;

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
  private final int THROTTLE_ON = 150;
  private final int THROTTLE_OFF = 0;
  private final double MAX_TIME_RECORD = 165;
  private final double LL_IMU_CORRECTION_RATE = 0.1;
  private final RobotSim robotSim;
  private final Mechanism2d mechanismRobot;
  private final SimWrapper m_simWrapper;
  private final double BROWNOUT_VOLTAGE = 6.4; // Limelight's minimum operating voltage is 3.3volts
  private static final double DATA_LOG_FLUSH_PERIOD_S = 1.0 / 14.0; // 14 Hz flush
  private final DriveStateNtLogger driveBaseSim;
  private final DriveStateSignalLogger logger;

  // Cached time for robot.periodic()
  private double LAST_TIME = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  protected Robot() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    // logging
    if (RobotBase.isReal()) {
      DataLogManager.start("", "", DATA_LOG_FLUSH_PERIOD_S);
      DriverStation.startDataLog(DataLogManager.getLog(), true);
    }
    PDH = new PowerDistribution(Hardware.PDH_ID, PowerDistribution.ModuleType.kRev);
    LiveWindow.disableAllTelemetry();
    LiveWindow.enableTelemetry(PDH);
    BuildInfo.logBuildInfo();

    // Set brownout Voltage
    RobotController.setBrownoutVoltage(BROWNOUT_VOLTAGE);

    // Loads the field layout before auto  to prevent any delay
    AllianceUtils.getHubTranslation2d();
    mechanismRobot = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(24));
    SmartDashboard.putData("Mechanism2d", mechanismRobot);
    subsystems = new Subsystems(mechanismRobot);

    // $VISIONSIM - Wrapper for sim features
    if (Robot.isSimulation()) {
      m_simWrapper = new SimWrapper(subsystems.drivebaseSubsystem, this::resetRobotPose);
    } else {
      m_simWrapper = null;
    }

    // $VISIONSIM - Wrapper for sim features
    if (Robot.isSimulation() && m_simWrapper != null) {
      ShowVisionOnField showVisionOnField =
          new ShowVisionOnField(null, m_simWrapper.getSimDebugField());
      if (subsystems.visionSubsystem != null) {
        subsystems.visionSubsystem.setShowVisionOnField(showVisionOnField);
      }
    }

    controls = new Controls(subsystems, m_simWrapper);

    if (DRIVEBASE_ENABLED) {
      AutoBuilderConfig.buildAuto(subsystems.drivebaseSubsystem, false);
    }
    AutoLogic.init(subsystems);
    if (Robot.isSimulation()) {
      robotSim = new RobotSim(subsystems.drivebaseSubsystem);
    } else {
      robotSim = null;
    }
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> DataLogManager.log("Command initialized: " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (command, interruptor) ->
                DataLogManager.log(
                    "Command interrupted: "
                        + command.getName()
                        + "; Cause: "
                        + interruptor.map(cmd -> cmd.getName()).orElse("<none>")));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> DataLogManager.log("Command finished: " + command.getName()));

    SmartDashboard.putData(CommandScheduler.getInstance());

    if (SubsystemConstants.DRIVEBASE_ENABLED) {
      AutoLogic.registerCommands(true);
      AutonomousField.initSmartDashBoard(() -> "Field", 0, 0, this::addPeriodic);

      AutoLogic.initSmartDashBoard();
      CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    logger = new DriveStateSignalLogger();
    subsystems.drivebaseSubsystem.registerTelemetry(logger::telemeterize);
    driveBaseSim = logger.DrivebaseSim(Controls.MaxSpeed);
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
    // Resume logging every X seconds
    double time = Timer.getFPGATimestamp();
    if (time - LAST_TIME >= 1) {
      LAST_TIME = time;
      DataLogManager.getLog().resume();
    }

    // $VISIONSIM - Wrapper for sim features
    if (Robot.isSimulation() && m_simWrapper != null) {
      // NOTE: We run the vision period FIRST in robotPeriodic, since it updates
      // NetworkTables with the limelight data, in-case any code in this loop
      // needs that info and doesnt want it delayed 20ms.
      m_simWrapper.robotPeriodic();
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    if (subsystems.visionSubsystem != null && subsystems.drivebaseSubsystem != null) {
      subsystems.visionSubsystem.update();
    }
    // var robotState = subsystems.drivebaseSubsystem.getState();
    // LauncherConstants.update(robotState.Pose, subsystems.drivebaseSubsystem);
    CommandScheduler.getInstance().run();
    driveBaseSim.update();
    LauncherConstants.UpdateNT(subsystems.drivebaseSubsystem.getState().Pose);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    if (subsystems.visionSubsystem != null) {
      if (subsystems.visionSubsystem.limelightaOnline) {
        setupLimelightForAprilTags(Hardware.LIMELIGHT_A, true);
        LimelightHelpers.setRewindEnabled(Hardware.LIMELIGHT_A, true);
      }
      if (subsystems.visionSubsystem.limelightbOnline) {
        setupLimelightForAprilTags(Hardware.LIMELIGHT_B, true);
        LimelightHelpers.setRewindEnabled(Hardware.LIMELIGHT_B, true);
      }
      if (subsystems.visionSubsystem.limelightcOnline) {
        setupLimelightForAprilTags(Hardware.LIMELIGHT_C, true);
        LimelightHelpers.setRewindEnabled(Hardware.LIMELIGHT_C, true);
      }
    }
    if (subsystems.turretSubsystem != null) {
      subsystems.turretSubsystem.coastTurret();
    }
    CommandScheduler.getInstance()
        .cancelAll(); // Prevent auto commands from persisting past auto or during testing.
  }

  @Override
  public void disabledExit() {
    if (subsystems.visionSubsystem != null) {
      if (subsystems.visionSubsystem.limelightaOnline) {
        setupLimelightForAprilTags(Hardware.LIMELIGHT_A, false);
      }
      if (subsystems.visionSubsystem.limelightbOnline) {
        setupLimelightForAprilTags(Hardware.LIMELIGHT_B, false);
      }
      if (subsystems.visionSubsystem.limelightcOnline) {
        setupLimelightForAprilTags(Hardware.LIMELIGHT_C, false);
      }
    }

    if (subsystems.turretSubsystem != null) {
      subsystems.turretSubsystem.brakeTurret();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // subsystems.ledSubsystem.setMode(LEDSubsystem.LEDMode.RAINBOW);
    if (AutoLogic.getSelectedAuto() != null) {
      if (Robot.isSimulation()) {
        robotSim.resetFuelSim();
      }

      CommandScheduler.getInstance().schedule(AutoLogic.getSelectedAuto());
      double initialYaw = SmartDashboard.getNumber("/Selected auto/Robot/2", 0);
      if (subsystems.visionSubsystem != null) {
        if (subsystems.visionSubsystem.limelightaOnline) {
          supplyRobotYawToLimelight(Hardware.LIMELIGHT_A, initialYaw);
        }
        if (subsystems.visionSubsystem.limelightbOnline) {
          supplyRobotYawToLimelight(Hardware.LIMELIGHT_B, initialYaw);
        }
        if (subsystems.visionSubsystem.limelightcOnline) {
          supplyRobotYawToLimelight(Hardware.LIMELIGHT_C, initialYaw);
        }
      }
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    supplyYawToAllLimelights();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    subsystems.ledSubsystem.setMode(LEDSubsystem.LEDMode.DEFAULT);
    HubShiftUtil.initialize();
  }

  @Override
  public void teleopPeriodic() {
    supplyYawToAllLimelights();
  }

  /** This function is called once when teleop mode is exited. */
  @Override
  public void teleopExit() {
    limelightsRecord();
  }

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
    // $VISIONSIM - Wrapper for sim features
    if (m_simWrapper != null) {
      m_simWrapper.simulationPeriodic();
    }

    robotSim.updateFuelSim();
  }

  private void setupLimelightForAprilTags(String limelightName, boolean isEnteringDisabled) {
    if (isEnteringDisabled) {
      LimelightHelpers.SetIMUAssistAlpha(limelightName, LL_IMU_CORRECTION_RATE);
      // Throttle to reduce heat
      LimelightHelpers.SetThrottle(limelightName, THROTTLE_ON);
      // seed internal limelight imu for mt2
      LimelightHelpers.SetIMUMode(limelightName, 1);
      LimelightHelpers.setPipelineIndex(limelightName, APRILTAG_PIPELINE);

    } else {
      // get rid of throttle to get rid of throttle "glazing"
      LimelightHelpers.SetThrottle(limelightName, THROTTLE_OFF);
      // Limelight Use internal IMU + external IMU
      LimelightHelpers.SetIMUMode(limelightName, 4);
    }
  }

  private void supplyYawToAllLimelights() {
    if (subsystems.visionSubsystem != null && subsystems.drivebaseSubsystem != null) {
      double heading = subsystems.drivebaseSubsystem.getState().Pose.getRotation().getDegrees();
      if (subsystems.visionSubsystem.limelightaOnline) {
        supplyRobotYawToLimelight(Hardware.LIMELIGHT_A, heading);
      }
      if (subsystems.visionSubsystem.limelightbOnline) {
        supplyRobotYawToLimelight(Hardware.LIMELIGHT_B, heading);
      }
      if (subsystems.visionSubsystem.limelightcOnline) {
        supplyRobotYawToLimelight(Hardware.LIMELIGHT_C, heading);
      }
    }
  }

  public void limelightsRecord() {
    if (subsystems.visionSubsystem != null) {
      if (subsystems.visionSubsystem.limelightaOnline) {
        LimelightHelpers.triggerRewindCapture(Hardware.LIMELIGHT_A, MAX_TIME_RECORD);
      }
      if (subsystems.visionSubsystem.limelightbOnline) {
        LimelightHelpers.triggerRewindCapture(Hardware.LIMELIGHT_B, MAX_TIME_RECORD);
      }
      if (subsystems.visionSubsystem.limelightcOnline) {
        LimelightHelpers.triggerRewindCapture(Hardware.LIMELIGHT_C, MAX_TIME_RECORD);
      }
    }
  }

  private void supplyRobotYawToLimelight(String limelightName, double heading) {
    LimelightHelpers.SetRobotOrientation(limelightName, heading, 0, 0, 0, 0, 0);
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
