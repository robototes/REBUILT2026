package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FEEDER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FLYWHEELS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.HOOD_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INDEXER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKE_ARM_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKE_ROLLERS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.LAUNCHER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.LEDS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.SPINDEXER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.TURRET_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.VISION_ENABLED;
import static robotutils.simlimelightproducer.VisionSimConstants.Vision.kSimCameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.sensors.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.index.Feeder;
import frc.robot.subsystems.index.IndexerSubsystem;
import frc.robot.subsystems.index.Spindexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.Flywheels;
import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.launcher.TurretSubsystem;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.robotType.RobotTypesEnum;
import java.util.Optional;
import robotutils.pub.RobotUtilsFactory;
import robotutils.pub.interfaces.FaultyDriveManagerInterface;
import robotutils.pub.interfaces.GroundTruthSimInterface;
import robotutils.pub.interfaces.SimLimelightProducerInterface;
import robotutils.pub.interfaces.dashboard.DashboardConstants;
import robotutils.pub.interfaces.dashboard.DashboardManagerInterface;
import robotutils.pub.interfaces.dashboard.Field2dMultipleObjectRenderer;
import robotutils.pub.interfaces.dashboard.Field2dObjectRenderer;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean INTAKE_ROLLERS_ENABLED = true;
    public static final boolean INTAKE_ARM_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean SPINDEXER_ENABLED = true;
    public static final boolean FEEDER_ENABLED = true;
    public static final boolean FLYWHEELS_ENABLED = true;
    public static final boolean HOOD_ENABLED = true;
    public static final boolean TURRET_ENABLED = true;
    public static final boolean INTAKE_ENABLED = INTAKE_ARM_ENABLED && INTAKE_ROLLERS_ENABLED;
    public static final boolean LAUNCHER_ENABLED = HOOD_ENABLED && FLYWHEELS_ENABLED;
    public static final boolean INDEXER_ENABLED = SPINDEXER_ENABLED && FEEDER_ENABLED;
    public static final boolean LEDS_ENABLED = true;
  }

  // Subsystems go here
  private final RobotUtilsFactory robotUtilsFactory = new RobotUtilsFactory();
  public final DashboardManagerInterface dashboardManager;
  public final GroundTruthSimInterface groundTruthSim;
  public final SimLimelightProducerInterface simLimelightProducer;
  public final FaultyDriveManagerInterface faultyDriveManager;
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final LauncherSubsystem launcherSubsystem;
  public final VisionSubsystem visionSubsystem;
  public final Flywheels flywheels;
  public final Hood hood;
  public final Spindexer spindexerSubsystem;
  public final Feeder feederSubsystem;
  public final IntakeRollers intakeRollers;
  public final IntakePivot intakePivot;
  public final IntakeSubsystem intakeSubsystem;
  public final TurretSubsystem turretSubsystem;
  public final IndexerSubsystem indexerSubsystem;
  public final LEDSubsystem ledSubsystem;

  public Subsystems(Mechanism2d mechanism2d) {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?

    // Pattern is

    // if (SUBSYSTEM_ENABLED){
    //   subsystem = new SubsystemName();
    // } else {
    //   subsystem = null;
    // }

    if (DRIVEBASE_ENABLED) {
      drivebaseSubsystem =
          (RobotType.TYPE == RobotTypesEnum.ALPHA)
              ? AlphaTunerConstants.createDrivetrain()
              : CompTunerConstants.createDrivetrain();
    } else {
      drivebaseSubsystem = null;
    }

    dashboardManager = robotUtilsFactory.createDashboardManager();

    if (DRIVEBASE_ENABLED) {
      groundTruthSim =
          robotUtilsFactory.createGroundTruthSim(
              Optional.of(dashboardManager), drivebaseSubsystem, this::resetRobotPose);
      if (groundTruthSim != null) {
        groundTruthSim.resetGroundTruthPoseForSim(drivebaseSubsystem.getState().Pose);
        drivebaseSubsystem.setHighFreqSimCallback(groundTruthSim::updateGroundTruthPose);
      }

      simLimelightProducer = robotUtilsFactory.createSimLimelightProducer(kSimCameras);
      if (RobotBase.isSimulation() && groundTruthSim != null && simLimelightProducer != null) {
        faultyDriveManager =
            robotUtilsFactory.createFaultyDriveManager(groundTruthSim, simLimelightProducer);
      } else {
        faultyDriveManager = null;
      }
      if (RobotBase.isSimulation() && simLimelightProducer != null) {
        Field2d simDebugField = simLimelightProducer.getSimDebugField();
        dashboardManager.addCustomRenderer(
            new Field2dObjectRenderer(simDebugField, DashboardConstants.kEstimatedPoseItemName),
            DashboardConstants.kGroundTruthProviderName,
            DashboardConstants.kEstimatedPoseItemName);
        dashboardManager.addCustomRenderer(
            new Field2dMultipleObjectRenderer(
                simDebugField, DashboardConstants.kEstimatedPoseModules, 4),
            DashboardConstants.kGroundTruthProviderName,
            DashboardConstants.kEstimatedPoseModules);
      }
    } else {
      groundTruthSim = null;
      simLimelightProducer = null;
      faultyDriveManager = null;
    }

    if (INTAKE_ROLLERS_ENABLED) {
      intakeRollers = new IntakeRollers();
    } else {
      intakeRollers = null;
    }

    if (INTAKE_ARM_ENABLED) {
      intakePivot = new IntakePivot();
    } else {
      intakePivot = null;
    }

    if (FLYWHEELS_ENABLED) {
      flywheels = new Flywheels();
    } else {
      flywheels = null;
    }

    if (HOOD_ENABLED) {
      hood = new Hood();
    } else {
      hood = null;
    }

    if (SPINDEXER_ENABLED) {
      spindexerSubsystem = new Spindexer();
    } else {
      spindexerSubsystem = null;
    }

    if (FEEDER_ENABLED) {
      feederSubsystem = new Feeder();
    } else {
      feederSubsystem = null;
    }

    if (INTAKE_ENABLED) {
      intakeSubsystem = new IntakeSubsystem(intakePivot, intakeRollers);
    } else {
      intakeSubsystem = null;
    }

    if (TURRET_ENABLED) {
      turretSubsystem = new TurretSubsystem(drivebaseSubsystem);
    } else {
      turretSubsystem = null;
    }

    if (LAUNCHER_ENABLED) {
      launcherSubsystem = new LauncherSubsystem(this);
    } else {
      launcherSubsystem = null;
    }

    if (INDEXER_ENABLED) {
      indexerSubsystem = new IndexerSubsystem(feederSubsystem, spindexerSubsystem);
    } else {
      indexerSubsystem = null;
    }

    if (LEDS_ENABLED) {
      ledSubsystem = new LEDSubsystem();
    } else {
      ledSubsystem = null;
    }

    if (VISION_ENABLED && DRIVEBASE_ENABLED && INTAKE_ARM_ENABLED) {
      visionSubsystem = new VisionSubsystem(drivebaseSubsystem, intakePivot);
      SmartDashboard.putData(visionSubsystem);
    } else {
      visionSubsystem = null;
    }
  }

  public void resetRobotPose(Pose2d pose) {
    if (drivebaseSubsystem == null) {
      return;
    }
    drivebaseSubsystem.resetPose(pose);
    if (groundTruthSim != null) {
      groundTruthSim.resetGroundTruthPoseForSim(pose);
    }
    if (simLimelightProducer != null) {
      simLimelightProducer.resetSimPose(pose);
    }
  }

  public void resetAllAutoSimFaults() {
    if (faultyDriveManager != null) {
      faultyDriveManager.resetAllAutoSimFaults();
    }
  }
}
