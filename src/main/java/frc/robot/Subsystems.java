package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FEEDER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FLYWHEELS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.GAMEPIECE_DETECTION_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.HOOD_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.LEDS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.SPINDEXER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.VISION_ENABLED;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.DetectionSubsystem;
import frc.robot.subsystems.DrivebaseWrapper;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Launcher.Flywheels;
import frc.robot.subsystems.Launcher.Hood;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean LEDS_ENABLED = true;
    public static final boolean SPINDEXER_ENABLED = true;
    public static final boolean FEEDER_ENABLED = true;
    public static final boolean FLYWHEELS_ENABLED = true;
    public static final boolean HOOD_ENABLED = true;
    public static final boolean GAMEPIECE_DETECTION_ENABLED = true;
  }

  // Subsystems go here
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final DrivebaseWrapper drivebaseWrapper;
  public final VisionSubsystem visionSubsystem;
  public final LEDSubsystem ledSubsystem;
  public final Flywheels flywheels;
  public final Hood hood;
  public final DetectionSubsystem detectionSubsystem;
  public final SpindexerSubsystem spindexerSubsystem;
  public final FeederSubsystem feederSubsystem;

  public Subsystems(Mechanism2d mechanism2d) {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {

      drivebaseSubsystem = CompTunerConstants.createDrivetrain();
      drivebaseWrapper = new DrivebaseWrapper(drivebaseSubsystem);
    } else {
      drivebaseSubsystem = null;
      drivebaseWrapper = new DrivebaseWrapper();
    }

    if (VISION_ENABLED) {
      visionSubsystem = new VisionSubsystem(drivebaseWrapper);
      SmartDashboard.putData(visionSubsystem);
    } else {
      visionSubsystem = null;
    }

    if (LEDS_ENABLED) {
      ledSubsystem = new LEDSubsystem();
    } else {
      ledSubsystem = null;
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
    if (GAMEPIECE_DETECTION_ENABLED) {
      detectionSubsystem = new DetectionSubsystem(drivebaseSubsystem);
    } else {
      detectionSubsystem = null;
    }
    if (SPINDEXER_ENABLED) {
      spindexerSubsystem = new SpindexerSubsystem();
    } else {
      spindexerSubsystem = null;
    }

    if (FEEDER_ENABLED) {
      feederSubsystem = new FeederSubsystem();
    } else {
      feederSubsystem = null;
    }
  }
}
