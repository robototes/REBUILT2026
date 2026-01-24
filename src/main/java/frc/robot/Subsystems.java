package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean INTAKEPIVOT_ENABLED = true;
    public static final boolean INTAKEROLLERS_ENABLED = true;
  }

  // Subsystems go here
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final IntakeSubsystem intakeSubsystem;

  public Subsystems(Mechanism2d mech) {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {

      drivebaseSubsystem = CompTunerConstants.createDrivetrain();
    } else {
      drivebaseSubsystem = null;
    }
    if (INTAKEPIVOT_ENABLED && INTAKEROLLERS_ENABLED) {
      intakeSubsystem = new IntakeSubsystem(mech, INTAKEPIVOT_ENABLED, INTAKEROLLERS_ENABLED);
      SmartDashboard.putData(intakeSubsystem);
    } else {
       intakeSubsystem = null;
    }
  }
}
