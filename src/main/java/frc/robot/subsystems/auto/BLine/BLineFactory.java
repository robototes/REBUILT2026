package frc.robot.subsystems.auto.BLine;

public final class BLineFactory {

  public enum Mode {
    NEW,
    LEGACY
  }

  private BLineFactory() {}

  public static BLineSystem create(Mode mode) {
    return switch (mode) {
      case NEW -> new NewBLineSystem();
      case LEGACY -> new LegacyBLineSystem();
    };
  }
}
