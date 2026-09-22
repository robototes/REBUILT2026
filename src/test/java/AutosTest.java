import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.auto.AutoLogic;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class AutosTest {
  @Test
  void validateFileName() {
    Path autosDirectory = Path.of("src", "main", "deploy", "pathplanner", "autos");

    assertTrue(
        !AutoLogic.getConfiguredAutoNames().isEmpty(), "No auto paths are configured in AutoLogic");

    for (String autoName : AutoLogic.getConfiguredAutoNames()) {
      assertTrue(
          Files.isRegularFile(autosDirectory.resolve(autoName + ".auto")),
          "No auto file matches " + autoName);
    }
  }
}
