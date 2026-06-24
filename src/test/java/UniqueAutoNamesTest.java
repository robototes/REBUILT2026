import frc.robot.util.UnitTestHelpers;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class UniqueAutoNamesTest {
  @Test
  public void validateUniqueAutoNames() {
    File[] files = UnitTestHelpers.getAutosFiles();
    if (files == null) return;

    List<String> invalidNames = new ArrayList<>();

    for (File file : files) {
      String filename = UnitTestHelpers.getFilenamWithoutExtension(file);

      if (filename.startsWith(" ")) {
        invalidNames.add(file.getName() + " (contains leading spaces)");
      }
      if (filename.endsWith(" ")) {
        invalidNames.add(file.getName() + " (contains trailing spaces)");
      }

      if (filename.matches(".*[0-9]$")) {
        invalidNames.add(file.getName() + " (ends with number)");
      }
    }

    if (!invalidNames.isEmpty()) {
      System.out.println("WARNING: Invalid auto names:");
      for (String name : invalidNames) {
        System.out.println("  - " + name);
      }
    } else {
      System.out.println("All auto names are valid");
    }
  }
}
