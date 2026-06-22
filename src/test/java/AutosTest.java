import static org.junit.jupiter.api.Assertions.assertNotNull;

import frc.robot.subsystems.auto.BLine.BLineLogic;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AutosTest {

  @BeforeEach
  public void setup() {
    BLineLogic.unitTestInit();
  }

  @Test
  public void unusedFileNames() {
    File subfolder = new File("src/main/deploy/autos/paths");
    assertNotNull(subfolder, "This folder should not be null.");

    if (!subfolder.exists() || !subfolder.isDirectory()) {
      System.out.println(
          "⚠️  WARNING: Autos folder does not exist at: " + subfolder.getAbsolutePath());
      return;
    }

    File[] files = subfolder.listFiles();
    if (files == null || files.length == 0) {
      System.out.println("⚠️  WARNING: Autos folder is empty");
      return;
    }

    List<String> validPathNames = BLineLogic.getBLinePathsNames();
    List<String> unusedFiles = new ArrayList<>();

    for (File file : files) {
      String filename = file.getName().split("\\.")[0];
      if (!validPathNames.contains(filename)) {
        unusedFiles.add(file.getName());
      }
    }

    if (!unusedFiles.isEmpty()) {
      System.out.println("⚠️  WARNING: Unused BLine Paths found:");
      for (String file : unusedFiles) {
        System.out.println("  - " + file);
      }
    } else {
      System.out.println("✓ All files in 'autos' folder are used");
    }
  }
}
