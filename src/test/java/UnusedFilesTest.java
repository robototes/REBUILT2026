import frc.robot.subsystems.auto.BLine.BLineLogic;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class UnusedFilesTest {
  @Test
  public void unusedFileNames() {
    File[] files = TestConstantsUtils.getAutosFiles();
    if (files == null) return;

    List<String> validPathNames = BLineLogic.getBLinePathsNames();
    List<String> unusedFiles = new ArrayList<>();

    for (File file : files) {
      String filename = TestConstantsUtils.getFilenamWithoutExtension(file);
      if (!validPathNames.contains(filename)) {
        unusedFiles.add(file.getName());
      }
    }

    if (!unusedFiles.isEmpty()) {
      System.out.println("WARNING: Unused BLine Paths found:");
      for (String file : unusedFiles) {
        System.out.println("  - " + file);
      }
    } else {
      System.out.println("All files in 'autos' folder are used");
    }
  }
}
