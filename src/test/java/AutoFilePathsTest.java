import frc.robot.subsystems.auto.BLine.BLineLogic;
import frc.robot.subsystems.auto.BLine.BLinePath;
import frc.robot.util.UnitTestHelpers;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class AutoFilePathsTest {
  @Test
  public void validatePathFiles() {
    BLineLogic.unitTestInit();
    File[] files = UnitTestHelpers.getAutosFiles();
    if (files == null) return;

    List<String> validFileNames = UnitTestHelpers.getValidFileNames(files);
    List<String> invalidPaths = new ArrayList<>();

    for (BLinePath auto : BLineLogic.getBLinePaths()) {
      for (String pathName : auto.getDisplayingNames()) {
        if (!validFileNames.contains(pathName)) {
          invalidPaths.add(
              "Auto '" + auto.getDisplayName() + "' references missing path: " + pathName);
        }
      }
    }

    if (!invalidPaths.isEmpty()) {
      System.out.println("WARNING: Autos reference invalid path files:");
      for (String invalid : invalidPaths) {
        System.out.println("  - " + invalid);
      }
    } else {
      System.out.println("All autos only reference valid path files");
    }
  }
}
