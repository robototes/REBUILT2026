import frc.robot.subsystems.auto.BLine.BLineFactory;
import frc.robot.subsystems.auto.BLine.BLinePathBase;
import frc.robot.subsystems.auto.BLine.BLineSystem;
import frc.robot.util.UnitTestHelpers;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class AutoFilePathsTest {
  private final BLineSystem bLineSystem = BLineFactory.create(BLineFactory.Mode.NEW);
  private final BLineSystem bLineSystem2 = BLineFactory.create(BLineFactory.Mode.LEGACY);

  public void validatePathFiles(BLineSystem system) {

    UnitTestHelpers.setup();

    File[] files = UnitTestHelpers.getAutosFiles();
    if (files == null) return;

    List<String> validFileNames = UnitTestHelpers.getValidFileNames(files);
    List<String> invalidPaths = new ArrayList<>();

    for (BLinePathBase auto : system.getBLinePaths()) {
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

  @Test
  public void testing() {
    validatePathFiles(bLineSystem);
    validatePathFiles(bLineSystem2);
  }
}
