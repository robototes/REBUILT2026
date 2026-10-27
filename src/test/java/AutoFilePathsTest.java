import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.Autos;
import frc.robot.util.UnitTestHelpers;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class AutoFilePathsTest {
  private static final UnitTestHelpers helper = new UnitTestHelpers();
  //TODO Fix the auto.toString() probably and getPaths initialization handling for unit testing

  @Test
  public void validatePathFiles() {
    helper.setup();
    File[] files = UnitTestHelpers.getAutosFiles();
    if (files == null) return;

    List<String> validFileNames = UnitTestHelpers.getValidFileNames(files);
    List<String> invalidPaths = new ArrayList<>();

    for (Path auto : Autos.getPaths()) {

        if (!validFileNames.contains(auto.toString())) {
          invalidPaths.add(
              "Auto '" + auto.toString() + "' references missing path: " + auto.toString());
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
