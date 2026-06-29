import edu.wpi.first.util.struct.parser.ParseException;
import frc.robot.subsystems.auto.BLine.BLineLogic;
import frc.robot.util.UnitTestHelpers;
import java.io.File;
import java.util.HashMap;
import java.util.List;
import org.junit.jupiter.api.Test;

public class JSONComparisionTest {
  private final int MINIMUM_FILE_COUNT = 2;

  @Test
  public void compareJSONContents()
      throws ParseException, ParseException, org.json.simple.parser.ParseException {
    BLineLogic.unitTestInit();
    File deployDir = new File(UnitTestHelpers.PATHS_PATH);

    if (!deployDir.exists() || !deployDir.isDirectory()) {
      System.out.println("WARNING: Deploy directory not found");
      return;
    }

    List<File> allJsonFiles = UnitTestHelpers.getAllJsonFiles(deployDir);

    if (allJsonFiles.size() < MINIMUM_FILE_COUNT) {
      System.out.println("WARNING: Not enough files to compare");
      return;
    }

    HashMap<Integer, File> seenHashes = new HashMap<>();

    for (File file : allJsonFiles) {
      Object parsedFile = UnitTestHelpers.JSONtoObject(file);

      int hash = parsedFile.hashCode();

      if (!seenHashes.containsKey(hash)) {

        seenHashes.put(hash, file);
      } else {
        System.out.println(
            seenHashes.get(UnitTestHelpers.JSONtoObject(file).hashCode()).getName()
                + " has duplicate file contents with "
                + file.getName());
      }
    }
  }
}
