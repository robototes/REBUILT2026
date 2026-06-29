import edu.wpi.first.util.struct.parser.ParseException;
import frc.robot.subsystems.auto.BLine.BLineLogic;
import frc.robot.util.UnitTestHelpers;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.junit.jupiter.api.Test;

public class JSONComparisionTest {
  private final int MINIMUM_FILE_COUNT = 2;

  @SuppressWarnings("unlikely-arg-type")
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
    HashMap<String, File> contents = new HashMap<>();
    HashMap<String, File> filesWithDuplicateContents = new HashMap<>();

    if (allJsonFiles.size() < MINIMUM_FILE_COUNT) {
      System.out.println("WARNING: Not enough files to compare");
      return;
    }

    HashMap<Integer, File> seenHashes = new HashMap<>();

    for (File file : allJsonFiles) {
      Object parsedFile = UnitTestHelpers.JSONtoObject(file);

      int hash = parsedFile.hashCode();

      if (!seenHashes.containsKey(hash)) {
        contents.put(file.getName(), file);

        seenHashes.put(hash, file);
      } else {

        filesWithDuplicateContents.put(file.getName(), file);
      }
    }
    // STICKLER FOR GRAMMAR
    List<File> iterableContents = new ArrayList<>();
    iterableContents.addAll(filesWithDuplicateContents.values());
    List<File> arrContents = new ArrayList<>();
    arrContents.addAll(contents.values());
    List<Integer> arrHash = new ArrayList<>();
    for (File file : seenHashes.values()) {
      arrHash.add(file.hashCode());
    }
    if (iterableContents.size() < 1) {
      System.out.println("No duplicate file contents");
    } else {

      for (File file : filesWithDuplicateContents.values()) {

        System.out.println(
            seenHashes.get(UnitTestHelpers.JSONtoObject(file).hashCode()).getName()
                + " has duplicate file contents with "
                + file.getName());
      }
    }
  }
}
