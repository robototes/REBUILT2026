import frc.robot.subsystems.auto.BLine.BLineLogic;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;

public class TestConstantsUtils {
  public static final String PATHS_PATH = "src/main/deploy/autos/paths";

  @BeforeEach
  public void setup() {
    BLineLogic.unitTestInit();
  }

  // ==================== METHODS ====================
  public static File[] getAutosFiles() {
    File subfolder = new File(PATHS_PATH);

    if (!subfolder.exists() || !subfolder.isDirectory()) {
      System.out.println("WARNING: Autos folder does not exist at: " + subfolder.getAbsolutePath());
      return null;
    }

    File[] files = subfolder.listFiles();
    if (files == null || files.length == 0) {
      System.out.println("WARNING: Autos folder is empty");
      return null;
    }

    return files;
  }

  public static String getFilenamWithoutExtension(File file) {
    return file.getName().split("\\.")[0];
  }

  public static List<String> getValidFileNames(File[] files) {
    List<String> validFileNames = new ArrayList<>();
    for (File file : files) {
      validFileNames.add(getFilenamWithoutExtension(file));
    }
    return validFileNames;
  }

  public static List<File> getAllJsonFiles(File dir) {
    List<File> jsonFiles = new ArrayList<>();
    collectJsonFiles(dir, jsonFiles);
    return jsonFiles;
  }

  public static void collectJsonFiles(File dir, List<File> jsonFiles) {
    if (!dir.exists() || !dir.isDirectory()) {
      return;
    }

    File[] files = dir.listFiles();
    if (files == null) return;

    for (File file : files) {
      if (file.isFile() && file.getName().endsWith(".json")) {
        jsonFiles.add(file);
      } else if (file.isDirectory()) {
        collectJsonFiles(file, jsonFiles);
      }
    }
  }
}
