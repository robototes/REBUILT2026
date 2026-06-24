import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.auto.BLine.BLineLogic;
import frc.robot.subsystems.auto.BLine.BLinePath;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AutosTest {

  private static final String PATHS_PATH = "src/main/deploy/autos/paths";

  @BeforeEach
  public void setup() {
    BLineLogic.unitTestInit();
  }

  // ==================== METHODS ====================

  private File[] getAutosFiles() {
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

  private String getFilenamWithoutExtension(File file) {
    return file.getName().split("\\.")[0];
  }

  private List<String> getValidFileNames(File[] files) {
    List<String> validFileNames = new ArrayList<>();
    for (File file : files) {
      validFileNames.add(getFilenamWithoutExtension(file));
    }
    return validFileNames;
  }

  // ==================== ACUTAL UNIT TESTS ====================

  @Test
  public void unusedFileNames() {
    File[] files = getAutosFiles();
    if (files == null) return;

    List<String> validPathNames = BLineLogic.getBLinePathsNames();
    List<String> unusedFiles = new ArrayList<>();

    for (File file : files) {
      String filename = getFilenamWithoutExtension(file);
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

  @Test
  public void validatePathFiles() {
    File[] files = getAutosFiles();
    if (files == null) return;

    List<String> validFileNames = getValidFileNames(files);
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

  @Test
  public void validateUniqueAutoNames() {
    File[] files = getAutosFiles();
    if (files == null) return;

    List<String> invalidNames = new ArrayList<>();

    for (File file : files) {
      String filename = getFilenamWithoutExtension(file);

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

  @Test
  public void compareJSONContents() throws IOException {
    File deployDir = Filesystem.getDeployDirectory();

    if (!deployDir.exists() || !deployDir.isDirectory()) {
      System.out.println("WARNING: Deploy directory not found");
      return;
    }

    List<File> allJsonFiles = getAllJsonFiles(deployDir);

    if (allJsonFiles.size() < 2) {
      System.out.println("WARNING: Not enough files to compare");
      return;
    }

    // Compare each file with every other file
    for (int i = 0; i < allJsonFiles.size(); i++) {
      String content1 = new String(java.nio.file.Files.readAllBytes(allJsonFiles.get(i).toPath()));

      for (int j = i + 1; j < allJsonFiles.size(); j++) {
        String content2 =
            new String(java.nio.file.Files.readAllBytes(allJsonFiles.get(j).toPath()));

        String file1Name = allJsonFiles.get(i).getName();
        String file2Name = allJsonFiles.get(j).getName();

        if (content1.equals(content2)) {
          System.out.println(file1Name + " and " + file2Name + " are IDENTICAL");
        }
      }
    }
  }

  private List<File> getAllJsonFiles(File dir) {
    List<File> jsonFiles = new ArrayList<>();
    collectJsonFiles(dir, jsonFiles);
    return jsonFiles;
  }

  private void collectJsonFiles(File dir, List<File> jsonFiles) {
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
