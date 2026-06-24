import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.UnitTestHelpers;
import java.io.File;
import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Test;

public class JSONComparisionTest {

  @Test
  public void compareJSONContents() throws IOException {
    File deployDir = Filesystem.getDeployDirectory();

    if (!deployDir.exists() || !deployDir.isDirectory()) {
      System.out.println("WARNING: Deploy directory not found");
      return;
    }

    List<File> allJsonFiles = UnitTestHelpers.getAllJsonFiles(deployDir);

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
}
