package frc.robot.util;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.wpi.first.util.struct.parser.ParseException;
import frc.robot.subsystems.auto.BLine.BLineLogic;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class UnitTestHelpers {
  public static final String PATHS_PATH = "src/main/deploy/autos/paths";

  // @BeforeEach
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

  // Source - https://stackoverflow.com/a/68606076
  // Posted by Aman Garg
  // Retrieved 2026-06-28, License - CC BY-SA 4.0

  @SuppressWarnings("unchecked")
  public static Object getObjectFromJsonFile(String jsonData, Class classObject) {
    Gson gson = new Gson();
    JsonParser parser = new JsonParser();
    JsonObject object = (JsonObject) parser.parse(jsonData);
    return gson.fromJson(object, classObject);
  }

  // Source - https://stackoverflow.com/a/42440060
  // Posted by Ankur Mahajan
  // Retrieved 2026-06-28, License - CC BY-SA 3.0

  public static JSONObject JSONtoObject(File file)
      throws ParseException, org.json.simple.parser.ParseException {
    try {
      JSONParser parser = new JSONParser();
      // Use JSONObject for simple JSON and JSONArray for array of JSON.
      JSONObject data =
          (JSONObject)
              parser.parse(new FileReader(file.getAbsolutePath())); // path to the JSON file.
      return data;
      // String json = data.toJSONString();
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }
}
