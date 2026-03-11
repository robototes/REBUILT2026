package frc.robot.util.robotType;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class ConfigShift {

  public static RobotConfig configFromRobot(String ROBOT) throws IOException, ParseException {

    BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "pathplanner/settings(" + ROBOT + ").json")));

    StringBuilder fileContentBuilder = new StringBuilder();
    String line;
    while ((line = br.readLine()) != null) {
      fileContentBuilder.append(line);
    }
    br.close();

    String fileContent = fileContentBuilder.toString();
    JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

    boolean isHolonomic = (boolean) json.get("holonomicMode");
    double massKG = ((Number) json.get("robotMass")).doubleValue();
    double MOI = ((Number) json.get("robotMOI")).doubleValue();
    double wheelRadius = ((Number) json.get("driveWheelRadius")).doubleValue();
    double gearing = ((Number) json.get("driveGearing")).doubleValue();
    double maxDriveSpeed = ((Number) json.get("maxDriveSpeed")).doubleValue();
    double wheelCOF = ((Number) json.get("wheelCOF")).doubleValue();
    String driveMotor = (String) json.get("driveMotorType");
    double driveCurrentLimit = ((Number) json.get("driveCurrentLimit")).doubleValue();

    int numMotors = isHolonomic ? 1 : 2;
    DCMotor gearbox =
        switch (driveMotor) {
          case "krakenX60" -> DCMotor.getKrakenX60(numMotors);
          case "krakenX60FOC" -> DCMotor.getKrakenX60Foc(numMotors);
          case "falcon500" -> DCMotor.getFalcon500(numMotors);
          case "falcon500FOC" -> DCMotor.getFalcon500Foc(numMotors);
          case "vortex" -> DCMotor.getNeoVortex(numMotors);
          case "NEO" -> DCMotor.getNEO(numMotors);
          case "CIM" -> DCMotor.getCIM(numMotors);
          case "miniCIM" -> DCMotor.getMiniCIM(numMotors);
          default -> throw new IllegalArgumentException("Invalid motor type: " + driveMotor);
        };
    gearbox = gearbox.withReduction(gearing);

    ModuleConfig moduleConfig =
        new ModuleConfig(
            wheelRadius, maxDriveSpeed, wheelCOF, gearbox, driveCurrentLimit, numMotors);

    if (isHolonomic) {
      Translation2d[] moduleOffsets =
          new Translation2d[] {
            new Translation2d(
                ((Number) json.get("flModuleX")).doubleValue(),
                ((Number) json.get("flModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("frModuleX")).doubleValue(),
                ((Number) json.get("frModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("blModuleX")).doubleValue(),
                ((Number) json.get("blModuleY")).doubleValue()),
            new Translation2d(
                ((Number) json.get("brModuleX")).doubleValue(),
                ((Number) json.get("brModuleY")).doubleValue())
          };

      return new RobotConfig(massKG, MOI, moduleConfig, moduleOffsets);
    } else {
      double trackwidth = ((Number) json.get("robotTrackwidth")).doubleValue();

      return new RobotConfig(massKG, MOI, moduleConfig, trackwidth);
    }
  }
}
