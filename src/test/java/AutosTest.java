
import org.json.simple.parser.ParseException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.auto.AutoBuilderConfig;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.auto.AutoPath;
import frc.robot.subsystems.auto.PathPlannerAutos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;


class Autostest {


@BeforeEach void setup() {
    AutoBuilderConfig.buildAuto(CompTunerConstants.createDrivetrain());
}
@Test
void validateFileName() throws IOException, ParseException {
    Dictionary<String, Boolean> pathDictionary = new Hashtable<>();
    for (AutoPath path : AutoLogic.getAutos()) { // Every auto in the auto chooser
        pathDictionary.put(path.getAutoName(), false);  // Putting each auto from the auto chooser in dictionary

    }

Enumeration<String> k = pathDictionary.keys();
    while (k.hasMoreElements()) { // As long as there are more autos in the list
        String key = k.nextElement();
         // Next entry in the dictionary
//Asserts false if file name does not exist

  assertFalse(PathPlannerAuto.getPathGroupFromAutoFile(key).isEmpty(),"No file path matches" + key);






    }



}
}
