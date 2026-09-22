package frc.robot.test;

import frc.robot.Robot;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.hardware.hal.simulation.DriverStationDataJNI;

public class AutomatedTestRobot extends Robot {
  private static void sleep(long durationMillis) {
    try {
      Thread.sleep(durationMillis);
    } catch (InterruptedException interrupt) {
      System.out.println("Interrupted");
    }
  }

  public AutomatedTestRobot() {
    System.out.println("Robot type: Automated test");
    new Thread(this::runTest).start();
  }

  @Override
  public void startCompetition() {
    try {
      super.startCompetition();
    } catch (Throwable throwable) {
      Throwable cause = throwable.getCause();
      if (cause != null) {
        throwable = cause;
      }
      DriverStationErrors.reportError(
          "Unhandled exception: " + throwable.toString(), throwable.getStackTrace());
      System.exit(-1);
    }
  }

  private void runTest() {
    System.out.println("Waiting two seconds for robot to finish startup");
    sleep(2000);

    System.out.println("Enabling autonomous mode and waiting 10 seconds");
    DriverStationDataJNI.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationDataJNI.setEnabled(true);

    sleep(10000);

    System.out.println("Disabling robot and waiting two seconds");
    DriverStationDataJNI.setEnabled(false);

    sleep(2000);

    System.out.println("Ending competition");
    suppressExitWarning(true);
    endCompetition();
  }
}
