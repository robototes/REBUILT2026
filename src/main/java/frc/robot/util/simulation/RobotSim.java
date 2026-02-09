package frc.robot.util.simulation;

import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class RobotSim {

    FuelSim fuelSim;
    CommandSwerveDrivetrain drive;
    final static public double UPDATE_S = 0.02; // 20 ms update rate

    public RobotSim(CommandSwerveDrivetrain drive){
      this.drive = drive;
      fuelSim = FuelSim.getInstance();
      fuelSim.spawnStartingFuel();
      fuelSim.registerRobot(
          0.8,
          0.8,
          0.7,
          () -> drive.getState().Pose,
          () -> drive.getState().Speeds);
      fuelSim.registerIntake(0.4, 0.8, 0.4, 0.8, () -> true);
      fuelSim.start();
    }

    public void resetFuelSim(){
        fuelSim.clearFuel();
        fuelSim.spawnStartingFuel();
    }

    public void updateFuelSim(){
        FuelSim.getInstance().updateSim();
    }
}
