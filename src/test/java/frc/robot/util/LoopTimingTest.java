package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class LoopTimingTest {
  private static final class TestRobot extends IterativeRobotBase {
    final CommandScheduler scheduler = CommandScheduler.getInstance();
    final LoopTiming timing = new LoopTiming(this, scheduler);

    TestRobot() {
      super(0.02);
    }

    @Override
    public void startCompetition() {}

    @Override
    public void endCompetition() {}

    @Override
    public void autonomousPeriodic() {
      SimHooks.stepTiming(0.001);
    }

    @Override
    public void teleopPeriodic() {
      SimHooks.stepTiming(0.002);
    }

    @Override
    public void robotPeriodic() {
      timing.resetCheckpoint();
      SimHooks.stepTiming(0.001);
      timing.checkpoint("ExplicitWork");
      scheduler.run();
      timing.checkpoint("Scheduler");
      timing.captureScheduler();
    }

    @Override
    public void simulationPeriodic() {}

    void cycle() {
      Logger.AdvancedHooks.invokePeriodicBeforeUser();
      long start = RobotController.getFPGATime();
      timing.beginLoop();
      super.loopFunc();
      timing.endLoop(start, getPeriod());
      Logger.AdvancedHooks.invokePeriodicAfterUser(RobotController.getFPGATime() - start, 0);
    }
  }

  @Test
  void exportsRealEpochsAndClearsInactiveWorkAcrossModes() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    BlockingQueue<LogTable> logs = new LinkedBlockingQueue<>();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.disableConsoleCapture();
    Logger.addDataReceiver(logs::add);
    Logger.start();
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.unregisterAllSubsystems();
    try (TestRobot robot = new TestRobot()) {
      new SubsystemBase("TimingSubsystem") {
        @Override
        public void periodic() {
          SimHooks.stepTiming(0.003);
        }

        @Override
        public void simulationPeriodic() {
          SimHooks.stepTiming(0.004);
        }
      };
      Command command =
          new Command() {
            @Override
            public void initialize() {
              SimHooks.stepTiming(0.002);
            }

            @Override
            public void execute() {
              SimHooks.stepTiming(0.005);
            }

            @Override
            public boolean isFinished() {
              SimHooks.stepTiming(0.001);
              return true;
            }

            @Override
            public void end(boolean interrupted) {
              SimHooks.stepTiming(0.002);
            }
          }.withName("TimingCommand");
      Sendable dashboard =
          builder ->
              builder.addDoubleProperty(
                  "Value",
                  () -> {
                    SimHooks.stepTiming(0.006);
                    return 1;
                  },
                  null);
      SmartDashboard.putData("LoopTimingTest", dashboard);
      Sendable liveWindow =
          builder ->
              builder.addDoubleProperty(
                  "Value",
                  () -> {
                    SimHooks.stepTiming(0.001);
                    return 1;
                  },
                  null);
      SendableRegistry.addLW(liveWindow, "LoopTimingTestLiveWindow");
      LiveWindow.enableTelemetry(liveWindow);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      edu.wpi.first.wpilibj.DriverStation.refreshData();
      AtomicBoolean schedule = new AtomicBoolean(true);
      scheduler
          .getDefaultButtonLoop()
          .bind(
              () -> {
                if (schedule.getAndSet(false)) {
                  scheduler.schedule(command);
                }
              });
      robot.cycle();
      LogTable first = outputs(logs);
      assertTrue(first.get("WatchdogEpochsAvailable", false));
      assertTrue(first.get("Thread/WallMS", -1.0) >= 0);
      if (first.get("Thread/CpuTimeAvailable", false)) {
        assertTrue(first.get("Thread/CpuMS", -1.0) >= 0);
        assertTrue(first.get("Thread/OffCpuMS", -1.0) >= 0);
      } else {
        assertEquals(-1.0, first.get("Thread/CpuMS", 0.0));
        assertEquals(-1.0, first.get("Thread/OffCpuMS", 0.0));
      }
      assertTrue(first.get("GC/CollectionCountAvailable", false));
      assertTrue(first.get("GC/Collections", -1L) >= 0);
      assertEquals(7.0, first.get("SchedulerEpochs/TimingSubsystem.periodic()MS", -1.0), 0.001);
      assertEquals(5.0, first.get("SchedulerEpochs/TimingCommand.execute()MS", -1.0), 0.001);
      assertEquals(2.0, first.get("SchedulerEpochs/TimingCommand.initialize()MS", -1.0), 0.001);
      assertEquals(3.0, first.get("SchedulerEpochs/TimingCommand.end(false)MS", -1.0), 0.001);
      assertEquals(1.0, first.get("RobotEpochs/autonomousPeriodic()MS", -1.0), 0.001);
      assertEquals(6.0, first.get("RobotEpochs/SmartDashboard.updateValues()MS", -1.0), 0.001);
      assertEquals(1.0, first.get("RobotEpochs/LiveWindow.updateValues()MS", -1.0), 0.001);
      assertEquals(1.0, first.get("RobotPeriodic/ExplicitWorkMS", -1.0), 0.001);
      assertTrue(first.get("TotalMS", -1.0) >= 23.0);
      assertTrue(first.get("Overrun", false));
      assertEquals(1L, first.get("OverrunCount", -1L));
      assertFalse(scheduler.isScheduled(command));

      DriverStationSim.setAutonomous(false);
      DriverStationSim.notifyNewData();
      robot.cycle();
      LogTable second = outputs(logs);
      assertEquals(0.0, second.get("SchedulerEpochs/TimingCommand.execute()MS", -1.0));
      assertEquals(0.0, second.get("SchedulerEpochs/TimingCommand.initialize()MS", -1.0));
      assertEquals(0.0, second.get("SchedulerEpochs/TimingCommand.end(false)MS", -1.0));
      assertEquals(0.0, second.get("RobotEpochs/autonomousPeriodic()MS", -1.0));
      assertEquals(2.0, second.get("RobotEpochs/teleopPeriodic()MS", -1.0), 0.001);
      assertFalse(second.get("Overrun", true));
      assertEquals(1L, second.get("OverrunCount", -1L));

      scheduler.disable();
      robot.cycle();
      LogTable disabled = outputs(logs);
      assertEquals(0.0, disabled.get("SchedulerEpochs/TimingSubsystem.periodic()MS", -1.0));
      assertEquals(0.0, disabled.get("SchedulerEpochs/buttons.run()MS", -1.0));
    } finally {
      scheduler.enable();
      scheduler.cancelAll();
      scheduler.unregisterAllSubsystems();
      scheduler.getDefaultButtonLoop().clear();
      LiveWindow.disableAllTelemetry();
      DriverStationSim.resetData();
      Logger.end();
      SimHooks.resumeTiming();
    }
  }

  private static LogTable outputs(BlockingQueue<LogTable> logs) throws InterruptedException {
    LogTable log = logs.poll(5, TimeUnit.SECONDS);
    assertNotNull(log, "AdvantageKit must publish the timing samples");
    return log.getSubtable("RealOutputs").getSubtable("LoopTiming");
  }
}
