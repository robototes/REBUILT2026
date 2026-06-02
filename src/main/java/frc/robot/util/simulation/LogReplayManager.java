package frc.robot.util.simulation;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Reads a .wpilog file and replays Limelight and DriveState data into NetworkTables so that the
 * VisionSubsystem and drivetrain can process them as if they were live.
 *
 * <p>This is designed to be driven from the robot's periodic loop. Call {@link #initReplay()} once
 * (e.g. in {@code simulationInit()}) to pause timing and record the starting FPGA time. Then call
 * {@link #updateReplay()} each loop iteration to publish all log entries whose timestamps fall
 * within the elapsed FPGA time.
 *
 * <p>The initial call to {@link #initReplay()} pauses the simulator clock via {@link
 * SimHooks#pauseTiming()} so the user can prepare before data flows. Use the sim GUI timing
 * controls (or {@link SimHooks#resumeTiming()}) to begin playback.
 *
 * <p>The log replay file path can be set via the environment variable {@code LOGREPLAY_FILE} (set
 * by the gradle property {@code logreplayFile}) or the system property {@code logreplay.file}.
 */
public class LogReplayManager implements Closeable {

  /**
   * Exact NT table prefixes we care about replaying. These match the top-level limelight and
   * DriveState tables only (not entries nested under CameraPublisher, SmartDashboard, etc.).
   */
  private static final String[] REPLAY_PREFIXES = {
    "NT:/limelight-a/",
    "NT:/limelight-b/",
    "NT:/limelight-c/",
    "NT:limelight-a/",
    "NT:limelight-b/",
    "NT:limelight-c/",
    "NT:/DriveState/",
    "NT:DriveState/",
  };

  /**
   * Prefixes that should NOT be matched even if they contain a replay prefix substring (e.g.
   * NT:/CameraPublisher/limelight-a/ or NT:/SmartDashboard/limelight-a_Stream).
   */
  private static final String[] EXCLUDED_PREFIXES = {
    "NT:/CameraPublisher/", "NT:/SmartDashboard/", "NT:CameraPublisher/", "NT:SmartDashboard/",
  };

  /**
   * A single data record to replay: its timestamp, the NT key, the data type, and the raw payload.
   */
  private record ReplayEntry(
      long timestampMicros, String ntKey, String type, DataLogRecord record) {}

  private final List<ReplayEntry> entries = new ArrayList<>();
  private final Map<Integer, DataLogRecord.StartRecordData> entryIdToStart = new HashMap<>();
  private final Map<String, Object> publishers = new HashMap<>();

  /** Names that have already produced an exception; suppresses repeated log spam. */
  private final java.util.Set<String> warnedEntries = new java.util.HashSet<>();

  /** Index of the next entry to publish. */
  private int nextIndex = 0;

  /** FPGA time (microseconds) when replay was started. */
  private long fpgaTimeAtStart = 0;

  /** Log timestamp of the first entry. */
  private long firstLogTimestamp = 0;

  /** Whether {@link #initReplay()} has been called. */
  private boolean initialized = false;

  /**
   * Optional replay drive state estimator. Set via {@link #setReplayDriveState} before calling
   * {@link #initReplay()}.
   */
  private ReplaySwerveDriveState replayDriveState = null;

  /**
   * Construct a LogReplayManager from a .wpilog file path.
   *
   * @param wpilogPath path to the .wpilog file
   */
  public LogReplayManager(String wpilogPath) {
    System.out.println("[LogReplay] Loading log file: " + wpilogPath);
    try {
      DataLogReader reader = new DataLogReader(wpilogPath);
      if (!reader.isValid()) {
        System.err.println("[LogReplay] Log file is not valid: " + wpilogPath);
        return;
      }
      indexEntries(reader);
      System.out.println("[LogReplay] Indexed " + entries.size() + " replayable data records.");
      printIndexSummary();
    } catch (IOException e) {
      System.err.println("[LogReplay] Failed to read log file: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /** Index all records from the log, keeping only those whose names match our replay prefixes. */
  private void indexEntries(DataLogReader reader) {
    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        DataLogRecord.StartRecordData startData = record.getStartData();
        entryIdToStart.put(startData.entry, startData);
      } else if (!record.isControl()) {
        DataLogRecord.StartRecordData startData = entryIdToStart.get(record.getEntry());
        if (startData != null && isReplayable(startData.name)) {
          entries.add(
              new ReplayEntry(record.getTimestamp(), startData.name, startData.type, record));
        }
      }
    }
    entries.sort((a, b) -> Long.compare(a.timestampMicros, b.timestampMicros));
  }

  /** Print a summary of what was indexed, grouped by source. */
  private void printIndexSummary() {
    Map<String, Integer> categoryCounts = new HashMap<>();
    for (ReplayEntry entry : entries) {
      String path = toNTPath(entry.ntKey);
      int slash = path.indexOf('/');
      String category = slash > 0 ? path.substring(0, slash) : path;
      categoryCounts.merge(category, 1, Integer::sum);
    }
    System.out.println("[LogReplay] Breakdown by source:");
    for (var e : categoryCounts.entrySet()) {
      System.out.println("[LogReplay]   " + e.getKey() + ": " + e.getValue() + " records");
    }
    if (!entries.isEmpty()) {
      long firstUs = entries.get(0).timestampMicros;
      long lastUs = entries.get(entries.size() - 1).timestampMicros;
      double durationSecs = (lastUs - firstUs) / 1_000_000.0;
      System.out.printf("[LogReplay] Time span: %.1f seconds%n", durationSecs);
    }
  }

  /** Check if a log entry name matches one of our replay prefixes (and not an excluded prefix). */
  private boolean isReplayable(String name) {
    for (String excluded : EXCLUDED_PREFIXES) {
      if (name.startsWith(excluded)) {
        return false;
      }
    }
    for (String prefix : REPLAY_PREFIXES) {
      if (name.startsWith(prefix)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Convert a log entry name (e.g. "NT:/limelight-a/botpose_wpiblue") to the NT path
   * ("limelight-a/botpose_wpiblue").
   */
  private String toNTPath(String logName) {
    String path = logName;
    if (path.startsWith("NT:/")) {
      path = path.substring(4);
    } else if (path.startsWith("NT:")) {
      path = path.substring(3);
    }
    return path;
  }

  /**
   * Initialize the replay. Pauses simulator timing and records the FPGA start time. Call this once
   * from {@code simulationInit()}.
   */
  public void initReplay() {
    if (entries.isEmpty()) {
      System.out.println("[LogReplay] No entries to replay.");
      return;
    }

    System.out.println("+==============================================================+");
    System.out.println("|  LOG REPLAY MODE ACTIVE                                      |");
    System.out.println("|  Simulator time is paused before first data point.           |");
    System.out.println("|  Use the sim GUI timing controls to resume/step.             |");
    System.out.println("+==============================================================+");

    SimHooks.pauseTiming();

    firstLogTimestamp = entries.get(0).timestampMicros;
    fpgaTimeAtStart = RobotController.getFPGATime();
    nextIndex = 0;
    initialized = true;
  }

  /**
   * Publish all log entries whose offset from the first log entry has been reached by the FPGA
   * clock. Call this every robot loop iteration (e.g. from {@code robotPeriodic()} or via {@code
   * addPeriodic()}).
   */
  public void updateReplay() {
    if (!initialized || nextIndex >= entries.size()) {
      return;
    }

    long currentFpga = RobotController.getFPGATime();
    long elapsed = currentFpga - fpgaTimeAtStart;

    while (nextIndex < entries.size()) {
      ReplayEntry entry = entries.get(nextIndex);
      long entryOffset = entry.timestampMicros - firstLogTimestamp;

      if (entryOffset > elapsed) {
        break; // not yet time for this entry
      }

      publishEntry(entry);
      nextIndex++;
    }

    if (nextIndex >= entries.size()) {
      System.out.println("[LogReplay] Replay complete. " + entries.size() + " records published.");
    }
  }

  /**
   * @return true if all entries have been published.
   */
  public boolean isFinished() {
    return initialized && nextIndex >= entries.size();
  }

  // ---- DriveState deserialization ----

  /** Feed a DriveState log entry into the replay pose estimator. */
  private void feedDriveState(String entryName, ReplayEntry entry) {
    double timestampSeconds = entry.timestampMicros / 1_000_000.0;
    switch (entryName) {
      case "ModulePositions":
        if (entry.type.startsWith("struct:")) {
          var positions = ReplaySwerveDriveState.deserializeModulePositions(entry.record);
          // Use the heading from the most recent logged Pose (cached in the replay state).
          // Do NOT use the estimator's own output here -- that closes a feedback loop and
          // causes the pose to drift.
          replayDriveState.updateOdometry(
              positions, replayDriveState.getLastHeading(), timestampSeconds);
        }
        break;
      case "Pose":
        if (entry.type.startsWith("struct:")) {
          var pose = ReplaySwerveDriveState.deserializePose2d(entry.record);
          replayDriveState.updatePose(pose, timestampSeconds);
        }
        break;
      case "Speeds":
        if (entry.type.startsWith("struct:")) {
          var speeds = ReplaySwerveDriveState.deserializeChassisSpeeds(entry.record);
          replayDriveState.updateSpeeds(speeds);
        }
        break;
      case "ModuleStates":
        if (entry.type.startsWith("struct:")) {
          var states = ReplaySwerveDriveState.deserializeModuleStates(entry.record);
          replayDriveState.updateModuleStates(states);
        }
        break;
      case "ModuleTargets":
        if (entry.type.startsWith("struct:")) {
          var targets = ReplaySwerveDriveState.deserializeModuleStates(entry.record);
          replayDriveState.updateModuleTargets(targets);
        }
        break;
      case "OdometryFrequency":
        if (entry.type.equals("double")) {
          double freq = entry.record.getDouble();
          if (freq > 0) {
            replayDriveState.updateOdometryPeriod(1.0 / freq);
          }
        }
        break;
      default:
        break;
    }
  }

  // ---- NT publishing helpers ----

  /** Publish a single log entry into NetworkTables, and feed DriveState entries to the replay. */
  private void publishEntry(ReplayEntry entry) {
    String ntPath = toNTPath(entry.ntKey);
    int lastSlash = ntPath.lastIndexOf('/');
    if (lastSlash < 0) {
      return;
    }
    String tablePath = ntPath.substring(0, lastSlash);
    String entryName = ntPath.substring(lastSlash + 1);

    // Feed DriveState struct entries into the replay pose estimator
    if (replayDriveState != null && tablePath.equals("DriveState")) {
      try {
        feedDriveState(entryName, entry);
      } catch (Exception e) {
        warnOnce("feedDriveState:" + entryName, e);
      }
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable(tablePath);

    try {
      switch (entry.type) {
        case "double":
          getOrCreateDoublePublisher(table, entryName, tablePath + "/" + entryName)
              .set(entry.record.getDouble());
          break;
        case "int64":
          getOrCreateIntegerPublisher(table, entryName, tablePath + "/" + entryName)
              .set(entry.record.getInteger());
          break;
        case "double[]":
          getOrCreateDoubleArrayPublisher(table, entryName, tablePath + "/" + entryName)
              .set(entry.record.getDoubleArray());
          break;
        case "string":
          getOrCreateStringPublisher(table, entryName, tablePath + "/" + entryName)
              .set(entry.record.getString());
          break;
        default:
          if (entry.type.startsWith("struct:") || entry.type.startsWith("structschema:")) {
            getOrCreateRawPublisher(table, entryName, tablePath + "/" + entryName, entry.type)
                .set(entry.record.getRaw());
          }
          break;
      }
    } catch (Exception e) {
      warnOnce("publish:" + entry.ntKey, e);
    }
  }

  /** Log a problem with a given entry once, then stay quiet about it. */
  private void warnOnce(String key, Exception e) {
    if (warnedEntries.add(key)) {
      System.err.println(
          "[LogReplay] Skipping "
              + key
              + " due to "
              + e.getClass().getSimpleName()
              + ": "
              + e.getMessage()
              + " (further occurrences suppressed)");
    }
  }

  private DoublePublisher getOrCreateDoublePublisher(
      NetworkTable table, String entryName, String fullKey) {
    return (DoublePublisher)
        publishers.computeIfAbsent(fullKey, k -> table.getDoubleTopic(entryName).publish());
  }

  private IntegerPublisher getOrCreateIntegerPublisher(
      NetworkTable table, String entryName, String fullKey) {
    return (IntegerPublisher)
        publishers.computeIfAbsent(fullKey, k -> table.getIntegerTopic(entryName).publish());
  }

  private DoubleArrayPublisher getOrCreateDoubleArrayPublisher(
      NetworkTable table, String entryName, String fullKey) {
    return (DoubleArrayPublisher)
        publishers.computeIfAbsent(fullKey, k -> table.getDoubleArrayTopic(entryName).publish());
  }

  private StringPublisher getOrCreateStringPublisher(
      NetworkTable table, String entryName, String fullKey) {
    return (StringPublisher)
        publishers.computeIfAbsent(fullKey, k -> table.getStringTopic(entryName).publish());
  }

  private RawPublisher getOrCreateRawPublisher(
      NetworkTable table, String entryName, String fullKey, String typeString) {
    return (RawPublisher)
        publishers.computeIfAbsent(fullKey, k -> table.getRawTopic(entryName).publish(typeString));
  }

  // ---- Static helpers ----

  /** Set the replay drive state estimator to feed DriveState data into. */
  public void setReplayDriveState(ReplaySwerveDriveState driveState) {
    this.replayDriveState = driveState;
  }

  /** Get the replay drive state estimator. */
  public ReplaySwerveDriveState getReplayDriveState() {
    return replayDriveState;
  }

  /** Get the total number of replayable entries found in the log. */
  public int getEntryCount() {
    return entries.size();
  }

  /** Check if log replay mode is enabled via system property or environment variable. */
  public static boolean isReplayEnabled() {
    String file = getReplayFilePath();
    return file != null && !file.isEmpty();
  }

  /** Get the log file path from system property or environment variable. */
  public static String getReplayFilePath() {
    String file = System.getProperty("logreplay.file");
    if (file != null && !file.isEmpty()) {
      return file;
    }
    file = System.getenv("LOGREPLAY_FILE");
    return file != null ? file : "";
  }

  /**
   * Create a LogReplayManager if replay is enabled.
   *
   * @return a LogReplayManager, or null if replay is not enabled
   */
  public static LogReplayManager createIfEnabled() {
    if (isReplayEnabled()) {
      String path = getReplayFilePath();
      System.out.println("[LogReplay] Log replay enabled with file: " + path);
      return new LogReplayManager(path);
    }
    return null;
  }

  @Override
  public void close() {
    for (Object pub : publishers.values()) {
      if (pub instanceof Closeable closeable) {
        try {
          closeable.close();
        } catch (IOException e) {
          // ignore
        }
      }
    }
    publishers.clear();
  }
}
