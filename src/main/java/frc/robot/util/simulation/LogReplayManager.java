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
 * <p>Usage: construct with a path to a .wpilog file. Call {@link #start()} to begin replay. The
 * manager will pause the simulator time (via {@link SimHooks#pauseTiming()}) before the first
 * relevant datapoint is published. Use the sim GUI's timing controls (or call {@link
 * SimHooks#resumeTiming()}) to begin playback. Data is published in sync with the FPGA clock from
 * {@link RobotController#getFPGATime()}.
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

  private volatile boolean stopped = false;
  private Thread replayThread;

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
        // Data record — check if this entry ID maps to a name we care about
        DataLogRecord.StartRecordData startData = entryIdToStart.get(record.getEntry());
        if (startData != null && isReplayable(startData.name)) {
          entries.add(
              new ReplayEntry(record.getTimestamp(), startData.name, startData.type, record));
        }
      }
    }
    // Sort by timestamp (should already be sorted, but just in case)
    entries.sort((a, b) -> Long.compare(a.timestampMicros, b.timestampMicros));
  }

  /** Print a summary of what was indexed, grouped by source. */
  private void printIndexSummary() {
    Map<String, Integer> categoryCounts = new HashMap<>();
    for (ReplayEntry entry : entries) {
      // Extract the table name (e.g. "limelight-a", "DriveState")
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
    // Exclude entries nested under tables we don't want (e.g. CameraPublisher, SmartDashboard)
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
   * Convert a log entry name (e.g. "NT:/limelight-a/botpose_wpiblue") to the NT table path
   * ("limelight-a") and entry name ("botpose_wpiblue").
   */
  private String toNTPath(String logName) {
    // Strip the "NT:" or "NT:/" prefix
    String path = logName;
    if (path.startsWith("NT:/")) {
      path = path.substring(4);
    } else if (path.startsWith("NT:")) {
      path = path.substring(3);
    }
    return path;
  }

  /** Start the replay. Spawns a background thread that replays data at the recorded pace. */
  public void start() {
    if (entries.isEmpty()) {
      System.out.println("[LogReplay] No entries to replay.");
      return;
    }

    replayThread =
        new Thread(
            () -> {
              try {
                runReplay();
              } catch (InterruptedException e) {
                System.out.println("[LogReplay] Replay thread interrupted.");
              }
            },
            "LogReplay");
    replayThread.setDaemon(true);
    replayThread.start();
  }

  private void runReplay() throws InterruptedException {
    System.out.println("+==============================================================+");
    System.out.println("|  LOG REPLAY MODE ACTIVE                                      |");
    System.out.println("|  Simulator time is paused before first data point.           |");
    System.out.println("|  Use the sim GUI timing controls to resume/step.             |");
    System.out.println("+==============================================================+");

    // Pause the simulator clock so the user can prepare before data starts flowing.
    SimHooks.pauseTiming();

    var statusPub =
        NetworkTableInstance.getDefault().getTable("logreplay").getStringTopic("status").publish();
    var progressPub =
        NetworkTableInstance.getDefault()
            .getTable("logreplay")
            .getDoubleTopic("progress")
            .publish();
    statusPub.set("PAUSED - Waiting to start replay");

    // Wait until the user resumes timing (via sim GUI or SimHooks.resumeTiming())
    while (SimHooks.isTimingPaused() && !stopped) {
      Thread.sleep(100);
    }

    if (stopped || entries.isEmpty()) {
      return;
    }

    System.out.println("[LogReplay] Replay starting...");
    statusPub.set("PLAYING");

    // Record the FPGA time at replay start and the log timestamp of the first entry.
    // All subsequent entries are published when the FPGA clock has advanced by the same
    // offset as in the original log.
    long firstLogTimestamp = entries.get(0).timestampMicros;
    long fpgaTimeAtStart = RobotController.getFPGATime();

    for (int i = 0; i < entries.size() && !stopped; i++) {
      ReplayEntry entry = entries.get(i);

      // Calculate how far into the log this entry is
      long offsetMicros = entry.timestampMicros - firstLogTimestamp;
      long targetFpgaTime = fpgaTimeAtStart + offsetMicros;

      // Wait until the FPGA clock reaches this entry's target time.
      // If timing is paused (via SimHooks), getFPGATime() won't advance,
      // so we naturally wait until the user steps or resumes.
      while (RobotController.getFPGATime() < targetFpgaTime && !stopped) {
        Thread.sleep(1);
      }

      publishEntry(entry);

      // Update progress periodically
      if (i % 100 == 0) {
        double progress = (double) i / entries.size() * 100.0;
        progressPub.set(progress);
      }
    }

    progressPub.set(100.0);
    statusPub.set("FINISHED");
    System.out.println("[LogReplay] Replay complete. " + entries.size() + " records replayed.");
  }

  /** Publish a single log entry into NetworkTables. */
  private void publishEntry(ReplayEntry entry) {
    String ntPath = toNTPath(entry.ntKey);
    // Split into table path and entry name
    int lastSlash = ntPath.lastIndexOf('/');
    if (lastSlash < 0) {
      return; // malformed path
    }
    String tablePath = ntPath.substring(0, lastSlash);
    String entryName = ntPath.substring(lastSlash + 1);

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
          // For struct types and other raw types, publish as raw bytes
          if (entry.type.startsWith("struct:") || entry.type.startsWith("structschema:")) {
            getOrCreateRawPublisher(table, entryName, tablePath + "/" + entryName, entry.type)
                .set(entry.record.getRaw());
          }
          break;
      }
    } catch (Exception e) {
      // Silently skip entries that can't be decoded — some may have been truncated
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

  /** Stop the replay entirely. */
  public void stop() {
    stopped = true;
    if (replayThread != null) {
      replayThread.interrupt();
    }
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
    // Check system property first (e.g. -Dlogreplay.file=...)
    String file = System.getProperty("logreplay.file");
    if (file != null && !file.isEmpty()) {
      return file;
    }
    // Fall back to environment variable (set by gradle wpi.sim.envVar)
    file = System.getenv("LOGREPLAY_FILE");
    return file != null ? file : "";
  }

  /**
   * Create a LogReplayManager from the system property if replay is enabled.
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
    stop();
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
