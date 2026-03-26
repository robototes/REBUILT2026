package frc.robot.util.simulation;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

/**
 * Standalone diagnostic tool that reads a .wpilog file and prints information about the entries it
 * contains. Run with: gradlew simulateJava -PlogreplayFile=... or directly as a main class.
 *
 * <p>Usage: java LogDiagnostic path/to/file.wpilog
 */
public class LogDiagnostic {

  public static void main(String[] args) throws IOException {
    String path;
    if (args.length > 0) {
      path = args[0];
    } else {
      path = System.getenv("LOGREPLAY_FILE");
      if (path == null || path.isEmpty()) {
        path = System.getProperty("logreplay.file", "");
      }
    }
    if (path.isEmpty()) {
      System.err.println("Usage: LogDiagnostic <path-to-wpilog>");
      System.exit(1);
    }

    System.out.println("Reading: " + path);
    DataLogReader reader = new DataLogReader(path);
    if (!reader.isValid()) {
      System.err.println("Log file is not valid!");
      System.exit(1);
    }

    Map<Integer, DataLogRecord.StartRecordData> starts = new HashMap<>();
    // entry name -> [type, count]
    Map<String, String> entryTypes = new TreeMap<>();
    Map<String, Integer> entryCounts = new TreeMap<>();
    Map<String, Long> entryFirstTimestamp = new TreeMap<>();
    Map<String, Long> entryLastTimestamp = new TreeMap<>();

    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        DataLogRecord.StartRecordData sd = record.getStartData();
        starts.put(sd.entry, sd);
        entryTypes.put(sd.name, sd.type);
        entryCounts.put(sd.name, 0);
      } else if (!record.isControl()) {
        DataLogRecord.StartRecordData sd = starts.get(record.getEntry());
        if (sd != null) {
          entryCounts.merge(sd.name, 1, Integer::sum);
          entryFirstTimestamp.putIfAbsent(sd.name, record.getTimestamp());
          entryLastTimestamp.put(sd.name, record.getTimestamp());
        }
      }
    }

    System.out.println("\n=== ALL ENTRIES ===");
    System.out.printf("%-70s %-30s %8s%n", "Name", "Type", "Count");
    System.out.println("-".repeat(115));
    for (var name : entryTypes.keySet()) {
      System.out.printf(
          "%-70s %-30s %8d%n", name, entryTypes.get(name), entryCounts.getOrDefault(name, 0));
    }

    System.out.println("\n=== LIMELIGHT & DRIVESTATE ENTRIES ===");
    System.out.printf(
        "%-70s %-30s %8s %15s %15s%n", "Name", "Type", "Count", "First (us)", "Last (us)");
    System.out.println("-".repeat(145));
    for (var name : entryTypes.keySet()) {
      if (name.contains("limelight")
          || name.contains("DriveState")
          || name.contains("drivestate")) {
        long first = entryFirstTimestamp.getOrDefault(name, 0L);
        long last = entryLastTimestamp.getOrDefault(name, 0L);
        System.out.printf(
            "%-70s %-30s %8d %15d %15d%n",
            name, entryTypes.get(name), entryCounts.getOrDefault(name, 0), first, last);
      }
    }

    System.out.println("\nTotal entry names: " + entryTypes.size());
    System.out.println(
        "Total limelight/DriveState entries: "
            + entryTypes.keySet().stream()
                .filter(n -> n.contains("limelight") || n.contains("DriveState"))
                .count());

    // Test LogReplayManager indexing
    System.out.println("\n=== LOG REPLAY MANAGER TEST ===");
    LogReplayManager manager = new LogReplayManager(path);
    System.out.println("LogReplayManager indexed " + manager.getEntryCount() + " entries.");
    manager.close();
  }
}
