# Skill: Package FRC Log Files

## Purpose

Bundle `.wpilog` robot log files with their corresponding `.hoot` signal logger folders into `.zip` archives for easy sharing and archival.

## Script

This skill is implemented as a standalone PowerShell script that can be run
directly by users — no Copilot required:

```
.github\skills\PackageLogs.ps1
```

### Running the script

```powershell
# Pass the folder path as an argument:
.\PackageLogs.ps1 -LogFolder "C:\Users\me\Downloads\logs"

# Or just run it and it will prompt you:
.\PackageLogs.ps1
```

### What it does

When invoked by Copilot, run the script in a terminal and answer any prompts
on the user's behalf based on context, or relay the prompts to the user.

1. **Scans** the given folder for `.wpilog` files and timestamped subdirectories containing `.hoot` files.
2. **Auto-matches** folders to `.wpilog` files by comparing their embedded timestamps (within a configurable tolerance, default 2 minutes).
3. **Prompts** the user to resolve any ambiguities:
   - Unmatched folders → assign to a `.wpilog`, ignore, or zip standalone.
   - Unmatched `.wpilog` files → zip alone or ignore.
4. **Shows the final plan** and asks for confirmation before creating any files.
5. **Creates `.zip` archives** — one per `.wpilog` file — containing the log and its matched folders.
6. **Prints a summary** of all created archives with sizes.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `-LogFolder` | *(prompt)* | Path to the folder with `.wpilog` files and timestamped subdirs |
| `-ToleranceMinutes` | `2` | Max minutes of timestamp drift to still auto-match |

## Reference

### Timestamp formats

- `.wpilog` filename: `FRC_YYYYMMDD_HHMMSS.wpilog` (no separators in date/time)
- Folder name: `YYYY-MM-DD_HH-MM-SS` (dashes in date, dashes in time)
- `.hoot` files inside folders have similar timestamps and don't need separate parsing

### Matching rules

- The `.wpilog` timestamp typically precedes or closely matches the folder timestamps because the robot code starts logging before the signal loggers initialize.
- Multiple folders can belong to one `.wpilog` session (e.g., when the robot is power-cycled during a practice session without the main log rolling over).
- A folder is matched to the `.wpilog` file with the **closest** timestamp, as long as the difference is within the tolerance (default 2 minutes).
- If no `.wpilog` is within tolerance, the folder is flagged as unmatched for the user to resolve.
- **The script never deletes or moves original files** — it only creates new `.zip` archives alongside them.
