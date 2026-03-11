# Skill: Package FRC Log Files

## Purpose

Bundle `.wpilog` robot log files with their corresponding `.hoot` signal
logger folders into `.zip` archives sized for OneDrive/SharePoint upload.

## Script

This skill is implemented as a standalone Windows PowerShell 5.1 script:

```
.github\skills\PackageLogs.ps1
```

### Running the script

```powershell
# With arguments:
.\PackageLogs.ps1 -LogFolder "D:\Temp\2026_BonneyLake\2412"

# Custom size limit and tolerance:
.\PackageLogs.ps1 -LogFolder "C:\logs" -MaxZipSizeMB 50 -ToleranceMinutes 3

# Interactive (prompts for folder):
.\PackageLogs.ps1
```

### What it does

When invoked by Copilot, run the script in a terminal and answer any prompts
on the user's behalf based on context, or relay the prompts to the user.

1. **Scans** the folder for `.wpilog` files and subdirectories (both
   timestamped like `2026-03-06_04-21-49` and event-labeled like `WABON_Q42`).
2. **Skips** zero-byte `.wpilog` files and files with no parseable timestamp.
3. **Auto-matches** folders to `.wpilog` files using:
   - **Event label matching** (highest priority): `WABON_P2` folder to
     `FRC_..._WABON_P2.wpilog`
   - **Timestamp proximity**: closest `.wpilog` within the tolerance window
   - For event-labeled folders, timestamps are extracted from `.hoot`
     filenames inside
4. **Event match `.wpilog` files are always kept** -- they are never listed
   as unmatched and never require user prompts.
5. **Prompts** the user to resolve remaining unmatched items, with **bulk
   options** for large sets (Ignore All / Zip All / Resolve Each).
6. **Groups** matched items into **batches** that stay under `MaxZipSizeMB`
   (default 100 MB uncompressed), splitting at the **largest timestamp gaps**
   to keep related logs together.
7. **Shows the final plan** and asks for confirmation.
8. **Creates `.zip` archives** and prints a summary with sizes.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `-LogFolder` | *(prompt)* | Path to the folder with `.wpilog` files and subdirs |
| `-ToleranceMinutes` | `2` | Max minutes of timestamp drift for auto-matching |
| `-MaxZipSizeMB` | `100` | Target max uncompressed size per zip (for OneDrive) |

## Reference

### Supported folder types

| Pattern | Example | Matching method |
|---------|---------|-----------------|
| Timestamped | `2026-03-06_04-21-49` | Closest `.wpilog` by timestamp |
| Event match | `WABON_Q42` | Name match to `FRC_..._WABON_Q42.wpilog` |
| Event match (empty) | `WABON_E4` (no .hoot files) | Name match only |

### Event match labels

Event match labels follow the format `<EventCode>_<Type><Number>`:
- **Event code**: uppercase letters (e.g., `WABON`, `CASJ`, `ORORE`)
- **Type**: `P` (practice), `Q` (qualification), `E` (elimination), `F` (finals)
- **Number**: match number (e.g., `1`, `42`, `65`)

Examples: `WABON_P2`, `WABON_Q65`, `CASJ_E14`, `ORORE_F3`

**Event-labeled `.wpilog` files are always included** -- they represent
official competition matches and are automatically zipped with or without
matching folders.  They never appear in the "unmatched" prompts.

### Timestamp formats

- `.wpilog` filename: `FRC_YYYYMMDD_HHMMSS.wpilog` or
  `FRC_YYYYMMDD_HHMMSS_WABON_Q42.wpilog`
- Timestamped folder: `YYYY-MM-DD_HH-MM-SS`
- `.hoot` files: `..._YYYY-MM-DD_HH-MM-SS.hoot`

### Matching rules

- **Event label match** takes priority -- if a folder name matches the
  `_XXXXX_YNN` suffix of a `.wpilog` file, they are linked regardless of
  timestamp.
- **Timestamp match** uses closest-match within the tolerance.
- Multiple folders can match one `.wpilog` file.
- Zero-byte `.wpilog` files and unparseable filenames are skipped.

### Batching rules

- All matched items are sorted chronologically and grouped into batches.
- If a batch exceeds `MaxZipSizeMB`, it is split at the **largest timestamp
  gap** within it, maximizing the natural separation between sessions.
- Single-item batches use the `.wpilog` basename as the zip filename.
- Multi-item batches are named `FRC_<first>_to_<last>.zip`.

### Safety

- **The script never deletes or moves original files** -- it only creates
  new `.zip` archives alongside them.
- Existing `.zip` files with the same name are overwritten after confirmation.
