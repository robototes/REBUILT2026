# Skill: Package FRC Log Files

## Purpose

Bundle `.wpilog` robot log files with their corresponding `.hoot` signal
logger folders into `.zip` archives sized for OneDrive/SharePoint upload.

See [`SPEC.md`](SPEC.md) for detailed documentation of FRC log file naming
conventions, timestamps, and folder structures.

## Script

This skill is implemented as a standalone Windows PowerShell 5.1 script:

```
.github\skills\PackageLogs.ps1
```

### Running the script

```powershell
# With arguments:
.\PackageLogs.ps1 -LogFolder "D:\Temp\2026_BonneyLake\2412"

# Custom size limit, tolerance, and parallelism:
.\PackageLogs.ps1 -LogFolder "C:\logs" -MaxZipSizeMB 50 -MaxParallel 8

# Interactive (prompts for folder):
.\PackageLogs.ps1
```

### What it does

When invoked by Copilot, run the script in a terminal and answer any prompts
on the user's behalf based on context, or relay the prompts to the user.

1. **Scans** the folder for `.wpilog` files and subdirectories (both
   timestamped like `2026-03-06_04-21-49` and event-labeled like `WABON_Q42`).
2. **Skips** zero-byte `.wpilog` files. Includes `FRC_TBD_*` files (pre-DS
   logs where the Driver Station never connected -- valid data, uses file
   modification time for sorting).
3. **Auto-matches** folders to `.wpilog` files using:
   - **Event label matching** (highest priority): `WABON_P2` folder to
     `FRC_..._WABON_P2.wpilog`
   - **Timestamp proximity**: closest `.wpilog` within the tolerance window
   - For event-labeled folders, timestamps are extracted from `.hoot`
     filenames inside
4. **Event match `.wpilog` files are always kept** -- they are never listed
   as unmatched and never require user prompts. Split-match logs (same
   label due to power loss) are merged into one zip.
5. **Prompts** the user to resolve remaining unmatched items, with **bulk
   options** for large sets (Ignore All / Zip All / Resolve Each).
6. **Groups** matched items into **batches** that stay under `MaxZipSizeMB`
   (default 100 MB uncompressed), splitting at the **largest timestamp gaps**
   to keep related logs together.
7. **Shows the final plan** and asks for confirmation.
8. **Creates `.zip` archives in parallel** (default 4 concurrent jobs) and
   prints a summary with sizes.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `-LogFolder` | *(prompt)* | Path to the folder with `.wpilog` files and subdirs |
| `-ToleranceMinutes` | `2` | Max minutes of timestamp drift for auto-matching |
| `-MaxZipSizeMB` | `100` | Target max uncompressed size per zip (for OneDrive) |
| `-MaxParallel` | `4` | Number of zip compression jobs to run concurrently |

## Reference

### `.wpilog` file naming (from WPILib docs)

Files go through up to three naming stages:

| Stage | Pattern | When |
|-------|---------|------|
| Pre-DS | `FRC_TBD_{random}.wpilog` | At boot, before Driver Station connects |
| DS connected | `FRC_YYYYMMDD_HHMMSS.wpilog` | After DS connects (UTC timestamp) |
| FMS match | `FRC_YYYYMMDD_HHMMSS_{event}_{match}.wpilog` | When FMS provides match info |

`FRC_TBD_*` files are valid logs -- the DS simply never connected. They are
preserved and included in batches.

### Supported folder types

| Pattern | Example | Matching method |
|---------|---------|-----------------|
| Timestamped | `2026-03-06_04-21-49` | Closest `.wpilog` by timestamp |
| Event match | `WABON_Q42` | Name match to `FRC_..._WABON_Q42.wpilog` |
| Event match (empty) | `WABON_E4` (no .hoot files) | Name match only |

### Event match labels

Format: `<EventCode>_<Type><Number>` where Type is `P` (practice),
`Q` (qualification), `E` (elimination), or `F` (finals).

Event-labeled `.wpilog` files are **always included** -- they never appear
in "unmatched" prompts. When multiple `.wpilog` files share the same
label (power loss mid-match), they are **merged into one zip** along with
the shared `.hoot` folder.

### Matching rules

- **Event label match** takes priority over timestamp matching.
- **Timestamp match** uses closest-match within the tolerance.
- Multiple folders can match one `.wpilog` file.
- Zero-byte `.wpilog` files are skipped; `FRC_TBD_*` files are kept.

### Batching rules

- All items are sorted chronologically and grouped into batches.
- Batches exceeding `MaxZipSizeMB` are split at the **largest timestamp
  gap**, maximizing natural separation between sessions.
- Single-item batches use the `.wpilog` basename as the zip filename.
- Multi-item batches are named `FRC_<first>_to_<last>.zip`.
- Event match items use `FRC_<label>.zip` (e.g., `FRC_WABON_Q65.zip`).

### Compression

- Uses `Start-Job` to run up to `-MaxParallel` (default 4) compression
  jobs concurrently, significantly reducing total time for large log sets.
- A warning is shown if any batch exceeds the ~2 GB `Compress-Archive`
  limit in PowerShell 5.1.

### Safety

- **The script never deletes or moves original files** -- it only creates
  new `.zip` archives alongside them.
- Existing `.zip` files with the same name are overwritten after confirmation.
