# FRC Robot Log File Specification

This document describes the file and folder naming conventions used by
FRC (FIRST Robotics Competition) robots running WPILib and CTRE Phoenix
software.  It serves as a reference for anyone working with these logs,
whether manually or via the `PackageLogs.ps1` script.

Reference: https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html

---

## Overview

An FRC robot produces two independent categories of log during operation:

| Source | Format | Description |
|--------|--------|-------------|
| WPILib DataLogManager | `.wpilog` | Main robot log -- sensor data, commands, DS/joystick data |
| CTRE Phoenix signal logger | `.hoot` | CAN bus signal recordings from Phoenix devices |

Both start logging within a few seconds of each other when the robot boots.
They are downloaded separately and end up in the same folder for archival.

---

## `.wpilog` File Lifecycle

When the robot code starts, `DataLogManager` creates a log file on the USB
flash drive (FAT32, in a folder called `logs`) or at `/home/lvuser/logs`.

### Naming stages

A `.wpilog` file goes through up to three naming stages during its life:

| Stage | Filename pattern | When |
|-------|-----------------|------|
| 1. Pre-DS | `FRC_TBD_{random}.wpilog` | Immediately at boot, before Driver Station connects |
| 2. DS connected | `FRC_YYYYMMDD_HHMMSS.wpilog` | Once the Driver Station connects (timestamp is UTC) |
| 3. FMS match | `FRC_YYYYMMDD_HHMMSS_{event}_{match}.wpilog` | When FMS provides a match number |

The file is **renamed in place** at each stage -- it is the same file, not
a new one.  The random identifier in stage 1 is a hex string.

### Naming pattern details

```
FRC_TBD_{random}.wpilog                       <- stage 1 (pre-DS)
FRC_YYYYMMDD_HHMMSS.wpilog                   <- stage 2 (DS connected)
FRC_YYYYMMDD_HHMMSS_<EventCode>_<Match>.wpilog  <- stage 3 (FMS match)
```

| Component | Format | Example | Notes |
|-----------|--------|---------|-------|
| Prefix | `FRC_` | `FRC_` | Always present |
| TBD marker | `TBD_{hex}` | `TBD_bdcc977b95194543` | Only if DS never connected |
| Date | `YYYYMMDD` | `20260308` | No separators, UTC |
| Time | `HHMMSS` | `172905` | 24-hour, no separators, UTC |
| Event label | `_{Code}_{Type}{Num}` | `_WABON_Q65` | Only when FMS is connected |

### Examples

```
FRC_TBD_bdcc977b95194543.wpilog         <- DS never connected (testing, brief power-on)
FRC_TBD_aad4a9c614288d68.wpilog         <- DS never connected (simulation)
FRC_20260306_043856.wpilog              <- practice / testing session
FRC_20260307_180601_WABON_P2.wpilog     <- practice match 2 at Bonney Lake
FRC_20260308_172905_WABON_Q65.wpilog    <- qualification match 65
FRC_20260308_225734_WABON_E14.wpilog    <- elimination match 14
```

### Startup cleanup (by DataLogManager)

When the robot code starts up:
1. All existing `FRC_TBD_*` log files on the drive are **deleted** by
   DataLogManager (since the DS never connected for those sessions).
2. If free space is below 50 MB, `FRC_*` files are deleted oldest-first
   until 50 MB is free or only 10 files remain.

**Important:** Once downloaded to a PC, `FRC_TBD_*` files are safe from
this cleanup and should be preserved -- they contain valid log data from
sessions where the DS never connected (e.g., bench testing, brief
power-ons, or simulation runs).

### Edge cases

| Case | Example | Notes |
|------|---------|-------|
| Zero bytes | `FRC_20260308_192235.wpilog` (0 B) | Robot rebooted before any data was logged |
| Pre-DS (TBD) | `FRC_TBD_bdcc977b95194543.wpilog` | Valid log, DS just never connected |
| TBD with suffix | `FRC_TBD_bdcc977b95194543.2.wpilog` | Multiple TBD files in same session |
| Duplicate event label | Two `..._WABON_Q65.wpilog` files | Power loss mid-match caused a split log |

### Duplicate event labels (power loss)

When the robot loses power during a competition match and reboots, a
**second** `.wpilog` file is created with the **same event label**:

```
FRC_20260308_172905_WABON_Q65.wpilog    <- first half (before power loss)
FRC_20260308_180752_WABON_Q65.wpilog    <- second half (after reboot)
```

Both files are critical -- together they form the complete log of that
match.  The corresponding `.hoot` folder will also contain recordings
from both sessions.

**All files with the same event label must be kept together** in one zip.

---

## `.hoot` Folders

Signal logger recordings are stored in folders.  There are two naming
styles depending on whether the recording happened during a competition
match or during practice/testing.

### Style 1: Timestamped folders (practice / testing)

```
YYYY-MM-DD_HH-MM-SS/
```

Example: `2026-03-06_04-38-48/`

Note the **dashes** in both the date and time portions -- this differs
from the `.wpilog` format which uses no separators.

**Contents:**

```
2026-03-06_04-38-48/
  +-- ECA0983C3353385320202034170A03FF_2026-03-06_04-38-51.hoot
  +-- rio_2026-03-06_04-38-51.hoot
```

### Style 2: Event-labeled folders (competition matches)

```
<EventCode>_<Type><Number>/
```

Example: `WABON_Q42/`

Event-labeled folders may contain recordings from **multiple sessions**
of the same match (e.g., if the robot lost power and rebooted).  Some
may be **empty** if the signal logger failed to start.

---

## `.hoot` Files

### Naming pattern

```
[<Prefix>_]<DeviceID>_YYYY-MM-DD_HH-MM-SS.hoot
[<Prefix>_]rio_YYYY-MM-DD_HH-MM-SS.hoot
```

| Component | Description | Example |
|-----------|-------------|---------|
| Prefix | Event label (only in match folders) | `WABON_Q65_` |
| Device ID | CAN device serial (hex) or `rio` | `ECA0983C...FF` or `rio` |
| Timestamp | `YYYY-MM-DD_HH-MM-SS` | `2026-03-08_17-29-01` |

The `rio` file records signals from the roboRIO.  The hex-ID file records
signals from a CTRE CAN device (e.g., Talon motor controller).

---

## Event Match Labels

### Format

```
<EventCode>_<Type><Number>
```

| Field | Values | Description |
|-------|--------|-------------|
| EventCode | `WABON`, `CASJ`, `ORORE`, etc. | FRC event code (uppercase) |
| Type | `P` / `Q` / `E` / `F` | Practice / Qualification / Elimination / Finals |
| Number | `1`, `42`, `65`, etc. | Match number within that type |

**Event-labeled logs are the most important** -- they represent official
competition performance and should always be preserved.

---

## Timestamp Relationships

When the robot boots, logging systems start up in this order:

1. **WPILib DataLogManager** starts -> creates `.wpilog` file (timestamp `T`)
2. **Phoenix signal logger** starts ~1-8s later -> creates `.hoot` folder

So the `.wpilog` timestamp is always **slightly before** the `.hoot`
folder timestamp.

### Typical session structure

```
  Time ---------------------------------------------------------------->

  FRC_20260308_030127.wpilog  <- robot boot 1
  2026-03-08_03-01-20/        <- hoot folder (7 sec later)

                     ~~~ 2 min gap (robot off) ~~~

  FRC_20260308_030436.wpilog  <- robot boot 2
  2026-03-08_03-04-29/        <- hoot folder (7 sec later)
```

### Power loss during a match

```
  FRC_20260308_172905_WABON_Q65.wpilog   <- match starts
  WABON_Q65/ (hoot session 1)

       ~~~ POWER LOSS ~~~

  FRC_20260308_180752_WABON_Q65.wpilog   <- robot reboots, same match label
  WABON_Q65/ (hoot session 2 added to same folder)
```

---

## Folder Structure Example

```
2412/                                       <- team number
  +-- FRC_TBD_aad4a9c614288d68.wpilog       <- bench test, DS never connected
  +-- FRC_20260306_043856.wpilog            <- practice day logs
  +-- 2026-03-06_04-38-48/                  <- matching hoot folder
  |
  +-- FRC_20260307_180601_WABON_P2.wpilog   <- competition match logs
  +-- WABON_P2/                             <- matching event folder
  |
  +-- FRC_20260308_172905_WABON_Q65.wpilog  <- split match (power loss)
  +-- FRC_20260308_180752_WABON_Q65.wpilog  <- same label, second half
  +-- WABON_Q65/                            <- folder has both sessions
  |
  +-- FRC_20260308_192235.wpilog            <- zero-byte (skip)
```
