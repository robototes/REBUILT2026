<#
.SYNOPSIS
    Package FRC log files (.wpilog) with matching .hoot folders into .zip archives.

.DESCRIPTION
    Scans a folder for .wpilog files and subdirectories containing .hoot files.
    Supports both timestamped folders (2026-03-06_04-21-49) and match-labeled
    folders (WABON_Q42).  Auto-matches, groups into size-limited batches by
    splitting at the largest timestamp gaps, and creates .zip archives.

.PARAMETER LogFolder
    Path to the folder containing .wpilog files and subdirectories.

.PARAMETER ToleranceMinutes
    Max minutes of timestamp drift for auto-matching.  Default 2.

.PARAMETER MaxZipSizeMB
    Target max uncompressed size per zip in MB.  Default 100.

.PARAMETER MaxParallel
    Number of zip compression jobs to run in parallel.  Default 4.

.EXAMPLE
    .\PackageLogs.ps1 -LogFolder "D:\Temp\2026_BonneyLake\2412"
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$LogFolder,
    [int]$ToleranceMinutes = 2,
    [int]$MaxZipSizeMB = 100,
    [int]$MaxParallel = 4
)
Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# ─────────────────────────────────────────────────────────────────────────────
# Script flow:
#   Step 0  Get the log folder path (prompt if not provided)
#   Step 1  Enumerate .wpilog files and hoot folders
#   Step 2  Auto-match folders to .wpilog files (by label, then timestamp)
#   Step 3  Resolve any unmatched folders/files (with bulk options)
#   Step 4  Group everything into size-limited zip batches
#   Step 5  Confirm plan and create .zip archives
#   Step 6  Print summary
#
# See SPEC.md in this folder for details on file naming conventions.
# ─────────────────────────────────────────────────────────────────────────────

#region Helpers

function Format-FileSize([long]$Bytes) {
    if ($Bytes -ge 1GB) { return "{0:N2} GB" -f ($Bytes / 1GB) }
    if ($Bytes -ge 1MB) { return "{0:N1} MB" -f ($Bytes / 1MB) }
    if ($Bytes -ge 1KB) { return "{0:N1} KB" -f ($Bytes / 1KB) }
    return "$Bytes B"
}

function Parse-WpilogTimestamp([string]$FileName) {
    if ($FileName -match '(\d{8})_(\d{6})') {
        return [datetime]::ParseExact("$($Matches[1])$($Matches[2])", "yyyyMMddHHmmss", $null)
    }
    return $null
}

function Parse-WpilogMatchLabel([string]$FileName) {
    # Match event labels like _WABON_Q65, _CASJ_E14, _ORORE_P2
    # Format: _<EventCode>_<Type><Number>
    #   EventCode = uppercase letters (FRC event code, e.g. WABON)
    #   Type      = P (practice), Q (qualification), E (elimination), F (finals)
    #   Number    = match number (1, 2, 42, 65, etc.)
    if ($FileName -match '_([A-Z]+_[PQEF]\d+)\.wpilog$') { return $Matches[1] }
    return $null
}

function Test-IsEventLog([PSCustomObject]$WpilogEntry) {
    # Returns true if the wpilog has an event match label
    return ($null -ne $WpilogEntry.MatchLabel)
}

function Parse-FolderTimestamp([string]$FolderName) {
    if ($FolderName -match '^(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})$') {
        $d = $Matches[1]
        $t = $Matches[2] -replace '-', ':'
        return [datetime]::Parse("$d $t")
    }
    return $null
}

function Get-HootTimestamp([string]$FolderPath) {
    $hoot = Get-ChildItem -Path $FolderPath -Filter "*.hoot" -File -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($hoot -and $hoot.Name -match '(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})\.hoot$') {
        $d = $Matches[1]
        $t = $Matches[2] -replace '-', ':'
        return [datetime]::Parse("$d $t")
    }
    return $null
}

function Get-DirSize([string]$Path) {
    $items = Get-ChildItem -Path $Path -Recurse -File -ErrorAction SilentlyContinue
    if ($items) { return ($items | Measure-Object -Property Length -Sum).Sum }
    return [long]0
}

function Write-Header([string]$Text) {
    $bar = "=" * 70
    Write-Host ""
    Write-Host $bar -ForegroundColor Cyan
    Write-Host "  $Text" -ForegroundColor Cyan
    Write-Host $bar -ForegroundColor Cyan
}

function Write-Sep() { Write-Host ("-" * 70) -ForegroundColor DarkGray }

#endregion

#region Step 0 - Get folder path

if (-not $LogFolder) {
    Write-Host ""
    $LogFolder = Read-Host "Enter the path to the folder containing .wpilog files"
}
$LogFolder = $LogFolder.Trim('"').Trim("'")
if (-not (Test-Path $LogFolder -PathType Container)) {
    Write-Host "ERROR: Folder not found: $LogFolder" -ForegroundColor Red
    exit 1
}
$LogFolder = (Resolve-Path $LogFolder).Path
Write-Host "Log folder : $LogFolder" -ForegroundColor Gray
Write-Host "Tolerance  : $ToleranceMinutes min" -ForegroundColor Gray
Write-Host "Max zip    : $MaxZipSizeMB MB (uncompressed)" -ForegroundColor Gray
Write-Host "Parallel   : $MaxParallel jobs" -ForegroundColor Gray

#endregion

#region Step 1 - Enumerate

Write-Header "Step 1: Scanning"

$script:skippedCount = 0
$wpilogFiles = @(Get-ChildItem -Path $LogFolder -Filter "*.wpilog" -File | ForEach-Object {
    if ($_.Length -eq 0) {
        Write-Host "  SKIP (0 bytes): $($_.Name)" -ForegroundColor DarkGray
        $script:skippedCount++
        return
    }
    $ts = Parse-WpilogTimestamp $_.Name
    $isTBD = $false
    if (-not $ts) {
        # FRC_TBD_{random}.wpilog -- DS never connected, but still valid data.
        # Use the file's last-write time as the timestamp for sorting/batching.
        if ($_.Name -match '^FRC_TBD_') {
            $ts = $_.LastWriteTime
            $isTBD = $true
            Write-Host "  TBD (pre-DS): $($_.Name) -- using file date $($ts.ToString('yyyy-MM-dd HH:mm:ss'))" -ForegroundColor DarkYellow
        } else {
            Write-Host "  SKIP (no timestamp): $($_.Name)" -ForegroundColor Yellow
            $script:skippedCount++
            return
        }
    }
    $label = Parse-WpilogMatchLabel $_.Name
    [PSCustomObject]@{
        Name       = $_.Name
        FullPath   = $_.FullName
        Timestamp  = $ts
        Size       = $_.Length
        BaseName   = [System.IO.Path]::GetFileNameWithoutExtension($_.Name)
        MatchLabel = $label
        IsTBD      = $isTBD
    }
} | Sort-Object Timestamp)

if ($script:skippedCount -gt 0) { Write-Host "  ($($script:skippedCount) file(s) skipped)" -ForegroundColor DarkGray }
if ($wpilogFiles.Count -eq 0) {
    Write-Host "  No usable .wpilog files found." -ForegroundColor Red
    exit 0
}

Write-Host ""
Write-Host "  $($wpilogFiles.Count) .wpilog files:" -ForegroundColor White
Write-Host ("  {0,-4} {1,-48} {2,-20} {3,10}" -f "#", "Filename", "Timestamp", "Size")
Write-Sep
for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
    $w = $wpilogFiles[$i]
    $displayName = $w.Name; if ($displayName.Length -gt 47) { $displayName = $displayName.Substring(0,44) + "..." }
    Write-Host ("  {0,-4} {1,-48} {2,-20} {3,10}" -f ($i+1), $displayName, $w.Timestamp.ToString("yyyy-MM-dd HH:mm:ss"), (Format-FileSize $w.Size))
}

#endregion

#region Step 1b - Enumerate folders

# Note: Inside ForEach-Object, "return" acts like "continue" (skips to the
# next pipeline item), NOT like a function return.  This is a PowerShell quirk.
$allFolders = @(Get-ChildItem -Path $LogFolder -Directory | ForEach-Object {
    $name = $_.Name
    $full = $_.FullName
    $ts = Parse-FolderTimestamp $name
    if ($ts) {
        [PSCustomObject]@{ Name=$name; FullPath=$full; Timestamp=$ts; MatchLabel=$null; IsEmpty=$false }
        return
    }
    if ($name -match '^([A-Z]+_[PQEF]\d+)$') {
        $matchLabel = $Matches[1]
        $hootTs = Get-HootTimestamp $full
        $empty = ($null -eq $hootTs)
        if (-not $hootTs) { $hootTs = [datetime]::MinValue }
        [PSCustomObject]@{ Name=$name; FullPath=$full; Timestamp=$hootTs; MatchLabel=$matchLabel; IsEmpty=$empty }
        return
    }
} | Sort-Object Timestamp)

$tsCount    = @($allFolders | Where-Object { -not $_.MatchLabel }).Count
$labelCount = @($allFolders | Where-Object { $null -ne $_.MatchLabel }).Count
Write-Host ""
Write-Host "  Folders: $tsCount timestamped, $labelCount match-labeled" -ForegroundColor White

#endregion

#region Step 2 - Match folders to wpilog files

Write-Header "Step 2: Matching folders to .wpilog files"

$toleranceSec = $ToleranceMinutes * 60
$matchMap = @{}
for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
    # [void] suppresses ArrayList.Add() return value (it returns the index,
    # which would pollute the pipeline output if not suppressed).
    $matchMap[$i] = New-Object System.Collections.ArrayList
}
$unmatchedFolders = New-Object System.Collections.ArrayList

foreach ($folder in $allFolders) {
    $matched = $false

    # ── Priority 1: Match by event label name ────────────────────────
    # If the folder is event-labeled (e.g. "WABON_Q42"), look for a
    # .wpilog file with the same label in its filename.
    if ($folder.MatchLabel) {
        for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
            if ($wpilogFiles[$i].MatchLabel -eq $folder.MatchLabel) {
                [void]$matchMap[$i].Add($folder)
                $matched = $true
                break  # assign to first (earliest) matching wpilog
            }
        }
        # If no label match found, fall back to timestamp matching
        # (but only if the folder has hoot files to get a timestamp from)
        if (-not $matched -and -not $folder.IsEmpty) {
            $bestIdx = -1; $bestDiff = [double]::MaxValue
            for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
                $absDiff = [Math]::Abs(($folder.Timestamp - $wpilogFiles[$i].Timestamp).TotalSeconds)
                if ($absDiff -lt $bestDiff) { $bestIdx = $i; $bestDiff = $absDiff }
            }
            if ($bestIdx -ge 0 -and $bestDiff -le $toleranceSec) {
                [void]$matchMap[$bestIdx].Add($folder)
                $matched = $true
            }
        }
    }

    # ── Priority 2: Match by closest timestamp ───────────────────────
    # For timestamped folders (e.g. "2026-03-06_04-38-48"), find the
    # .wpilog file whose timestamp is closest, within tolerance.
    if (-not $matched -and -not $folder.MatchLabel) {
        $bestIdx = -1; $bestDiff = [double]::MaxValue
        for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
            $absDiff = [Math]::Abs(($folder.Timestamp - $wpilogFiles[$i].Timestamp).TotalSeconds)
            if ($absDiff -lt $bestDiff) { $bestIdx = $i; $bestDiff = $absDiff }
        }
        if ($bestIdx -ge 0 -and $bestDiff -le $toleranceSec) {
            [void]$matchMap[$bestIdx].Add($folder)
            $matched = $true
        }
    }

    if (-not $matched) { [void]$unmatchedFolders.Add($folder) }
}

# Display
$matchedWpiCount = 0; $noMatchWpiCount = 0
Write-Host ""
for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
    $flist = $matchMap[$i]
    if ($flist.Count -gt 0) {
        $matchedWpiCount++
        Write-Host ("  {0}. {1}" -f ($i+1), $wpilogFiles[$i].Name) -ForegroundColor White
        foreach ($f in $flist) { Write-Host "       + $($f.Name)" -ForegroundColor Green }
    } else { $noMatchWpiCount++ }
}
if ($noMatchWpiCount -gt 0) {
    Write-Host ""; Write-Host "  ($noMatchWpiCount .wpilog file(s) have no matched folders)" -ForegroundColor Yellow
}
if ($unmatchedFolders.Count -gt 0) {
    Write-Host "  ($($unmatchedFolders.Count) folder(s) unmatched)" -ForegroundColor Yellow
}

#endregion

#region Step 3 - Resolve unmatched items

if ($unmatchedFolders.Count -gt 0) {
    Write-Header "Step 3a: Resolve $($unmatchedFolders.Count) unmatched folder(s)"
    Write-Host ""
    for ($j = 0; $j -lt $unmatchedFolders.Count; $j++) {
        $uf = $unmatchedFolders[$j]
        $extra = ""; if ($uf.IsEmpty) { $extra = " (empty)" }
        Write-Host "    $($j+1). $($uf.Name)$extra"
    }
    Write-Host ""
    Write-Host "  Bulk options:" -ForegroundColor White
    Write-Host "    [A] Ignore ALL unmatched folders"
    Write-Host "    [R] Resolve each one individually"
    Write-Host ""
    $bulkChoice = (Read-Host "  Choose [A/R]").Trim().ToUpper()

    if ($bulkChoice -ne "A") {
        foreach ($folder in $unmatchedFolders) {
            Write-Host ""
            $extra = ""; if ($folder.IsEmpty) { $extra = " (empty)" }
            Write-Host "  Folder '$($folder.Name)'$extra" -ForegroundColor Yellow
            Write-Host "    [I]   Ignore"
            Write-Host "    [S]   Standalone zip"
            Write-Host "    [1-N] Assign to wpilog #"
            Write-Host ""
            $valid = $false
            while (-not $valid) {
                $ch = (Read-Host "  Choice").Trim().ToUpper()
                if ($ch -eq "I") {
                    Write-Host "    -> Ignored" -ForegroundColor DarkGray; $valid = $true
                } elseif ($ch -eq "S") {
                    $sIdx = $wpilogFiles.Count
                    $sEntry = [PSCustomObject]@{ Name=$null; FullPath=$null; Timestamp=$folder.Timestamp; Size=[long]0; BaseName=$folder.Name; MatchLabel=$null; IsTBD=$false }
                    $wpilogFiles += $sEntry
                    $matchMap[$sIdx] = New-Object System.Collections.ArrayList
                    [void]$matchMap[$sIdx].Add($folder)
                    Write-Host "    -> Standalone zip" -ForegroundColor Cyan; $valid = $true
                } else {
                    $num = 0
                    if ([int]::TryParse($ch, [ref]$num) -and $num -ge 1 -and $num -le $wpilogFiles.Count) {
                        $idx = $num - 1
                        if (-not $matchMap.ContainsKey($idx)) { $matchMap[$idx] = New-Object System.Collections.ArrayList }
                        [void]$matchMap[$idx].Add($folder)
                        Write-Host "    -> Assigned to $($wpilogFiles[$idx].Name)" -ForegroundColor Green; $valid = $true
                    } else { Write-Host "    Invalid." -ForegroundColor Red }
                }
            }
        }
    } else {
        Write-Host "    -> All $($unmatchedFolders.Count) folders ignored" -ForegroundColor DarkGray
    }
}

$unmatchedWpilogs = @()
$autoKeptEventLogs = @()
for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
    if ($null -eq $wpilogFiles[$i].FullPath) { continue }
    if ($matchMap[$i].Count -eq 0) {
        if (Test-IsEventLog $wpilogFiles[$i]) {
            # Event match logs are always kept — never prompt
            $autoKeptEventLogs += $i
        } else {
            $unmatchedWpilogs += $i
        }
    }
}

if ($autoKeptEventLogs.Count -gt 0) {
    Write-Host ""
    Write-Host "  $($autoKeptEventLogs.Count) event match log(s) auto-included (no folders needed):" -ForegroundColor Cyan
    foreach ($idx in $autoKeptEventLogs) {
        Write-Host "    $($wpilogFiles[$idx].Name)" -ForegroundColor Cyan
    }
}

if ($unmatchedWpilogs.Count -gt 0) {
    Write-Header "Step 3b: $($unmatchedWpilogs.Count) .wpilog file(s) have no folders"
    Write-Host ""
    foreach ($idx in $unmatchedWpilogs) {
        Write-Host "    $($wpilogFiles[$idx].Name)  ($(Format-FileSize $wpilogFiles[$idx].Size))"
    }
    Write-Host ""
    Write-Host "  Bulk options:" -ForegroundColor White
    Write-Host "    [Z] Include ALL in batches (zip alone)"
    Write-Host "    [I] Ignore ALL"
    Write-Host "    [R] Resolve each individually"
    Write-Host ""
    $bulkChoice = (Read-Host "  Choose [Z/I/R]").Trim().ToUpper()
    if ($bulkChoice -eq "I") {
        foreach ($idx in $unmatchedWpilogs) { $matchMap.Remove($idx) }
        Write-Host "    -> All ignored" -ForegroundColor DarkGray
    } elseif ($bulkChoice -eq "R") {
        foreach ($idx in $unmatchedWpilogs) {
            $w = $wpilogFiles[$idx]
            Write-Host ""
            Write-Host "  '$($w.Name)' ($(Format-FileSize $w.Size))" -ForegroundColor Yellow
            Write-Host "    [Z] Zip alone  [I] Ignore"
            $valid = $false
            while (-not $valid) {
                $c = (Read-Host "  Choice").Trim().ToUpper()
                if ($c -eq "Z") { Write-Host "    -> Include" -ForegroundColor Cyan; $valid = $true }
                elseif ($c -eq "I") { $matchMap.Remove($idx); Write-Host "    -> Ignored" -ForegroundColor DarkGray; $valid = $true }
                else { Write-Host "    Enter Z or I." -ForegroundColor Red }
            }
        }
    } else {
        Write-Host "    -> All will be included in batches" -ForegroundColor Cyan
    }
}

#endregion

#region Step 4 - Build batches

Write-Header "Step 4: Building zip batches"

$planItems = @()
$planCount = ($matchMap.Keys | Measure-Object).Count
$planCurrent = 0
foreach ($idx in ($matchMap.Keys | Sort-Object)) {
    $planCurrent++
    Write-Host "`r  Calculating sizes... $planCurrent / $planCount" -NoNewline
    $w = $wpilogFiles[$idx]
    $folders = @($matchMap[$idx])
    $isStandalone = ($null -eq $w.FullPath)
    [long]$itemSize = 0
    if (-not $isStandalone) { $itemSize += $w.Size }
    foreach ($f in $folders) { $itemSize += (Get-DirSize $f.FullPath) }
    $planItems += [PSCustomObject]@{
        WpilogFiles = @($w)          # array — may grow when merging split matches
        Folders     = $folders
        IsStandalone = $isStandalone
        Timestamp   = $w.Timestamp   # earliest timestamp (for sorting/batching)
        TotalSize   = $itemSize
        MatchLabel  = $w.MatchLabel   # event label or $null
    }
}
$planItems = @($planItems | Sort-Object Timestamp)
Write-Host ""  # clear the progress line

# ── Merge split-match items ──────────────────────────────────────────
# When multiple .wpilog files share the same event label (e.g., two
# WABON_Q65 files due to a mid-match power loss), merge them into a
# single plan item so they end up in the same zip file.
$mergedItems = New-Object System.Collections.ArrayList
$labelGroups = @{}  # label -> index in $mergedItems

foreach ($item in $planItems) {
    if ($item.MatchLabel) {
        if ($labelGroups.ContainsKey($item.MatchLabel)) {
            # Merge into existing item
            $existingIdx = $labelGroups[$item.MatchLabel]
            $existing = $mergedItems[$existingIdx]
            $existing.WpilogFiles += $item.WpilogFiles
            $existing.TotalSize  += $item.TotalSize
            # Merge folders (avoid duplicates by path)
            foreach ($f in $item.Folders) {
                $alreadyHave = $false
                foreach ($ef in $existing.Folders) {
                    if ($ef.FullPath -eq $f.FullPath) { $alreadyHave = $true; break }
                }
                if (-not $alreadyHave) {
                    $existing.Folders += $f
                    # Note: folder size is NOT added here because it was already
                    # counted in $item.TotalSize which was added above.
                }
            }
            # Keep the earliest timestamp
            if ($item.Timestamp -lt $existing.Timestamp) {
                $existing.Timestamp = $item.Timestamp
            }
        } else {
            $labelGroups[$item.MatchLabel] = $mergedItems.Count
            [void]$mergedItems.Add($item)
        }
    } else {
        [void]$mergedItems.Add($item)
    }
}

$planItems = @($mergedItems | Sort-Object Timestamp)

# Report any merges
$mergeCount = 0
foreach ($item in $planItems) {
    if ($item.WpilogFiles.Count -gt 1) {
        $mergeCount++
        $label = $item.MatchLabel
        Write-Host "  Merged $($item.WpilogFiles.Count) split logs for $label" -ForegroundColor Cyan
    }
}
if ($mergeCount -gt 0) { Write-Host "" }

if ($planItems.Count -eq 0) {
    Write-Host "  Nothing to do." -ForegroundColor Yellow
    exit 0
}

$maxBytes = [long]$MaxZipSizeMB * 1MB

# ── Batch-splitting algorithm ────────────────────────────────────────
# Start with ALL plan items in a single batch.  If that batch exceeds
# the size limit, find the largest time gap between consecutive items
# and split there.  Repeat until every batch is under the limit (or
# can't be split further because it contains only one item).
# This maximizes the natural time separation between zips.
$batchList = New-Object System.Collections.ArrayList
[void]$batchList.Add(@($planItems))

$splitAgain = $true
while ($splitAgain) {
    $splitAgain = $false
    $newList = New-Object System.Collections.ArrayList
    foreach ($batch in $batchList) {
        [long]$bSize = 0
        foreach ($it in $batch) { $bSize += $it.TotalSize }
        if ($bSize -gt $maxBytes -and $batch.Count -gt 1) {
            $bestGap = -1; $bestGapSec = [double]-1
            for ($g = 0; $g -lt ($batch.Count - 1); $g++) {
                $gapSec = ($batch[$g+1].Timestamp - $batch[$g].Timestamp).TotalSeconds
                if ($gapSec -gt $bestGapSec) { $bestGap = $g; $bestGapSec = $gapSec }
            }
            if ($bestGap -ge 0) {
                $left  = @($batch[0..$bestGap])
                $right = @($batch[($bestGap+1)..($batch.Count-1)])
                [void]$newList.Add($left)
                [void]$newList.Add($right)
                $splitAgain = $true
            } else {
                [void]$newList.Add($batch)
            }
        } else {
            [void]$newList.Add($batch)
        }
    }
    $batchList = $newList
}

$batchPlans = @()
for ($b = 0; $b -lt $batchList.Count; $b++) {
    $batch = $batchList[$b]
    $first = $batch[0].Timestamp
    $last  = $batch[$batch.Count - 1].Timestamp
    if ($batch.Count -eq 1) {
        # Single item — use the label or basename
        $item = $batch[0]
        if ($item.MatchLabel) {
            $zipBase = "FRC_" + $item.MatchLabel
        } else {
            $zipBase = $item.WpilogFiles[0].BaseName
        }
    } else {
        $zipBase = "FRC_" + $first.ToString("yyyyMMdd_HHmmss") + "_to_" + $last.ToString("yyyyMMdd_HHmmss")
    }
    $zipName = "$zipBase.zip"
    $zipPath = Join-Path $LogFolder $zipName
    [long]$batchSize = 0
    foreach ($it in $batch) { $batchSize += $it.TotalSize }
    $batchPlans += [PSCustomObject]@{
        BatchNum=$b+1; Items=$batch; ZipName=$zipName; ZipPath=$zipPath; TotalSize=$batchSize
    }
}

Write-Host ""
Write-Host "  $($batchPlans.Count) zip archive(s) planned:" -ForegroundColor White

# Warn about Compress-Archive 2GB limit in Windows PowerShell 5.1
foreach ($bp in $batchPlans) {
    if ($bp.TotalSize -gt 2GB) {
        Write-Host ""
        Write-Host "  WARNING: Batch '$($bp.ZipName)' is ~$(Format-FileSize $bp.TotalSize) uncompressed." -ForegroundColor Red
        Write-Host "  Compress-Archive in PowerShell 5.1 has a ~2 GB limit and may fail." -ForegroundColor Red
        Write-Host "  Consider reducing -MaxZipSizeMB to split it further." -ForegroundColor Red
    }
}

Write-Host ""
foreach ($bp in $batchPlans) {
    $sizeStr = Format-FileSize $bp.TotalSize
    Write-Host "  Batch $($bp.BatchNum): $($bp.ZipName)" -ForegroundColor Cyan
    Write-Host "    (~$sizeStr uncompressed, $($bp.Items.Count) item(s))" -ForegroundColor Gray
    foreach ($it in $bp.Items) {
        if ($it.IsStandalone) {
            Write-Host "    [standalone]" -ForegroundColor DarkGray
        } else {
            foreach ($wf in $it.WpilogFiles) {
                Write-Host "    $($wf.Name)  ($(Format-FileSize $wf.Size))" -ForegroundColor White
            }
        }
        foreach ($f in $it.Folders) { Write-Host "      + $($f.Name)/" -ForegroundColor Green }
    }
    Write-Host ""
}

#endregion

#region Step 5 - Confirm and create archives

$confirm = (Read-Host "Proceed? [Y/N]").Trim().ToUpper()
if ($confirm -ne "Y") { Write-Host "Cancelled." -ForegroundColor Yellow; exit 0 }

Write-Header "Step 5: Creating archives ($MaxParallel parallel)"

# Build a list of compression tasks (each is a hashtable with paths)
$tasks = @()
foreach ($bp in $batchPlans) {
    $itemsToCompress = @()
    [int]$wpiCount = 0; [int]$folderCount = 0
    foreach ($it in $bp.Items) {
        if (-not $it.IsStandalone) {
            foreach ($wf in $it.WpilogFiles) {
                $itemsToCompress += $wf.FullPath
                $wpiCount++
            }
        }
        foreach ($f in $it.Folders) { $itemsToCompress += $f.FullPath; $folderCount++ }
    }
    if ($itemsToCompress.Count -eq 0) { continue }
    $tasks += @{
        ZipName  = $bp.ZipName
        ZipPath  = $bp.ZipPath
        Items    = $itemsToCompress
        WpiCount = $wpiCount
        FolderCount = $folderCount
    }
}

# Remove any pre-existing zips that we are about to recreate
foreach ($t in $tasks) {
    if (Test-Path $t.ZipPath) { Remove-Item $t.ZipPath -Force }
}

# Launch compression jobs in parallel, throttled to $MaxParallel
$runningJobs = @{}   # jobId -> task hashtable
$completedIdx = 0
$totalTasks = $tasks.Count
$createdZips = @()
$failedZips  = @()

for ($taskIdx = 0; $taskIdx -lt $tasks.Count; $taskIdx++) {
    # Wait if we've hit the concurrency limit
    while ($runningJobs.Count -ge $MaxParallel) {
        # @() forces array even when there's only one running job
        $finished = Get-Job -Id @($runningJobs.Keys) | Where-Object { $_.State -ne 'Running' } | Select-Object -First 1
        if ($finished) {
            $fTask = $runningJobs[$finished.Id]
            $runningJobs.Remove($finished.Id)
            if ($finished.State -eq 'Completed') {
                $zipSize = [long](Receive-Job $finished)
                $completedIdx++
                Write-Host ("  [{0}/{1}] [OK] {2} ({3})" -f $completedIdx, $totalTasks, $fTask.ZipName, (Format-FileSize $zipSize)) -ForegroundColor Green
                $createdZips += [PSCustomObject]@{ ZipName=$fTask.ZipName; ZipSize=$zipSize; WpiCount=$fTask.WpiCount; FolderCount=$fTask.FolderCount }
            } else {
                $errMsg = (Receive-Job $finished -ErrorAction SilentlyContinue 2>&1) -join "; "
                $completedIdx++
                Write-Host ("  [{0}/{1}] [FAIL] {2}: {3}" -f $completedIdx, $totalTasks, $fTask.ZipName, $errMsg) -ForegroundColor Red
                $failedZips += $fTask.ZipName
            }
            Remove-Job $finished
        } else {
            Start-Sleep -Milliseconds 500
        }
    }

    # Launch this task
    $t = $tasks[$taskIdx]
    Write-Host "  Starting $($t.ZipName) ..." -ForegroundColor Gray
    $job = Start-Job -ScriptBlock {
        param($zipPath, $items)
        Compress-Archive -Path $items -DestinationPath $zipPath -CompressionLevel Optimal -Force
        (Get-Item $zipPath).Length
    } -ArgumentList $t.ZipPath, $t.Items
    $runningJobs[$job.Id] = $t
}

# Wait for remaining jobs to finish
while ($runningJobs.Count -gt 0) {
    $finished = Get-Job -Id @($runningJobs.Keys) | Where-Object { $_.State -ne 'Running' } | Select-Object -First 1
    if ($finished) {
        $fTask = $runningJobs[$finished.Id]
        $runningJobs.Remove($finished.Id)
        if ($finished.State -eq 'Completed') {
            $zipSize = [long](Receive-Job $finished)
            $completedIdx++
            Write-Host ("  [{0}/{1}] [OK] {2} ({3})" -f $completedIdx, $totalTasks, $fTask.ZipName, (Format-FileSize $zipSize)) -ForegroundColor Green
            $createdZips += [PSCustomObject]@{ ZipName=$fTask.ZipName; ZipSize=$zipSize; WpiCount=$fTask.WpiCount; FolderCount=$fTask.FolderCount }
        } else {
            $errMsg = (Receive-Job $finished -ErrorAction SilentlyContinue 2>&1) -join "; "
            $completedIdx++
            Write-Host ("  [{0}/{1}] [FAIL] {2}: {3}" -f $completedIdx, $totalTasks, $fTask.ZipName, $errMsg) -ForegroundColor Red
            $failedZips += $fTask.ZipName
        }
        Remove-Job $finished
    } else {
        Start-Sleep -Milliseconds 500
    }
}

#endregion

#region Step 6 - Summary

Write-Header "Done!"
if ($createdZips.Count -eq 0) {
    Write-Host "  No archives created." -ForegroundColor Yellow
} else {
    Write-Host ""
    Write-Host "  Created $($createdZips.Count) zip file(s):" -ForegroundColor White
    [long]$totalSize = 0
    foreach ($z in $createdZips) {
        $desc = "$($z.WpiCount) wpilog + $($z.FolderCount) folder(s)"
        Write-Host ("    {0,-55} {1,10}  ({2})" -f $z.ZipName, (Format-FileSize $z.ZipSize), $desc)
        $totalSize += $z.ZipSize
    }
    Write-Host ""
    Write-Host "  Total: $(Format-FileSize $totalSize)" -ForegroundColor Cyan
}
if ($failedZips.Count -gt 0) {
    Write-Host ""
    Write-Host "  $($failedZips.Count) archive(s) FAILED:" -ForegroundColor Red
    foreach ($f in $failedZips) { Write-Host "    $f" -ForegroundColor Red }
}
Write-Host ""

#endregion
