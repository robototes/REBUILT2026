<#
.SYNOPSIS
    Package FRC log files (.wpilog) with their matching .hoot signal-logger
    folders into .zip archives.

.DESCRIPTION
    This script scans a folder for .wpilog files and timestamped subdirectories
    containing .hoot files. It automatically matches folders to .wpilog files
    based on timestamp proximity, lets you resolve any ambiguities, and then
    creates one .zip archive per .wpilog file containing the log and its
    associated folders.

.PARAMETER LogFolder
    Path to the folder that contains .wpilog files and timestamped subdirectories.
    If omitted, the script will prompt you to enter one.

.PARAMETER ToleranceMinutes
    Maximum number of minutes a folder timestamp may differ from a .wpilog
    timestamp and still be considered an automatic match.  Default is 2.

.EXAMPLE
    .\PackageLogs.ps1 -LogFolder "C:\Users\me\Downloads\logs"

.EXAMPLE
    .\PackageLogs.ps1
    # (will prompt for the folder path)
#>

[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$LogFolder,

    [int]$ToleranceMinutes = 2
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# ── Helpers ──────────────────────────────────────────────────────────────────

function Format-FileSize([long]$Bytes) {
    if ($Bytes -ge 1GB) { return "{0:N2} GB" -f ($Bytes / 1GB) }
    if ($Bytes -ge 1MB) { return "{0:N1} MB" -f ($Bytes / 1MB) }
    if ($Bytes -ge 1KB) { return "{0:N1} KB" -f ($Bytes / 1KB) }
    return "$Bytes B"
}

function Parse-WpilogTimestamp([string]$FileName) {
    # FRC_YYYYMMDD_HHMMSS.wpilog
    if ($FileName -match '(\d{8})_(\d{6})\.wpilog$') {
        $d = $Matches[1]   # 20260306
        $t = $Matches[2]   # 042157
        return [datetime]::ParseExact("$d$t", "yyyyMMddHHmmss", $null)
    }
    return $null
}

function Parse-FolderTimestamp([string]$FolderName) {
    # 2026-03-06_04-21-49
    if ($FolderName -match '^(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})$') {
        $d = $Matches[1]                          # 2026-03-06
        $t = $Matches[2] -replace '-', ':'        # 04:21:49
        return [datetime]::Parse("$d $t")
    }
    return $null
}

function Write-Header([string]$Text) {
    $bar = "=" * 65
    Write-Host ""
    Write-Host $bar -ForegroundColor Cyan
    Write-Host "  $Text" -ForegroundColor Cyan
    Write-Host $bar -ForegroundColor Cyan
}

function Write-Separator() {
    Write-Host ("-" * 65) -ForegroundColor DarkGray
}

# ── Step 0: Get the folder path ─────────────────────────────────────────────

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
Write-Host "Log folder: $LogFolder" -ForegroundColor Gray

# ── Step 1: Enumerate .wpilog files and folders ─────────────────────────────

Write-Header "Step 1: Scanning log folder"

# Gather .wpilog files
$wpilogFiles = @(Get-ChildItem -Path $LogFolder -Filter "*.wpilog" -File | ForEach-Object {
    $ts = Parse-WpilogTimestamp $_.Name
    if ($ts) {
        [PSCustomObject]@{
            Name      = $_.Name
            FullPath  = $_.FullName
            Timestamp = $ts
            Size      = $_.Length
            BaseName  = [System.IO.Path]::GetFileNameWithoutExtension($_.Name)
        }
    } else {
        Write-Host "  WARNING: Could not parse timestamp from $($_.Name), skipping." -ForegroundColor Yellow
    }
} | Sort-Object Timestamp)

if ($wpilogFiles.Count -eq 0) {
    Write-Host "  No .wpilog files found in $LogFolder" -ForegroundColor Red
    exit 0
}

Write-Host ""
Write-Host "  .wpilog files found:" -ForegroundColor White
Write-Host ("  {0,-4} {1,-35} {2,-22} {3,12}" -f "#", "Filename", "Timestamp", "Size")
Write-Separator
for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
    $w = $wpilogFiles[$i]
    Write-Host ("  {0,-4} {1,-35} {2,-22} {3,12}" -f `
        ($i + 1), $w.Name, `
        $w.Timestamp.ToString("yyyy-MM-dd HH:mm:ss"), `
        (Format-FileSize $w.Size))
}

# Gather timestamped folders
$logFolders = @(Get-ChildItem -Path $LogFolder -Directory | ForEach-Object {
    $ts = Parse-FolderTimestamp $_.Name
    if ($ts) {
        [PSCustomObject]@{
            Name      = $_.Name
            FullPath  = $_.FullName
            Timestamp = $ts
        }
    }
} | Sort-Object Timestamp)

Write-Host ""
if ($logFolders.Count -eq 0) {
    Write-Host "  No timestamped folders found." -ForegroundColor Yellow
} else {
    Write-Host "  Timestamped folders found: $($logFolders.Count)" -ForegroundColor White
    foreach ($f in $logFolders) {
        Write-Host "    $($f.Name)  ($($f.Timestamp.ToString('yyyy-MM-dd HH:mm:ss')))"
    }
}

# ── Step 2: Auto-match folders to .wpilog files ─────────────────────────────

Write-Header "Step 2: Matching folders to .wpilog files"

$toleranceSec = $ToleranceMinutes * 60

# Build a hashtable: wpilog index -> list of folder objects
# (Named $matchMap to avoid collision with the automatic $Matches variable)
$matchMap = @{}
for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
    $matchMap[$i] = New-Object System.Collections.ArrayList
}
$unmatchedFolders = New-Object System.Collections.ArrayList

foreach ($folder in $logFolders) {
    # Find the closest .wpilog by absolute timestamp difference.
    # The folder typically starts within a few seconds of the .wpilog.
    $bestIdx  = -1
    $bestDiff = [double]::MaxValue

    for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
        $wTs    = $wpilogFiles[$i].Timestamp
        $absDiff = [Math]::Abs(($folder.Timestamp - $wTs).TotalSeconds)

        if ($absDiff -lt $bestDiff) {
            $bestIdx  = $i
            $bestDiff = $absDiff
        }
    }

    # Only auto-match if within tolerance
    if ($bestIdx -ge 0 -and $bestDiff -le $toleranceSec) {
        [void]$matchMap[$bestIdx].Add($folder)
    } else {
        [void]$unmatchedFolders.Add($folder)
    }
}

# Display proposed matches
Write-Host ""
for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
    $w = $wpilogFiles[$i]
    $flist = $matchMap[$i]
    Write-Host ("  {0}. {1}" -f ($i + 1), $w.Name) -ForegroundColor White
    if ($flist.Count -eq 0) {
        Write-Host "       (no folders matched)" -ForegroundColor Yellow
    } else {
        foreach ($f in $flist) {
            Write-Host "       + $($f.Name)" -ForegroundColor Green
        }
    }
}

# ── Step 3: Resolve unmatched items ──────────────────────────────────────────

if ($unmatchedFolders.Count -gt 0) {
    Write-Header "Step 3a: Resolve unmatched folders"

    foreach ($folder in $unmatchedFolders) {
        Write-Host ""
        Write-Host "  Folder '$($folder.Name)' has no clear .wpilog match." -ForegroundColor Yellow
        Write-Host "  Choose an action:"
        for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
            Write-Host ("    [{0}] Assign to {1}" -f ($i + 1), $wpilogFiles[$i].Name)
        }
        Write-Host "    [I] Ignore this folder"
        Write-Host "    [S] Create a standalone .zip for just this folder"
        Write-Host ""

        $valid = $false
        while (-not $valid) {
            $choice = (Read-Host "  Your choice").Trim().ToUpper()

            if ($choice -eq "I") {
                Write-Host "    -> Ignoring $($folder.Name)" -ForegroundColor DarkGray
                $valid = $true
            }
            elseif ($choice -eq "S") {
                # Create a pseudo wpilog entry for standalone folders
                $standaloneIdx = $wpilogFiles.Count
                $standaloneEntry = [PSCustomObject]@{
                    Name      = $null
                    FullPath  = $null
                    Timestamp = $folder.Timestamp
                    Size      = [long]0
                    BaseName  = $folder.Name
                }
                $wpilogFiles += $standaloneEntry
                $matchMap[$standaloneIdx] = New-Object System.Collections.ArrayList
                [void]$matchMap[$standaloneIdx].Add($folder)
                Write-Host "    -> Will create standalone zip for $($folder.Name)" -ForegroundColor Cyan
                $valid = $true
            }
            else {
                $num = 0
                if ([int]::TryParse($choice, [ref]$num) -and $num -ge 1 -and $num -le $wpilogFiles.Count) {
                    $idx = $num - 1
                    if (-not $matchMap.ContainsKey($idx)) {
                        $matchMap[$idx] = New-Object System.Collections.ArrayList
                    }
                    [void]$matchMap[$idx].Add($folder)
                    Write-Host "    -> Assigned to $($wpilogFiles[$idx].Name)" -ForegroundColor Green
                    $valid = $true
                } else {
                    Write-Host "    Invalid choice. Try again." -ForegroundColor Red
                }
            }
        }
    }
}

# Check for .wpilog files with no matched folders
$unmatchedWpilogs = @()
for ($i = 0; $i -lt $wpilogFiles.Count; $i++) {
    $w = $wpilogFiles[$i]
    if ($null -eq $w.FullPath) { continue }  # skip standalone folder entries
    if ($matchMap[$i].Count -eq 0) {
        $unmatchedWpilogs += $i
    }
}

if ($unmatchedWpilogs.Count -gt 0) {
    Write-Header "Step 3b: Resolve unmatched .wpilog files"

    foreach ($idx in $unmatchedWpilogs) {
        $w = $wpilogFiles[$idx]
        Write-Host ""
        Write-Host "  '$($w.Name)' has no matching folders." -ForegroundColor Yellow
        Write-Host "    [Z] Zip it alone (no hoot folders)"
        Write-Host "    [I] Ignore / skip this file"
        Write-Host ""

        $valid = $false
        while (-not $valid) {
            $choice = (Read-Host "  Your choice").Trim().ToUpper()

            if ($choice -eq "Z") {
                Write-Host "    -> Will zip alone" -ForegroundColor Cyan
                $valid = $true
            }
            elseif ($choice -eq "I") {
                Write-Host "    -> Ignoring $($w.Name)" -ForegroundColor DarkGray
                $matchMap.Remove($idx)
                $valid = $true
            }
            else {
                Write-Host "    Invalid choice. Enter Z or I." -ForegroundColor Red
            }
        }
    }
}

# ── Step 4: Confirm plan ────────────────────────────────────────────────────

Write-Header "Step 4: Final Plan"

$plan = @()
foreach ($idx in ($matchMap.Keys | Sort-Object)) {
    $w = $wpilogFiles[$idx]
    $folders = $matchMap[$idx]

    $isStandalone = ($null -eq $w.FullPath)
    $zipName = "$($w.BaseName).zip"
    $zipPath = Join-Path $LogFolder $zipName

    $plan += [PSCustomObject]@{
        Index        = $idx
        WpilogFile   = $w
        Folders      = $folders
        ZipName      = $zipName
        ZipPath      = $zipPath
        IsStandalone = $isStandalone
    }
}

if ($plan.Count -eq 0) {
    Write-Host "  Nothing to do -- all items were ignored." -ForegroundColor Yellow
    exit 0
}

foreach ($p in $plan) {
    if ($p.IsStandalone) {
        Write-Host "  Standalone folder:" -ForegroundColor White
    } else {
        Write-Host "  $($p.WpilogFile.Name)" -ForegroundColor White
    }
    foreach ($f in $p.Folders) {
        Write-Host "     + $($f.Name)/" -ForegroundColor Green
    }
    Write-Host "     -> $($p.ZipName)" -ForegroundColor Cyan
    Write-Host ""
}

$confirm = (Read-Host "Proceed? [Y/N]").Trim().ToUpper()
if ($confirm -ne "Y") {
    Write-Host "Cancelled." -ForegroundColor Yellow
    exit 0
}

# ── Step 5: Create .zip archives ────────────────────────────────────────────

Write-Header "Step 5: Creating archives"

$createdZips = @()

foreach ($p in $plan) {
    $zipPath = $p.ZipPath

    # Remove existing zip if present
    if (Test-Path $zipPath) {
        Remove-Item $zipPath -Force
    }

    # Collect items to add to the archive
    $itemsToCompress = @()

    # Add the .wpilog file (if not a standalone folder zip)
    if (-not $p.IsStandalone) {
        $itemsToCompress += $p.WpilogFile.FullPath
    }

    # Add each matched folder
    foreach ($f in $p.Folders) {
        $itemsToCompress += $f.FullPath
    }

    $zipLabel = $p.ZipName
    Write-Host ""
    Write-Host "  Creating $zipLabel ..." -ForegroundColor White -NoNewline

    try {
        Compress-Archive -Path $itemsToCompress -DestinationPath $zipPath -CompressionLevel Optimal -Force
        $zipSize = (Get-Item $zipPath).Length
        $folderCount = $p.Folders.Count
        if ($p.IsStandalone) { $wpiCount = 0 } else { $wpiCount = 1 }

        Write-Host ""
        Write-Host "  [OK] Created $zipLabel ($(Format-FileSize $zipSize))" -ForegroundColor Green

        $createdZips += [PSCustomObject]@{
            ZipName     = $zipLabel
            ZipSize     = $zipSize
            WpiCount    = $wpiCount
            FolderCount = $folderCount
        }
    }
    catch {
        Write-Host ""
        Write-Host "  [FAIL] Error creating ${zipLabel}: $_" -ForegroundColor Red
    }
}

# ── Step 6: Summary ─────────────────────────────────────────────────────────

Write-Header "Done!"

if ($createdZips.Count -eq 0) {
    Write-Host "  No archives were created." -ForegroundColor Yellow
} else {
    Write-Host ""
    Write-Host "  Created $($createdZips.Count) zip file(s):" -ForegroundColor White
    [long]$totalSize = 0
    foreach ($z in $createdZips) {
        $desc = "$($z.WpiCount) wpilog + $($z.FolderCount) folder(s)"
        Write-Host ("    {0,-40} {1,12}  ({2})" -f $z.ZipName, (Format-FileSize $z.ZipSize), $desc)
        $totalSize += $z.ZipSize
    }
    Write-Host ""
    Write-Host "  Total: $(Format-FileSize $totalSize)" -ForegroundColor Cyan
}

Write-Host ""
