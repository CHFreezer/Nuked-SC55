#requires -Version 5.1
<#
.SYNOPSIS
    M4 oracle -- n=28 audio null test:
    stock interpreter vs M4 native engine (`-mk2cpp -voices:28`).

.DESCRIPTION
    Dry-run by default: validates arguments, probes the GT build for the
    frozen oracle options and prints the exact GT commands, timeouts and
    comparison plan. It starts NO GT process.

    With -Execute (and -UserPresent) it starts the real GT per mode -- LCD
    window and audio ON, never SDL dummy, per tools/docs/plan_256.md §6 --
    polls for the audio window's .meta marker, then force-kills the process
    in a finally block. It compares stock vs M4 captures:
      - -audiohash (deterministic FNV-1a64 at a fixed cycle) -- preferred,
      - WAV PCM payload SHA256 / byte compare (pcmdiff.exe for details),
      - layout-independent state scalars from -hashdump (W3 gate 3 helper).

    Requires the frozen GT options (W2 08_m4_pcm_api.md §3.5 +
    W4 10_m4_oracle.md §2.2):
        -wav:<file>                 producer-side int16 WAV tap (default off)
        -audiowin <start> <end>     fixed cycle window; writes <file>.meta and
                                    backfills the RIFF/data sizes at <end>
        -audiohash <cycles> <file>  FNV-1a64 checkpoint (W2)
    and, with -HandOff, the same-binary A/B switch
        -mk2cpp-hand:0|1            (default 1; W3 09_m4_integration.md §5.4)
    Default GT behavior must be unchanged when these options are absent.
    Missing options are reported as clear ERROR lines even in dry-run; add
    -RequireReady to make the dry-run fail (exit 2) until they exist.

    With -HandOff it adds the A/B run
        -mk2cpp -voices:28 -mk2cpp-hand:0
    and compares it against the hand-on run (W3 §5.3 gate 5).

    With -CheckBaseline it additionally gates on the frozen stock capture in
    tools\baselines\m4_audio (G6, 10_m4_oracle.md §5.1): first SHA256SUMS.txt is
    re-verified (mismatch/missing listed file = FAIL), then the stock run's WAV
    payload SHA256 and -audiohash are compared against the frozen files
    (differences = FAIL). A missing baseline directory/manifest/files = SKIP,
    never FAIL. Off by default: without it dry-run and -Execute are unchanged.

.NOTES
    Pure PowerShell 5.1, no Python, no SDL dependency. Outputs go to -OutDir
    (default is under the gitignored mk2cpp/out/). The user must be present
    for real runs; the A/B listen is manual and cannot be automated.
#>
[CmdletBinding()]
param(
    [ValidateSet('demo', 'midi')]
    [string]$Scenario = 'demo',

    [uint64]$AudioStart = 300000000,
    [uint64]$AudioEnd   = 320000000,

    # State sample cycle for -hashdump; 0 -> AudioEnd. Must be >= 200M.
    [uint64]$StateAt = 0,

    [int]$TimeoutSec = 0,

    [string]$OutDir = 'mk2cpp\out\m4\audio_null',

    # -Scenario midi: schedule file for the frozen -midiseq option.
    [string]$MidiSchedule = '',

    # Optional -gain:<x db> / -gain:<x>; empty = GT default (1.0).
    [string]$Gain = '',

    # Optional G6 gate: verify the frozen stock baseline and require the stock
    # run's WAV payload/audiohash to match it. Off by default.
    [switch]$CheckBaseline,

    # Frozen baseline directory and file stem; relative paths resolve against
    # the repo root. Custom windows need a matching frozen <stem>.
    [string]$BaselineDir = 'tools\baselines\m4_audio',
    [string]$BaselineName = 'stock_300M_320M',

    # Add the -mk2cpp-hand:0 A/B run (W3 gate 5).
    [switch]$HandOff,

    # Dry-run only: exit 2 when required GT options are missing.
    [switch]$RequireReady,

    [switch]$Execute,
    [switch]$UserPresent
)

$ErrorActionPreference = 'Stop'

# ---- paths ----------------------------------------------------------------
# PSScriptRoot = <repo>\mk2cpp\tests  ->  repo root is 2 levels up.
$script:RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$script:WorkDir  = Join-Path $script:RepoRoot 'build'
$script:ExePath  = Join-Path $script:WorkDir 'nuked-sc55.exe'
$script:Pcmdiff  = Join-Path $script:RepoRoot 'mk2cpp\tools\pcmdiff\pcmdiff.exe'

# ---- timeout: ceil(cycles/24e6 * 2) + 15 (plan_256.md §6) -----------------
function Get-GtTimeoutSec {
    param([uint64]$Cycles)
    $seconds = ([double]$Cycles / 24e6) * 2.0 + 15.0
    if ($seconds -lt 10.0) { $seconds = 10.0 }
    return [int][Math]::Ceiling($seconds)
}

function Exit-Setup {
    param([string]$Message)
    Write-Host ("ERROR: " + $Message)
    exit 2
}

# ---- GT capability probe (help text only; never launches a GT run) --------
function Get-HelpText {
    $oldEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $lines = @(& $script:ExePath -h 2>&1 | ForEach-Object { [string]$_ })
    }
    catch { $lines = @() }
    $ErrorActionPreference = $oldEap
    return ($lines -join [Environment]::NewLine)
}

function Get-GtCapabilities {
    $help = Get-HelpText
    $caps = [ordered]@{
        'wav'      = [bool]($help -match [regex]::Escape('-wav:'))
        'audiowin' = [bool]($help -match [regex]::Escape('-audiowin'))
        'audiohash' = [bool]($help -match [regex]::Escape('-audiohash'))
        'midiseq'  = [bool]($help -match [regex]::Escape('-midiseq'))
        'hand'     = [bool]($help -match [regex]::Escape('-mk2cpp-hand'))
    }
    return $caps
}

function Get-RequiredCapabilities {
    param($Caps)
    $required = New-Object System.Collections.Generic.List[string]
    if (-not $Caps['wav'])      { [void]$required.Add('-wav:<file>') }
    if (-not $Caps['audiowin']) { [void]$required.Add('-audiowin <start> <end>') }
    if (-not $Caps['audiohash']) { [void]$required.Add('-audiohash <cycles> <file>') }
    if ($Scenario -eq 'midi' -and -not $Caps['midiseq']) {
        [void]$required.Add('-midiseq <file> [start]')
    }
    if ($HandOff -and -not $Caps['hand']) {
        [void]$required.Add('-mk2cpp-hand:0|1')
    }
    return $required
}

function Write-CapabilityReport {
    param($Caps)
    Write-Host ("GT capability probe (`"$script:ExePath`" -h):")
    $order = @(
        @('wav',      '-wav:<file>',                'G1  (10_m4_oracle.md §2.2)'),
        @('audiowin', '-audiowin <start> <end>',    'G1  (10_m4_oracle.md §2.2)'),
        @('audiohash', '-audiohash <cycles> <file>', 'G1  (10_m4_oracle.md §2.2)'),
        @('midiseq',  '-midiseq <file> [start]',    'G5  (10_m4_oracle.md §3.1)'),
        @('hand',     '-mk2cpp-hand:0|1',           'W3  (09_m4_integration.md §5.4)')
    )
    foreach ($row in $order) {
        $state = 'ok'
        if (-not $Caps[$row[0]]) { $state = 'MISSING' }
        Write-Host ("  {0,-28} {1,-8} {2}" -f $row[1], $state, $row[2])
    }
}

# ---- validation -----------------------------------------------------------
if (-not (Test-Path -LiteralPath $script:ExePath)) {
    Exit-Setup ("GT exe not found: " + $script:ExePath)
}
if ($AudioEnd -le $AudioStart) {
    Exit-Setup ("invalid audio window [" + $AudioStart + "," + $AudioEnd + ")")
}
if ($StateAt -eq 0) { $StateAt = $AudioEnd }
if ($StateAt -lt 200000000) {
    Exit-Setup ("StateAt=" + $StateAt + " is below the 200M-cycle state sampling floor")
}
if ($Scenario -eq 'midi') {
    if ([string]::IsNullOrWhiteSpace($MidiSchedule)) {
        Exit-Setup "-Scenario midi requires -MidiSchedule <file>"
    }
    if (-not (Test-Path -LiteralPath $MidiSchedule)) {
        Exit-Setup ("MIDI schedule not found: " + $MidiSchedule)
    }
}
if ($TimeoutSec -le 0) { $TimeoutSec = Get-GtTimeoutSec -Cycles $AudioEnd }

if (-not [System.IO.Path]::IsPathRooted($OutDir)) {
    $OutDir = Join-Path $script:RepoRoot $OutDir
}
$OutDir = [System.IO.Path]::GetFullPath($OutDir)
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$stockWav    = Join-Path $OutDir 'stock.wav'
$m4Wav       = Join-Path $OutDir 'm4.wav'
$handoffWav  = Join-Path $OutDir 'handoff.wav'
$stockHash   = Join-Path $OutDir 'stock.audiohash'
$m4Hash      = Join-Path $OutDir 'm4.audiohash'
$handoffHash = Join-Path $OutDir 'handoff.audiohash'
$stockState  = Join-Path $OutDir 'stock.state.hash'
$m4State     = Join-Path $OutDir 'm4.state.hash'
$handoffState = Join-Path $OutDir 'handoff.state.hash'
$summaryPath = Join-Path $OutDir 'summary.txt'
$planPath    = Join-Path $OutDir 'plan.txt'

# ---- frozen baseline paths (only read with -CheckBaseline) ----------------
if (-not [System.IO.Path]::IsPathRooted($BaselineDir)) {
    $BaselineDir = Join-Path $script:RepoRoot $BaselineDir
}
$BaselineDir = [System.IO.Path]::GetFullPath($BaselineDir)
$script:BaselineDir  = $BaselineDir
$script:BaselineName = $BaselineName
$script:BaselineWav  = Join-Path $BaselineDir ($BaselineName + '.wav')
$script:BaselineHash = Join-Path $BaselineDir ($BaselineName + '.audiohash')
$script:BaselineSums = Join-Path $BaselineDir 'SHA256SUMS.txt'

$caps = Get-GtCapabilities
$missing = @(Get-RequiredCapabilities -Caps $caps)

# ---- command builders -----------------------------------------------------
function Get-ModeArgs {
    param([string]$Mode, [string]$WavPath, [string]$HashPath, [string]$StatePath)
    $a = New-Object System.Collections.Generic.List[string]
    [void]$a.Add('-mk2')
    if ($Mode -eq 'm4') {
        [void]$a.Add('-mk2cpp')
        [void]$a.Add('-voices:28')
    }
    elseif ($Mode -eq 'm4handoff') {
        [void]$a.Add('-mk2cpp')
        [void]$a.Add('-voices:28')
        [void]$a.Add('-mk2cpp-hand:0')
    }
    if ($Scenario -eq 'demo') {
        [void]$a.Add('-demo')
    }
    else {
        [void]$a.Add('-midiseq')
        [void]$a.Add($MidiSchedule)
        [void]$a.Add('200000000')
    }
    if (-not [string]::IsNullOrWhiteSpace($Gain)) {
        [void]$a.Add('-gain:' + $Gain)
    }
    [void]$a.Add('-wav:' + $WavPath)
    [void]$a.Add('-audiowin')
    [void]$a.Add([string]$AudioStart)
    [void]$a.Add([string]$AudioEnd)
    [void]$a.Add('-audiohash')
    [void]$a.Add([string]$AudioEnd)
    [void]$a.Add($HashPath)
    [void]$a.Add('-hashdump')
    [void]$a.Add([string]$StateAt)
    [void]$a.Add($StatePath)
    return ,$a.ToArray()
}

function Format-Cmd {
    param([string[]]$ArgList)
    $quoted = @($ArgList | ForEach-Object {
        if ($_ -match '\s') { '"' + $_ + '"' } else { $_ }
    })
    return ($quoted -join ' ')
}

$stockArgs = Get-ModeArgs -Mode 'stock' -WavPath $stockWav -HashPath $stockHash -StatePath $stockState
$m4Args    = Get-ModeArgs -Mode 'm4'    -WavPath $m4Wav    -HashPath $m4Hash    -StatePath $m4State
$handoffArgs = @()
if ($HandOff) {
    $handoffArgs = Get-ModeArgs -Mode 'm4handoff' -WavPath $handoffWav -HashPath $handoffHash -StatePath $handoffState
}

# ---- frozen stock baseline (G6) -------------------------------------------
# Integrity is defined by <BaselineDir>\SHA256SUMS.txt. A missing directory or
# manifest is SKIP; a hash mismatch or a missing listed file is FAIL.
function Get-BaselineStatus {
    if (-not (Test-Path -LiteralPath $script:BaselineDir)) {
        return [pscustomobject]@{ Result = 'SKIP'; Detail = ('baseline dir missing: ' + $script:BaselineDir) }
    }
    if (-not (Test-Path -LiteralPath $script:BaselineSums)) {
        return [pscustomobject]@{ Result = 'SKIP'; Detail = ('SHA256SUMS.txt missing: ' + $script:BaselineSums) }
    }
    $bad = New-Object System.Collections.Generic.List[string]
    $count = 0
    foreach ($line in Get-Content -LiteralPath $script:BaselineSums) {
        if ($line -notmatch '^\s*([0-9a-fA-F]{64})\s+(.+?)\s*$') { continue }
        $count++
        $expected = $Matches[1].ToLower()
        $name = $Matches[2]
        $path = Join-Path $script:BaselineDir $name
        if (-not (Test-Path -LiteralPath $path)) {
            [void]$bad.Add($name + ': missing')
            continue
        }
        $actual = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLower()
        if ($actual -ne $expected) { [void]$bad.Add($name + ': sha256 mismatch') }
    }
    if ($count -eq 0) {
        return [pscustomobject]@{ Result = 'SKIP'; Detail = 'SHA256SUMS.txt has no parsable entries' }
    }
    if ($bad.Count -gt 0) {
        return [pscustomobject]@{ Result = 'FAIL'; Detail = ('SHA256SUMS mismatch: ' + ($bad -join '; ')) }
    }
    return [pscustomobject]@{ Result = 'PASS'; Detail = ('SHA256SUMS verified (' + $count + ' files)') }
}

function Get-ShortHash {
    param([string]$Hash)
    if ([string]::IsNullOrEmpty($Hash)) { return '?' }
    return $Hash.Substring(0, [Math]::Min(16, $Hash.Length))
}

# ---- dry-run (default) ----------------------------------------------------
function Write-Plan {
    $lines = New-Object System.Collections.Generic.List[string]
    [void]$lines.Add("m4_audio_null.ps1 -- plan (dry-run unless -Execute)")
    [void]$lines.Add(("scenario     : {0}" -f $Scenario))
    [void]$lines.Add(("audio window : [{0},{1})" -f $AudioStart, $AudioEnd))
    [void]$lines.Add(("state sample : hashdump @ {0}" -f $StateAt))
    [void]$lines.Add(("timeout      : {0}s  (ceil(End/24e6*2)+15)" -f $TimeoutSec))
    [void]$lines.Add(("outdir       : {0}" -f $OutDir))
    [void]$lines.Add(("pcmdiff      : {0}" -f $(if (Test-Path -LiteralPath $script:Pcmdiff) { 'present' } else { 'not built (SHA256 fallback)' })))
    if ($CheckBaseline) {
        $bl = Get-BaselineStatus
        [void]$lines.Add(("baseline     : {0} [{1}]" -f $script:BaselineDir, $script:BaselineName))
        [void]$lines.Add(("baseline chk : {0} - {1}" -f $bl.Result, $bl.Detail))
    }
    [void]$lines.Add("")
    [void]$lines.Add("stock  : " + (Format-Cmd -ArgList $stockArgs))
    [void]$lines.Add("m4     : " + (Format-Cmd -ArgList $m4Args))
    if ($HandOff) {
        [void]$lines.Add("handoff: " + (Format-Cmd -ArgList $handoffArgs))
    }
    [void]$lines.Add("")
    [void]$lines.Add("checks (automated, 10_m4_oracle.md §2.1-2.4 / W3 §5.3):")
    [void]$lines.Add("  1. -audiohash files exist and are equal (deterministic FNV-1a64)")
    [void]$lines.Add("  2. WAV payload SHA256 / pcmdiff --tolerance 0 bit-exact (gate: 0 tolerance)")
    [void]$lines.Add("  3. .meta windows/rate/length identical; WAV header finalized (nonzero data size)")
    [void]$lines.Add("  4. layout-independent state scalars equal (mcu.pc/sr/cycles, pcm cfg 3c/3d)")
    [void]$lines.Add("  5. stdout LCDEN 0 <= 2 (boot + demo power-cycle)")
    [void]$lines.Add("  6. CPU duty report (process CPU / wall; INFO only)")
    if ($CheckBaseline) {
        [void]$lines.Add("  B0. frozen baseline integrity via SHA256SUMS.txt (mismatch = FAIL, missing = SKIP)")
        [void]$lines.Add("  B1. stock WAV payload SHA256 == frozen WAV payload (bit-exact)")
        [void]$lines.Add("  B2. stock -audiohash == frozen audio_fnv1a")
    }
    if ($HandOff) {
        [void]$lines.Add("  7. W3 gate 5: handoff vs hand-on audiohash + WAV payload byte-identical")
    }
    [void]$lines.Add("checks (manual, user present):")
    [void]$lines.Add("  A. LCD window renders; audio device is real (no SDL dummy)")
    [void]$lines.Add("  B. A/B listen: stock vs M4 must be indistinguishable")
    $text = ($lines -join [Environment]::NewLine)
    Set-Content -LiteralPath $planPath -Value $text -Encoding UTF8
    Write-Host $text
}

function Write-MissingOptions {
    param([string[]]$Missing)
    if ($Missing.Count -eq 0) { return }
    Write-Host ""
    Write-Host ("ERROR: this GT build lacks {0} option(s) required by m4_audio_null.ps1:" -f $Missing.Count)
    foreach ($m in $Missing) {
        Write-Host ("  ERROR: missing GT option " + $m)
    }
    Write-Host "       implementer (Wave 0a/0b): land the frozen options listed in"
    Write-Host "       mk2cpp/docs/10_m4_oracle.md §2.2/§5.1 (G1/G5) and advertise them in -h;"
    Write-Host "       -audiowin must backfill the RIFF/data sizes and fclose the WAV,"
    Write-Host "       then write <file>.meta last as the completion marker."
    Write-Host "       Dry-run continues; -Execute will refuse until these options exist."
}

Write-Plan
Write-Host ""
Write-CapabilityReport -Caps $caps
Write-MissingOptions -Missing $missing

if (-not $Execute) {
    if ($RequireReady -and $missing.Count -gt 0) {
        Write-Host ("ERROR: -RequireReady: " + $missing.Count + " required GT option(s) missing.")
        exit 2
    }
    Write-Host ""
    Write-Host ("DRY-RUN OK ({0} required GT option(s) missing). Re-run with -Execute -UserPresent when the GT side is ready." -f $missing.Count)
    exit 0
}

if (-not $UserPresent) {
    Write-Host "ERROR: real GT runs must be observed on site (LCD + audio)."
    Write-Host "       Re-run with -Execute -UserPresent only when the user is present."
    exit 2
}

if ($missing.Count -gt 0) {
    Write-Host ("ERROR: refusing to launch GT: " + $missing.Count + " required option(s) missing (see above).")
    exit 2
}

# ---- helpers --------------------------------------------------------------
$script:checks = New-Object System.Collections.Generic.List[object]

function Add-Check {
    param([string]$Check, [string]$Result, [string]$Detail)
    [void]$script:checks.Add([pscustomobject]@{
        Check  = $Check
        Result = $Result
        Detail = $Detail
    })
}

function Test-NonEmptyFile {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { return $false }
    try { return ((Get-Item -LiteralPath $Path).Length -gt 0) } catch { return $false }
}

function Test-RunReady {
    param([string]$WavPath, [string]$HashPath, [string]$StatePath)
    return ((Test-NonEmptyFile -Path ($WavPath + '.meta')) -and
            (Test-NonEmptyFile -Path $HashPath) -and
            (Test-NonEmptyFile -Path $StatePath))
}

function Invoke-GtWindowed {
    param(
        [string]$Label,
        [string[]]$ArgList,
        [string]$WavPath,
        [string]$HashPath,
        [string]$StatePath,
        [string]$LogBase
    )
    $stdoutPath = $LogBase + '.stdout.txt'
    $stderrPath = $LogBase + '.stderr.txt'
    $metaPath   = $WavPath + '.meta'
    foreach ($f in @($WavPath, $metaPath, $HashPath, $StatePath, $stdoutPath, $stderrPath)) {
        Remove-Item -LiteralPath $f -Force -ErrorAction SilentlyContinue
    }
    $quoted = @($ArgList | ForEach-Object {
        if ($_ -match '\s') { '"' + $_ + '"' } else { $_ }
    })

    Write-Host ("[{0}] " -f $Label + (Format-Cmd -ArgList $ArgList))
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $proc = Start-Process -FilePath $script:ExePath -WorkingDirectory $script:WorkDir `
        -PassThru -ArgumentList $quoted `
        -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath

    $status = 'timeout'
    $cpuSec = $null
    try {
        $deadline = (Get-Date).AddSeconds($TimeoutSec)
        while ($true) {
            if (Test-RunReady -WavPath $WavPath -HashPath $HashPath -StatePath $StatePath) {
                $status = 'ready'; break
            }
            try { $proc.Refresh() } catch { }
            if ($proc.HasExited) { $status = 'exited'; break }
            if ((Get-Date) -ge $deadline) { $status = 'timeout'; break }
            Start-Sleep -Milliseconds 500
        }
        if ($status -ne 'ready') {
            if (Test-RunReady -WavPath $WavPath -HashPath $HashPath -StatePath $StatePath) {
                $status = 'ready'
            }
        }
        try { $cpuSec = $proc.TotalProcessorTime.TotalSeconds } catch { }
    }
    finally {
        try { $proc.Refresh() } catch { }
        if (-not $proc.HasExited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
            try { [void]$proc.WaitForExit(5000) } catch { }
        }
        $sw.Stop()
    }

    $wall = $sw.Elapsed.TotalSeconds
    $duty = 'n/a'
    if ($null -ne $cpuSec -and $wall -gt 0) {
        $duty = ('{0:P1}' -f ($cpuSec / $wall))
    }
    return [pscustomobject]@{
        Label      = $Label
        Status     = $status
        WavPath    = $WavPath
        MetaPath   = $metaPath
        HashPath   = $HashPath
        StatePath  = $StatePath
        StdoutPath = $stdoutPath
        StderrPath = $stderrPath
        WallSec    = $wall
        CpuSec     = $cpuSec
        CpuDuty    = $duty
    }
}

function Read-Meta {
    param([string]$Path)
    $h = @{}
    if (-not (Test-Path -LiteralPath $Path)) { return $h }
    foreach ($line in Get-Content -LiteralPath $Path) {
        if ($line -match '^\s*([A-Za-z0-9_]+)\s*=\s*(.+?)\s*$') {
            $h[$Matches[1]] = $Matches[2]
        }
    }
    return $h
}

function Read-Scalars {
    param([string]$Path)
    $h = @{}
    if (-not (Test-Path -LiteralPath $Path)) { return $h }
    foreach ($line in Get-Content -LiteralPath $Path) {
        if ($line -match '^\s*([A-Za-z0-9_.]+)\s*=\s*(\S+)\s*$') {
            $h[$Matches[1]] = $Matches[2]
        }
    }
    return $h
}

function Read-AudioHash {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { return '' }
    foreach ($line in Get-Content -LiteralPath $Path) {
        if ($line -match 'audio_fnv1a\s*=\s*([0-9a-fA-F]+)') { return $Matches[1].ToLower() }
    }
    return ''
}

function Get-WavDataInfo {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { return $null }
    $bytes = [System.IO.File]::ReadAllBytes($Path)
    if ($bytes.Length -lt 44) { return $null }
    $pos = 12
    while ($pos + 8 -le $bytes.Length) {
        $id = [System.Text.Encoding]::ASCII.GetString($bytes, $pos, 4)
        $size = [uint64][BitConverter]::ToUInt32($bytes, $pos + 4)
        $remaining = [uint64]($bytes.Length - ($pos + 8))
        if ($id -eq 'data') {
            return [pscustomobject]@{
                Offset   = $pos + 8
                Declared = $size
                Actual   = $remaining
                Bytes    = $bytes
            }
        }
        if ($size -gt $remaining) { return $null }
        $pos += 8 + [int]$size + [int]($size -band 1)
    }
    return $null
}

function Get-WavPayloadHash {
    param($Info)
    if ($null -eq $Info) { return '' }
    if ($Info.Declared -eq 0 -and $Info.Actual -gt 0) { return '' }
    $len = [int][Math]::Min($Info.Declared, $Info.Actual)
    if ($len -le 0) { return '' }
    $sha = [System.Security.Cryptography.SHA256]::Create()
    try {
        return ([BitConverter]::ToString($sha.ComputeHash($Info.Bytes, $Info.Offset, $len)) -replace '-', '').ToLower()
    }
    finally { $sha.Dispose() }
}

function Write-PcmdiffReport {
    param([string]$RefPath, [string]$DutPath, [string]$ReportPath)
    if (-not (Test-Path -LiteralPath $script:Pcmdiff)) { return $null }
    $oldEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $out = @(& $script:Pcmdiff --ref $RefPath --dut $DutPath --tolerance 0 --report $ReportPath 2>&1 |
            ForEach-Object { [string]$_ })
        $code = $LASTEXITCODE
    }
    catch {
        $out = @([string]$_)
        $code = 3
    }
    $ErrorActionPreference = $oldEap
    return [pscustomobject]@{ ExitCode = $code; Output = ($out -join ' ') }
}

function Compare-WavDumps {
    param([string]$RefPath, [string]$DutPath, [string]$ReportPath)
    $refInfo = Get-WavDataInfo -Path $RefPath
    $dutInfo = Get-WavDataInfo -Path $DutPath
    if ($null -eq $refInfo -or $null -eq $dutInfo) {
        return [pscustomobject]@{ Result = 'FAIL'; Detail = 'missing/unparsable WAV' }
    }
    if ($refInfo.Declared -eq 0 -and $refInfo.Actual -gt 0) {
        return [pscustomobject]@{ Result = 'FAIL'; Detail = 'ref WAV header not finalized (data size 0)' }
    }
    if ($dutInfo.Declared -eq 0 -and $dutInfo.Actual -gt 0) {
        return [pscustomobject]@{ Result = 'FAIL'; Detail = 'dut WAV header not finalized (data size 0)' }
    }

    $refHash = Get-WavPayloadHash -Info $refInfo
    $dutHash = Get-WavPayloadHash -Info $dutInfo
    if ($refHash -and $refHash -eq $dutHash) {
        return [pscustomobject]@{ Result = 'PASS'; Detail = ('payload bit-exact sha256=' + $refHash.Substring(0, 16)) }
    }

    if (Test-Path -LiteralPath $script:Pcmdiff) {
        $pd = Write-PcmdiffReport -RefPath $RefPath -DutPath $DutPath -ReportPath $ReportPath
        $detail = ('pcmdiff exit={0}; see {1}' -f $pd.ExitCode, $ReportPath)
        if ($pd.ExitCode -eq 0) { return [pscustomobject]@{ Result = 'PASS'; Detail = $detail } }
        return [pscustomobject]@{ Result = 'FAIL'; Detail = $detail }
    }
    $rs = 'ref=?'
    $ds = 'dut=?'
    if ($refHash) { $rs = 'ref=' + $refHash.Substring(0, 16) }
    if ($dutHash) { $ds = 'dut=' + $dutHash.Substring(0, 16) }
    return [pscustomobject]@{ Result = 'FAIL'; Detail = ('WAV payload differs (' + $rs + ' ' + $ds + '); build pcmdiff for sample stats') }
}

function Get-LcdenResetCount {
    param([string]$StdoutPath)
    if (-not (Test-Path -LiteralPath $StdoutPath)) { return -1 }
    return @(Select-String -LiteralPath $StdoutPath -Pattern 'LCDEN 0' -ErrorAction SilentlyContinue).Count
}

# ---- run ------------------------------------------------------------------
$runStock = Invoke-GtWindowed -Label 'stock' -ArgList $stockArgs -WavPath $stockWav `
    -HashPath $stockHash -StatePath $stockState -LogBase (Join-Path $OutDir 'stock')
$runM4 = Invoke-GtWindowed -Label 'm4' -ArgList $m4Args -WavPath $m4Wav `
    -HashPath $m4Hash -StatePath $m4State -LogBase (Join-Path $OutDir 'm4')

$runs = @($runStock, $runM4)
if ($HandOff) {
    $runHandoff = Invoke-GtWindowed -Label 'handoff' -ArgList $handoffArgs -WavPath $handoffWav `
        -HashPath $handoffHash -StatePath $handoffState -LogBase (Join-Path $OutDir 'handoff')
    $runs += $runHandoff
}

foreach ($run in $runs) {
    if ($run.Status -ne 'ready') {
        Add-Check -Check ('run:' + $run.Label) -Result 'FAIL' -Detail ('status=' + $run.Status)
    }
    else {
        Add-Check -Check ('run:' + $run.Label) -Result 'PASS' `
            -Detail ('meta/hash/state ready; wall={0:N1}s cpu={1}s duty={2}' -f $run.WallSec, $run.CpuSec, $run.CpuDuty)
    }
}

# 1) deterministic audio hash checkpoint
$hStock = Read-AudioHash -Path $runStock.HashPath
$hM4    = Read-AudioHash -Path $runM4.HashPath
if ([string]::IsNullOrWhiteSpace($hStock) -or [string]::IsNullOrWhiteSpace($hM4)) {
    Add-Check -Check 'audio:hash' -Result 'FAIL' -Detail 'missing audio_fnv1a line'
}
elseif ($hStock -eq $hM4) {
    Add-Check -Check 'audio:hash' -Result 'PASS' -Detail ('fnv1a=' + $hStock.Substring(0, 16))
}
else {
    Add-Check -Check 'audio:hash' -Result 'FAIL' -Detail ('stock=' + $hStock.Substring(0, 16) + ' m4=' + $hM4.Substring(0, 16))
}

# 2) meta sanity + WAV payload compare
$metaStock = Read-Meta -Path $runStock.MetaPath
$metaM4    = Read-Meta -Path $runM4.MetaPath
$metaFields = @('start_cycles', 'end_cycles', 'rate', 'channels', 'format', 'sample_pairs')
$metaDiffs = New-Object System.Collections.Generic.List[string]
foreach ($f in $metaFields) {
    if ($metaStock[$f] -ne $metaM4[$f]) {
        [void]$metaDiffs.Add(('{0}: {1} != {2}' -f $f, $metaStock[$f], $metaM4[$f]))
    }
}
if ($metaDiffs.Count -gt 0) {
    Add-Check -Check 'audio:meta' -Result 'FAIL' -Detail ($metaDiffs -join '; ')
}
else {
    Add-Check -Check 'audio:meta' -Result 'PASS' `
        -Detail ('pairs={0} rate={1} [{2},{3})' -f $metaStock['sample_pairs'], $metaStock['rate'], $metaStock['start_cycles'], $metaStock['end_cycles'])
}
if ($metaStock['payload_fnv1a'] -and $metaM4['payload_fnv1a'] -and
    $metaStock['payload_fnv1a'] -ne $metaM4['payload_fnv1a']) {
    Add-Check -Check 'audio:meta-fnv' -Result 'FAIL' -Detail 'payload_fnv1a differs'
}
elseif ($metaStock['payload_fnv1a'] -and $metaM4['payload_fnv1a']) {
    Add-Check -Check 'audio:meta-fnv' -Result 'PASS' -Detail ('payload_fnv1a=' + $metaStock['payload_fnv1a'].Substring(0, [Math]::Min(16, $metaStock['payload_fnv1a'].Length)))
}

$cmp = Compare-WavDumps -RefPath $stockWav -DutPath $m4Wav -ReportPath (Join-Path $OutDir 'audio_diff.md')
Add-Check -Check 'audio:null' -Result $cmp.Result -Detail $cmp.Detail

# 2b) frozen stock baseline (G6; only with -CheckBaseline)
if ($CheckBaseline) {
    $bs = Get-BaselineStatus
    if ($bs.Result -eq 'SKIP') {
        Add-Check -Check 'baseline:integrity' -Result 'SKIP' -Detail $bs.Detail
        Add-Check -Check 'baseline:wav' -Result 'SKIP' -Detail 'frozen baseline absent; nothing to compare'
        Add-Check -Check 'baseline:audiohash' -Result 'SKIP' -Detail 'frozen baseline absent; nothing to compare'
    }
    else {
        Add-Check -Check 'baseline:integrity' -Result $bs.Result -Detail $bs.Detail

        $baseInfo  = Get-WavDataInfo -Path $script:BaselineWav
        if ($null -eq $baseInfo) {
            Add-Check -Check 'baseline:wav' -Result 'SKIP' -Detail ('frozen WAV missing/unparsable: ' + $script:BaselineWav)
        }
        else {
            $baseHash  = Get-WavPayloadHash -Info $baseInfo
            $stockInfo = Get-WavDataInfo -Path $runStock.WavPath
            $stockHash = Get-WavPayloadHash -Info $stockInfo
            if ($baseHash -and $baseHash -eq $stockHash) {
                Add-Check -Check 'baseline:wav' -Result 'PASS' -Detail ('payload bit-exact sha256=' + (Get-ShortHash -Hash $baseHash))
            }
            else {
                Add-Check -Check 'baseline:wav' -Result 'FAIL' `
                    -Detail ('payload differs (frozen=' + (Get-ShortHash -Hash $baseHash) + ' stock=' + (Get-ShortHash -Hash $stockHash) + ')')
            }
        }

        $baseAh = Read-AudioHash -Path $script:BaselineHash
        if (-not $baseAh) {
            Add-Check -Check 'baseline:audiohash' -Result 'SKIP' -Detail ('frozen audiohash missing/unparsable: ' + $script:BaselineHash)
        }
        else {
            $stockAh = Read-AudioHash -Path $runStock.HashPath
            if ($baseAh -eq $stockAh) {
                Add-Check -Check 'baseline:audiohash' -Result 'PASS' -Detail ('fnv1a=' + (Get-ShortHash -Hash $baseAh))
            }
            else {
                Add-Check -Check 'baseline:audiohash' -Result 'FAIL' `
                    -Detail ('frozen=' + (Get-ShortHash -Hash $baseAh) + ' stock=' + (Get-ShortHash -Hash $stockAh))
            }
        }
    }
}

# 3) layout-independent state scalars (W3 gate 3 helper; D4: native data
# structures may differ, so only layout-free scalars are compared)
$scalarsStock = Read-Scalars -Path $runStock.StatePath
$scalarsM4    = Read-Scalars -Path $runM4.StatePath
if ($scalarsStock.Count -eq 0 -or $scalarsM4.Count -eq 0) {
    Add-Check -Check 'state:scalars' -Result 'FAIL' -Detail 'missing hashdump'
}
else {
    $scalarKeys = @('mcu.cp', 'mcu.pc', 'mcu.sr', 'mcu.cycles', 'pcm.config_reg_3c', 'pcm.config_reg_3d', 'pcm.irq_assert')
    $scalarDiffs = New-Object System.Collections.Generic.List[string]
    foreach ($k in $scalarKeys) {
        if ($scalarsStock[$k] -ne $scalarsM4[$k]) {
            [void]$scalarDiffs.Add(('{0}: {1} != {2}' -f $k, $scalarsStock[$k], $scalarsM4[$k]))
        }
    }
    if ($scalarDiffs.Count -gt 0) {
        Add-Check -Check 'state:scalars' -Result 'FAIL' -Detail ($scalarDiffs -join '; ')
    }
    else {
        Add-Check -Check 'state:scalars' -Result 'PASS' `
            -Detail ('cycles={0} cfg3d={1}' -f $scalarsStock['mcu.cycles'], $scalarsStock['pcm.config_reg_3d'])
    }
}

# 4) reset heuristic
foreach ($run in $runs) {
    $resets = Get-LcdenResetCount -StdoutPath $run.StdoutPath
    if ($resets -lt 0) {
        Add-Check -Check ('lcd:' + $run.Label) -Result 'FAIL' -Detail 'stdout missing'
    }
    elseif ($resets -le 2) {
        Add-Check -Check ('lcd:' + $run.Label) -Result 'PASS' -Detail ('LCDEN 0 count={0}' -f $resets)
    }
    else {
        Add-Check -Check ('lcd:' + $run.Label) -Result 'FAIL' -Detail ('LCDEN 0 count={0} suspects reset loop' -f $resets)
    }
}

# 5) W3 gate 5: hand on/off same-binary A/B
if ($HandOff) {
    $hOff = Read-AudioHash -Path $runHandoff.HashPath
    if ($hOff -and $hM4 -and ($hOff -eq $hM4)) {
        Add-Check -Check 'audio:handoff' -Result 'PASS' -Detail ('fnv1a=' + $hOff.Substring(0, 16))
    }
    else {
        Add-Check -Check 'audio:handoff' -Result 'FAIL' -Detail 'hand-off vs hand-on audiohash differs'
    }
    $cmpOff = Compare-WavDumps -RefPath $m4Wav -DutPath $handoffWav -ReportPath (Join-Path $OutDir 'audio_diff_handoff.md')
    Add-Check -Check 'audio:handoff-wav' -Result $cmpOff.Result -Detail $cmpOff.Detail

    $scalarsOff = Read-Scalars -Path $runHandoff.StatePath
    if ($scalarsOff.Count -eq 0) {
        Add-Check -Check 'state:handoff' -Result 'FAIL' -Detail 'missing hashdump'
    }
    else {
        $handDiffs = New-Object System.Collections.Generic.List[string]
        foreach ($k in @('mcu.cp', 'mcu.pc', 'mcu.sr', 'mcu.cycles', 'pcm.config_reg_3c', 'pcm.config_reg_3d', 'pcm.irq_assert')) {
            if ($scalarsOff[$k] -ne $scalarsM4[$k]) {
                [void]$handDiffs.Add(('{0}: {1} != {2}' -f $k, $scalarsOff[$k], $scalarsM4[$k]))
            }
        }
        if ($handDiffs.Count -gt 0) {
            Add-Check -Check 'state:handoff' -Result 'FAIL' -Detail ($handDiffs -join '; ')
        }
        else {
            Add-Check -Check 'state:handoff' -Result 'PASS' -Detail 'handoff scalars == hand-on scalars'
        }
    }
}

# ---- summary --------------------------------------------------------------
$table = ($script:checks | Format-Table -AutoSize | Out-String -Width 200).TrimEnd()
Write-Host ""
Write-Host "== summary =="
Write-Host $table
Set-Content -LiteralPath $summaryPath -Value $table -Encoding UTF8

$failCount = @($script:checks | Where-Object { $_.Result -eq 'FAIL' }).Count
if ($failCount -gt 0) {
    Write-Host ("RESULT: FAIL ({0} check(s); outputs in {1})" -f $failCount, $OutDir)
    Write-Host "REMINDER: user must confirm the A/B listen before accepting the run."
    exit 1
}
Write-Host ("RESULT: PASS (automated part; outputs in {0})" -f $OutDir)
Write-Host "REMINDER: A/B listen by the user is still required (plan_256.md §6)."
exit 0
