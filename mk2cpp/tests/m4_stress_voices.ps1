#requires -Version 5.1
<#
.SYNOPSIS
    M4 oracle -- n=32/64/128/255 polyphony stress and long-run checks
    (S1-S7 judgement parsing, O1-O10 cross-check mapping).

.DESCRIPTION
    Dry-run by default: validates arguments, probes the GT build for the
    frozen oracle options and prints, per voice level, the exact GT command,
    the timeout and the expected S1-S7 / O1-O10 checks. It starts NO GT.

    With -Execute (and -UserPresent) it starts GT per level -- LCD window and
    audio ON, never SDL dummy, per tools/docs/plan_256.md §6 -- waits for the
    -wav:/-audiowin <file>.meta marker, the -audiohash checkpoint, the
    -hashdump state file and the -tracepc tail, then force-kills the process
    in a finally block and parses:
      - pcm_trace.log   -> config/select_channel/mask popcount, O5/O6 PCs
      - hashdump        -> pcm.config_reg_3d and state scalars (v1, unchanged)
      - snapinfo        -> pcm.ext_voices (S4/O4) and O2/O3/O7 scalars
        (when supported; plan A per out/m4/12_cfg3d_voice_count.md)
      - tracepc tail    -> window completion / 00:037A stall signature
      - stdout LCDEN    -> O1 reset/heartbeat heuristic
      - process CPU     -> duty / real-time factor (report only)
    The manual part (listen for dropped notes / crackle / LCD anomalies) is
    required per plan_256.md §6 and is NOT automated. SKIP is not PASS.

    Frozen GT interfaces this script calls (10_m4_oracle.md §2.2/§3/§5.1):
      -wav:<file> / -audiowin <start> <end> / -audiohash <cycles> <file>
      -midiseq <file> [start]         (G5, schedule cycles are relative to start)
      -pcmtrace extended to ext writes (G2) for S6/O6
      -snapinfo <cycles> <file> (G3): pcm.ext_voices for S4/O4
        (plan A, out/m4/12 §4.3). -hashdump stays v1 and is NOT required to
        carry ext_voices; a missing snapinfo scalar is a SKIP, never a PASS.
    Missing required options are reported as clear ERROR lines even in
    dry-run; add -RequireReady to make the dry-run fail (exit 2).

.NOTES
    Pure PowerShell 5.1, no Python, no SDL dependency. MIDI material is not
    committed: put poly32/poly64/poly128/poly255.sched (or a single
    -MidiSchedule) under mk2cpp/out/m4/corpus/ (gitignored). A missing
    schedule is a SKIP with a midisched hint, never a PASS.
#>
[CmdletBinding()]
param(
    # Accepts "32,64" and "32 64" (string array: -File binds comma lists as one
    # token in Windows PowerShell 5.1, so the script splits them itself).
    [string[]]$VoiceLevels = @('32', '64', '128', '255'),

    [ValidateSet('midi', 'demo')]
    [string]$Scenario = 'midi',

    # Override for all levels; empty -> <CorpusDir>\poly<n>.sched.
    [string]$MidiSchedule = '',

    [string]$CorpusDir = 'mk2cpp\out\m4\corpus',

    [uint64]$RunTo = 400000000,

    [uint64]$AudioStart = 300000000,
    [uint64]$AudioEnd   = 320000000,

    # 0 -> RunTo; must be >= 200M cycles.
    [uint64]$HashAt = 0,
    # 0 -> RunTo-2M / RunTo
    [uint64]$TraceFrom = 0,
    [uint64]$TraceTo   = 0,

    [int]$TimeoutSec = 0,

    [string]$OutDir = 'mk2cpp\out\m4\stress',

    # Dry-run only: exit 2 when required GT options/material are missing.
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
$script:Midisched = Join-Path $script:RepoRoot 'mk2cpp\tools\midisched\midisched.exe'

# CLI cap is 28..255 (PCM_MAX_VOICE = 255, capacity 256); the milestone text
# says "256 voices" but the accepted oracle wording is -voices:255 (D1).
$script:MinVoices = 28
$script:MaxVoices = 255

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
    return [ordered]@{
        'wav'       = [bool]($help -match [regex]::Escape('-wav:'))
        'audiowin'  = [bool]($help -match [regex]::Escape('-audiowin'))
        'audiohash' = [bool]($help -match [regex]::Escape('-audiohash'))
        'midiseq'   = [bool]($help -match [regex]::Escape('-midiseq'))
        'hand'      = [bool]($help -match [regex]::Escape('-mk2cpp-hand'))
        'snapinfo'  = [bool]($help -match [regex]::Escape('-snapinfo'))
    }
}

function Write-CapabilityReport {
    param($Caps)
    Write-Host ("GT capability probe (`"$script:ExePath`" -h):")
    $order = @(
        @('wav',       '-wav:<file>',                 'G1   required (10_m4_oracle.md §2.2)'),
        @('audiowin',  '-audiowin <start> <end>',     'G1   required (10_m4_oracle.md §2.2)'),
        @('audiohash', '-audiohash <cycles> <file>',  'G1   required (10_m4_oracle.md §2.2)'),
        @('midiseq',   '-midiseq <file> [start]',     'G5   required for -Scenario midi'),
        @('snapinfo',  '-snapinfo <cycles> <file>',   'G3   optional (S4/O4 ext_voices; O2/O3/O7)'),
        @('hand',      '-mk2cpp-hand:0|1',            'W3   optional A/B switch')
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
if ($AudioEnd -le $AudioStart -or $AudioEnd -gt $RunTo) {
    Exit-Setup ("invalid audio window [" + $AudioStart + "," + $AudioEnd + ") vs RunTo=" + $RunTo)
}
if ($HashAt -eq 0) { $HashAt = $RunTo }
if ($HashAt -lt 200000000) {
    Exit-Setup ("HashAt=" + $HashAt + " is below the 200M-cycle state sampling floor")
}
if ($TraceFrom -eq 0 -and $TraceTo -eq 0) {
    $TraceFrom = $RunTo - 2000000
    $TraceTo   = $RunTo
}
if ($TraceTo -le $TraceFrom) {
    Exit-Setup ("invalid trace window [" + $TraceFrom + "," + $TraceTo + ")")
}
if ($TraceTo -gt ($RunTo + 12)) {
    Exit-Setup ("TraceTo=" + $TraceTo + " is beyond RunTo=" + $RunTo)
}
if ($Scenario -eq 'midi' -and -not [string]::IsNullOrWhiteSpace($MidiSchedule)) {
    if (-not (Test-Path -LiteralPath $MidiSchedule)) {
        Exit-Setup ("MIDI schedule not found: " + $MidiSchedule)
    }
}
if ($TimeoutSec -le 0) { $TimeoutSec = Get-GtTimeoutSec -Cycles $RunTo }

$levelTokens = New-Object System.Collections.Generic.List[int]
foreach ($item in $VoiceLevels) {
    foreach ($tok in ($item -split '[,\s]+')) {
        if ([string]::IsNullOrWhiteSpace($tok)) { continue }
        $n = 0
        if (-not [int]::TryParse($tok, [ref]$n)) {
            Exit-Setup ("invalid -VoiceLevels token '" + $tok + "'")
        }
        [void]$levelTokens.Add($n)
    }
}
$levels = New-Object System.Collections.Generic.List[int]
foreach ($v in ($levelTokens | Sort-Object -Unique)) {
    if ($v -lt $script:MinVoices -or $v -gt $script:MaxVoices) {
        Write-Host ("WARN: -voices:{0} outside the current CLI range {1}..{2}; skipped (10_m4_oracle.md §5.4 D1; capacity is 256, accepted cap is 255)." -f $v, $script:MinVoices, $script:MaxVoices)
        continue
    }
    [void]$levels.Add([int]$v)
}
if ($levels.Count -eq 0) { Exit-Setup "no supported voice level" }

if (-not [System.IO.Path]::IsPathRooted($OutDir)) {
    $OutDir = Join-Path $script:RepoRoot $OutDir
}
$OutDir = [System.IO.Path]::GetFullPath($OutDir)
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$summaryPath = Join-Path $OutDir 'summary.txt'
$planPath    = Join-Path $OutDir 'plan.txt'

if (-not [System.IO.Path]::IsPathRooted($CorpusDir)) {
    $CorpusDir = Join-Path $script:RepoRoot $CorpusDir
}
$script:CorpusDir = [System.IO.Path]::GetFullPath($CorpusDir)

function Get-MidischedHint {
    param([int]$Voices)
    $out = Join-Path $script:CorpusDir ('poly' + $Voices + '.sched')
    return ('missing ' + $out + ' (build with mk2cpp\tools\midisched; see mk2cpp\tools\README.md)')
}

function Write-MidischedRecipe {
    Write-Host "  recipe: clang -O2 -o mk2cpp\tools\midisched\midisched.exe mk2cpp\tools\midisched\midisched.c"
    Write-Host "          mk2cpp\tools\midisched\midisched.exe <polyNN.smf> -o mk2cpp\out\m4\corpus\polyNN.sched"
}

function Resolve-SchedulePath {
    param([int]$Voices)
    if (-not [string]::IsNullOrWhiteSpace($MidiSchedule)) { return $MidiSchedule }
    return (Join-Path $script:CorpusDir ('poly' + $Voices + '.sched'))
}

$caps = Get-GtCapabilities

$missing = New-Object System.Collections.Generic.List[string]
if (-not $caps['wav'])       { [void]$missing.Add('-wav:<file>') }
if (-not $caps['audiowin'])  { [void]$missing.Add('-audiowin <start> <end>') }
if (-not $caps['audiohash']) { [void]$missing.Add('-audiohash <cycles> <file>') }
if ($Scenario -eq 'midi' -and -not $caps['midiseq']) { [void]$missing.Add('-midiseq <file> [start]') }

$schedulePlan = @{}
foreach ($v in $levels) {
    $path = Resolve-SchedulePath -Voices $v
    $schedulePlan[$v] = [pscustomobject]@{
        Path = $path
        Ok   = (Test-Path -LiteralPath $path)
    }
}

# ---- per-level command builder --------------------------------------------
function Get-LevelArgs {
    param([int]$Voices, [string]$LevelDir, [string]$SchedulePath, $Caps)
    $wav   = Join-Path $LevelDir 'audio.wav'
    $ahash = Join-Path $LevelDir 'audio.hash'
    $hash  = Join-Path $LevelDir 'state.hash'
    $trace = Join-Path $LevelDir 'trace.txt'

    $a = New-Object System.Collections.Generic.List[string]
    [void]$a.Add('-mk2')
    [void]$a.Add('-mk2cpp')
    [void]$a.Add('-voices:' + [string]$Voices)
    if ($Scenario -eq 'demo') {
        [void]$a.Add('-demo')
    }
    else {
        [void]$a.Add('-midiseq')
        [void]$a.Add($SchedulePath)
        [void]$a.Add('200000000')
    }
    [void]$a.Add('-pcmtrace')
    [void]$a.Add('-wav:' + $wav)
    [void]$a.Add('-audiowin')
    [void]$a.Add([string]$AudioStart)
    [void]$a.Add([string]$AudioEnd)
    [void]$a.Add('-audiohash')
    [void]$a.Add([string]$RunTo)
    [void]$a.Add($ahash)
    [void]$a.Add('-hashdump')
    [void]$a.Add([string]$HashAt)
    [void]$a.Add($hash)
    [void]$a.Add('-tracepc')
    [void]$a.Add($trace)
    [void]$a.Add([string]$TraceFrom)
    [void]$a.Add([string]$TraceTo)
    if ($Caps['snapinfo']) {
        [void]$a.Add('-snapinfo')
        [void]$a.Add([string]$HashAt)
        [void]$a.Add((Join-Path $LevelDir 'state.snapinfo'))
    }
    return ,$a.ToArray()
}

function Format-Cmd {
    param([string[]]$ArgList)
    $quoted = @($ArgList | ForEach-Object {
        if ($_ -match '\s') { '"' + $_ + '"' } else { $_ }
    })
    return ($quoted -join ' ')
}

# ---- dry-run (default) ----------------------------------------------------
function Write-Plan {
    $lines = New-Object System.Collections.Generic.List[string]
    [void]$lines.Add("m4_stress_voices.ps1 -- plan (dry-run unless -Execute)")
    [void]$lines.Add(("scenario   : {0}" -f $Scenario))
    [void]$lines.Add(("run to     : {0}   audio [{1},{2})   hash@{3}" -f $RunTo, $AudioStart, $AudioEnd, $HashAt))
    [void]$lines.Add(("trace      : [{0},{1})" -f $TraceFrom, $TraceTo))
    [void]$lines.Add(("timeout    : {0}s per level (ceil(RunTo/24e6*2)+15)" -f $TimeoutSec))
    [void]$lines.Add(("outdir     : {0}" -f $OutDir))
    [void]$lines.Add(("corpus     : {0}" -f $script:CorpusDir))
    [void]$lines.Add("")
    foreach ($v in $levels) {
        $levelDir = Join-Path $OutDir ('n' + $v)
        $sp = $schedulePlan[$v]
        if ($Scenario -eq 'midi' -and -not $sp.Ok) {
            [void]$lines.Add(("n={0}: SKIP -- {1}" -f $v, (Get-MidischedHint -Voices $v)))
            continue
        }
        $cmdArgs = Get-LevelArgs -Voices $v -LevelDir $levelDir -SchedulePath $sp.Path -Caps $caps
        $extNote = 'n/a'
        if ($v -gt 32) { $extNote = 'required (G2 ext pcmtrace)' }
        [void]$lines.Add(("n={0} (per-process voice slots; capacity 256)" -f $v))
        [void]$lines.Add("  cmd: " + (Format-Cmd -ArgList $cmdArgs))
        [void]$lines.Add(("  expect: cfg3d=7b; ext_voices={0}; select_channel 0..{1}; low32 mask popcount={2}; ext={3}" -f $v, ($v - 1), [Math]::Min($v, 32), $extNote))
    }
    if ($Scenario -eq 'midi' -and @($levels | Where-Object { -not $schedulePlan[$_].Ok }).Count -gt 0) {
        [void]$lines.Add("")
        [void]$lines.Add("midisched recipe (corpus material is not committed):")
        [void]$lines.Add("  clang -O2 -o mk2cpp\tools\midisched\midisched.exe mk2cpp\tools\midisched\midisched.c")
        [void]$lines.Add("  mk2cpp\tools\midisched\midisched.exe <polyNN.smf> -o mk2cpp\out\m4\corpus\polyNN.sched")
    }
    [void]$lines.Add("")
    [void]$lines.Add("automated checks per level (S1-S7, 10_m4_oracle.md §3.3):")
    [void]$lines.Add("  S1 no reset/stall : waits for wav .meta + audiohash + hashdump + trace tail >= TraceTo-12; 00:037A not stuck; LCDEN 0 <=2")
    [void]$lines.Add("  S2 PCM active     : audio.wav payload non-zero; audiohash present (pcmdiff --stats when built)")
    [void]$lines.Add("  S3 CPU duty       : report cpu/wall + real-time factor")
    [void]$lines.Add("  S4 cfg3d/extv     : cfg3d == 0x7b (hashdump/snapinfo) AND pcm.ext_voices == n (snapinfo only; hashdump stays v1); missing ext_voices -> SKIP")
    [void]$lines.Add("  S5 select_channel : unique reg=3e values == n, max == n-1")
    [void]$lines.Add("  S6 voice_enable   : max popcount of latest reg 00..03 == min(n,32); ext writes needed for n>32 (G2)")
    [void]$lines.Add("  S7 IRQ slot >=32  : -snapinfo pcm.irq_channel or ext read 0xE820 (G2/G3); else SKIP")
    [void]$lines.Add("")
    [void]$lines.Add("O1-O10 mapping (10_m4_oracle.md §3.4; automatic rows are parsed per level):")
    [void]$lines.Add("  O1  LCD enable      : stdout LCDEN 1 present; LCDEN 0 <=2")
    [void]$lines.Add("  O2  Heartbeat       : snapinfo isr scalar >0; else SKIP")
    [void]$lines.Add("  O3  Idle state      : snapinfo sleep/iml/pend reported; else SKIP")
    [void]$lines.Add("  O4  Config byte     : same evidence as S4 (cfg3d == 0x7b + pcm.ext_voices == n via -snapinfo, plan A / out/m4/12)")
    [void]$lines.Add("  O5  mask flush PC   : pcmtrace reg<=3 at pc 00:5527 / 00:5664")
    [void]$lines.Add("  O6  ext writes      : pcmtrace ext reg/val non-zero for n>32; else SKIP (G2)")
    [void]$lines.Add("  O7  IRQ slot        : same evidence as S7")
    [void]$lines.Add("  O8  power-cycle demo: LCDEN 0 count includes the 145.72M/163.94M cycle (demo only)")
    [void]$lines.Add("  O9  no-flag regress : covered by two_mode_check.ps1 -Scenario demo200 + tools/baselines")
    [void]$lines.Add("  O10 stall signature : 00:037A hits + repeated LCDEN 0 (S1)")
    [void]$lines.Add("")
    [void]$lines.Add("manual checks (user present, required):")
    [void]$lines.Add("  M1 LCD renders; no boot animation loop / reset")
    [void]$lines.Add("  M2 listen: no dropped notes under 255, no crackle/instability")
    $text = ($lines -join [Environment]::NewLine)
    Set-Content -LiteralPath $planPath -Value $text -Encoding UTF8
    Write-Host $text
}

Write-Plan
Write-Host ""
Write-CapabilityReport -Caps $caps

if ($missing.Count -gt 0) {
    Write-Host ""
    Write-Host ("ERROR: this GT build lacks {0} option(s) required by m4_stress_voices.ps1:" -f $missing.Count)
    foreach ($m in $missing) {
        Write-Host ("  ERROR: missing GT option " + $m)
    }
    Write-Host "       implementer (Wave 0a/0b): land the frozen options in"
    Write-Host "       mk2cpp/out/m4/10_m4_oracle.md §2.2/§3.1/§5.1 (G1/G2/G3/G5);"
    Write-Host "       -audiowin must backfill RIFF/data sizes and write <file>.meta last;"
    Write-Host "       schedule cycles are relative to the -midiseq start argument."
}

$missingSchedules = @($levels | Where-Object { $Scenario -eq 'midi' -and -not $schedulePlan[$_].Ok })
if ($missingSchedules.Count -gt 0) {
    Write-Host ""
    Write-Host ("SKIP: {0}/{1} level(s) have no MIDI schedule (corpus is not committed)." -f $missingSchedules.Count, $levels.Count)
    foreach ($v in $missingSchedules) {
        Write-Host ("  SKIP n={0}: {1}" -f $v, (Get-MidischedHint -Voices $v))
    }
    Write-MidischedRecipe
}

if (-not $Execute) {
    if ($RequireReady -and ($missing.Count -gt 0 -or $missingSchedules.Count -gt 0)) {
        Write-Host ("ERROR: -RequireReady: {0} GT option(s) and {1} schedule(s) missing." -f $missing.Count, $missingSchedules.Count)
        exit 2
    }
    Write-Host ""
    Write-Host ("DRY-RUN OK ({0} required GT option(s) missing, {1} level schedule(s) missing). Re-run with -Execute -UserPresent when ready." -f $missing.Count, $missingSchedules.Count)
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
    param([string]$Level, [string]$Check, [string]$Result, [string]$Detail)
    [void]$script:checks.Add([pscustomobject]@{
        Level  = $Level
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

function Get-Scalar {
    param($Table, [string[]]$Keys)
    if ($null -eq $Table) { return '' }
    foreach ($k in $Keys) {
        if ($Table.ContainsKey($k) -and -not [string]::IsNullOrWhiteSpace($Table[$k])) { return [string]$Table[$k] }
    }
    return ''
}

function Get-TraceTailInfo {
    param([string]$Path, [uint64]$TraceEnd)
    if (-not (Test-Path -LiteralPath $Path)) { return $null }
    $tail = Get-Content -LiteralPath $Path -Tail 1
    if ([string]::IsNullOrWhiteSpace($tail)) { return $null }
    $parts = $tail -split '\s+'
    if ($parts.Count -lt 3) { return $null }
    $cyc = [uint64]$parts[1]
    return [pscustomobject]@{
        Cycle   = $cyc
        Pc      = $parts[2]
        Reached = ($cyc -ge ($TraceEnd - 12))
    }
}

function Get-LcdenResetCount {
    param([string]$StdoutPath)
    if (-not (Test-Path -LiteralPath $StdoutPath)) { return -1 }
    return @(Select-String -LiteralPath $StdoutPath -Pattern 'LCDEN 0' -ErrorAction SilentlyContinue).Count
}

function Get-LcdenEnableCount {
    param([string]$StdoutPath)
    if (-not (Test-Path -LiteralPath $StdoutPath)) { return -1 }
    return @(Select-String -LiteralPath $StdoutPath -Pattern 'LCDEN 1' -ErrorAction SilentlyContinue).Count
}

function Get-StallCount {
    param([string]$TracePath)
    if (-not (Test-Path -LiteralPath $TracePath)) { return -1 }
    return @(Select-String -LiteralPath $TracePath -Pattern ' 00:037a$' -ErrorAction SilentlyContinue).Count
}

function Read-PcmTrace {
    param([string]$Path)
    $list = New-Object System.Collections.Generic.List[object]
    if (-not (Test-Path -LiteralPath $Path)) { return ,$list }
    foreach ($line in Get-Content -LiteralPath $Path) {
        if ($line -match '^pcm\s+(?:(ext)\s+)?reg=([0-9a-fA-F]{2})\s+val=([0-9a-fA-F]{2})\s+pc=([0-9a-fA-F]{2}):([0-9a-fA-F]{4})\s+cyc=(\d+)') {
            $isExt = ($Matches[1] -eq 'ext')
            [void]$list.Add([pscustomobject]@{
                Ext = $isExt
                Reg = [Convert]::ToInt32($Matches[2], 16)
                Val = [Convert]::ToInt32($Matches[3], 16)
                Pc  = ($Matches[4] + ':' + $Matches[5])
                Cyc = [uint64]$Matches[6]
            })
        }
    }
    return ,$list
}

function Get-MaxMaskPop {
    param($Entries)
    $cur = @(0, 0, 0, 0)
    $maxPop = 0
    foreach ($e in ($Entries | Where-Object { -not $_.Ext -and $_.Reg -le 3 } | Sort-Object Cyc)) {
        $cur[$e.Reg] = $e.Val
        $pop = 0
        for ($r = 0; $r -lt 4; $r++) {
            $v = $cur[$r]
            while ($v -gt 0) {
                $pop += ($v -band 1)
                $v = $v -shr 1
            }
        }
        if ($pop -gt $maxPop) { $maxPop = $pop }
    }
    return $maxPop
}

function Test-WavNonZero {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { return $false }
    $bytes = [System.IO.File]::ReadAllBytes($Path)
    if ($bytes.Length -le 44) { return $false }
    $take = [int][Math]::Min(4194304, $bytes.Length - 44)
    for ($i = 44; $i -lt (44 + $take); $i++) {
        if ($bytes[$i] -ne 0) { return $true }
    }
    return $false
}

function Invoke-GtWindowed {
    param(
        [string]$Label,
        [string[]]$ArgList,
        [string]$WavPath,
        [string]$LogBase,
        [string[]]$WaitFiles = @(),
        [string]$TracePath = '',
        [uint64]$TraceEnd = 0
    )
    $stdoutPath = $LogBase + '.stdout.txt'
    $stderrPath = $LogBase + '.stderr.txt'
    $metaPath   = $WavPath + '.meta'
    foreach ($f in @($WavPath, $metaPath, $stdoutPath, $stderrPath)) {
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
            $ready = (Test-NonEmptyFile -Path $metaPath)
            if ($ready) {
                foreach ($wf in $WaitFiles) {
                    if (-not (Test-NonEmptyFile -Path $wf)) { $ready = $false; break }
                }
            }
            if ($ready -and $TracePath -and $TraceEnd -gt 0) {
                $tailInfo = Get-TraceTailInfo -Path $TracePath -TraceEnd $TraceEnd
                if ($null -eq $tailInfo -or -not $tailInfo.Reached) { $ready = $false }
            }
            if ($ready) { $status = 'ready'; break }
            try { $proc.Refresh() } catch { }
            if ($proc.HasExited) { $status = 'exited'; break }
            if ((Get-Date) -ge $deadline) { $status = 'timeout'; break }
            Start-Sleep -Milliseconds 500
        }
        if ($status -ne 'ready') {
            $ready = (Test-NonEmptyFile -Path $metaPath)
            if ($ready) {
                foreach ($wf in $WaitFiles) {
                    if (-not (Test-NonEmptyFile -Path $wf)) { $ready = $false; break }
                }
            }
            if ($ready) {
                if ($TracePath -and $TraceEnd -gt 0) {
                    $tailInfo = Get-TraceTailInfo -Path $TracePath -TraceEnd $TraceEnd
                    if ($null -eq $tailInfo -or -not $tailInfo.Reached) { $ready = $false }
                }
            }
            if ($ready) { $status = 'ready' }
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
        StdoutPath = $stdoutPath
        StderrPath = $stderrPath
        WallSec    = $wall
        CpuSec     = $cpuSec
        CpuDuty    = $duty
    }
}

# ---- run per level --------------------------------------------------------
foreach ($v in $levels) {
    $label = 'n' + $v
    $levelDir = Join-Path $OutDir $label
    New-Item -ItemType Directory -Force -Path $levelDir | Out-Null
    $sp = $schedulePlan[$v]

    if ($Scenario -eq 'midi' -and -not $sp.Ok) {
        Add-Check -Level $label -Check 'S0:schedule' -Result 'SKIP' -Detail (Get-MidischedHint -Voices $v)
        continue
    }

    $cmdArgs = Get-LevelArgs -Voices $v -LevelDir $levelDir -SchedulePath $sp.Path -Caps $caps
    $wavPath    = Join-Path $levelDir 'audio.wav'
    $ahashPath  = Join-Path $levelDir 'audio.hash'
    $hashPath   = Join-Path $levelDir 'state.hash'
    $tracePath  = Join-Path $levelDir 'trace.txt'
    $snapPath   = Join-Path $levelDir 'state.snapinfo'

    $waitFiles = @($ahashPath, $hashPath)
    if ($caps['snapinfo']) { $waitFiles += $snapPath }

    $run = Invoke-GtWindowed -Label $label -ArgList $cmdArgs -WavPath $wavPath -LogBase (Join-Path $levelDir 'run') `
        -WaitFiles $waitFiles -TracePath $tracePath -TraceEnd $TraceTo

    $pcmSrc = Join-Path $script:WorkDir 'pcm_trace.log'
    $pcmDst = Join-Path $levelDir 'pcm_trace.log'
    if (Test-Path -LiteralPath $pcmSrc) {
        Move-Item -LiteralPath $pcmSrc -Destination $pcmDst -Force
    }

    if ($run.Status -ne 'ready') {
        Add-Check -Level $label -Check 'S1:run' -Result 'FAIL' -Detail ('status=' + $run.Status)
        continue
    }

    $realTimeFactor = 'n/a'
    if ($run.WallSec -gt 0) {
        $realTimeFactor = ('{0:N2}x' -f ($RunTo / ($run.WallSec * 24e6)))
    }
    Add-Check -Level $label -Check 'S3:cpu' -Result 'INFO' `
        -Detail ('wall={0:N1}s cpu={1:N1}s duty={2} rt={3}' -f $run.WallSec, $run.CpuSec, $run.CpuDuty, $realTimeFactor)

    $tail = Get-TraceTailInfo -Path $tracePath -TraceEnd $TraceTo
    $stall = Get-StallCount -TracePath $tracePath
    if ($null -eq $tail -or -not $tail.Reached) {
        Add-Check -Level $label -Check 'S1:trace' -Result 'FAIL' -Detail 'trace tail < window end (stall?)'
        Add-Check -Level $label -Check 'O10:stall' -Result 'FAIL' -Detail 'trace incomplete'
    }
    elseif ($stall -gt 64) {
        Add-Check -Level $label -Check 'S1:trace' -Result 'FAIL' -Detail ('00:037A hits={0} (O10 signature)' -f $stall)
        Add-Check -Level $label -Check 'O10:stall' -Result 'FAIL' -Detail ('00:037A hits={0}' -f $stall)
    }
    else {
        Add-Check -Level $label -Check 'S1:trace' -Result 'PASS' -Detail ('tail c{0} pc={1}' -f $tail.Cycle, $tail.Pc)
        Add-Check -Level $label -Check 'O10:stall' -Result 'PASS' -Detail ('00:037A hits={0}' -f $stall)
    }

    $resets = Get-LcdenResetCount -StdoutPath $run.StdoutPath
    if ($resets -lt 0) {
        Add-Check -Level $label -Check 'S1:lcd' -Result 'FAIL' -Detail 'stdout missing'
    }
    elseif ($resets -le 2) {
        Add-Check -Level $label -Check 'S1:lcd' -Result 'PASS' -Detail ('LCDEN 0 count={0}' -f $resets)
    }
    else {
        Add-Check -Level $label -Check 'S1:lcd' -Result 'FAIL' -Detail ('LCDEN 0 count={0} (reset loop?)' -f $resets)
    }

    $enables = Get-LcdenEnableCount -StdoutPath $run.StdoutPath
    if ($enables -gt 0) {
        Add-Check -Level $label -Check 'O1:lcd-enable' -Result 'PASS' -Detail ('LCDEN 1 count={0}' -f $enables)
    }
    else {
        Add-Check -Level $label -Check 'O1:lcd-enable' -Result 'FAIL' -Detail 'no LCDEN 1 in stdout'
    }
    if ($Scenario -eq 'demo' -and $resets -ge 1) {
        Add-Check -Level $label -Check 'O8:power-cycle' -Result 'PASS' -Detail ('LCDEN 0 count={0} (demo power cycle)' -f $resets)
    }
    else {
        Add-Check -Level $label -Check 'O8:power-cycle' -Result 'SKIP' -Detail 'demo-only evidence (O8)'
    }
    Add-Check -Level $label -Check 'O9:regress' -Result 'INFO' -Detail 'covered by two_mode_check.ps1 -Scenario demo200 + tools/baselines'

    if (Test-WavNonZero -Path $wavPath) {
        Add-Check -Level $label -Check 'S2:pcm' -Result 'PASS' -Detail 'audio.wav payload has non-zero samples'
    }
    else {
        Add-Check -Level $label -Check 'S2:pcm' -Result 'FAIL' -Detail 'audio.wav silent/empty'
    }

    $scalars = Read-Scalars -Path $hashPath
    $snapScalars = @{}
    if ($caps['snapinfo'] -and (Test-Path -LiteralPath $snapPath)) {
        $snapScalars = Read-Scalars -Path $snapPath
    }
    if ($snapScalars.Count -eq 0) { $snapScalars = $scalars }
    $diagScalars = @{}
    foreach ($k in $scalars.Keys) { $diagScalars[$k] = $scalars[$k] }
    foreach ($k in $snapScalars.Keys) { $diagScalars[$k] = $snapScalars[$k] }

    # Plan A (out/m4/12, 2026-09-11): config_reg_3d keeps stock 0x7b
    # (bit5 = ROM bank mode) in every mode; extended voice count is reported
    # by -snapinfo as the pcm.ext_voices scalar (-hashdump stays v1).
    # Missing scalar -> SKIP, never PASS.
    $cfgStr = Get-Scalar -Table $diagScalars -Keys @('pcm.config_reg_3d')
    $cfgVal = -1
    $cfgOk = $false
    if (-not [string]::IsNullOrWhiteSpace($cfgStr)) {
        try { $cfgVal = [Convert]::ToInt32($cfgStr, 16); $cfgOk = $true } catch { $cfgOk = $false }
    }
    if (-not $cfgOk) {
        Add-Check -Level $label -Check 'S4:cfg3d' -Result 'FAIL' -Detail ('hashdump missing/unparsable pcm.config_reg_3d (' + $cfgStr + ')')
    }
    elseif ($cfgVal -ne 0x7b) {
        Add-Check -Level $label -Check 'S4:cfg3d' -Result 'FAIL' -Detail ('got 0x{0:x2}, want 0x7b (plan A, out/m4/12)' -f $cfgVal)
    }
    else {
        $extStr = Get-Scalar -Table $diagScalars -Keys @('pcm.ext_voices')
        $extVal = -1
        $extOk = $false
        if (-not [string]::IsNullOrWhiteSpace($extStr)) {
            try { $extVal = [Convert]::ToInt32($extStr, 10); $extOk = $true } catch { $extOk = $false }
        }
        if (-not $extOk) {
            Add-Check -Level $label -Check 'S4:cfg3d' -Result 'SKIP' `
                -Detail ('cfg3d=0x7b OK but pcm.ext_voices scalar missing/unparsable in -snapinfo; ' + 'got "' + $extStr + '" (plan A, out/m4/12 §4.3)')
        }
        elseif ($extVal -eq $v) {
            Add-Check -Level $label -Check 'S4:cfg3d' -Result 'PASS' -Detail ('cfg3d=0x7b ext_voices={0}' -f $extVal)
        }
        else {
            Add-Check -Level $label -Check 'S4:cfg3d' -Result 'FAIL' -Detail ('cfg3d=0x7b but ext_voices={0}, want {1} (plan A, out/m4/12)' -f $extVal, $v)
        }
    }

    $entries = Read-PcmTrace -Path $pcmDst
    if ($entries.Count -eq 0) {
        Add-Check -Level $label -Check 'S5:select' -Result 'FAIL' -Detail 'pcm_trace.log missing/empty'
        Add-Check -Level $label -Check 'S6:mask' -Result 'FAIL' -Detail 'pcm_trace.log missing/empty'
        Add-Check -Level $label -Check 'O5:flush' -Result 'SKIP' -Detail 'no pcmtrace'
        Add-Check -Level $label -Check 'O6:ext-write' -Result 'SKIP' -Detail 'no pcmtrace'
    }
    else {
        $selVals = @($entries | Where-Object { -not $_.Ext -and $_.Reg -eq 0x3e } | ForEach-Object { [int]$_.Val })
        if ($selVals.Count -eq 0) {
            Add-Check -Level $label -Check 'S5:select' -Result 'FAIL' -Detail 'no reg=3e writes'
        }
        else {
            $selMax = ($selVals | Measure-Object -Maximum).Maximum
            $selUnique = @($selVals | Sort-Object -Unique).Count
            if ($selMax -eq ($v - 1) -and $selUnique -eq $v) {
                Add-Check -Level $label -Check 'S5:select' -Result 'PASS' -Detail ('unique={0} max={1}' -f $selUnique, $selMax)
            }
            else {
                Add-Check -Level $label -Check 'S5:select' -Result 'FAIL' -Detail ('unique={0} max={1}, want n={2} max={3}' -f $selUnique, $selMax, $v, ($v - 1))
            }
        }

        $maxPop = Get-MaxMaskPop -Entries $entries
        $wantPop = [Math]::Min($v, 32)
        if ($maxPop -eq $wantPop) {
            $detail = ('max popcount={0}' -f $maxPop)
            if ($v -gt 32) { $detail += ' (low 32 only; ext writes still needed for 4..n-1 - G2)' }
            Add-Check -Level $label -Check 'S6:mask' -Result 'PASS' -Detail $detail
        }
        else {
            Add-Check -Level $label -Check 'S6:mask' -Result 'FAIL' -Detail ('max popcount={0}, want {1}' -f $maxPop, $wantPop)
        }

        $flushPcs = @('00:5527', '00:5664')
        $flushHits = @($entries | Where-Object { -not $_.Ext -and $_.Reg -le 3 -and ($flushPcs -contains $_.Pc) })
        if ($flushHits.Count -ge 2) {
            Add-Check -Level $label -Check 'O5:flush' -Result 'PASS' -Detail ('hits={0} (5527/5664)' -f $flushHits.Count)
        }
        else {
            Add-Check -Level $label -Check 'O5:flush' -Result 'FAIL' -Detail ('hits={0}, want both 00:5527 and 00:5664' -f $flushHits.Count)
        }

        $extWrites = @($entries | Where-Object { $_.Ext -and $_.Val -ne 0 })
        if ($v -le 32) {
            Add-Check -Level $label -Check 'O6:ext-write' -Result 'SKIP' -Detail 'n<=32 does not need ext writes (O6)'
        }
        elseif ($extWrites.Count -eq 0) {
            Add-Check -Level $label -Check 'O6:ext-write' -Result 'SKIP' -Detail 'no ext pcmtrace evidence (G2); low-32 mask only'
        }
        else {
            Add-Check -Level $label -Check 'O6:ext-write' -Result 'PASS' -Detail ('non-zero ext writes={0}' -f $extWrites.Count)
        }
    }

    $isr = Get-Scalar -Table $snapScalars -Keys @('isr4fe', 'isr', 'heartbeat.isr')
    $isrVal = [uint64]0
    $isrOk = $false
    if ($isr -ne '') { $isrOk = [uint64]::TryParse($isr, [ref]$isrVal) }
    if ($isrOk) {
        if ($isrVal -gt 0) {
            Add-Check -Level $label -Check 'O2:isr' -Result 'PASS' -Detail ('isr={0}' -f $isr)
        }
        else {
            Add-Check -Level $label -Check 'O2:isr' -Result 'FAIL' -Detail 'isr=0 (heartbeat stopped, O10 signature)'
        }
    }
    else {
        Add-Check -Level $label -Check 'O2:isr' -Result 'SKIP' -Detail ('no parsable isr scalar (G3: -snapinfo; got "' + $isr + '")')
    }

    $sleep = Get-Scalar -Table $snapScalars -Keys @('sleep', 'mcu.sleep')
    $iml = Get-Scalar -Table $snapScalars -Keys @('iml', 'mcu.iml')
    $pend = Get-Scalar -Table $snapScalars -Keys @('pend', 'mcu.pend')
    if ($sleep -ne '' -or $iml -ne '' -or $pend -ne '') {
        Add-Check -Level $label -Check 'O3:idle' -Result 'INFO' -Detail ('sleep={0} iml={1} pend={2}' -f $sleep, $iml, $pend)
    }
    else {
        Add-Check -Level $label -Check 'O3:idle' -Result 'SKIP' -Detail 'no sleep/iml/pend scalars (G3)'
    }

    $irqCh = Get-Scalar -Table $snapScalars -Keys @('pcm.irq_channel', 'irq_channel', 'd15c')
    if ($v -le 32) {
        Add-Check -Level $label -Check 'S7:irq' -Result 'SKIP' -Detail 'n<=32 not required by O7'
    }
    elseif ($irqCh -ne '') {
        $irqVal = -1
        $irqOk = $false
        try { $irqVal = [Convert]::ToInt32($irqCh, 16); $irqOk = $true } catch { $irqOk = $false }
        if (-not $irqOk) {
            Add-Check -Level $label -Check 'S7:irq' -Result 'SKIP' -Detail ('unparsable irq_channel "' + $irqCh + '"')
        }
        elseif ($irqVal -lt 0x80 -and $irqVal -le ($v - 1)) {
            Add-Check -Level $label -Check 'S7:irq' -Result 'PASS' -Detail ('irq_channel=0x{0:x2} (< n)' -f $irqVal)
        }
        else {
            Add-Check -Level $label -Check 'S7:irq' -Result 'FAIL' -Detail ('irq_channel=0x{0:x2} out of range (n={1})' -f $irqVal, $v)
        }
    }
    else {
        Add-Check -Level $label -Check 'S7:irq' -Result 'SKIP' -Detail 'no irq_channel evidence (G2 ext read 0xE820 / G3 snapinfo)'
    }

    if (Test-Path -LiteralPath $script:Pcmdiff) {
        $report = Join-Path $levelDir 'audio_stats.md'
        $oldEap = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        try { [void]@(& $script:Pcmdiff --ref $wavPath --dut $wavPath --stats --report $report 2>&1) } catch { }
        $ErrorActionPreference = $oldEap
        Add-Check -Level $label -Check 'S2:stats' -Result 'INFO' -Detail ('pcmdiff stats -> ' + $report)
    }
}

# ---- summary --------------------------------------------------------------
$table = ($script:checks | Format-Table -AutoSize | Out-String -Width 220).TrimEnd()
Write-Host ""
Write-Host "== summary =="
Write-Host $table
Set-Content -LiteralPath $summaryPath -Value $table -Encoding UTF8

$failCount = @($script:checks | Where-Object { $_.Result -eq 'FAIL' }).Count
$skipCount = @($script:checks | Where-Object { $_.Result -eq 'SKIP' }).Count
if ($failCount -gt 0) {
    Write-Host ("RESULT: FAIL ({0} check(s), {1} SKIP; outputs in {2})" -f $failCount, $skipCount, $OutDir)
    Write-Host "REMINDER: manual listen is still required (plan_256.md §6)."
    exit 1
}
Write-Host ("RESULT: PASS automated part ({0} SKIP; outputs in {1})" -f $skipCount, $OutDir)
Write-Host "REMINDER: SKIPs are not PASSes; manual listen is still required (plan_256.md §6)."
exit 0
