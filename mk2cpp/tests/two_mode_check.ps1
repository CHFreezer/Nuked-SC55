#requires -Version 5.1
<#
.SYNOPSIS
    Two-mode regression check for the mk2cpp integration:
    default GT interpreter vs `-mk2cpp` translated core, same scenario/window.

.DESCRIPTION
    Runs build\nuked-sc55.exe twice for one scenario (with cwd=build so the ROM
    BasePath resolves), polling for the -hashdump file and trace-window
    completion, then force-kills the process (GT never self-exits).

    Compares:
      1. def.trace vs tr.mk2cpp.trace          (tracediff.exe)
      2. def.hash  vs tr.mk2cpp.hash            (Get-FileHash SHA256)
      3. def.trace vs tools\baselines\... when scenario+window matches a
         frozen fixture (boot = [0,3M), demo200 = [200M,202M))

    With no translated code linked, `-mk2cpp` is pure interpreter fallback and
    every comparison is expected to pass.

.NOTES
    Pure PowerShell 5.1. No Python. All outputs live under -OutDir.
#>
[CmdletBinding()]
param(
    [ValidateSet('boot', 'demo200', 'custom')]
    [string]$Scenario = 'boot',

    [ValidateSet('auto', 'boot3m', '200m', 'custom')]
    [string]$Window = 'auto',

    [uint64]$WindowFrom = 0,
    [uint64]$WindowTo = 0,
    [uint64]$HashCycle = 0,

    [int]$TimeoutSec = 60,

    [string]$OutDir = 'mk2cpp\out\twomode',

    [string[]]$ScenarioArgs = @(),

    [switch]$KeepGoing
)

$ErrorActionPreference = 'Stop'

# ---- paths ---------------------------------------------------------------
$script:RepoRoot  = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$script:WorkDir   = Join-Path $script:RepoRoot 'build'
$script:ExePath   = Join-Path $script:WorkDir 'nuked-sc55.exe'
$script:Tracediff = Join-Path $script:RepoRoot 'mk2cpp\tools\tracediff\tracediff.exe'

if (-not (Test-Path -LiteralPath $script:ExePath)) {
    Write-Host "ERROR: GT exe not found: $script:ExePath"
    exit 2
}
if (-not (Test-Path -LiteralPath $script:Tracediff)) {
    Write-Host "ERROR: tracediff not found: $script:Tracediff"
    exit 2
}

if (-not [System.IO.Path]::IsPathRooted($OutDir)) {
    $OutDir = Join-Path $script:RepoRoot $OutDir
}
$OutDir = [System.IO.Path]::GetFullPath($OutDir)
$logDir = Join-Path $OutDir 'logs'
New-Item -ItemType Directory -Force -Path $OutDir, $logDir | Out-Null

# ---- environment (headless deterministic GT) ------------------------------
$env:SDL_VIDEODRIVER  = 'dummy'
$env:SDL_AUDIODRIVER  = 'dummy'
$env:SDL_RENDER_DRIVER = 'software'

# ---- resolve scenario args, window and hash cycle -------------------------
$presetFrom = [uint64]0
$presetTo   = [uint64]0
$presetHash = [uint64]0

switch ($Scenario) {
    'boot' {
        $presetFrom = 0
        $presetTo   = 3000000
        $presetHash = 3000000
        $modeArgs = @('-mk2')
    }
    'demo200' {
        $presetFrom = 200000000
        $presetTo   = 202000000
        $presetHash = 202000000
        $modeArgs = @('-mk2', '-demo')
    }
    'custom' {
        $modeArgs = @('-mk2') + @($ScenarioArgs)
    }
}

if ($Window -eq 'boot3m') {
    $presetFrom = 0
    $presetTo   = 3000000
    if ($presetHash -eq 0) { $presetHash = 3000000 }
}
elseif ($Window -eq '200m') {
    $presetFrom = 200000000
    $presetTo   = 202000000
    if ($presetHash -eq 0) { $presetHash = 202000000 }
}

$script:TraceFrom = $presetFrom
$script:TraceTo   = $presetTo
$script:HashAt    = $presetHash

if ($WindowFrom -gt 0) { $script:TraceFrom = $WindowFrom }
if ($WindowTo   -gt 0) { $script:TraceTo   = $WindowTo }
if ($HashCycle  -gt 0) { $script:HashAt    = $HashCycle }
if ($script:HashAt -eq 0) { $script:HashAt = $script:TraceTo }

if ($script:TraceTo -le $script:TraceFrom) {
    Write-Host "ERROR: invalid window [$($script:TraceFrom),$($script:TraceTo)); for -Scenario custom pass -WindowFrom/-WindowTo."
    exit 2
}
if ($Scenario -ne 'custom' -and $ScenarioArgs.Count -gt 0) {
    Write-Host "WARNING: -ScenarioArgs is only honored for -Scenario custom; ignored."
}

# ---- outputs / stale-file cleanup -----------------------------------------
$defTrace   = Join-Path $OutDir 'def.trace'
$defHash    = Join-Path $OutDir 'def.hash'
$mkTrace    = Join-Path $OutDir 'tr.mk2cpp.trace'
$mkHash     = Join-Path $OutDir 'tr.mk2cpp.hash'
$defStdout  = Join-Path $logDir 'def.stdout.txt'
$defStderr  = Join-Path $logDir 'def.stderr.txt'
$mkStdout   = Join-Path $logDir 'tr.mk2cpp.stdout.txt'
$mkStderr   = Join-Path $logDir 'tr.mk2cpp.stderr.txt'
$diffTraceReport = Join-Path $OutDir 'diff_trace.md'
$diffTraceDiv    = Join-Path $OutDir 'diff_trace_div.txt'
$diffBaseReport  = Join-Path $OutDir 'diff_baseline.md'
$diffBaseDiv     = Join-Path $OutDir 'diff_baseline_div.txt'
$summaryPath     = Join-Path $OutDir 'summary.txt'

$stale = @(
    $defTrace, $defHash, $mkTrace, $mkHash,
    $defStdout, $defStderr, $mkStdout, $mkStderr,
    $diffTraceReport, $diffTraceDiv, $diffBaseReport, $diffBaseDiv,
    $summaryPath
)
foreach ($f in $stale) {
    Remove-Item -LiteralPath $f -Force -ErrorAction SilentlyContinue
}

# ---- helpers ---------------------------------------------------------------
$script:checks = New-Object System.Collections.Generic.List[object]

function Add-Check {
    param([string]$Check, [string]$Result, [string]$Detail)
    [void]$script:checks.Add([pscustomobject]@{
        Check  = $Check
        Result = $Result
        Detail = $Detail
    })
}

function Quote-Arg {
    param([string]$Value)
    if ($null -eq $Value) { return '""' }
    if ($Value -match '\s') { return '"' + $Value + '"' }
    return $Value
}

function Test-NonEmptyFile {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { return $false }
    try { return ((Get-Item -LiteralPath $Path).Length -gt 0) } catch { return $false }
}

function Get-TailLines {
    param([string]$Path, [int]$Count)
    if (-not (Test-Path -LiteralPath $Path)) { return @() }
    try { return @(Get-Content -LiteralPath $Path -Tail $Count -ErrorAction Stop) } catch { return @() }
}

function Get-TraceTailCycle {
    param([string]$Path)
    try {
        $tail = Get-Content -LiteralPath $Path -Tail 1 -ErrorAction Stop
        if ($null -eq $tail) { return $null }
        $line = [string]$tail
        if ([string]::IsNullOrWhiteSpace($line)) { return $null }
        $parts = $line -split '\s+'
        if ($parts.Count -lt 2) { return $null }
        $cycle = [uint64]0
        if ([uint64]::TryParse($parts[1], [ref]$cycle)) { return $cycle }
        return $null
    } catch {
        return $null
    }
}

function Wait-GtOutputs {
    param(
        $Process,
        [string]$TracePath,
        [string]$HashPath,
        [int]$TimeoutSec,
        [uint64]$TraceEnd
    )
    $tailFloor = [uint64]0
    if ($TraceEnd -gt 12) { $tailFloor = $TraceEnd - 12 }

    $deadline  = (Get-Date).AddSeconds($TimeoutSec)
    $lastSize  = [int64]-1
    $stableHit = 0

    while ($true) {
        $hashReady = Test-NonEmptyFile -Path $HashPath
        $traceReady = $false

        if (Test-Path -LiteralPath $TracePath) {
            try { $size = (Get-Item -LiteralPath $TracePath).Length } catch { $size = [int64]-1 }
            if ($size -eq $lastSize) { $stableHit++ } else { $stableHit = 0 }
            $lastSize = $size

            $tailCycle = Get-TraceTailCycle -Path $TracePath
            if ($null -ne $tailCycle -and $tailCycle -ge $tailFloor) {
                $traceReady = $true
            }
            elseif ($size -gt 0 -and $stableHit -ge 4) {
                $traceReady = $true
            }
        }
        else {
            $stableHit = 0
        }

        if ($hashReady -and $traceReady) { return 'ready' }
        if ((Get-Date) -ge $deadline)      { return 'timeout' }

        try { $Process.Refresh() } catch { }
        if ($Process.HasExited) { return 'exited' }

        Start-Sleep -Milliseconds 500
    }
}

function Invoke-GtRun {
    param(
        [string]$Label,
        [string[]]$ModeArgs,
        [string]$TracePath,
        [string]$HashPath,
        [string]$StdoutPath,
        [string]$StderrPath,
        [int]$TimeoutSec
    )

    $argList = New-Object System.Collections.Generic.List[string]
    foreach ($a in $ModeArgs) { [void]$argList.Add($a) }
    [void]$argList.Add('-tracepc')
    [void]$argList.Add($TracePath)
    [void]$argList.Add([string]$script:TraceFrom)
    [void]$argList.Add([string]$script:TraceTo)
    [void]$argList.Add('-hashdump')
    [void]$argList.Add([string]$script:HashAt)
    [void]$argList.Add($HashPath)

    $quoted = @($argList | ForEach-Object { Quote-Arg -Value $_ })

    $proc   = $null
    $status = 'not-started'
    $exitedEarly = $false

    try {
        $proc = Start-Process -FilePath $script:ExePath -WorkingDirectory $script:WorkDir `
            -PassThru -ArgumentList $quoted `
            -RedirectStandardOutput $StdoutPath -RedirectStandardError $StderrPath
        $status = Wait-GtOutputs -Process $proc -TracePath $TracePath -HashPath $HashPath `
            -TimeoutSec $TimeoutSec -TraceEnd $script:TraceTo
        if ($status -eq 'exited') { $exitedEarly = $true }
    }
    finally {
        if ($null -ne $proc) {
            try { $proc.Refresh() } catch { }
            if (-not $proc.HasExited) {
                Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
                try { [void]$proc.WaitForExit(5000) } catch { }
            }
        }
    }

    return [pscustomobject]@{
        Label       = $Label
        Status      = $status
        ExitedEarly = $exitedEarly
        TraceOk     = (Test-NonEmptyFile -Path $TracePath)
        HashOk      = (Test-NonEmptyFile -Path $HashPath)
        TracePath   = $TracePath
        HashPath    = $HashPath
        StdoutPath  = $StdoutPath
        StderrPath  = $StderrPath
    }
}

function Show-RunFailure {
    param($Run)
    Write-Host ("  [{0}] status={1} trace={2} hash={3}" -f $Run.Label, $Run.Status, $Run.TraceOk, $Run.HashOk)
    $so = Get-TailLines -Path $Run.StdoutPath -Count 8
    if ($so.Count -gt 0) {
        Write-Host ("  [{0}] last stdout:" -f $Run.Label)
        foreach ($l in $so) { Write-Host ("    " + $l) }
    }
    $se = Get-TailLines -Path $Run.StderrPath -Count 4
    if ($se.Count -gt 0) {
        Write-Host ("  [{0}] last stderr:" -f $Run.Label)
        foreach ($l in $se) { Write-Host ("    " + $l) }
    }
}

function Invoke-TraceDiff {
    param(
        [string]$Ref,
        [string]$Dut,
        [string]$Report,
        [string]$Divergence
    )
    $output = @(& $script:Tracediff --ref $Ref --dut $Dut `
        --history 64 --report $Report --divergence $Divergence 2>&1)
    $exit = $LASTEXITCODE
    return [pscustomobject]@{
        ExitCode = $exit
        Output   = @($output | ForEach-Object { [string]$_ })
    }
}

# ---- run both modes --------------------------------------------------------
Write-Host "== mk2cpp two-mode check =="
Write-Host ("scenario  : {0}" -f $Scenario)
Write-Host ("window    : [{0},{1})   hash@{2}   timeout={3}s" -f $script:TraceFrom, $script:TraceTo, $script:HashAt, $TimeoutSec)
Write-Host ("outdir    : {0}" -f $OutDir)
Write-Host ""

Write-Host ("[1/2] default interpreter : {0}" -f (($modeArgs + @('-tracepc', $defTrace, [string]$script:TraceFrom, [string]$script:TraceTo, '-hashdump', [string]$script:HashAt, $defHash)) -join ' '))
$runDef = Invoke-GtRun -Label 'def' -ModeArgs $modeArgs -TracePath $defTrace -HashPath $defHash `
    -StdoutPath $defStdout -StderrPath $defStderr -TimeoutSec $TimeoutSec

Write-Host ("[2/2] -mk2cpp translated   : {0}" -f (($modeArgs + @('-mk2cpp', '-tracepc', $mkTrace, [string]$script:TraceFrom, [string]$script:TraceTo, '-hashdump', [string]$script:HashAt, $mkHash)) -join ' '))
$runMk = Invoke-GtRun -Label 'tr.mk2cpp' -ModeArgs ($modeArgs + '-mk2cpp') -TracePath $mkTrace -HashPath $mkHash `
    -StdoutPath $mkStdout -StderrPath $mkStderr -TimeoutSec $TimeoutSec
Write-Host ""

# run-output checks
foreach ($run in @($runDef, $runMk)) {
    if (-not $run.TraceOk -or -not $run.HashOk -or $run.ExitedEarly -or $run.Status -eq 'timeout') {
        Add-Check -Check ("run:" + $run.Label) -Result 'FAIL' `
            -Detail ("status={0} trace={1} hash={2}" -f $run.Status, $run.TraceOk, $run.HashOk)
        Show-RunFailure -Run $run
    }
    else {
        Add-Check -Check ("run:" + $run.Label) -Result 'PASS' `
            -Detail ("status=ready trace={0}B hash={1}B" -f (Get-Item -LiteralPath $run.TracePath).Length, (Get-Item -LiteralPath $run.HashPath).Length)
    }
}

# trace compare: def vs -mk2cpp
if ($runDef.TraceOk -and $runMk.TraceOk) {
    Write-Host "tracediff def.trace vs tr.mk2cpp.trace:"
    $td = Invoke-TraceDiff -Ref $defTrace -Dut $mkTrace -Report $diffTraceReport -Divergence $diffTraceDiv
    foreach ($line in $td.Output) { Write-Host ("  " + $line) }
    if ($td.ExitCode -eq 0) {
        Add-Check -Check 'trace:def-vs-tr.mk2cpp' -Result 'PASS' -Detail 'identical'
    }
    else {
        Add-Check -Check 'trace:def-vs-tr.mk2cpp' -Result 'FAIL' -Detail ("tracediff exit={0}; see {1}" -f $td.ExitCode, $diffTraceDiv)
    }
}
else {
    Add-Check -Check 'trace:def-vs-tr.mk2cpp' -Result 'FAIL' -Detail 'missing trace input'
}

# hash compare: def vs -mk2cpp (file SHA256)
if ((Test-NonEmptyFile -Path $defHash) -and (Test-NonEmptyFile -Path $mkHash)) {
    $hDef = (Get-FileHash -LiteralPath $defHash -Algorithm SHA256).Hash
    $hMk  = (Get-FileHash -LiteralPath $mkHash  -Algorithm SHA256).Hash
    if ($hDef -eq $hMk) {
        Add-Check -Check 'hash:def-vs-tr.mk2cpp' -Result 'PASS' -Detail ("sha256=" + $hDef.Substring(0, 16))
    }
    else {
        Add-Check -Check 'hash:def-vs-tr.mk2cpp' -Result 'FAIL' -Detail ("def={0} mk2cpp={1}" -f $hDef.Substring(0, 16), $hMk.Substring(0, 16))
    }
}
else {
    Add-Check -Check 'hash:def-vs-tr.mk2cpp' -Result 'FAIL' -Detail 'missing hash input'
}

# baseline compare: default trace vs frozen fixture
$baseline = $null
if ($Scenario -eq 'boot' -and $script:TraceFrom -eq 0 -and $script:TraceTo -eq 3000000) {
    $baseline = Join-Path $script:RepoRoot 'tools\baselines\trace_boot3m_base.txt'
}
elseif ($Scenario -eq 'demo200' -and $script:TraceFrom -eq 200000000 -and $script:TraceTo -eq 202000000) {
    $baseline = Join-Path $script:RepoRoot 'tools\baselines\trace_200m_base.txt'
}

if ($null -eq $baseline) {
    Add-Check -Check 'trace:def-vs-baseline' -Result 'SKIP' `
        -Detail ("no baseline for scenario={0} window=[{1},{2})" -f $Scenario, $script:TraceFrom, $script:TraceTo)
}
elseif (-not (Test-Path -LiteralPath $baseline)) {
    Add-Check -Check 'trace:def-vs-baseline' -Result 'SKIP' -Detail ("baseline absent: " + $baseline)
}
elseif (-not $runDef.TraceOk) {
    Add-Check -Check 'trace:def-vs-baseline' -Result 'FAIL' -Detail 'missing default trace'
}
else {
    Write-Host ("tracediff baseline {0} vs def.trace:" -f (Split-Path -Leaf $baseline))
    $bd = Invoke-TraceDiff -Ref $baseline -Dut $defTrace -Report $diffBaseReport -Divergence $diffBaseDiv
    foreach ($line in $bd.Output) { Write-Host ("  " + $line) }
    if ($bd.ExitCode -eq 0) {
        Add-Check -Check 'trace:def-vs-baseline' -Result 'PASS' -Detail (Split-Path -Leaf $baseline)
    }
    else {
        Add-Check -Check 'trace:def-vs-baseline' -Result 'FAIL' -Detail ("tracediff exit={0}; see {1}" -f $bd.ExitCode, $diffBaseDiv)
    }
}

# ---- summary ---------------------------------------------------------------
$table = ($script:checks | Format-Table -AutoSize | Out-String -Width 200).TrimEnd()
Write-Host ""
Write-Host "== summary =="
Write-Host $table
Set-Content -LiteralPath $summaryPath -Value $table

$failCount = @($script:checks | Where-Object { $_.Result -eq 'FAIL' }).Count
Write-Host ""
if ($failCount -gt 0) {
    Write-Host ("RESULT: FAIL ({0} check(s); outputs in {1})" -f $failCount, $OutDir)
    if ($KeepGoing) {
        Write-Host "-KeepGoing set: exiting 0 despite failures."
        exit 0
    }
    exit 1
}

Write-Host ("RESULT: PASS (outputs in {0})" -f $OutDir)
exit 0
