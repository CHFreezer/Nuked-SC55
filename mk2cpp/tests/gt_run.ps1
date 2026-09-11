# Runs the GT emulator under a hard timeout.
#
# GT never exits on its own. This wrapper starts it, waits until -WaitFile
# appears (hashdump / WAV .meta / trace file) or the timeout expires, then
# force-kills the process. Exit 0 = WaitFile appeared, 1 = timeout/exit.
#
# Usage (note: -Args is ONE space-separated string; `powershell -File` cannot
# bind a PowerShell array across tokens):
#   powershell -NoProfile -ExecutionPolicy Bypass -File mk2cpp\tests\gt_run.ps1 `
#     -Exe build\nuked-sc55.exe `
#     -Args "-mk2 -mk2cpp -hashdump 3000000 out.hash" `
#     -WaitFile out.hash -TimeoutSec 60
param(
    [Parameter(Mandatory = $true)][string]$Exe,
    [Parameter(Mandatory = $true)][string]$Args,
    [Parameter(Mandatory = $true)][string]$WaitFile,
    [int]$TimeoutSec = 120,
    [string]$StdOut = '',
    [string]$StdErr = ''
)
$ErrorActionPreference = 'Stop'

$argList = @($Args -split '\s+' | Where-Object { $_ -ne '' })
if ($argList.Count -eq 0) { Write-Error '-Args is empty'; exit 2 }

$waitFull = [System.IO.Path]::GetFullPath($WaitFile)
if (Test-Path -LiteralPath $waitFull) { Remove-Item -LiteralPath $waitFull -Force }

$startArgs = @{ FilePath = $Exe; ArgumentList = $argList; PassThru = $true }
if ($StdOut) { $startArgs.RedirectStandardOutput = [System.IO.Path]::GetFullPath($StdOut) }
if ($StdErr) { $startArgs.RedirectStandardError = [System.IO.Path]::GetFullPath($StdErr) }

$proc = Start-Process @startArgs
$ok = $false
try {
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        if (Test-Path -LiteralPath $waitFull) { $ok = $true; break }
        if ($proc.HasExited) { break }
        Start-Sleep -Milliseconds 200
    }
} finally {
    if (-not $proc.HasExited) {
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
}

if ($ok) { exit 0 } else { exit 1 }
