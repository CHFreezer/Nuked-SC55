# Build all tools in tools/ using clang.
# Usage: powershell -ExecutionPolicy Bypass -File build.ps1
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$clang = "C:\Program Files\LLVM\bin\clang.exe"
if (-not (Test-Path $clang)) { $clang = "clang" }

$targets = @(
    @{ Out = "vm\h8vm.exe";         Src = @("vm\h8vm_main.c", "vm\h8vm_body.c", "vm\h8vm_sm.c", "vm\h8vm_pcm.c") },
    @{ Out = "disasm\h8dasm.exe";  Src = @("disasm\h8dasm.c") },
    @{ Out = "diff\extract_pc.exe"; Src = @("diff\extract_pc.c") },
    @{ Out = "diff\extract_flow.exe"; Src = @("diff\extract_flow.c") },
    @{ Out = "diff\pcdiff.exe";     Src = @("diff\pcdiff.c") },
    @{ Out = "diff\readdiff.exe";   Src = @("diff\readdiff.c") },
    @{ Out = "diff\ramdiff.exe";    Src = @("diff\ramdiff.c") },
    @{ Out = "diff\seq.exe";        Src = @("diff\seq.c") },
    @{ Out = "diff\seqdiff.exe";    Src = @("diff\seqdiff.c") },
    @{ Out = "diff\collapse.exe";   Src = @("diff\collapse.c") },
    @{ Out = "diff\segdump.exe";    Src = @("diff\segdump.c") },
    @{ Out = "diff\segdiff.exe";    Src = @("diff\segdiff.c") },
    @{ Out = "misc\dumpbanner.exe"; Src = @("misc\dumpbanner.c") },
    @{ Out = "misc\findstr.exe";    Src = @("misc\findstr.c") },
    @{ Out = "misc\missctx.exe";    Src = @("misc\missctx.c") },
    @{ Out = "misc\missentry.exe";  Src = @("misc\missentry.c") },
    @{ Out = "misc\misspc.exe";     Src = @("misc\misspc.c") },
    @{ Out = "misc\rtstarget.exe";  Src = @("misc\rtstarget.c") }
)

$failed = 0
foreach ($t in $targets) {
    & $clang -O2 -Wno-deprecated-declarations -o (Join-Path $root $t.Out) @($t.Src | ForEach-Object { Join-Path $root $_ })
    if ($LASTEXITCODE -ne 0) { Write-Warning "FAILED: $($t.Out)"; $failed = 1 }
    else { Write-Host "built $($t.Out)" }
}
exit $failed
