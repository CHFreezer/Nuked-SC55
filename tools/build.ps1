# Build all tools in tools/ using clang.
# Usage: powershell -ExecutionPolicy Bypass -File build.ps1
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$clang = "C:\Program Files\LLVM\bin\clang.exe"
if (-not (Test-Path $clang)) { $clang = "clang" }

$targets = @(
    @{ Out = "vm\h8vm.exe";                  Src = @("vm\h8vm_main.c", "vm\h8vm_body.c", "vm\h8vm_sm.c", "vm\h8vm_pcm.c") },
    @{ Out = "disasm\h8dasm.exe";           Src = @("disasm\h8dasm.c") },
    @{ Out = "disasm\smdasm.exe";           Src = @("disasm\smdasm.c") },
    @{ Out = "disasm\logproc.exe";          Src = @("disasm\logproc.c") },
    @{ Out = "verify\verify_dasm.exe";      Src = @("verify\verify_dasm.c") },
    @{ Out = "probe\memprobe_gen.exe";      Src = @("probe\memprobe_gen.c") },
    @{ Out = "probe\memprobe_map.exe";      Src = @("probe\memprobe_map.c") },
    @{ Out = "probe\memprobe_read.exe";     Src = @("probe\memprobe_read.c") },
    @{ Out = "probe\smvec_check.exe";       Src = @("probe\smvec_check.c") }
)

$failed = 0
foreach ($t in $targets) {
    & $clang -O2 -Wno-deprecated-declarations -o (Join-Path $root $t.Out) @($t.Src | ForEach-Object { Join-Path $root $_ })
    if ($LASTEXITCODE -ne 0) { Write-Warning "FAILED: $($t.Out)"; $failed = 1 }
    else { Write-Host "built $($t.Out)" }
}
exit $failed
