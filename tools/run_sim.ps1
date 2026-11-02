# =============================================================================
# run_sim.ps1 -- J280 Balance_Ctrl simulation runner (ASCII only on purpose)
# -----------------------------------------------------------------------------
# NOTE: this file is intentionally ASCII-only. Windows PowerShell reads .ps1
# files using the ANSI code page, so a UTF-8 file with Chinese comments gets
# mis-decoded and the parser fails with "missing closing brace".
# All Chinese documentation lives in README.md and the Word deliverable.
# -----------------------------------------------------------------------------
# Why mirror to a pure-ASCII path:
#   ModelSim vlog/vsim fail on paths containing non-ASCII characters
#   (observed: sqlite3_open C:/Users/<mojibake>/... unable to open database).
#   Icarus Verilog has the same limitation. So we copy src/ and sim/ to
#   C:\J280_work\Balance_Ctrl\ before invoking the tools.
#
# Why `vsim -batch` instead of `vsim -c`:
#   On this machine ModelSim 2020.4 with `-c` always reports
#       Error: can't read "FileWatch(fileName)": no such element in array
#   and the design never loads, so nothing is simulated. `-batch` does NOT
#   trigger that defect and runs the testbench normally. Measured with a
#   trivial counter tb: -c fails, -batch prints the tb output and finishes.
#
# Usage:
#   powershell -File tools\run_sim.ps1
#   powershell -File tools\run_sim.ps1 -Only tb_pid_pipeline
# =============================================================================
param(
    [string]$SrcRoot = "C:\Users\1\Desktop\Balance_Ctrl",
    [string]$Work    = "C:\J280_work\Balance_Ctrl",
    [string]$Only    = ""
)

$ErrorActionPreference = "Continue"
$env:MODEL_TECH = "D:\MODELISM\win64"
$env:PATH       = "D:\MODELISM\win64;" + $env:PATH

$VLOG = "D:\MODELISM\win64\vlog.exe"
$VLIB = "D:\MODELISM\win64\vlib.exe"
$VSIM = "D:\MODELISM\win64\vsim.exe"

$srcDst = Join-Path $Work "src"
$simDst = Join-Path $Work "sim"

Write-Output "===== 1) mirror project to pure-ASCII path ====="
# Clean the mirror first: Copy-Item does not delete files that no longer exist
# in the source, and a stale copy in the mirror silently shadows the real one
# (seen: an old src\tb_oled.v kept getting compiled next to sim\tb_oled.v,
#  producing "Existing module 'tb_oled' ... will be overwritten").
# Only *.v files are removed - never the whole directory (the work library lives
# in this tree and a wrong path here would be destructive).
New-Item -ItemType Directory -Force -Path $srcDst, $simDst | Out-Null
Get-ChildItem $srcDst, $simDst -Filter *.v -File -ErrorAction SilentlyContinue | Remove-Item -Force
Copy-Item (Join-Path $SrcRoot "src\*") $srcDst -Recurse -Force
Copy-Item (Join-Path $SrcRoot "sim\*.v") $simDst -Force
Write-Output ("  src -> " + $srcDst)
Write-Output ("  sim -> " + $simDst)

Set-Location $simDst

Write-Output ""
Write-Output "===== 2) rebuild work library and compile all RTL + tb ====="
Remove-Item (Join-Path $simDst "work") -Recurse -Force -ErrorAction SilentlyContinue
& $VLIB work | Out-Null

$srcFiles = @(Get-ChildItem (Join-Path $srcDst "*.v") | ForEach-Object { $_.FullName })
$tbFiles  = @(Get-ChildItem (Join-Path $simDst "tb_*.v") | ForEach-Object { $_.FullName })
# sim_*.v = simulation-only models (e.g. Gowin primitives such as BUFG that
# ModelSim has no library for). They must NEVER be added to the synthesis file
# list, otherwise they collide with the real Gowin primitives.
$primFiles = @(Get-ChildItem (Join-Path $simDst "sim_*.v") -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName })

Write-Output ("  RTL files = " + $srcFiles.Count + " , tb files = " + $tbFiles.Count + " , sim models = " + $primFiles.Count)

$vlogLog = Join-Path $simDst "compile.log"
& $VLOG -work work @primFiles @srcFiles @tbFiles 2>&1 | Tee-Object -FilePath $vlogLog
$vlogExit = $LASTEXITCODE

Write-Output ""
Write-Output "===== 3) run each testbench with vsim -batch ====="

Set-Content -Path (Join-Path $simDst "run.do") -Encoding ascii -Value @("run -all", "quit -f")

$tbList = @(Get-ChildItem (Join-Path $simDst "tb_*.v") | ForEach-Object { $_.BaseName } | Sort-Object)
if ($Only -ne "") { $tbList = @($Only) }

$results = @()
foreach ($tb in $tbList) {
    $logFile = Join-Path $simDst ("result_" + $tb + ".txt")
    Write-Output ""
    Write-Output ("---------- " + $tb + " ----------")
    $p = Start-Process -FilePath $VSIM `
         -ArgumentList @("-batch", ("work." + $tb), "-do", "run.do") `
         -WorkingDirectory $simDst -NoNewWindow -PassThru `
         -RedirectStandardOutput $logFile -RedirectStandardError ($logFile + ".err")
    if (-not $p.WaitForExit(600000)) {
        try { $p.Kill() } catch {}
        Write-Output "  [TIMEOUT 600s -- killed]"
    }
    $out = @()
    if (Test-Path $logFile) { $out = Get-Content $logFile }
    $out | Where-Object { $_ -match 'TEST_RESULT' -or $_ -match 'FAIL' -or $_ -match 'INFO' -or $_ -match 'Error' } |
        Select-Object -First 60 | ForEach-Object { Write-Output $_ }

    $res = ($out | Select-String -Pattern 'TEST_RESULT:' | Select-Object -Last 1)
    if ($res) {
        $line = $res.Line.Trim()
        Write-Output ("  => " + $line)
        $results += , @($tb, $line)
    } else {
        Write-Output "  => NO TEST_RESULT LINE (compile error or sim did not finish)"
        $results += , @($tb, "NO_RESULT")
    }
}

Write-Output ""
Write-Output "================================================================"
Write-Output " SIMULATION SUMMARY"
Write-Output "================================================================"
$pass = 0; $fail = 0
foreach ($r in $results) {
    Write-Output ("  {0,-22} {1}" -f $r[0], $r[1])
    if ($r[1] -match 'PASS') { $pass++ } elseif ($r[1] -match 'FAIL') { $fail++ }
}
Write-Output ("  PASS = " + $pass + " , FAIL = " + $fail)
Write-Output ("  vlog exit code = " + $vlogExit)
Write-Output ("  result logs in = " + $simDst)
