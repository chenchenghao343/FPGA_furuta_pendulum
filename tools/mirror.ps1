# =============================================================================
# mirror.ps1 -- copy J280 v6.0 project to a pure-ASCII build path
# -----------------------------------------------------------------------------
# ASCII-only on purpose (Windows PowerShell reads .ps1 with the ANSI code page).
#
# Why: Gowin gw_sh and ModelSim both fail on paths containing non-ASCII
# characters. The project lives under C:\Users\<chinese>\Desktop\..., so every
# build/sim run must first be mirrored to C:\J280_work\Balance_Ctrl.
#
# Usage:  powershell -File tools\mirror.ps1
# =============================================================================
param(
    [string]$Src = "C:\Users\1\Desktop\J280_v6.0_work",
    [string]$Dst = "C:\J280_work\Balance_Ctrl"
)

$ErrorActionPreference = "Stop"

Write-Output "===== mirror project to pure-ASCII path ====="
New-Item -ItemType Directory -Force -Path $Dst | Out-Null

# wipe previous mirror so a deleted source file cannot linger
Get-ChildItem -Path $Dst -Force | Remove-Item -Recurse -Force -ErrorAction SilentlyContinue

& robocopy $Src $Dst /E /XD ".git" "impl\pnr" "impl\gwsynthesis" "sim\work" /NFL /NDL /NJH /NJS /NP | Out-Null
if ($LASTEXITCODE -ge 8) { throw ("robocopy failed, exit=" + $LASTEXITCODE) }

Write-Output ("  src  -> " + (Join-Path $Dst "src"))
Write-Output ("  sim  -> " + (Join-Path $Dst "sim"))
Write-Output ("  rtl files = " + (Get-ChildItem (Join-Path $Dst "src\*.v")).Count)
Write-Output ("  tb  files = " + (Get-ChildItem (Join-Path $Dst "sim\tb_*.v")).Count)
