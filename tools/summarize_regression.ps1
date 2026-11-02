# =============================================================================
# summarize_regression.ps1 -- 从 ModelSim 逐 tb 日志汇总回归结果
# -----------------------------------------------------------------------------
# 用法：
#     & "tools\summarize_regression.ps1"                  # 默认扫 ..\sim
#     & "tools\summarize_regression.ps1" -SimDir <目录> -OutFile <输出>
#
# 输入：目录里每个 tb 一份日志。默认按 <SimDir>\f_<tb>.txt 找（最终回归的
#       命名）；找不到就退回 <SimDir>\r_<tb>.txt（早期命名）。
#
# -----------------------------------------------------------------------------
# ★★ 判定规则（写死在这里，因为【我在这上面错过两次】）★★
#
#   各 tb 实际打印的结束行只有这几种：
#       TEST_RESULT: PASS  (all checks passed)
#       TEST_RESULT: tb_xxx PASS   ...
#       TEST_RESULT: tb_xxx FAIL  失败项数=1
#       PASS: 全部用例通过
#       TEST_RESULT: FAIL ...              （分析型 tb）
#
#   所以：
#     1) 必须【先查 FAIL，再查 PASS】。反过来会把 FAIL 误判成 PASS。
#     2) 不要写 "TEST_RESULT:\s*\S+\s+PASS" 这种"要求第二个 PASS 记号"的
#        正则 —— "TEST_RESULT: PASS" 里冒号后面【直接】就是 PASS，
#        这种写法会让所有这种格式的 tb 全部变成 NORESULT。
#        （2026-10-15 就是这么把 5 个 PASS 判成 NORESULT 的。）
#     3) NORESULT 不代表失败，代表【这个 tb 没有机读结论】。
#        见到 NORESULT 应该去补 tb 的 TEST_RESULT 行，而不是当它通过。
# =============================================================================
param(
    [string]$SimDir = (Join-Path $PSScriptRoot "..\sim"),
    [string]$OutFile = ""
)

$failPat = 'TEST_RESULT:.*FAIL|失败项数\s*=\s*[1-9]|=>\s*FAIL'
$passPat = 'TEST_RESULT:.*PASS|PASS:\s'

$rows = @()
foreach ($tb in @(Get-ChildItem (Join-Path $SimDir "tb_*.v") -ErrorAction SilentlyContinue |
                  ForEach-Object { $_.BaseName } | Sort-Object)) {
    $log = Join-Path $SimDir "f_$tb.txt"
    if (-not (Test-Path $log)) { $log = Join-Path $SimDir "r_$tb.txt" }
    $txt = Get-Content $log -Raw -ErrorAction SilentlyContinue
    $v = "NORESULT"
    if ($txt) {
        if     ($txt -match $failPat) { $v = "FAIL" }
        elseif ($txt -match $passPat) { $v = "PASS" }
    }
    $nFail = 0
    if ($txt -and ($txt -match '失败项数\s*=\s*(\d+)')) { $nFail = [int]$Matches[1] }
    $rows += [pscustomobject]@{ TB = $tb; Result = $v; FailItems = $nFail }
}

$rows | ForEach-Object { "{0,-24} {1}" -f $_.TB, $_.Result }
$p = ($rows | Where-Object Result -eq "PASS").Count
$f = ($rows | Where-Object Result -eq "FAIL").Count
$n = ($rows | Where-Object Result -eq "NORESULT").Count
"---"
"PASS=$p  FAIL=$f  NORESULT=$n  TOTAL=$($rows.Count)"
if ($OutFile) { $rows | ForEach-Object { "{0,-24} {1}" -f $_.TB, $_.Result } |
                Set-Content $OutFile -Encoding UTF8 }
