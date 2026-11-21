<#
  文件名 : pack_submission.ps1
  用途   : 按《初赛提交要求（2026）》把仓库整理成可提交的文件夹并压缩

  为什么需要它：
    提交要求是一套目录结构（设计报告 / 工程文件 / 仿真实测结果 / 其他附件），
    而仓库里的东西是另一种结构。手工挑文件最容易漏的恰恰是：
      - results/ 下的实测 CSV（它们被 .gitignore 忽略，git status 里根本看不见）
      - 报告 PDF（要点名，不能交 .md）
      - 波形/报告截图（分散在 docs/figs 与 docs/reports）
    这个脚本按清单复制，并打印一份"里有什么/缺什么"，漏了自己会报出来。

  用法：
    powershell -ExecutionPolicy Bypass -File scripts\pack_submission.ps1 -Team "热管理小队"
    powershell -ExecutionPolicy Bypass -File scripts\pack_submission.ps1 -Team "xxx" -IncludeBit

  产物：
    out\数字FPGA类-1-<队名>\        提交文件夹
    out\数字FPGA类-1-<队名>.zip     压缩包（要求 ≤1 GB）
#>

param(
    [Parameter(Mandatory = $true)][string]$Team,
    [string]$Root = "",
    [switch]$IncludeBit
)

$ErrorActionPreference = "Stop"
if ([string]::IsNullOrEmpty($Root)) { $Root = Split-Path -Parent $PSScriptRoot }
if ([string]::IsNullOrEmpty($Root)) { $Root = (Get-Location).Path }

$name = "数字FPGA类-1-$Team"
$stage = Join-Path $Root "out\$name"
$zip = Join-Path $Root "out\$name.zip"

if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
New-Item -ItemType Directory -Force $stage | Out-Null
New-Item -ItemType Directory -Force (Join-Path $stage "01_设计报告") | Out-Null
New-Item -ItemType Directory -Force (Join-Path $stage "02_工程文件") | Out-Null
New-Item -ItemType Directory -Force (Join-Path $stage "03_仿真与实测") | Out-Null
New-Item -ItemType Directory -Force (Join-Path $stage "04_其他附件") | Out-Null

$copied = 0
$missing = @()

function Copy-ItemSafe($src, $dstDir, $label) {
    if (Test-Path $src) {
        Copy-Item $src $dstDir -Recurse -Force
        $script:copied++
        Write-Host ("  [OK]   " + $label)
    } else {
        $script:missing += $label
        Write-Host ("  [MISS] " + $label) -ForegroundColor Yellow
    }
}

Write-Host "=== 1/4 设计报告（核心文档）==="
Copy-ItemSafe (Join-Path $Root "docs\设计报告.pdf") (Join-Path $stage "01_设计报告") "设计报告.pdf"
Copy-ItemSafe (Join-Path $Root "docs\07_设计报告_初稿.md") (Join-Path $stage "01_设计报告") "设计报告源文件(.md)"

Write-Host "=== 2/4 工程文件 ==="
foreach ($d in @("rtl", "constr", "sim", "scripts", "sw")) {
    Copy-ItemSafe (Join-Path $Root $d) (Join-Path $stage "02_工程文件") $d
}
Copy-ItemSafe (Join-Path $Root "README.md") (Join-Path $stage "02_工程文件") "README.md"

Write-Host "=== 3/4 仿真与实测结果 ==="
Copy-ItemSafe (Join-Path $Root "docs\figs") (Join-Path $stage "03_仿真与实测") "docs/figs（证据图与 CSV）"
Copy-ItemSafe (Join-Path $Root "docs\reports") (Join-Path $stage "03_仿真与实测") "docs/reports（时序/资源/功耗报告）"
Copy-ItemSafe (Join-Path $Root "results") (Join-Path $stage "03_仿真与实测") "results（板上实测 CSV）"
Copy-ItemSafe (Join-Path $Root ".work\regress_report.txt") (Join-Path $stage "03_仿真与实测") "回归结果汇总"
foreach ($m in @("docs\05_证据表.md", "docs\06_图表清单.md", "docs\08_上板实测记录.md", "docs\03_测试记录.md")) {
    Copy-ItemSafe (Join-Path $Root $m) (Join-Path $stage "03_仿真与实测") (Split-Path $m -Leaf)
}

Write-Host "=== 4/4 其他附件 ==="
foreach ($m in @("docs\01_架构与仿真方案.md", "docs\09_外设清单与接线说明.md", "docs\10_交接说明.md")) {
    Copy-ItemSafe (Join-Path $Root $m) (Join-Path $stage "04_其他附件") (Split-Path $m -Leaf)
}
if ($IncludeBit) {
    $bit = Get-ChildItem $Root -Recurse -Filter "*.bit" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($bit) { Copy-ItemSafe $bit.FullName (Join-Path $stage "02_工程文件") ("比特流 " + $bit.Name) } else { $missing += "比特流(.bit)"; Write-Host "  [MISS] 比特流（未找到 .bit）" -ForegroundColor Yellow }
}

# 清掉中间产物，避免把 work/ .vvp 之类塞进提交包
Get-ChildItem $stage -Recurse -Include "*.vvp", "*.log", "*.jou", "*.str", "*.pb" -ErrorAction SilentlyContinue |
    Remove-Item -Force -ErrorAction SilentlyContinue

# 清单
$manifest = Join-Path $stage "清单.txt"
$tree = Get-ChildItem $stage -Recurse -File | Sort-Object FullName
"提交清单  $name" | Out-File $manifest -Encoding UTF8
"生成时间  $(Get-Date -Format 'yyyy-MM-dd HH:mm')" | Out-File $manifest -Append -Encoding UTF8
"文件数    $($tree.Count)" | Out-File $manifest -Append -Encoding UTF8
"总大小    $([math]::Round(($tree | Measure-Object Length -Sum).Sum / 1MB, 1)) MB" | Out-File $manifest -Append -Encoding UTF8
"" | Out-File $manifest -Append -Encoding UTF8
$tree | ForEach-Object {
    "{0,10:N0}  {1}" -f $_.Length, $_.FullName.Substring($stage.Length + 1)
} | Out-File $manifest -Append -Encoding UTF8

if (Test-Path $zip) { Remove-Item $zip -Force }
Compress-Archive -Path $stage -DestinationPath $zip -CompressionLevel Optimal
$zipMB = [math]::Round((Get-Item $zip).Length / 1MB, 1)

Write-Host ""
Write-Host "=================================================================="
Write-Host ("  文件夹 : " + $stage)
Write-Host ("  压缩包 : " + $zip + "  (" + $zipMB + " MB)")
Write-Host ("  文件数 : " + $tree.Count + "，复制成功 " + $copied + " 项")
if ($missing.Count -gt 0) {
    Write-Host ("  缺失   : " + ($missing -join "、")) -ForegroundColor Yellow
} else {
    Write-Host "  缺失   : 无" -ForegroundColor Green
}
if ($zipMB -gt 1000) { Write-Host "  ⚠ 超过 1 GB，要求是 ≤1 GB" -ForegroundColor Yellow }
Write-Host "=================================================================="
Write-Host "提醒：演示视频(.mp4)与实物照片需要手工放进 04_其他附件/"
