<#
  文件名 : gen_filelist.ps1
  用途   : 由当前目录树重新生成 sim/filelist.f

  为什么需要：
    filelist.f 是手工维护的清单，已经两次落后于目录（漏 rtl/vplant、又漏 tb_ext_loop、
    还留着已删除的 fan_tach.v）。xsim 走 sim.bat 用这份清单编译，
    清单错了 → xsim 编的根本不是 regress.bat 那套设计，却不会报错得明显。

  用法：
    powershell -ExecutionPolicy Bypass -File scripts\gen_filelist.ps1
#>
param([string]$Root = "")

$ErrorActionPreference = "Stop"
if ([string]::IsNullOrEmpty($Root)) { $Root = Split-Path -Parent $PSScriptRoot }
if ([string]::IsNullOrEmpty($Root)) { $Root = (Get-Location).Path }

$fl = Join-Path $Root "sim\filelist.f"
$lines = New-Object System.Collections.Generic.List[string]

# 路径相对 sim/filelist.f 的位置：../rtl/... 与 ../sim/...
$dirs = @("common", "clk", "acquire", "dsp", "ctrl", "load", "telemetry", "ui", "vplant", "top")
foreach ($d in $dirs) {
    Get-ChildItem (Join-Path $Root "rtl\$d") -Filter "*.v" -ErrorAction SilentlyContinue |
        Sort-Object Name | ForEach-Object { $lines.Add("../rtl/$d/" + $_.Name) }
    Get-ChildItem (Join-Path $Root "rtl\$d") -Filter "*.vh" -ErrorAction SilentlyContinue |
        Sort-Object Name | ForEach-Object { $lines.Add("../rtl/$d/" + $_.Name) }
}
Get-ChildItem (Join-Path $Root "sim\models") -Filter "*.v" | Sort-Object Name |
    ForEach-Object { $lines.Add("../sim/models/" + $_.Name) }
Get-ChildItem (Join-Path $Root "sim\tb") -Filter "*.sv" | Sort-Object Name |
    ForEach-Object { $lines.Add("../sim/tb/" + $_.Name) }

$utf8 = New-Object System.Text.UTF8Encoding($false)
[IO.File]::WriteAllLines($fl, $lines, $utf8)

Write-Host ("生成 " + $fl + "  共 " + $lines.Count + " 行")
Write-Host ("  其中 rtl: " + ($lines | Where-Object { $_ -like "../rtl/*" }).Count +
            "  models: " + ($lines | Where-Object { $_ -like "../sim/models/*" }).Count +
            "  tb: " + ($lines | Where-Object { $_ -like "../sim/tb/*" }).Count)
