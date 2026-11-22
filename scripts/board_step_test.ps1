<#
board_step_test.ps1 —— 在板上做一次设定值阶跃，采全过程 → CSV + 指标

用法：
    pwsh -File scripts\board_step_test.ps1 -Port COM4 -StepTo 60.0 -Seconds 90

做法：先稳定 -PreSeconds 秒，然后通过命令通道发 "S=60.0"，继续采 -Seconds 秒。
输出 CSV 列与 host_logger.py 一致，可直接喂 sw\analyze_thermal.py 出图。
#>
param(
    [string]$Port = "COM4",
    [int]$Baud = 115200,
    [double]$StepTo = 60.0,
    [int]$PreSeconds = 20,
    [int]$PostSeconds = 70,
    [ValidateSet("tj", "text")]
    [string]$Target = "tj",       # tj=改结温设定值(S=)，text=改传感器设定值(SE=)
    [string]$Csv = "results\board_step.csv"
)

$ErrorActionPreference = "Stop"
function Get-Crc16([byte[]]$d, [int]$len) {
    $crc = 0xFFFF
    for ($i = 0; $i -lt $len; $i++) {
        $crc = $crc -bxor ([int]$d[$i] -shl 8)
        for ($b = 0; $b -lt 8; $b++) {
            if ($crc -band 0x8000) { $crc = (($crc -shl 1) -bxor 0x1021) -band 0xFFFF }
            else { $crc = ($crc -shl 1) -band 0xFFFF }
        }
    }
    return $crc
}
function Q8([int]$lo, [int]$hi) {
    $v = $lo -bor ($hi -shl 8); if ($v -ge 32768) { $v -= 65536 }; return $v / 256.0
}

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 500
$sp.Open()
Write-Host "已打开 $Port，先稳定 $PreSeconds 秒..."

$buf = New-Object System.Collections.Generic.List[byte]
$rows = New-Object System.Collections.Generic.List[string]
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$sent = $false
$stepT = $null
$frames = @()

function Pump([int]$ms) {
    $t0 = [System.Diagnostics.Stopwatch]::StartNew()
    while ($t0.Elapsed.TotalMilliseconds -lt $ms) {
        $chunk = New-Object byte[] 512
        $n = 0
        try { $n = $sp.Read($chunk, 0, 512) } catch { $n = 0 }
        if ($n -le 0) { continue }
        for ($i = 0; $i -lt $n; $i++) { $global:buf.Add($chunk[$i]) }
        while ($true) {
            $sync = -1
            for ($i = 0; $i -lt $global:buf.Count - 1; $i++) {
                if ($global:buf[$i] -eq 0xA5 -and $global:buf[$i + 1] -eq 0x5A) { $sync = $i; break }
            }
            if ($sync -lt 0) { break }
            if ($sync -gt 0) { $global:buf.RemoveRange(0, $sync) }
            if ($global:buf.Count -lt 32) { break }
            $f = $global:buf.GetRange(0, 32).ToArray()
            if (([int]$f[30] -bor ([int]$f[31] -shl 8)) -ne (Get-Crc16 ($f[2..29]) 28)) {
                $global:buf.RemoveRange(0, 1); continue
            }
            $global:buf.RemoveRange(0, 32)
            $rec = [pscustomobject]@{
                T   = $sw.Elapsed.TotalSeconds
                tj  = Q8 $f[10] $f[11]
                tset= Q8 $f[16] $f[17]
                text= Q8 $f[12] $f[13]
                rh  = (([int]$f[14] -bor ([int]$f[15] -shl 8))) / 10.0
                duty= [int]$f[18] -bor ([int]$f[19] -shl 3)
                # 新版遥测帧：b20 高 5 位是加热占空比低 5 位，b21 高 6 位是高 6 位
                # 所以 gear/state 必须掩码，否则会读到 234 这种值（旧解析器的 bug）
                gear= ([int]$f[20]) -band 0x07
                st  = ([int]$f[21]) -band 0x03
                hd  = (([int]$f[20] -shr 3) -bor (([int]$f[21] -shr 2) -shl 5))
                tse = Q8 $f[28] $f[29]
                f   = (([int]$f[22] -bor ([int]$f[23] -shl 8))) / 10.0
                p   = [int]$f[24] -bor ([int]$f[25] -shl 8)
                seq = [int]$f[4] -bor ([int]$f[5] -shl 8)
            }
            $global:frames += $rec
            $rows.Add(("{0:F2},{1:F2},{2:F2},{3:F1},{4:F2},{5},{6},{7},{8:F1},{9},{10},{11:F2},{12}" -f `
                $rec.T, $rec.tj, $rec.text, $rec.rh, $rec.tset, $rec.duty, $rec.gear, $rec.st,
                $rec.f, $rec.p, $rec.seq, $rec.tse, $rec.hd))
        }
    }
}

Pump ($PreSeconds * 1000)
# 发命令
if ($Target -eq "text") { $cmd = "SE={0:F1}`r" -f $StepTo }
else                     { $cmd = "S={0:F1}`r"  -f $StepTo }
$sp.Write($cmd)
$tgtName = if ($Target -eq "text") { "传感器设定值" } else { "结温设定值" }
Write-Host ("已发送命令 {0}（{1}），继续采 {2} 秒..." -f ($cmd.Trim()), $tgtName, $PostSeconds)
$stepT = $sw.Elapsed.TotalSeconds
Pump ($PostSeconds * 1000)
$sp.Close()

$dir = Split-Path -Parent $Csv
if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
"t_s,tj_c,text_c,rh_pct,tset_c,duty,gear,state,f_mhz,p_mw,seq,tset_ext_c,duty_heat" | Set-Content $Csv -Encoding utf8
$rows | Add-Content $Csv -Encoding utf8

# ---- 指标 ----
$pre = $frames | Where-Object { $_.T -lt $stepT }
$post = $frames | Where-Object { $_.T -ge $stepT }
# 指标在"所选通道"上算：tj=结温列，text=环境传感器列
$colName = if ($Target -eq "text") { "text" } else { "tj" }
$preV  = $pre  | ForEach-Object { $_.$colName }
$postV = $post | ForEach-Object { $_.$colName }
$t0   = ($preV  | Measure-Object -Average).Average
$tmax = ($postV | Measure-Object -Maximum).Maximum
$tend = ($postV | Select-Object -Last 20 | Measure-Object -Average).Average
$ov = if ($StepTo -gt $t0) { [Math]::Max(0.0, ($tmax - $StepTo) / ($StepTo - $t0) * 100) } else { 0 }
$band = 0.5
$settle = $null
for ($i = 0; $i -lt $post.Count; $i++) {
    $rem = $post[$i..($post.Count - 1)]
    if ((($rem | ForEach-Object { [Math]::Abs($_.$colName - $StepTo) } | Measure-Object -Maximum).Maximum) -lt $band) {
        $settle = $rem[0].T - $stepT; break
    }
}

Write-Host ""
Write-Host "==== 阶跃测试结果 ===="
$chanName = if ($Target -eq "text") { "环境传感器 AM2302" } else { "结温 XADC" }
Write-Host ("测试通道    : {0}   命令 {1}" -f $chanName, $cmd.Trim())
Write-Host ("设定值      : {0:F1} ℃（从 {1:F1} ℃ 起跳）" -f $StepTo, $t0)
Write-Host ("峰值        : {0:F2} ℃   超调 {1:F1} %" -f $tmax, $ov)
Write-Host ("末段稳态    : {0:F2} ℃   稳态误差 {1:F2} ℃" -f $tend, ($tend - $StepTo))
if ($null -ne $settle) { Write-Host ("调节时间    : {0:F1} s（±{1} ℃ 带内）" -f $settle, $band) }
else { Write-Host "调节时间    : 未在采集窗口内进入误差带" }
Write-Host ("帧数        : {0}   本次发送命令后 {1} 秒" -f $frames.Count, $PostSeconds)
Write-Host ("档位范围   : {0} ~ {1}（应为 0~4；若出现 234 说明解帧未掩码）" -f `
    ($frames | Measure-Object -Property gear -Minimum).Minimum, `
    ($frames | Measure-Object -Property gear -Maximum).Maximum)
Write-Host "CSV -> $Csv"
