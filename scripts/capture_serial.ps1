<#
capture_serial.ps1 —— 读 EGO1 的 32 字节遥测帧 → CSV（不依赖 pyserial）

为什么用 PowerShell 的 SerialPort 类：这台机器上没装 pyserial，
而装一个包要联网/装东西，没必要。System.IO.Ports 是 .NET 自带的。

用法：
    powershell -ExecutionPolicy Bypass -File scripts\capture_serial.ps1 -Port COM4 -Seconds 60
    powershell -ExecutionPolicy Bypass -File scripts\capture_serial.ps1 -Port COM4 -Seconds 1800 -Csv results\soak.csv

输出：
    - 每帧解析结果（可选 -Verbose 逐帧打印）
    - CSV：t_s,tj_c,text_c,rh_pct,tset_c,duty,gear,state,f_mhz,p_mw,age,flags,seq
    - 汇总：帧数、CRC 错、帧头错、seq 跳变次数、Tj 最小/最大
#>
param(
    [string]$Port = "COM4",
    [int]$Baud = 115200,
    [int]$Seconds = 60,
    [string]$Csv = "",
    [switch]$Quiet
)

$ErrorActionPreference = "Stop"

function Get-Crc16([byte[]]$data, [int]$len) {
    $crc = 0xFFFF
    for ($i = 0; $i -lt $len; $i++) {
        $crc = $crc -bxor ([int]$data[$i] -shl 8)
        for ($b = 0; $b -lt 8; $b++) {
            if ($crc -band 0x8000) { $crc = (($crc -shl 1) -bxor 0x1021) -band 0xFFFF }
            else { $crc = ($crc -shl 1) -band 0xFFFF }
        }
    }
    return $crc
}

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 500
$sp.Open()
if (-not $Quiet) { Write-Host "已打开 $Port @ $Baud，采集 $Seconds 秒..." }

$buf = New-Object System.Collections.Generic.List[byte]
$rows = New-Object System.Collections.Generic.List[string]
$nFrame = 0; $nCrc = 0; $nHdr = 0; $nSeqJump = 0
$lastSeq = -1
$tjMin = 999.0; $tjMax = -999.0
$t0 = $null
$sw = [System.Diagnostics.Stopwatch]::StartNew()

function Q8_8([int]$lo, [int]$hi) {
    $v = $lo -bor ($hi -shl 8)
    if ($v -ge 32768) { $v -= 65536 }
    return $v / 256.0
}

while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    $chunk = New-Object byte[] 512
    $n = 0
    try { $n = $sp.Read($chunk, 0, 512) } catch { $n = 0 }
    if ($n -le 0) { continue }
    for ($i = 0; $i -lt $n; $i++) { $buf.Add($chunk[$i]) }

    # 在缓冲里找完整帧
    while ($true) {
        $sync = -1
        for ($i = 0; $i -lt $buf.Count - 1; $i++) {
            if ($buf[$i] -eq 0xA5 -and $buf[$i + 1] -eq 0x5A) { $sync = $i; break }
        }
        if ($sync -lt 0) { if ($buf.Count -gt 1) { $buf.RemoveRange(0, $buf.Count - 1) }; break }
        if ($sync -gt 0) { $buf.RemoveRange(0, $sync) }
        if ($buf.Count -lt 32) { break }

        $f = $buf.GetRange(0, 32).ToArray()
        $crcRecv = [int]$f[30] -bor ([int]$f[31] -shl 8)
        $crcCalc = Get-Crc16 ($f[2..29]) 28   # 规范：CRC 覆盖字节 2..29，不含帧头
        if ($crcRecv -ne $crcCalc) {
            $nCrc++
            $buf.RemoveRange(0, 1)
            continue
        }
        $buf.RemoveRange(0, 32)
        $nFrame++

        $seq   = [int]$f[4] -bor ([int]$f[5] -shl 8)
        $ts    = [BitConverter]::ToUInt32($f, 6)
        $tj    = Q8_8 $f[10] $f[11]
        $text  = Q8_8 $f[12] $f[13]
        $rh    = (([int]$f[14] -bor ([int]$f[15] -shl 8))) / 10.0
        $tset  = Q8_8 $f[16] $f[17]
        $duty  = [int]$f[18] -bor ([int]$f[19] -shl 3)
        $gear  = [int]$f[20]
        $state = [int]$f[21]
        $fq    = (([int]$f[22] -bor ([int]$f[23] -shl 8))) / 10.0
        $p     = [int]$f[24] -bor ([int]$f[25] -shl 8)
        $age   = [int]$f[26]
        $flags = [int]$f[27]

        if ($lastSeq -ge 0 -and (($lastSeq + 1) -band 0xFFFF) -ne $seq) { $nSeqJump++ }
        $lastSeq = $seq
        if ($null -eq $t0) { $t0 = $ts }
        $trel = ($ts - $t0) / 1000.0

        if ($tj -lt $tjMin) { $tjMin = $tj }
        if ($tj -gt $tjMax) { $tjMax = $tj }

        $rows.Add(("{0:F2},{1:F2},{2:F2},{3:F1},{4:F2},{5},{6},{7},{8:F1},{9},{10},{11},{12}" -f `
            $trel, $tj, $text, $rh, $tset, $duty, $gear, $state, $fq, $p, $age, $flags, $seq))

        if (-not $Quiet -and ($nFrame % 10) -eq 0) {
            Write-Host ("帧{0,4}  T={1,6:F2}℃  Tset={2:F1}  TEXT={3:F2}  RH={4:F1}%  duty={5,3}  gear={6}  f={7:F1}MHz  P={8}mW  state={9}" -f `
                $nFrame, $tj, $tset, $text, $rh, $duty, $gear, $fq, $p, $state)
        }
    }
}
$sp.Close()

if ($Csv -ne "") {
    $dir = Split-Path -Parent $Csv
    if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    "t_s,tj_c,text_c,rh_pct,tset_c,duty,gear,state,f_mhz,p_mw,age,flags,seq" | Set-Content -Path $Csv -Encoding utf8
    $rows | Add-Content -Path $Csv -Encoding utf8
    if (-not $Quiet) { Write-Host "CSV -> $Csv ($($rows.Count) 行)" }
}

Write-Host ""
Write-Host "==== 汇总 ===="
Write-Host ("帧数        : {0}  （约 {1:F1} 帧/s）" -f $nFrame, ($nFrame / [Math]::Max(1, $sw.Elapsed.TotalSeconds)))
Write-Host ("CRC 错      : {0}" -f $nCrc)
Write-Host ("seq 跳变    : {0}" -f $nSeqJump)
if ($nFrame -gt 0) {
    Write-Host ("Tj 范围     : {0:F2} … {1:F2} ℃" -f $tjMin, $tjMax)
}
if ($nFrame -eq 0) { Write-Host "没有收到任何有效帧 —— 检查波特率/接线/bitstream 是否已下载" }

