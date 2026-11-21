param(
    [Parameter(Mandatory = $true, Position = 0)][string]$Cmd,
    [string]$Port = "COM3",
    [int]$Baud = 115200,
    [int]$Ms = 600
)

# serial_cmd.ps1 —— 从命令行发一条 ASCII 命令给板子，并把回复打印出来
# 用法：  scripts\serial_cmd.bat "?"
#         scripts\serial_cmd.bat "S=36.0"
#         scripts\serial_cmd.bat "Z"
# 说明：  内部会自动把 32 字节的二进制遥测帧剔掉，只显示 ASCII 回复。

$ErrorActionPreference = "Stop"

try {
    $sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
    $sp.ReadTimeout = 20
    $sp.Open()
} catch {
    Write-Host "打开 $Port 失败：$($_.Exception.Message)"
    Write-Host "常见原因：串口被别的程序占用（PuTTY / 另一个采集脚本）"
    exit 1
}

$sp.DiscardInBuffer()
Start-Sleep -Milliseconds 120
$sp.Write($Cmd + "`r")
Start-Sleep -Milliseconds 250

$buf = New-Object System.Collections.Generic.List[byte]
$t0 = Get-Date
while (((Get-Date) - $t0).TotalMilliseconds -lt $Ms) {
    try { $buf.Add([byte]$sp.ReadByte()) } catch { }
}
$sp.Close()

# 剔除二进制遥测帧（A5 5A + 32 字节）
$a = $buf.ToArray()
$nf = New-Object System.Collections.Generic.List[byte]
$i = 0
while ($i -lt $a.Length - 1) {
    if ($a[$i] -eq 0xA5 -and $a[$i + 1] -eq 0x5A) { $i += 32; continue }
    $nf.Add($a[$i]); $i++
}

$s = -join ($nf | ForEach-Object { if ($_ -ge 32 -and $_ -lt 127) { [char]$_ } else { "" } })
Write-Host ("[" + $s.Trim() + "]")
