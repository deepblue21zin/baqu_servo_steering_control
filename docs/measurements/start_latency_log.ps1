# Double-click runner for PuTTY serial latency logging
# 1) Edit $Port once (e.g., COM5 -> your port)
# 2) Double-click this file to start logging

$Port = "COM5"   # TODO: change to your STM32 COM port
$Baud = 115200

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Timestamp = Get-Date -Format "yyyy-MM-dd_HHmmss"
$LogFile = Join-Path $ScriptDir ("latency_{0}.csv" -f $Timestamp)
$MetaFile = Join-Path $ScriptDir ("latency_{0}_meta.md" -f $Timestamp)

$PuttyCandidates = @(
    "C:\Program Files\PuTTY\putty.exe",
    "C:\Program Files (x86)\PuTTY\putty.exe"
)
$PuttyPath = $PuttyCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1

if (-not $PuttyPath) {
    Write-Host "PuTTY not found. Install PuTTY first: https://www.putty.org/"
    Read-Host "Press Enter to exit"
    exit 1
}

$Meta = @"
# Latency Measurement Meta

- timestamp: $Timestamp
- com_port: $Port
- baud: $Baud
- log_file: $(Split-Path -Leaf $LogFile)
- build_option: -O2
- latency_auto_report_samples: 2000
- note: fill Git SHA, input sequence ID, board info after run
"@
Set-Content -Path $MetaFile -Value $Meta -Encoding UTF8

Write-Host "Starting PuTTY logging..."
Write-Host "Port: $Port  Baud: $Baud"
Write-Host "CSV : $LogFile"
Write-Host "Meta: $MetaFile"
Write-Host "Close PuTTY window to finish logging."

Start-Process -FilePath $PuttyPath -ArgumentList "-serial $Port -sercfg $Baud,8,n,1,N -sessionlog \"$LogFile\" -logoverwrite"
