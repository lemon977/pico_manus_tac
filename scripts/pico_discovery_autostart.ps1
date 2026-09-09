[CmdletBinding()]
param(
    [Parameter(Position = 0, Mandatory = $true)]
    [ValidateSet("ensure", "install", "start", "status", "stop", "uninstall")]
    [string]$Action
)

$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$RunDir = Join-Path $ProjectRoot ".run"
$DiscoveryScript = Join-Path $ProjectRoot "pico_discovery.py"
$PicoReceiverScript = Join-Path $ProjectRoot "pico_receiver.py"
$StatusFile = Join-Path $RunDir "pico_discovery.status.json"
$LogFile = Join-Path $RunDir "pico_discovery.autostart.log"
$RawDir = Join-Path $ProjectRoot "data\sessions"
$PicoProcessRecord = Join-Path $RunDir "pico.windows.json"
$PicoStdoutLog = Join-Path $RunDir "pico.windows.out.log"
$PicoStderrLog = Join-Path $RunDir "pico.windows.err.log"
$PicoControlPort = 63910
$RunRegistryPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
$RunRegistryName = "PICOEgoDiscovery"
$Interval = if ($env:PICO_DISCOVERY_INTERVAL) { $env:PICO_DISCOVERY_INTERVAL } else { "1" }
$RescanInterval = if ($env:PICO_DISCOVERY_RESCAN_INTERVAL) { $env:PICO_DISCOVERY_RESCAN_INTERVAL } else { "5" }

# Registry autostart does not inherit collect_windows.ps1's environment.
# Force UTF-8 so hidden-service logs can always be read by Show-ServiceLogs.
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

New-Item -ItemType Directory -Force -Path $RunDir, $RawDir | Out-Null
if (-not (Test-Path -LiteralPath $DiscoveryScript -PathType Leaf)) {
    throw "缺少 PICO 发现广播程序: $DiscoveryScript"
}
if (-not (Test-Path -LiteralPath $PicoReceiverScript -PathType Leaf)) {
    throw "缺少 PICO 接收程序: $PicoReceiverScript"
}

function Resolve-Python {
    if ($env:EGO_PYTHON_EXE) {
        if (-not (Test-Path -LiteralPath $env:EGO_PYTHON_EXE -PathType Leaf)) {
            throw "EGO_PYTHON_EXE 不存在: $env:EGO_PYTHON_EXE"
        }
        return (Resolve-Path -LiteralPath $env:EGO_PYTHON_EXE).Path
    }
    $command = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $command) { throw "找不到 python；无法启动 PICO 发现广播" }
    return $command.Source
}

function Quote-ProcessArgument([string]$Value) {
    if ($null -eq $Value -or $Value.Length -eq 0) { return '""' }
    if ($Value -notmatch '[\s"]') { return $Value }
    return '"' + $Value.Replace('"', '\"') + '"'
}

$PythonExe = Resolve-Python
$PythonWindowExe = Join-Path (Split-Path -Parent $PythonExe) "pythonw.exe"
if (-not (Test-Path -LiteralPath $PythonWindowExe -PathType Leaf)) {
    $PythonWindowExe = $PythonExe
}
$ArgumentValues = @(
    "-u", $DiscoveryScript,
    "--interval", $Interval,
    "--rescan-interval", $RescanInterval,
    "--status-file", $StatusFile,
    "--log-file", $LogFile
)
$ProcessArguments = ($ArgumentValues | ForEach-Object { Quote-ProcessArgument $_ }) -join " "
$PicoArgumentValues = @(
    "-u", $PicoReceiverScript,
    "--service", "--print-hz", "0", "--log-dir", $RawDir,
    "--video", "--video-size", "4096x1536", "--video-fps", "30",
    "--no-view", "--no-broadcast"
)
$PicoProcessArguments = ($PicoArgumentValues | ForEach-Object { Quote-ProcessArgument $_ }) -join " "
$ControllerCommand = 'powershell.exe -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File ' +
    (Quote-ProcessArgument $PSCommandPath) + ' start'

function Get-AutostartCommand {
    try {
        return [string](Get-ItemPropertyValue -LiteralPath $RunRegistryPath `
            -Name $RunRegistryName -ErrorAction Stop)
    } catch {
        return $null
    }
}

function Install-Autostart {
    New-Item -Path $RunRegistryPath -Force | Out-Null
    New-ItemProperty -LiteralPath $RunRegistryPath -Name $RunRegistryName `
        -PropertyType String -Value $ControllerCommand -Force | Out-Null
    Write-Host "[PICO-DISCOVERY] 已安装 Windows 登录自启动项: $RunRegistryName" -ForegroundColor Green
}

function Read-DiscoveryStatus {
    if (-not (Test-Path -LiteralPath $StatusFile -PathType Leaf)) { return $null }
    try {
        return Get-Content -LiteralPath $StatusFile -Raw -Encoding UTF8 | ConvertFrom-Json
    } catch {
        return $null
    }
}

function Write-PicoProcessRecord([System.Diagnostics.Process]$Process) {
    $record = [ordered]@{
        schema = "pico_ego_windows_process_v1"
        name = "pico"
        pid = $Process.Id
        start_utc = $Process.StartTime.ToUniversalTime().ToString("o")
        command = "$PythonExe $PicoProcessArguments"
    }
    $record | ConvertTo-Json | Set-Content -LiteralPath $PicoProcessRecord -Encoding UTF8
}

function Get-PicoProcess {
    if (-not (Test-Path -LiteralPath $PicoProcessRecord -PathType Leaf)) { return $null }
    try {
        $record = Get-Content -LiteralPath $PicoProcessRecord -Raw -Encoding UTF8 | ConvertFrom-Json
        $process = Get-Process -Id ([int]$record.pid) -ErrorAction SilentlyContinue
        if ($null -eq $process) { return $null }
        $expected = [datetime]::Parse([string]$record.start_utc).ToUniversalTime()
        if ([math]::Abs(($process.StartTime.ToUniversalTime() - $expected).TotalSeconds) -gt 3.0) {
            return $null
        }
        return $process
    } catch {
        return $null
    }
}

function Send-PicoControl([string]$Command, [int]$TimeoutMs = 1500) {
    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $connect = $client.ConnectAsync("127.0.0.1", $PicoControlPort)
        if (-not $connect.Wait($TimeoutMs)) { throw "connect timeout" }
        $stream = $client.GetStream()
        $stream.ReadTimeout = $TimeoutMs
        $stream.WriteTimeout = $TimeoutMs
        $writer = New-Object System.IO.StreamWriter($stream, [Text.UTF8Encoding]::new($false), 1024, $true)
        $writer.NewLine = "`n"
        $writer.AutoFlush = $true
        $reader = New-Object System.IO.StreamReader($stream, [Text.UTF8Encoding]::new($false), $false, 1024, $true)
        $writer.WriteLine($Command)
        return $reader.ReadLine()
    } finally {
        $client.Dispose()
    }
}

function Test-PicoControl {
    try { return (Send-PicoControl "PING" 1000) -eq "PONG" } catch { return $false }
}

function Stop-PicoProcess {
    try { [void](Send-PicoControl "STOP" 3000) } catch { }
    # STOP keeps the warm stream alive for normal session changes. A service
    # restart is different: explicitly stop the headset encoder first so the
    # next process receives a fresh SPS/PPS and applies any new resolution.
    try { [void](Send-PicoControl "VST_OFF" 3000) } catch { }
    Start-Sleep -Milliseconds 350
    $process = Get-PicoProcess
    if ($null -ne $process) {
        Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
        try { [void]$process.WaitForExit(5000) } catch { }
    }
    Remove-Item -LiteralPath $PicoProcessRecord -Force -ErrorAction SilentlyContinue
}

function Start-PicoProcess {
    $process = Get-PicoProcess
    $record = if (Test-Path -LiteralPath $PicoProcessRecord -PathType Leaf) {
        try { Get-Content -LiteralPath $PicoProcessRecord -Raw -Encoding UTF8 | ConvertFrom-Json }
        catch { $null }
    } else { $null }
    $expectedCommand = "$PythonExe $PicoProcessArguments"
    if ($null -ne $process -and (Test-PicoControl) -and
        $null -ne $record -and [string]$record.command -eq $expectedCommand) {
        return $process
    }
    Stop-PicoProcess
    Set-Content -LiteralPath $PicoStdoutLog -Value "" -Encoding UTF8
    Set-Content -LiteralPath $PicoStderrLog -Value "" -Encoding UTF8
    $process = Start-Process -FilePath $PythonExe -ArgumentList $PicoProcessArguments `
        -WorkingDirectory $ProjectRoot -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput $PicoStdoutLog -RedirectStandardError $PicoStderrLog
    Write-PicoProcessRecord $process
    Write-Host "[PICO-LINK] 已启动常驻接收服务 pid=$($process.Id)"
    $deadline = [datetime]::UtcNow.AddSeconds(15)
    while ([datetime]::UtcNow -lt $deadline) {
        if ($process.HasExited) { break }
        if (Test-PicoControl) { return $process }
        Start-Sleep -Milliseconds 250
    }
    foreach ($path in @($PicoStdoutLog, $PicoStderrLog)) {
        if (Test-Path -LiteralPath $path -PathType Leaf) {
            Write-Host "--- $path ---"
            Get-Content -LiteralPath $path -Tail 30 -Encoding UTF8
        }
    }
    throw "PICO 常驻接收服务未在 15 秒内就绪"
}

function Get-StatusProcess([object]$Status) {
    if ($null -eq $Status) { return $null }
    $process = Get-Process -Id ([int]$Status.pid) -ErrorAction SilentlyContinue
    if ($null -eq $process) { return $null }
    try {
        $expected = [datetime]::Parse([string]$Status.started_utc).ToUniversalTime()
        $actual = $process.StartTime.ToUniversalTime()
        if ([math]::Abs(($actual - $expected).TotalSeconds) -gt 3.0) { return $null }
    } catch {
        return $null
    }
    return $process
}

function Get-LiveDiscoveryStatus {
    $status = Read-DiscoveryStatus
    if ($null -eq (Get-StatusProcess $status)) { return $null }
    try {
        $updated = [datetime]::Parse([string]$status.updated_utc).ToUniversalTime()
        if (([datetime]::UtcNow - $updated).TotalSeconds -gt 20) { return $null }
    } catch {
        return $null
    }
    return $status
}

function Stop-DiscoveryProcess {
    $status = Read-DiscoveryStatus
    $process = Get-StatusProcess $status
    if ($null -ne $process) {
        Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
        try { [void]$process.WaitForExit(5000) } catch { }
    }
}

function Start-DiscoveryProcess {
    $live = Get-LiveDiscoveryStatus
    if ($null -ne $live) { return $live }

    # A stale status can represent a hung older broadcaster. Stop only when
    # PID and process start time still match, then launch one clean instance.
    Stop-DiscoveryProcess
    $process = Start-Process -FilePath $PythonWindowExe `
        -ArgumentList $ProcessArguments -WorkingDirectory $ProjectRoot `
        -WindowStyle Hidden -PassThru
    Write-Host "[PICO-DISCOVERY] 已启动常驻广播 pid=$($process.Id)"

    $deadline = [datetime]::UtcNow.AddSeconds(15)
    while ([datetime]::UtcNow -lt $deadline) {
        Start-Sleep -Milliseconds 250
        $live = Get-LiveDiscoveryStatus
        if ($null -ne $live) { return $live }
        if ($process.HasExited) { break }
    }
    if (Test-Path -LiteralPath $LogFile -PathType Leaf) {
        Write-Host "--- $LogFile ---"
        Get-Content -LiteralPath $LogFile -Tail 30 -Encoding UTF8
    }
    throw "PICO 发现广播已启动，但 15 秒内未报告存活状态"
}

function Show-DiscoveryStatus {
    $installed = (Get-AutostartCommand) -eq $ControllerCommand
    $live = Get-LiveDiscoveryStatus
    if ($null -eq $live) {
        $label = if ($installed) { "installed" } else { "not_installed" }
        Write-Host "[PICO-DISCOVERY] autostart=$label process=down" -ForegroundColor Yellow
        return $false
    }
    $ips = @($live.ips) -join ","
    if ([string]::IsNullOrWhiteSpace($ips)) { $ips = "(none)" }
    $label = if ($installed) { "installed" } else { "not_installed" }
    Write-Host "[PICO-DISCOVERY] autostart=$label pid=$($live.pid) ips=$ips sent=$($live.sent) errors=$($live.errors) updated=$($live.updated_utc)"
    return $installed
}

function Show-PicoLinkStatus {
    $process = Get-PicoProcess
    if ($null -eq $process -or -not (Test-PicoControl)) {
        Write-Host "[PICO-LINK] process=down control=down" -ForegroundColor Yellow
        return $false
    }
    try {
        $reply = Send-PicoControl "STATUS" 1500
        Write-Host "[PICO-LINK] pid=$($process.Id) $reply"
    } catch {
        Write-Host "[PICO-LINK] pid=$($process.Id) control=down" -ForegroundColor Yellow
        return $false
    }
    return $true
}

switch ($Action) {
    "install" {
        Install-Autostart
        $live = Start-DiscoveryProcess
        $pico = Start-PicoProcess
        [void](Show-DiscoveryStatus)
        [void](Show-PicoLinkStatus)
    }
    "ensure" {
        if ((Get-AutostartCommand) -ne $ControllerCommand) {
            Install-Autostart
        }
        $live = Start-DiscoveryProcess
        $pico = Start-PicoProcess
        [void](Show-DiscoveryStatus)
        [void](Show-PicoLinkStatus)
    }
    "start" {
        $live = Start-DiscoveryProcess
        $pico = Start-PicoProcess
        [void](Show-DiscoveryStatus)
        [void](Show-PicoLinkStatus)
    }
    "status" {
        $discoveryOk = Show-DiscoveryStatus
        $picoOk = Show-PicoLinkStatus
        if (-not $discoveryOk -or -not $picoOk) { exit 1 }
    }
    "stop" {
        Stop-PicoProcess
        Stop-DiscoveryProcess
        Write-Host "[PICO-LINK] 常驻接收和发现广播已停止；登录自启动配置仍保留。" -ForegroundColor Yellow
    }
    "uninstall" {
        Stop-PicoProcess
        Stop-DiscoveryProcess
        Remove-ItemProperty -LiteralPath $RunRegistryPath -Name $RunRegistryName `
            -ErrorAction SilentlyContinue
        Write-Host "[PICO-DISCOVERY] 已卸载 Windows 登录自启动项。" -ForegroundColor Yellow
    }
}
