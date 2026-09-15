param(
    [Parameter(Position = 0, Mandatory = $true)]
    [ValidateSet("start", "restart", "stop", "status", "check", "pipeline", "doctor")]
    [string]$Action,

    [Parameter(Position = 1)]
    [string]$Session,

    [switch]$NoVideo,
    [switch]$SkipManus,
    [switch]$SkipTactile
)

$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$RunDir = Join-Path $ProjectRoot ".run"
$DataRoot = Join-Path $ProjectRoot "data"
$SessionsDir = Join-Path $DataRoot "sessions"
$RawDir = $SessionsDir
$SessionDataTools = Join-Path $PSScriptRoot "session_data_tools.ps1"
$PicoDiscoveryController = Join-Path $PSScriptRoot "pico_discovery_autostart.ps1"
if (-not (Test-Path -LiteralPath $SessionDataTools -PathType Leaf)) {
    throw "缺少批次数据工具: $SessionDataTools"
}
. $SessionDataTools
$PicoControlPort = 63910
$ManusControlPort = 63911
$TactileControlPort = 63912
$PicoTrackingPort = 63901
$PicoVideoPort = 63902
$TactileRateHz = if ($env:TACTILE_RATE_HZ) { [double]$env:TACTILE_RATE_HZ } else { 60.0 }
$TactileTransientLimit = if ($env:TACTILE_TRANSIENT_LIMIT) { [int]$env:TACTILE_TRANSIENT_LIMIT } else { 3 }
$TactileTransientWindowS = if ($env:TACTILE_TRANSIENT_WINDOW_S) { [double]$env:TACTILE_TRANSIENT_WINDOW_S } else { 5.0 }

Set-Location -LiteralPath $ProjectRoot
$env:PYTHONUTF8 = "1"
foreach ($dir in @($RunDir, $SessionsDir)) {
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
}
$machinePath = [Environment]::GetEnvironmentVariable("Path", "Machine")
$userPath = [Environment]::GetEnvironmentVariable("Path", "User")
$env:Path = "$env:Path;$machinePath;$userPath"
$env:MPLCONFIGDIR = Join-Path $RunDir "matplotlib"
New-Item -ItemType Directory -Force -Path $env:MPLCONFIGDIR | Out-Null

function Resolve-Python {
    if ($env:EGO_PYTHON_EXE) {
        if (-not (Test-Path -LiteralPath $env:EGO_PYTHON_EXE -PathType Leaf)) {
            throw "EGO_PYTHON_EXE 不存在: $env:EGO_PYTHON_EXE"
        }
        return (Resolve-Path -LiteralPath $env:EGO_PYTHON_EXE).Path
    }
    $command = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $command) { throw "找不到 python；请先安装 64 位 Python 3.10+" }
    return $command.Source
}

$PythonExe = Resolve-Python

function Quote-ProcessArgument([string]$Value) {
    if ($null -eq $Value -or $Value.Length -eq 0) { return '""' }
    if ($Value -notmatch '[\s"]') { return $Value }
    return '"' + $Value.Replace('"', '\"') + '"'
}

function Join-ProcessArguments([string[]]$Values) {
    return (($Values | ForEach-Object { Quote-ProcessArgument $_ }) -join " ")
}

function Write-ProcessRecord([string]$Name, [System.Diagnostics.Process]$Process, [string]$Command) {
    $record = [ordered]@{
        schema = "pico_ego_windows_process_v1"
        name = $Name
        pid = $Process.Id
        start_utc = $Process.StartTime.ToUniversalTime().ToString("o")
        command = $Command
    }
    $path = Join-Path $RunDir "$Name.windows.json"
    $record | ConvertTo-Json | Set-Content -LiteralPath $path -Encoding UTF8
}

function Read-ProcessRecord([string]$Name) {
    $path = Join-Path $RunDir "$Name.windows.json"
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { return $null }
    try {
        return Get-Content -LiteralPath $path -Raw -Encoding UTF8 | ConvertFrom-Json
    } catch {
        Write-Warning "无法读取进程记录 $path：$($_.Exception.Message)"
        return $null
    }
}

function Get-VerifiedProcess([string]$Name) {
    $record = Read-ProcessRecord $Name
    if ($null -eq $record) { return $null }
    $process = Get-Process -Id ([int]$record.pid) -ErrorAction SilentlyContinue
    if ($null -eq $process) { return $null }
    try {
        $expected = [datetime]::Parse([string]$record.start_utc).ToUniversalTime()
        $actual = $process.StartTime.ToUniversalTime()
        if ([math]::Abs(($actual - $expected).TotalSeconds) -gt 2.0) {
            Write-Warning "$Name PID=$($record.pid) 已被复用，拒绝操作"
            return $null
        }
    } catch {
        Write-Warning "$Name 进程身份校验失败，拒绝操作"
        return $null
    }
    return $process
}

function Reset-LogFile([string]$Path, [int]$TimeoutMs = 5000) {
    $deadline = [datetime]::UtcNow.AddMilliseconds($TimeoutMs)
    $lastError = $null
    while ([datetime]::UtcNow -lt $deadline) {
        try {
            Set-Content -LiteralPath $Path -Value "" -Encoding UTF8
            return
        } catch {
            $lastError = $_.Exception
            Start-Sleep -Milliseconds 100
        }
    }
    throw "日志文件在 ${TimeoutMs}ms 内未释放: $Path；$($lastError.Message)"
}

function Start-ManagedPython([string]$Name, [string[]]$Arguments) {
    $stdout = Join-Path $RunDir "$Name.windows.out.log"
    $stderr = Join-Path $RunDir "$Name.windows.err.log"
    Reset-LogFile $stdout
    Reset-LogFile $stderr
    $argumentLine = Join-ProcessArguments $Arguments
    $process = Start-Process -FilePath $PythonExe -ArgumentList $argumentLine `
        -WorkingDirectory $ProjectRoot -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    Write-ProcessRecord $Name $process "$PythonExe $argumentLine"
    Write-Host "[start] $Name pid=$($process.Id)"
    return $process
}

function Start-TactileWindow {
    $wrapper = Join-Path $PSScriptRoot "tactile_service_windows.ps1"
    $arguments = @(
        "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $wrapper,
        "-Python", $PythonExe, "-ProjectRoot", $ProjectRoot,
        "-RateHz", ([string]$TactileRateHz),
        "-TransientIncidentLimit", ([string]$TactileTransientLimit),
        "-TransientWindowSeconds", ([string]$TactileTransientWindowS)
    )
    $argumentLine = Join-ProcessArguments $arguments
    # 触觉配对必须由用户交互，因此显式打开可见窗口。
    $process = Start-Process -FilePath "powershell.exe" -ArgumentList $argumentLine `
        -WorkingDirectory $ProjectRoot -WindowStyle Normal -PassThru
    Write-ProcessRecord "tactile" $process "powershell.exe $argumentLine"
    Write-Host "[start] tactile window pid=$($process.Id)"
    return $process
}

function Send-Control([int]$Port, [string]$Command, [int]$TimeoutMs = 3000) {
    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $connect = $client.ConnectAsync("127.0.0.1", $Port)
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

function Wait-Control([string]$Name, [int]$Port, [int]$TimeoutSeconds) {
    $deadline = [datetime]::UtcNow.AddSeconds($TimeoutSeconds)
    while ([datetime]::UtcNow -lt $deadline) {
        $process = Get-VerifiedProcess $Name
        if ($null -eq $process -or $process.HasExited) { return $false }
        try {
            $reply = Send-Control $Port "PING" 1000
            if ($reply -eq "PONG") { return $true }
        } catch { }
        Start-Sleep -Milliseconds 250
    }
    return $false
}

function Show-ServiceLogs([string]$Name) {
    foreach ($suffix in @("out.log", "err.log")) {
        $path = Join-Path $RunDir "$Name.windows.$suffix"
        if (Test-Path -LiteralPath $path) {
            Write-Host "--- $path ---"
            Get-Content -LiteralPath $path -Tail 40 -Encoding UTF8
        }
    }
}

function Stop-RecordedTree([string]$Name) {
    $path = Join-Path $RunDir "$Name.windows.json"
    $process = Get-VerifiedProcess $Name
    if ($null -ne $process) {
        Write-Host "[stop] $Name pid=$($process.Id)"
        & taskkill.exe /PID $process.Id /T /F 2>&1 | ForEach-Object { Write-Host "  $_" }
        try {
            if (-not $process.HasExited) {
                [void]$process.WaitForExit(5000)
            }
        } catch { }
        $deadline = [datetime]::UtcNow.AddSeconds(5)
        while ([datetime]::UtcNow -lt $deadline) {
            if ($null -eq (Get-Process -Id $process.Id -ErrorAction SilentlyContinue)) {
                break
            }
            Start-Sleep -Milliseconds 100
        }
    }
    if (Test-Path -LiteralPath $path) {
        Remove-Item -LiteralPath $path -Force
    }
}

function Stop-EgoServices([switch]$BestEffort) {
    Write-Host "======== STOP ========"
    $failures = @()
    $items = @(
        @{ Name = "tactile"; Port = $TactileControlPort; Timeout = 25000 },
        @{ Name = "manus"; Port = $ManusControlPort; Timeout = 25000 },
        @{ Name = "pico"; Port = $PicoControlPort; Timeout = 50000 }
    )
    foreach ($item in $items) {
        $process = Get-VerifiedProcess $item.Name
        if ($null -eq $process) { continue }
        try {
            $status = Send-Control $item.Port "STATUS" 2500
            $isIdle = ($item.Name -eq "tactile" -and $status -match '\bstate=PAIRED_IDLE\b') -or
                ($item.Name -ne "tactile" -and $status -match '^OK idle\b')
            if (-not $isIdle) {
                $reply = Send-Control $item.Port "STOP" $item.Timeout
                Write-Host "[$($item.Name)] $reply"
                $validStop = ($item.Name -eq "tactile" -and
                    $reply -match '^OK tactile_raw_stopped\b') -or
                    ($item.Name -ne "tactile" -and $reply -match '^OK stopped\b')
                if (-not $validStop) {
                    $failures += "$($item.Name) STOP 未返回有效完成证明: $reply"
                    continue
                }
            }
            $finalStatus = Send-Control $item.Port "STATUS" 2500
            $isIdle = ($item.Name -eq "tactile" -and
                $finalStatus -match '\bstate=PAIRED_IDLE\b') -or
                ($item.Name -ne "tactile" -and $finalStatus -match '^OK idle\b')
            if (-not $isIdle) {
                $failures += "$($item.Name) STOP 后未回到 idle: $finalStatus"
            }
        } catch {
            $failures += "$($item.Name) 无法确认安全停录: $($_.Exception.Message)"
        }
    }

    # MANUS and tactile now expose an idle-only graceful shutdown command.
    foreach ($item in $items | Where-Object { $_.Name -in @("tactile", "manus") }) {
        $process = Get-VerifiedProcess $item.Name
        if ($null -eq $process) { continue }
        try {
            $reply = Send-Control $item.Port "SHUTDOWN" 3000
            if ($reply -ne "OK shutting_down") {
                $failures += "$($item.Name) 拒绝优雅退出: $reply"
            }
        } catch {
            $failures += "$($item.Name) 优雅退出命令失败: $($_.Exception.Message)"
        }
        try { [void]$process.WaitForExit(8000) } catch { }
        if (-not $process.HasExited) {
            $failures += "$($item.Name) 优雅退出超时，已执行兜底终止"
            Stop-RecordedTree $item.Name
        } else {
            $recordPath = Join-Path $RunDir "$($item.Name).windows.json"
            Remove-Item -LiteralPath $recordPath -Force -ErrorAction SilentlyContinue
        }
    }
    $picoProcess = Get-VerifiedProcess "pico"
    if ($null -ne $picoProcess) {
        Write-Host "[keep] pico link pid=$($picoProcess.Id)；采集已停止，PICO 连接保持常驻。" -ForegroundColor Cyan
    }
    if ($failures.Count -gt 0) {
        $failures | ForEach-Object { Write-Host "[!] $_" -ForegroundColor Red }
        if (-not $BestEffort) {
            throw "服务未全部取得安全停止/退出证明"
        }
    }
}

function Test-PythonImports {
    $previousPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        $output = & $PythonExe -c "import numpy, scipy, h5py, cv2, matplotlib, serial; print('Python modules OK')" 2>&1
        $code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previousPreference
    }
    $output | ForEach-Object { Write-Host $_ }
    return ($code -eq 0)
}

function Invoke-Doctor(
    [switch]$AllowMissingManus,
    [switch]$AllowMissingTactile
) {
    Write-Host "======== WINDOWS DOCTOR ========"
    $failed = $false
    Write-Host "project: $ProjectRoot"
    $versionOutput = & $PythonExe --version 2>&1
    $versionOutput | ForEach-Object { Write-Host $_ }
    if (-not (Test-PythonImports)) { $failed = $true }

    foreach ($command in @("ffmpeg", "ffprobe")) {
        $found = Get-Command $command -ErrorAction SilentlyContinue
        if ($null -eq $found) {
            Write-Host "MISS $command" -ForegroundColor Red
            $failed = $true
        } else {
            Write-Host "OK   $command -> $($found.Source)" -ForegroundColor Green
        }
    }

    foreach ($relative in @(
        "pico_discovery.py", "scripts\pico_discovery_autostart.ps1",
        "pico_receiver.py", "manus_collector.py", "pico_record.py",
        "tactile_collector.py", "align_pico_manus.py", "export_dataset.py",
        "config\tactile_pairing.json", "config\calib_wrist.json",
        "config\pico_cam\vst_cam.json"
    )) {
        if (-not (Test-Path -LiteralPath (Join-Path $ProjectRoot $relative))) {
            Write-Host "MISS $relative" -ForegroundColor Red
            $failed = $true
        }
    }

    foreach ($side in @("left", "right")) {
        $calibration = Join-Path $ProjectRoot "config\manus_$side.mcal"
        $valid = (Test-Path -LiteralPath $calibration -PathType Leaf) -and
            ((Get-Item -LiteralPath $calibration).Length -gt 0)
        if (-not $valid) {
            $color = if ($AllowMissingManus) { "Yellow" } else { "Red" }
            Write-Host "MISS MANUS $side 个人标定: $calibration" -ForegroundColor $color
            if (-not $AllowMissingManus) { $failed = $true }
        } else {
            $hash = (Get-FileHash -LiteralPath $calibration -Algorithm SHA256).Hash.ToLowerInvariant()
            Write-Host "OK   MANUS $side calibration sha256=$($hash.Substring(0, 12))..." `
                -ForegroundColor Green
        }
    }

    $pairingOutput = & $PythonExe (Join-Path $ProjectRoot "tactile_pairing.py") --check-config 2>&1
    $pairingCode = $LASTEXITCODE
    $pairingOutput | ForEach-Object { Write-Host $_ }
    if ($pairingCode -ne 0) { $failed = $true }
    Write-Host "--- tactile ports ---"
    $portOutput = & $PythonExe (Join-Path $ProjectRoot "tactile_probe.py") --list 2>&1
    $portOutput | ForEach-Object { Write-Host $_ }
    $candidateCount = @(
        $portOutput | Where-Object { [string]$_ -match '^\s*candidate_[0-9]+\s+' }
    ).Count
    if ($candidateCount -ne 2) {
        $portColor = if ($AllowMissingTactile) { "Yellow" } else { "Red" }
        Write-Host "MISS 双手触觉设备：需要2只，当前识别到 $candidateCount 只" `
            -ForegroundColor $portColor
        if (-not $AllowMissingTactile) { $failed = $true }
    }

    $bridge = Join-Path $ProjectRoot "manus_ndjson_bridge\manus_ndjson_bridge.exe"
    $dll = Get-ChildItem -LiteralPath (Join-Path $ProjectRoot "manus_ndjson_bridge") `
        -Filter "ManusSDK*.dll" -File -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not (Test-Path -LiteralPath $bridge -PathType Leaf)) {
        $bridgeColor = if ($AllowMissingManus) { "Yellow" } else { "Red" }
        Write-Host "MISS MANUS Windows bridge: $bridge" -ForegroundColor $bridgeColor
        Write-Host "     运行 scripts\setup_manus_bridge.ps1（需要厂商 Windows SDK）"
        if (-not $AllowMissingManus) { $failed = $true }
    } elseif ($null -eq $dll) {
        $dllColor = if ($AllowMissingManus) { "Yellow" } else { "Red" }
        Write-Host "MISS MANUS runtime DLL beside bridge" -ForegroundColor $dllColor
        if (-not $AllowMissingManus) { $failed = $true }
    } else {
        Write-Host "OK   MANUS bridge + $($dll.Name)" -ForegroundColor Green
    }

    Write-Host "--- PICO firewall ---"
    $firewallSpecs = @(
        @{ Name = "PICO Ego Tracking TCP"; Protocol = "TCP"; Port = "63901" },
        @{ Name = "PICO Ego Discovery UDP"; Protocol = "UDP"; Port = "29888" },
        @{ Name = "PICO Ego VST TCP"; Protocol = "TCP"; Port = "63902" }
    )
    foreach ($spec in $firewallSpecs) {
        try {
            $matching = @(
                Get-NetFirewallRule -DisplayName $spec.Name -ErrorAction Stop |
                    Where-Object {
                        $portFilter = $_ | Get-NetFirewallPortFilter
                        $addressFilter = $_ | Get-NetFirewallAddressFilter
                        $_.Enabled -eq "True" -and
                        $_.Direction -eq "Inbound" -and
                        $_.Action -eq "Allow" -and
                        ($_.Profile.ToString() -match "Private") -and
                        $portFilter.Protocol.ToString() -eq $spec.Protocol -and
                        $portFilter.LocalPort.ToString() -eq $spec.Port -and
                        @($addressFilter.RemoteAddress) -contains "LocalSubnet"
                    }
            )
        } catch {
            $matching = @()
        }
        if ($matching.Count -eq 0) {
            Write-Host "MISS $($spec.Name): $($spec.Protocol) $($spec.Port) Private/LocalSubnet" -ForegroundColor Red
            $failed = $true
        } else {
            Write-Host "OK   $($spec.Name): $($spec.Protocol) $($spec.Port) Private/LocalSubnet" -ForegroundColor Green
        }
    }
    if ($failed) {
        Write-Host "修复命令: scripts\configure_pico_firewall.ps1" -ForegroundColor Yellow
    }
    return (-not $failed)
}

function Start-EgoServices {
    Write-Host "======== START (WINDOWS) ========"
    Write-Host "--- PICO always-on discovery ---"
    & $PicoDiscoveryController ensure
    if (-not $SkipManus) {
        $env:MANUS_CALIB_LEFT = Join-Path $ProjectRoot "config\manus_left.mcal"
        $env:MANUS_CALIB_RIGHT = Join-Path $ProjectRoot "config\manus_right.mcal"
    }

    $bridgeExe = Join-Path $ProjectRoot "manus_ndjson_bridge\manus_ndjson_bridge.exe"
    $bridgeSource = Join-Path $ProjectRoot "manus_ndjson_bridge\manus_ndjson_bridge.cpp"
    if (-not $SkipManus -and (Test-Path -LiteralPath $bridgeExe) -and
        (Test-Path -LiteralPath $bridgeSource) -and
        ((Get-Item -LiteralPath $bridgeSource).LastWriteTimeUtc -gt
         (Get-Item -LiteralPath $bridgeExe).LastWriteTimeUtc)) {
        Write-Host "[svc] MANUS bridge 源码已更新，正在重新编译……" -ForegroundColor Cyan
        & (Join-Path $ProjectRoot "scripts\setup_manus_bridge.ps1")
        if ($LASTEXITCODE -ne 0) { throw "MANUS bridge 重新编译失败" }
    }

    if (-not (Invoke-Doctor -AllowMissingManus:$SkipManus `
            -AllowMissingTactile:$SkipTactile)) {
        throw "Windows 部署自检未通过；服务未启动"
    }
    Stop-EgoServices -BestEffort

    try {
        if (-not (Wait-Control "pico" $PicoControlPort 15)) {
            Show-ServiceLogs "pico"
            throw "PICO 常驻控制服务未就绪"
        }

        if (-not $SkipManus) {
            $manusArgs = @("-u", (Join-Path $ProjectRoot "manus_collector.py"),
                "--service", "--print-hz", "0", "--hand-motion", "none",
                "--log-dir", $RawDir)
            Start-ManagedPython "manus" $manusArgs | Out-Null
            if (-not (Wait-Control "manus" $ManusControlPort 20)) {
                Show-ServiceLogs "manus"
                throw "MANUS 控制服务未就绪"
            }
        }

        if (-not $SkipTactile) {
            Write-Host "即将打开触觉交互窗口，请在该窗口完成两步配对。" -ForegroundColor Cyan
            Start-TactileWindow | Out-Null
            if (-not (Wait-Control "tactile" $TactileControlPort 300)) {
                $transcript = Join-Path $RunDir "tactile_service.windows.log"
                if (Test-Path -LiteralPath $transcript -PathType Leaf) {
                    Write-Host "--- 触觉窗口最近日志 ---"
                    Get-Content -LiteralPath $transcript -Tail 30
                }
                throw "触觉服务未在 300 秒内完成配对"
            }
        }
    } catch {
        Stop-EgoServices -BestEffort
        throw
    }

    $readyServices = @("PICO-DISCOVERY", "PICO")
    if (-not $SkipManus) { $readyServices += "MANUS" }
    if (-not $SkipTactile) { $readyServices += "TACTILE" }
    Write-Host "$($readyServices -join ' + ') Windows 服务已就绪。" -ForegroundColor Green
    Write-Host "采集: python pico_record.py start <任务名>"
    Write-Host "停止服务: powershell -ExecutionPolicy Bypass -File .\ego_ctl.ps1 stop"
}

function Show-Status {
    Write-Host "======== STATUS ========"
    & powershell.exe -NoProfile -ExecutionPolicy Bypass `
        -File $PicoDiscoveryController status
    foreach ($item in @(
        @{ Name = "pico"; Port = $PicoControlPort },
        @{ Name = "manus"; Port = $ManusControlPort },
        @{ Name = "tactile"; Port = $TactileControlPort }
    )) {
        $process = Get-VerifiedProcess $item.Name
        $processText = if ($null -eq $process) { "process=down" } else { "pid=$($process.Id)" }
        try {
            $reply = Send-Control $item.Port "STATUS" 1500
            Write-Host "[$($item.Name.ToUpper())] $processText $reply"
        } catch {
            Write-Host "[$($item.Name.ToUpper())] $processText control=down" -ForegroundColor Yellow
        }
    }
}

function Invoke-Check {
    $ok = Invoke-Doctor
    Show-Status
    $core = @(
        "pico_discovery.py", "pico_receiver.py", "manus_collector.py", "pico_record.py",
        "record_control.py", "tactile_protocol.py", "tactile_probe.py",
        "tactile_pairing.py", "tactile_collector.py", "align_pico_manus.py",
        "export_dataset.py"
    ) | ForEach-Object { Join-Path $ProjectRoot $_ }
    & $PythonExe -m py_compile @core
    if ($LASTEXITCODE -ne 0) { $ok = $false }
    if (-not $ok) { throw "Windows health check 未通过" }
    Write-Host "Windows health check PASS" -ForegroundColor Green
}

function Invoke-PythonStep([string]$Label, [string[]]$Arguments) {
    $previousPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & $PythonExe @Arguments
        $code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previousPreference
    }
    if ($code -ne 0) { throw "$Label 失败 (exit=$code)" }
}

function Invoke-Pipeline([string]$Name) {
    if ([string]::IsNullOrWhiteSpace($Name)) {
        $Name = Get-LatestRawSessionName -ProjectRoot $ProjectRoot
        if ([string]::IsNullOrWhiteSpace($Name)) { throw "没有可用会话，请指定 session" }
    }
    $paths = Get-SessionStoragePaths -ProjectRoot $ProjectRoot -Session $Name -PreferExisting
    $sessionDir = $paths.Raw
    $pico = Join-Path $sessionDir "pico.jsonl"
    $manus = Join-Path $sessionDir "manus.jsonl"
    if (-not (Test-Path -LiteralPath $pico -PathType Leaf)) { throw "缺 $pico" }
    if (-not (Test-Path -LiteralPath $manus -PathType Leaf)) { throw "缺 $manus" }

    $aligned = $paths.Aligned
    $hdf5 = $paths.Export
    $tactile = Join-Path $paths.Tactile "tactile.jsonl"
    $tactileMeta = Join-Path $paths.Tactile "tactile.meta.json"
    foreach ($parent in @((Split-Path $aligned -Parent), (Split-Path $hdf5 -Parent))) {
        New-Item -ItemType Directory -Path $parent -Force | Out-Null
    }
    $tactileArgs = @()
    if ((Test-Path -LiteralPath $tactile) -and (Test-Path -LiteralPath $tactileMeta)) {
        $tactileArgs = @("--tactile", $tactile, "--tactile-meta", $tactileMeta)
    } elseif ((Test-Path -LiteralPath "$tactile.partial") -or
              (Test-Path -LiteralPath $tactile) -or
              (Test-Path -LiteralPath $tactileMeta)) {
        throw "触觉资产不完整，拒绝导出: $(Split-Path $tactile -Parent)"
    }

    Write-Host "======== PIPELINE session=$Name ========"
    Invoke-PythonStep "analyze_quality" @(
        (Join-Path $ProjectRoot "analyze_quality.py"), $pico, $manus
    )

    $alignArgs = @((Join-Path $ProjectRoot "align_pico_manus.py"), $pico, $manus,
        "-o", $aligned, "--full", "--max-skew-ms", "30",
        "--gate-ms", "40", "--tactile-gate-ms", "40") + $tactileArgs
    Invoke-PythonStep "align" $alignArgs

    $exportArgs = @((Join-Path $ProjectRoot "export_dataset.py"), $pico, $manus,
        "-o", $hdf5, "--calib", (Join-Path $ProjectRoot "config\calib_wrist.json"),
        "--gate-ms", "40", "--video-gate-ms", "40", "--tactile-gate-ms", "40",
        "--min-hand-coverage", "0.95", "--min-video-coverage", "0.95",
        "--min-tactile-coverage", "0.95", "--min-complete-coverage", "0.95",
        "--max-p95-skew-ms", "30",
        "--max-skew-ms", "40", "--fps", "30") + $tactileArgs
    $vst = Join-Path $sessionDir "vst.h264"
    $vstTs = Join-Path $sessionDir "vst.ts.jsonl"
    $vstQpcTs = Join-Path $sessionDir "vst.qpc.ts.jsonl"
    $vstPresent = @(
        (Test-Path -LiteralPath $vst -PathType Leaf),
        (Test-Path -LiteralPath $vstTs -PathType Leaf),
        (Test-Path -LiteralPath $vstQpcTs -PathType Leaf)
    )
    $vstCount = @($vstPresent | Where-Object { $_ }).Count
    if ($vstCount -notin @(0, 3)) {
        throw "VST 资产不完整，必须同时存在 h264/wall-QPC sidecar：$sessionDir"
    }
    if ($vstCount -eq 3) {
        $exportArgs += @("--vst", $vst, "--vst-ts", $vstTs,
            "--vst-qpc-ts", $vstQpcTs)
    }
    Invoke-PythonStep "export" $exportArgs

    Invoke-PythonStep "catalog" @(
        (Join-Path $ProjectRoot "data_catalog.py"), $Name, "--write-manifest"
    )
    Write-Host "aligned: $aligned"
    Write-Host "hdf5:    $hdf5"
}

try {
    switch ($Action) {
        "start" { Start-EgoServices }
        "restart" { Stop-EgoServices; Start-EgoServices }
        "stop" { Stop-EgoServices }
        "status" { Show-Status }
        "check" { Invoke-Check }
        "doctor" {
            if (-not (Invoke-Doctor)) { exit 1 }
        }
        "pipeline" { Invoke-Pipeline $Session }
    }
    exit 0
} catch {
    Write-Host "[ego_ctl] $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
