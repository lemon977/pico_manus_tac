[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [Alias("Session")]
    [string]$TaskPrefix,

    [ValidateRange(10, 3600)]
    [int]$ReadyTimeoutSeconds = 300,

    [switch]$NoVideo,
    [switch]$NoExport,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$Controller = Join-Path $ProjectRoot "ego_ctl.ps1"
$Recorder = Join-Path $ProjectRoot "pico_record.py"
$RejectedRoot = Join-Path $ProjectRoot "data\rejected"
$RunRoot = Join-Path $ProjectRoot ".run"
$WorkflowLock = Join-Path $RunRoot "collect_windows.pid"
$script:WorkflowFailed = $false
$script:ServicesStarted = $false
$script:ControllerExitCode = 0
$script:WorkflowLockOwned = $false
$script:WorkflowLockStream = $null
$script:PendingFailureArchive = $null
$RestartRecordExitCode = 10
$QuitRecordExitCode = 11
$ReadyWaitExitCode = 12
$SensorFaultExitCode = 13
$SessionDataTools = Join-Path $PSScriptRoot "session_data_tools.ps1"
if (-not (Test-Path -LiteralPath $SessionDataTools -PathType Leaf)) {
    throw "缺少批次数据工具: $SessionDataTools"
}
. $SessionDataTools

Set-Location -LiteralPath $ProjectRoot
$env:PYTHONUTF8 = "1"
$machinePath = [Environment]::GetEnvironmentVariable("Path", "Machine")
$userPath = [Environment]::GetEnvironmentVariable("Path", "User")
$env:Path = "$env:Path;$machinePath;$userPath"

function Resolve-Python {
    if ($env:EGO_PYTHON_EXE) {
        if (-not (Test-Path -LiteralPath $env:EGO_PYTHON_EXE -PathType Leaf)) {
            throw "EGO_PYTHON_EXE 不存在: $env:EGO_PYTHON_EXE"
        }
        return (Resolve-Path -LiteralPath $env:EGO_PYTHON_EXE).Path
    }
    $command = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $command) {
        throw "找不到 Python。请先运行 scripts\setup_windows.ps1。"
    }
    return $command.Source
}

function Invoke-Controller([string[]]$Arguments) {
    # 不把 stdout 接到 PowerShell pipeline。触觉交互窗口是长期子进程；若通过
    # ``| ForEach-Object`` 捕获输出，它会继承管道句柄，使主窗口一直等不到 EOF。
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Controller @Arguments
    $script:ControllerExitCode = [int]$LASTEXITCODE
}

function Test-SessionName([string]$Name) {
    if ([string]::IsNullOrWhiteSpace($Name)) { return $false }
    # 预留自动后缀 ``_001`` 及未来更大序号，生成后的 session 仍不超过128字符。
    if ($Name -ne $Name.Trim() -or $Name.Length -gt 120) { return $false }
    if ($Name -in @('.', '..') -or $Name.EndsWith('.') -or $Name.EndsWith(' ')) {
        return $false
    }
    $hasWhitespace = @(
        $Name.ToCharArray() | Where-Object { [char]::IsWhiteSpace($_) }
    ).Count -gt 0
    $hasInvalidCharacter = $Name.IndexOfAny(
        [IO.Path]::GetInvalidFileNameChars()
    ) -ge 0
    if ($hasWhitespace -or $hasInvalidCharacter) {
        return $false
    }
    $stem = $Name.Split(".")[0].ToUpperInvariant()
    $reserved = @("CON", "PRN", "AUX", "NUL")
    $reserved += "CLOCK" + [char]36
    $reserved += 1..9 | ForEach-Object { "COM$_" }
    $reserved += 1..9 | ForEach-Object { "LPT$_" }
    return ($stem -notin $reserved)
}

function Test-SessionAvailable([string]$Name) {
    return -not (Test-SessionAssetsExist -ProjectRoot $ProjectRoot -Session $Name)
}

function Read-TaskPrefix([string]$Initial) {
    $candidate = $Initial
    while ($true) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            $defaultPrefix = Get-Date -Format "yyyyMMdd"
            $entered = Read-Host "请输入本批任务前缀（示例 S01_pick；直接回车使用 $defaultPrefix）"
            $candidate = if ([string]::IsNullOrWhiteSpace($entered)) { $defaultPrefix } else { $entered }
        }
        if (-not (Test-SessionName $candidate)) {
            Write-Host "[!] 前缀无效：不能含空格或 Windows 文件名非法字符。" -ForegroundColor Red
            if (-not [string]::IsNullOrWhiteSpace($Initial)) {
                throw "无效任务前缀: $candidate"
            }
            $candidate = $null
            continue
        }
        return $candidate
    }
}

function Get-NextSession([string]$Prefix) {
    $indices = @(Get-BatchIndices -ProjectRoot $ProjectRoot -Prefix $Prefix)
    $maximum = if ($indices.Count -gt 0) {
        [int]($indices | Measure-Object -Maximum).Maximum
    } else { 0 }
    $next = $maximum + 1
    while ($true) {
        $candidate = Get-BatchSessionName -Prefix $Prefix -Index $next
        if (Test-SessionAvailable $candidate) { return $candidate }
        $next++
    }
}

function Read-OperatorKey([string]$Prompt, [switch]$AllowQuit, [switch]$AllowUndo) {
    Write-Host $Prompt -NoNewline -ForegroundColor Yellow
    while ($true) {
        $key = [Console]::ReadKey($true)
        if ($key.Key -eq [ConsoleKey]::Enter) {
            Write-Host ""
            return "ENTER"
        }
        if ($AllowQuit -and $key.Key -eq [ConsoleKey]::Q) {
            Write-Host "Q"
            return "Q"
        }
        if ($AllowUndo -and $key.Key -eq [ConsoleKey]::H) {
            Write-Host "H"
            return "H"
        }
    }
}

function Move-SessionToRejected([string]$Session, [string]$Reason) {
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss_fff"
    $batch = Split-BatchSessionName -Session $Session
    $archiveGroup = if ($null -ne $batch) { $batch.Prefix } else { "standalone" }
    $destination = Join-Path (Join-Path $RejectedRoot $archiveGroup) `
        "${Session}__${Reason}_$stamp"
    $moved = @(Move-SessionAssetsToDirectory -ProjectRoot $ProjectRoot `
        -Session $Session -Destination $destination)

    [ordered]@{
        schema = "pico_retry_archive_v1"
        session = $Session
        reason = $Reason
        archived_at = [DateTimeOffset]::Now.ToString("o")
        assets = $moved
    } | ConvertTo-Json -Depth 3 | Set-Content `
        -LiteralPath (Join-Path $destination "retry.meta.json") -Encoding UTF8
    return $destination
}

function Move-MarkedFailuresToRejected([string]$Prefix) {
    $batchRoot = Join-Path (Join-Path $ProjectRoot "data\sessions") $Prefix
    if (-not (Test-Path -LiteralPath $batchRoot -PathType Container)) { return }
    foreach ($indexDir in @(Get-ChildItem -LiteralPath $batchRoot -Directory `
            -ErrorAction SilentlyContinue)) {
        if ($indexDir.Name -notmatch '^[0-9]{3,4}$') { continue }
        $marker = Join-Path (Join-Path $indexDir.FullName "raw") "capture.failure.json"
        if (-not (Test-Path -LiteralPath $marker -PathType Leaf)) { continue }
        $session = "$Prefix`_$($indexDir.Name)"
        $archive = Move-SessionToRejected $session "marked_failure"
        Write-Host "[清理] 已将带失败标记的 $session 移出正式目录：$archive" `
            -ForegroundColor Yellow
    }
}

function Write-WorkflowFailureMarker([string]$Session, [string]$Reason) {
    $paths = Get-SessionStoragePaths -ProjectRoot $ProjectRoot `
        -Session $Session -PreferExisting
    $raw = $paths.Raw
    if (-not (Test-Path -LiteralPath $raw -PathType Container)) { return }
    $target = Join-Path $raw "capture.failure.json"
    if (Test-Path -LiteralPath $target -PathType Leaf) { return }
    $partial = "$target.partial"
    [ordered]@{
        schema = "capture_failure_v1"
        session = $Session
        outcome = "invalid_workflow_exit"
        stage = "collection_workflow"
        reason = $Reason
        detected_at = [DateTimeOffset]::Now.ToString("o")
        policy = "stop all routes; never export failed attempt; archive before retry"
    } | ConvertTo-Json -Depth 3 | Set-Content -LiteralPath $partial -Encoding UTF8
    Move-Item -LiteralPath $partial -Destination $target -Force
}

function Assert-NoIncompleteOfficialSessions([string]$Prefix) {
    $batchRoot = Join-Path (Join-Path $ProjectRoot "data\sessions") $Prefix
    if (-not (Test-Path -LiteralPath $batchRoot -PathType Container)) { return }
    $incomplete = @()
    foreach ($indexDir in @(Get-ChildItem -LiteralPath $batchRoot -Directory `
            -ErrorAction SilentlyContinue)) {
        if ($indexDir.Name -notmatch '^[0-9]{3,4}$') { continue }
        $raw = Join-Path $indexDir.FullName "raw"
        if (-not (Test-Path -LiteralPath $raw -PathType Container)) { continue }
        $dataset = Join-Path $indexDir.FullName "dataset.hdf5"
        $manifest = Join-Path $indexDir.FullName "manifest.json"
        if (-not (Test-Path -LiteralPath $dataset -PathType Leaf) -or
            -not (Test-Path -LiteralPath $manifest -PathType Leaf)) {
            $incomplete += "$Prefix`_$($indexDir.Name)"
        }
    }
    if ($incomplete.Count -gt 0) {
        throw ("正式目录存在尚未完成 pipeline 的会话：{0}。先运行 " +
            ".\ego_ctl.ps1 pipeline <会话名> 修复，不能跳号继续采集。" -f
            ($incomplete -join ", "))
    }
}

function Show-Preparation {
    Clear-Host
    Write-Host "============================================================" -ForegroundColor Cyan
    Write-Host " PICO + MANUS + 双手触觉 一键采集" -ForegroundColor Cyan
    Write-Host "============================================================" -ForegroundColor Cyan
    Write-Host "开始前确认："
    Write-Host "  1. PICO、两只手柄、MANUS dongle/手套、两只触觉设备均已连接。"
    Write-Host "  2. PICO App 已打开 Send + Head + Controller。"
    if (-not $NoVideo) {
        Write-Host "  3. PICO VST 视频推流已开启。"
    }
    Write-Host "  4. 触觉配对窗口弹出后，按提示完成松手基线和左手按压。"
    Write-Host ""
}

$PythonExe = Resolve-Python
if (-not (Test-Path -LiteralPath $Controller -PathType Leaf)) {
    throw "缺少控制脚本: $Controller"
}
if (-not (Test-Path -LiteralPath $Recorder -PathType Leaf)) {
    throw "缺少录制脚本: $Recorder"
}

Show-Preparation

if ($DryRun) {
    $preview = if ([string]::IsNullOrWhiteSpace($TaskPrefix)) { "<交互输入>" } else { $TaskPrefix }
    Write-Host "DRY RUN PASS" -ForegroundColor Green
    Write-Host "project: $ProjectRoot"
    Write-Host "python:  $PythonExe"
    Write-Host "task prefix: $preview"
    if (-not [string]::IsNullOrWhiteSpace($TaskPrefix)) {
        if (-not (Test-SessionName $TaskPrefix)) { throw "无效任务前缀: $TaskPrefix" }
        Write-Host "next session: $(Get-NextSession $TaskPrefix)"
    }
    Write-Host "flow: start services -> Enter start/stop -> pipeline/export -> repeat"
    exit 0
}

$TaskPrefix = Read-TaskPrefix $TaskPrefix
Write-Host "[*] 本批任务前缀：$TaskPrefix；序号将自动递增。" -ForegroundColor Cyan

New-Item -ItemType Directory -Path $RunRoot -Force | Out-Null
try {
    # Hold an exclusive OS file handle for the whole workflow. Unlike a
    # check-then-write PID file, this cannot race when two windows start.
    $script:WorkflowLockStream = [IO.File]::Open(
        $WorkflowLock,
        [IO.FileMode]::OpenOrCreate,
        [IO.FileAccess]::ReadWrite,
        [IO.FileShare]::None
    )
    $script:WorkflowLockStream.SetLength(0)
    $lockBytes = [Text.Encoding]::ASCII.GetBytes([string]$PID)
    $script:WorkflowLockStream.Write($lockBytes, 0, $lockBytes.Length)
    $script:WorkflowLockStream.Flush($true)
    $script:WorkflowLockOwned = $true
} catch {
    throw "另一个一键采集窗口仍在运行，或采集锁无法独占：$WorkflowLock"
}

try {
    Move-MarkedFailuresToRejected $TaskPrefix
    if (-not $NoExport) {
        Assert-NoIncompleteOfficialSessions $TaskPrefix
    }
    Write-Host "[*] 正在进行部署自检并启动 PICO 持续发现、接收、MANUS 和触觉服务……" -ForegroundColor Cyan
    $startArgs = @("start")
    Invoke-Controller $startArgs
    $startCode = $script:ControllerExitCode
    if ($startCode -ne 0) {
        throw "服务启动失败 (exit=$startCode)"
    }
    $script:ServicesStarted = $true

    $endBatch = $false
    while (-not $endBatch) {
        $currentSession = Get-NextSession $TaskPrefix
        Write-Host ""
        Write-Host "============================================================" -ForegroundColor Green
        Write-Host " 自动生成本条名称：$currentSession" -ForegroundColor Green
        Write-Host "============================================================" -ForegroundColor Green
        Write-Host "控制说明：Enter 开始/停止；录制中 H 重采本条；待机时 H 撤销上一条。"
        Write-Host "          录制中 Q 停止、导出并结束本批；待机时 Q 直接结束本批。"
        Write-Host "防呆机制：开录前缺设备会等待稳定；录制中掉线会停录归档，并重采同一编号。"
        $idleChoice = Read-OperatorKey `
            "[待机] Enter 开始 $currentSession；H 撤销上一条；Q 结束本批：" `
            -AllowQuit -AllowUndo
        if ($idleChoice -eq "Q") {
            $endBatch = $true
            continue
        }
        if ($idleChoice -eq "H") {
            $indices = @(Get-BatchIndices -ProjectRoot $ProjectRoot -Prefix $TaskPrefix)
            if ($indices.Count -eq 0) {
                Write-Host "[H] 当前批次没有上一条可撤销。" -ForegroundColor Yellow
            } else {
                $lastIndex = [int]($indices | Measure-Object -Maximum).Maximum
                $previousSession = Get-BatchSessionName -Prefix $TaskPrefix -Index $lastIndex
                $archive = Move-SessionToRejected $previousSession "paused_undo"
                Write-Host "[H] 上一条 $previousSession 已从正式数据中移除。" `
                    -ForegroundColor Yellow
                Write-Host "    可恢复归档：$archive" -ForegroundColor DarkYellow
            }
            continue
        }

        $retryCurrentSession = $true
        while ($retryCurrentSession) {
            $recordArgs = @($Recorder, "start", $currentSession,
                "--ready-timeout", [string]$ReadyTimeoutSeconds,
                "--ready-stable-seconds", "2",
                "--record-fault-grace-seconds", "1.5",
                "--operator-control")
            if ($NoVideo) { $recordArgs += "--no-vst-recording" }
            & $PythonExe @recordArgs
            $recordCode = $LASTEXITCODE

            if ($recordCode -eq $RestartRecordExitCode) {
                $archive = Move-SessionToRejected $currentSession "recording_restart"
                Write-Host "[H] 本次误采已归档（可恢复）：$archive" -ForegroundColor Yellow
                Write-Host "[*] 正在重新开始同一条：$currentSession" -ForegroundColor Cyan
                continue
            }

            if ($recordCode -eq $ReadyWaitExitCode) {
                Write-Host "[等待未就绪] 尚未创建本条数据，编号仍为 $currentSession。" `
                    -ForegroundColor Yellow
                $waitChoice = Read-OperatorKey `
                    "[等待未就绪] Enter 再次等待全部传感器稳定；Q 结束本批：" `
                    -AllowQuit
                if ($waitChoice -eq "Q") {
                    $endBatch = $true
                    $retryCurrentSession = $false
                }
                continue
            }

            if ($recordCode -eq $SensorFaultExitCode) {
                $archive = Move-SessionToRejected $currentSession "sensor_dropout"
                Write-Host "[传感器故障] 本次数据禁止导出，已移出正式数据目录。" `
                    -ForegroundColor Red
                Write-Host "    失败归档：$archive" -ForegroundColor DarkYellow
                Write-Host "    修复设备后仍会重采同一编号：$currentSession" `
                    -ForegroundColor Cyan
                $faultChoice = Read-OperatorKey `
                    "[恢复等待] Enter 等待全路连续稳定并重采；Q 结束本批：" `
                    -AllowQuit
                if ($faultChoice -eq "Q") {
                    $endBatch = $true
                    $retryCurrentSession = $false
                }
                continue
            }

            $quitAfterExport = ($recordCode -eq $QuitRecordExitCode)
            $recordSucceeded = ($recordCode -eq 0 -or $quitAfterExport)
            if (-not $recordSucceeded) {
                $script:WorkflowFailed = $true
                $script:PendingFailureArchive = $currentSession
                Write-WorkflowFailureMarker $currentSession `
                    "录制进程异常退出 exit=$recordCode"
                Write-Host "[!] 本条录制未通过完整性门禁：$currentSession (exit=$recordCode)" `
                    -ForegroundColor Red
                Write-Host "[!] 结果未知，立即结束本批；安全停服后再隔离本条，禁止跳到下一编号。" `
                    -ForegroundColor Red
                throw "录制事务异常退出 (session=$currentSession exit=$recordCode)"
            } elseif (-not $NoExport) {
                Write-Host "[*] 原始数据已封存，开始自动质检、对齐并导出 HDF5……" -ForegroundColor Cyan
                Invoke-Controller @("pipeline", $currentSession)
                $pipelineCode = $script:ControllerExitCode
                if ($pipelineCode -ne 0) {
                    $script:WorkflowFailed = $true
                    Write-Host "[!] 导出失败，但原始数据已经保留，不会丢失。" -ForegroundColor Red
                    throw "pipeline 失败 (session=$currentSession exit=$pipelineCode)；禁止跳到下一编号"
                } else {
                    $paths = Get-SessionStoragePaths -ProjectRoot $ProjectRoot `
                        -Session $currentSession -PreferExisting
                    $hdf5 = $paths.Export
                    Write-Host "[OK] 本条采集及导出完成：$hdf5" -ForegroundColor Green
                }
            } else {
                Write-Host "[OK] 原始数据已封存；本次按 -NoExport 跳过导出。" -ForegroundColor Green
            }

            if ($quitAfterExport) {
                $endBatch = $true
            }
            $retryCurrentSession = $false
        }
    }
} catch {
    $script:WorkflowFailed = $true
    Write-Host "[!] 一键采集流程失败：$($_.Exception.Message)" -ForegroundColor Red
} finally {
    if ($script:ServicesStarted) {
        Write-Host "[*] 正在安全关闭采集服务……" -ForegroundColor Cyan
        Invoke-Controller @("stop")
        $stopCode = $script:ControllerExitCode
        if ($stopCode -ne 0) {
            $script:WorkflowFailed = $true
            Write-Host "[!] 服务关闭未完全成功，请运行 ego_ctl.ps1 stop。" -ForegroundColor Red
        }
    }
    if ($null -ne $script:PendingFailureArchive -and $stopCode -eq 0 -and
        (Test-SessionAssetsExist -ProjectRoot $ProjectRoot `
            -Session $script:PendingFailureArchive)) {
        try {
            $archive = Move-SessionToRejected $script:PendingFailureArchive "unexpected_exit"
            Write-Host "[!] 未知失败数据已隔离到：$archive" -ForegroundColor Yellow
        } catch {
            $script:WorkflowFailed = $true
            Write-Host "[!] 失败数据隔离失败，禁止继续采集：$($_.Exception.Message)" `
                -ForegroundColor Red
        }
    }
    if ($null -ne $script:WorkflowLockStream) {
        try { $script:WorkflowLockStream.Dispose() } catch { }
        $script:WorkflowLockStream = $null
    }
    if ($script:WorkflowLockOwned -and (Test-Path -LiteralPath $WorkflowLock)) {
        Remove-Item -LiteralPath $WorkflowLock -Force -ErrorAction SilentlyContinue
    }
}

if ($script:WorkflowFailed) { exit 1 }
Write-Host "[OK] 全部采集任务完成。" -ForegroundColor Green
exit 0
