param(
    [Parameter(Mandatory = $true)][string]$Python,
    [Parameter(Mandatory = $true)][string]$ProjectRoot,
    [double]$RateHz = 60.0,
    [int]$TransientIncidentLimit = 3,
    [double]$TransientWindowSeconds = 5.0
)

$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $ProjectRoot
$env:PYTHONUTF8 = "1"
$Host.UI.RawUI.WindowTitle = "PICO tactile service - keep this window open"

$runDir = Join-Path $ProjectRoot ".run"
New-Item -ItemType Directory -Force -Path $runDir | Out-Null
$transcript = Join-Path $runDir "tactile_service.windows.log"

try {
    Start-Transcript -Path $transcript -Append | Out-Null
} catch {
    Write-Warning "无法启动 transcript；仍继续运行触觉服务: $($_.Exception.Message)"
}

try {
    Write-Host ""
    Write-Host "这是触觉常驻服务窗口。请按提示完成静止基线和左手按压配对。" -ForegroundColor Cyan
    Write-Host "配对成功后请保持此窗口开启；采集由另一个终端的 pico_record.py 控制。" -ForegroundColor Cyan
    & $Python -u (Join-Path $ProjectRoot "tactile_collector.py") `
        --service `
        --log-dir (Join-Path $ProjectRoot "data\sessions") `
        --rate-hz $RateHz `
        --transient-incident-limit $TransientIncidentLimit `
        --transient-window-seconds $TransientWindowSeconds
    $code = $LASTEXITCODE
} catch {
    Write-Error $_
    $code = 1
} finally {
    try { Stop-Transcript | Out-Null } catch { }
}

if ($code -ne 0) {
    Write-Host ""
    Write-Host "触觉服务已异常退出（exit=$code）。窗口将在 3 秒后关闭，主窗口会立即报告失败。" `
        -ForegroundColor Red
    Start-Sleep -Seconds 3
}
exit $code
