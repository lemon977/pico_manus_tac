param(
    [switch]$InstallFFmpeg,
    [switch]$ConfigureFirewall
)

$ErrorActionPreference = "Stop"
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location -LiteralPath $projectRoot
$env:PYTHONUTF8 = "1"

function Require-Command([string]$Name) {
    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($null -eq $command) {
        throw "找不到命令: $Name"
    }
    return $command.Source
}

$python = if ($env:EGO_PYTHON_EXE) {
    (Resolve-Path -LiteralPath $env:EGO_PYTHON_EXE).Path
} else {
    Require-Command "python"
}

Write-Host "[setup] project=$projectRoot"
Write-Host "[setup] python=$python"
& $python -m pip install --upgrade -r (Join-Path $projectRoot "requirements-windows.txt")
if ($LASTEXITCODE -ne 0) {
    throw "Python 依赖安装失败"
}

if ($InstallFFmpeg -and -not (Get-Command ffmpeg -ErrorAction SilentlyContinue)) {
    $winget = Require-Command "winget"
    & $winget install --id Gyan.FFmpeg --exact `
        --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) {
        throw "FFmpeg 安装失败"
    }
    Write-Host "[setup] FFmpeg 已安装；若当前终端仍找不到 ffmpeg，请新开 PowerShell。"
}

$machinePath = [Environment]::GetEnvironmentVariable("Path", "Machine")
$userPath = [Environment]::GetEnvironmentVariable("Path", "User")
$env:Path = "$env:Path;$machinePath;$userPath"

if ($ConfigureFirewall) {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass `
        -File (Join-Path $PSScriptRoot "configure_pico_firewall.ps1")
    if ($LASTEXITCODE -ne 0) {
        throw "PICO 防火墙配置失败"
    }
}

foreach ($dir in @(".run", "data\sessions")) {
    New-Item -ItemType Directory -Force -Path (Join-Path $projectRoot $dir) | Out-Null
}

& (Join-Path $PSScriptRoot "pico_discovery_autostart.ps1") ensure

& $python (Join-Path $projectRoot "tactile_pairing.py") --check-config
if ($LASTEXITCODE -ne 0) {
    throw "触觉配置校验失败"
}

Write-Host ""
Write-Host "[setup] Windows 通用依赖部署完成。" -ForegroundColor Green
Write-Host "下一步运行: powershell -ExecutionPolicy Bypass -File .\ego_ctl.ps1 doctor"
Write-Host "MANUS 若显示缺失，请先按 docs\WINDOWS_DEPLOY_ZH.md 放入 Windows SDK，再运行 setup_manus_bridge.ps1。"
