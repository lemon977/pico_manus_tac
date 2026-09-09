[CmdletBinding()]
param(
    [Parameter(Position = 0)][string]$Prefix,
    [Parameter(Position = 1)][string]$Indices,
    [switch]$Yes,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$Manager = Join-Path $ProjectRoot "batch_data_manager.py"
Set-Location -LiteralPath $ProjectRoot
$env:PYTHONUTF8 = "1"

$python = Get-Command python -ErrorAction SilentlyContinue
if ($null -eq $python) { throw "找不到 Python，请先运行 Windows 部署脚本。" }
if (-not (Test-Path -LiteralPath $Manager -PathType Leaf)) {
    throw "缺少批次管理程序: $Manager"
}

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host " 批次数据删除（保留原编号）" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "说明：指定条目会从正式数据中移除，但保存在 data\deleted 以便恢复。"
Write-Host "原始数据、触觉、aligned、HDF5 和复核资产会一起处理。"
Write-Host "其余条目保持原编号，允许出现编号空缺；后续采集从当前最大编号继续递增。"
Write-Host ""

if ([string]::IsNullOrWhiteSpace($Prefix)) {
    $Prefix = Read-Host "请输入批次前缀（例如 expert）"
}
if ([string]::IsNullOrWhiteSpace($Indices)) {
    $Indices = Read-Host "请输入要删除的编号（例如 2,5,7-9）"
}

& $python.Source $Manager --prefix $Prefix --delete $Indices `
    --project-root $ProjectRoot --dry-run
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
if ($DryRun) { exit 0 }

if (-not $Yes) {
    Write-Host ""
    $answer = Read-Host "确认按上面的方案执行？输入 Y 确认"
    if ($answer.Trim().ToUpperInvariant() -ne "Y") {
        Write-Host "已取消，数据没有变化。" -ForegroundColor Yellow
        exit 0
    }
}

& $python.Source $Manager --prefix $Prefix --delete $Indices `
    --project-root $ProjectRoot
exit $LASTEXITCODE
