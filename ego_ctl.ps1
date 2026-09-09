# Windows 兼容入口；生命周期逻辑只维护在 scripts/ego_ctl.ps1。
$controller = Join-Path $PSScriptRoot "scripts\ego_ctl.ps1"
& $controller @args
exit $LASTEXITCODE
