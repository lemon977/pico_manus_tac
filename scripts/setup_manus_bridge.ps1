param(
    [string]$SdkRoot = $env:MANUS_SDK_WINDOWS,
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$bridgeDir = Join-Path $projectRoot "manus_ndjson_bridge"

if ([string]::IsNullOrWhiteSpace($SdkRoot)) {
    $SdkRoot = Join-Path $projectRoot "vendor\manus_sdk_windows"
}
if (-not (Test-Path -LiteralPath $SdkRoot -PathType Container)) {
    throw @"
未找到 MANUS Windows SDK: $SdkRoot
请从 MANUS Download Center 下载 Windows C++ SDK，解压到:
  $projectRoot\vendor\manus_sdk_windows
或设置 MANUS_SDK_WINDOWS 后重试。
"@
}
$SdkRoot = (Resolve-Path -LiteralPath $SdkRoot).Path

function Find-One([string[]]$Names, [string]$Kind) {
    foreach ($name in $Names) {
        $hit = Get-ChildItem -LiteralPath $SdkRoot -Recurse -File -Filter $name -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($null -ne $hit) { return $hit.FullName }
    }
    throw "MANUS Windows SDK 中找不到 $Kind ($($Names -join ', '))"
}

$header = Find-One @("ManusSDK.h") "ManusSDK.h"
$types = Find-One @("ManusSDKTypes.h") "ManusSDKTypes.h"
$initializers = Find-One @("ManusSDKTypeInitializers.h") "ManusSDKTypeInitializers.h"
$importLib = Find-One @("ManusSDK.lib", "ManusSDK_Integrated.lib") "导入库"
$dll = Find-One @("ManusSDK.dll", "ManusSDK_Integrated.dll") "运行时 DLL"

function Import-MsvcEnvironment {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) {
        return $false
    }

    $installationPath = & $vswhere -latest -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath | Select-Object -First 1
    if ([string]::IsNullOrWhiteSpace($installationPath)) {
        return $false
    }

    $vsDevCmd = Join-Path $installationPath "Common7\Tools\VsDevCmd.bat"
    if (-not (Test-Path -LiteralPath $vsDevCmd -PathType Leaf)) {
        return $false
    }

    $vsCommand = 'call "' + $vsDevCmd + '" -arch=x64 -host_arch=x64 >nul && set'
    $environment = & $env:ComSpec /d /s /c $vsCommand
    if ($LASTEXITCODE -ne 0) {
        return $false
    }
    foreach ($line in $environment) {
        $separator = $line.IndexOf("=")
        if ($separator -le 0) { continue }
        $name = $line.Substring(0, $separator)
        $value = $line.Substring($separator + 1)
        Set-Item -LiteralPath "Env:$name" -Value $value
    }
    return $true
}

$cl = Get-Command cl.exe -ErrorAction SilentlyContinue
if ($null -eq $cl) {
    [void](Import-MsvcEnvironment)
    $cl = Get-Command cl.exe -ErrorAction SilentlyContinue
}
if ($null -eq $cl) {
    throw @"
找不到 cl.exe。请安装 Visual Studio 2022 Build Tools，勾选“使用 C++ 的桌面开发”，
或在“x64 Native Tools Command Prompt for VS 2022”中运行本脚本。
"@
}

$includeDir = Join-Path $bridgeDir "ManusSDK\include"
$libDir = Join-Path $bridgeDir "ManusSDK\lib\windows"
$binDir = Join-Path $bridgeDir "ManusSDK\bin"
New-Item -ItemType Directory -Force -Path $includeDir, $libDir, $binDir | Out-Null
Copy-Item -LiteralPath $header -Destination (Join-Path $includeDir "ManusSDK.h") -Force
Copy-Item -LiteralPath $types -Destination (Join-Path $includeDir "ManusSDKTypes.h") -Force
Copy-Item -LiteralPath $initializers -Destination (Join-Path $includeDir "ManusSDKTypeInitializers.h") -Force
Copy-Item -LiteralPath $importLib -Destination (Join-Path $libDir (Split-Path $importLib -Leaf)) -Force
$runtimeDlls = Get-ChildItem -LiteralPath (Split-Path $dll -Parent) -Filter "*.dll" -File
foreach ($runtimeDll in $runtimeDlls) {
    Copy-Item -LiteralPath $runtimeDll.FullName -Destination (Join-Path $binDir $runtimeDll.Name) -Force
    Copy-Item -LiteralPath $runtimeDll.FullName -Destination (Join-Path $bridgeDir $runtimeDll.Name) -Force
}

$out = Join-Path $bridgeDir "manus_ndjson_bridge.exe"
$source = Join-Path $bridgeDir "manus_ndjson_bridge.cpp"
$libName = Split-Path $importLib -Leaf
Push-Location $bridgeDir
try {
    & $cl.Source /nologo /EHsc /std:c++17 /utf-8 /O2 /MD `
        "/I$includeDir" $source `
        /link "/LIBPATH:$libDir" $libName "/OUT:$out"
    if ($LASTEXITCODE -ne 0) {
        throw "MANUS bridge 编译失败"
    }
} finally {
    Pop-Location
}

Write-Host "[setup] MANUS Windows bridge: $out" -ForegroundColor Green
Write-Host "[setup] runtime DLL: $(Join-Path $bridgeDir (Split-Path $dll -Leaf))"
