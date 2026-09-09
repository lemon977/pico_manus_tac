param(
    [switch]$Elevated
)

$ErrorActionPreference = "Stop"
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$runDir = Join-Path $projectRoot ".run"
$logPath = Join-Path $runDir "pico_firewall_setup.log"
New-Item -ItemType Directory -Force -Path $runDir | Out-Null

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

if (-not (Test-IsAdministrator)) {
    if ($Elevated) {
        throw "The elevated firewall process does not have an administrator token."
    }

    $scriptPath = $MyInvocation.MyCommand.Path
    $arguments = @(
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-File", ('"' + $scriptPath + '"'),
        "-Elevated"
    )
    Write-Host "Requesting Windows administrator permission..." -ForegroundColor Yellow
    try {
        $process = Start-Process -FilePath "powershell.exe" -Verb RunAs `
            -ArgumentList $arguments -Wait -PassThru
    } catch {
        throw "Administrator permission was cancelled or unavailable: $($_.Exception.Message)"
    }
    if (Test-Path -LiteralPath $logPath -PathType Leaf) {
        Get-Content -LiteralPath $logPath -Encoding UTF8 | ForEach-Object { Write-Host $_ }
    }
    if ($process.ExitCode -ne 0) {
        throw "Firewall setup failed with exit code $($process.ExitCode). See $logPath"
    }
    exit 0
}

$rules = @(
    @{ Name = "PICO Ego Tracking TCP"; Protocol = "TCP"; Port = "63901" },
    @{ Name = "PICO Ego VST TCP"; Protocol = "TCP"; Port = "63902" },
    @{ Name = "PICO Ego Discovery UDP"; Protocol = "UDP"; Port = "29888" }
)

try {
    $results = foreach ($spec in $rules) {
        $existing = @(Get-NetFirewallRule -DisplayName $spec.Name -ErrorAction SilentlyContinue)
        if ($existing.Count -eq 0) {
            New-NetFirewallRule -DisplayName $spec.Name -Direction Inbound `
                -Action Allow -Enabled True -Protocol $spec.Protocol `
                -LocalPort $spec.Port -Profile Private `
                -RemoteAddress LocalSubnet | Out-Null
        }

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
        if ($matching.Count -eq 0) {
            throw "Firewall rule exists but does not match the required scope: $($spec.Name)"
        }
        [pscustomobject]@{
            Name = $spec.Name
            Protocol = $spec.Protocol
            LocalPort = $spec.Port
            Profile = "Private"
            RemoteAddress = "LocalSubnet"
            Status = "OK"
        }
    }

    $text = $results | Format-Table -AutoSize | Out-String
    @(
        "PICO firewall setup completed: $(Get-Date -Format o)"
        $text.TrimEnd()
    ) | Set-Content -LiteralPath $logPath -Encoding UTF8
    Get-Content -LiteralPath $logPath -Encoding UTF8 | ForEach-Object { Write-Host $_ }
    exit 0
} catch {
    @(
        "PICO firewall setup failed: $(Get-Date -Format o)"
        $_.Exception.Message
    ) | Set-Content -LiteralPath $logPath -Encoding UTF8
    Write-Error $_.Exception.Message
    exit 1
}

