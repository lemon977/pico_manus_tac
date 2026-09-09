function Split-BatchSessionName {
    [CmdletBinding()]
    param([Parameter(Mandatory = $true)][string]$Session)

    $match = [regex]::Match($Session, '^(.+)_([0-9]{3,4})$')
    if (-not $match.Success) { return $null }
    [pscustomobject]@{
        Prefix = $match.Groups[1].Value
        IndexText = $match.Groups[2].Value
        Index = [int]$match.Groups[2].Value
    }
}

function Get-BatchSessionName {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string]$Prefix,
        [Parameter(Mandatory = $true)][ValidateRange(1, 9999)][int]$Index
    )
    return '{0}_{1:D3}' -f $Prefix, $Index
}

function Get-SessionStoragePaths {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string]$ProjectRoot,
        [Parameter(Mandatory = $true)][string]$Session,
        [switch]$PreferExisting
    )

    $projectFull = [IO.Path]::GetFullPath($ProjectRoot)
    $dataRoot = Join-Path $projectFull 'data'
    $batch = Split-BatchSessionName -Session $Session
    $sessionBase = if ($null -ne $batch) {
        Join-Path (Join-Path (Join-Path $dataRoot 'sessions') $batch.Prefix) $batch.IndexText
    } else {
        Join-Path (Join-Path $dataRoot 'sessions') $Session
    }
    $taskFirst = [pscustomobject]@{
        Session = $Session
        Prefix = if ($null -ne $batch) { $batch.Prefix } else { $null }
        Index = if ($null -ne $batch) { $batch.Index } else { $null }
        IndexText = if ($null -ne $batch) { $batch.IndexText } else { $null }
        SessionRoot = $sessionBase
        Raw = Join-Path $sessionBase 'raw'
        Tactile = Join-Path $sessionBase 'raw'
        Aligned = Join-Path $sessionBase 'aligned.jsonl'
        Export = Join-Path $sessionBase 'dataset.hdf5'
        ReviewOverlay = Join-Path (Join-Path $sessionBase 'review') 'overlay.mp4'
        ReviewProbes = Join-Path (Join-Path $sessionBase 'review') 'probes'
        TaskFirst = $true
        Grouped = $true
    }
    if (-not $PreferExisting) { return $taskFirst }
    foreach ($path in @($taskFirst.SessionRoot, $taskFirst.Raw, $taskFirst.Aligned,
            $taskFirst.Export, $taskFirst.ReviewOverlay, $taskFirst.ReviewProbes)) {
        if (Test-Path -LiteralPath $path) { return $taskFirst }
    }

    # 兼容旧的 data/<type>/<prefix>/<index> 类型优先布局。
    if ($null -ne $batch) {
        $grouped = [pscustomobject]@{
            Session = $Session
            Prefix = $batch.Prefix
            Index = $batch.Index
            IndexText = $batch.IndexText
            Raw = Join-Path (Join-Path (Join-Path $dataRoot 'raw') $batch.Prefix) $batch.IndexText
            Tactile = Join-Path (Join-Path (Join-Path $dataRoot 'tactile_raw') $batch.Prefix) $batch.IndexText
            Aligned = Join-Path (Join-Path (Join-Path $dataRoot 'aligned') $batch.Prefix) ($batch.IndexText + '.jsonl')
            Export = Join-Path (Join-Path (Join-Path $dataRoot 'export') $batch.Prefix) ($batch.IndexText + '.hdf5')
            ReviewOverlay = Join-Path (Join-Path (Join-Path $dataRoot 'review') $batch.Prefix) ('overlay_' + $batch.IndexText + '.mp4')
            ReviewProbes = Join-Path (Join-Path (Join-Path $dataRoot 'review') $batch.Prefix) ('probes_' + $batch.IndexText)
            SessionRoot = $null
            TaskFirst = $false
            Grouped = $true
        }
        if (-not $PreferExisting) { return $grouped }
        foreach ($path in @($grouped.Raw, $grouped.Tactile, $grouped.Aligned,
                $grouped.Export, $grouped.ReviewOverlay, $grouped.ReviewProbes)) {
            if (Test-Path -LiteralPath $path) { return $grouped }
        }
    }

    # 兼容现有的 data/<type>/<prefix>_<index> 扁平数据。
    $legacyPrefix = $null
    $legacyIndex = $null
    $legacyIndexText = $null
    if ($null -ne $batch) {
        $legacyPrefix = $batch.Prefix
        $legacyIndex = $batch.Index
        $legacyIndexText = $batch.IndexText
    }
    return [pscustomobject]@{
        Session = $Session
        Prefix = $legacyPrefix
        Index = $legacyIndex
        IndexText = $legacyIndexText
        Raw = Join-Path (Join-Path $dataRoot 'raw') $Session
        Tactile = Join-Path (Join-Path $dataRoot 'tactile_raw') $Session
        Aligned = Join-Path (Join-Path $dataRoot 'aligned') ($Session + '.jsonl')
        Export = Join-Path (Join-Path $dataRoot 'export') ($Session + '_check.hdf5')
        ReviewOverlay = Join-Path (Join-Path $dataRoot 'review') ('overlay_' + $Session + '.mp4')
        ReviewProbes = Join-Path (Join-Path $dataRoot 'review') ('probes_' + $Session)
        SessionRoot = $null
        TaskFirst = $false
        Grouped = $false
    }
}

function Test-SessionAssetsExist {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string]$ProjectRoot,
        [Parameter(Mandatory = $true)][string]$Session
    )
    $paths = Get-SessionStoragePaths -ProjectRoot $ProjectRoot -Session $Session -PreferExisting
    foreach ($path in @($paths.SessionRoot, $paths.Raw, $paths.Tactile, $paths.Aligned, $paths.Export,
            $paths.ReviewOverlay, $paths.ReviewProbes)) {
        if ($null -ne $path -and (Test-Path -LiteralPath $path)) { return $true }
    }
    return $false
}

function Get-BatchIndices {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string]$ProjectRoot,
        [Parameter(Mandatory = $true)][string]$Prefix
    )
    $dataRoot = Join-Path ([IO.Path]::GetFullPath($ProjectRoot)) 'data'
    $indices = [Collections.Generic.HashSet[int]]::new()

    $sessionsBatch = Join-Path (Join-Path $dataRoot 'sessions') $Prefix
    Get-ChildItem -LiteralPath $sessionsBatch -Directory -ErrorAction SilentlyContinue |
        ForEach-Object {
            $value = 0
            if ($_.Name -match '^[0-9]{3,4}$' -and
                [int]::TryParse($_.Name, [ref]$value) -and $value -gt 0) {
                [void]$indices.Add($value)
            }
        }

    foreach ($type in @('raw', 'tactile_raw')) {
        $batchDir = Join-Path (Join-Path $dataRoot $type) $Prefix
        Get-ChildItem -LiteralPath $batchDir -Directory -ErrorAction SilentlyContinue |
            ForEach-Object {
                $value = 0
                if ($_.Name -match '^[0-9]{3,4}$' -and
                    [int]::TryParse($_.Name, [ref]$value) -and $value -gt 0) {
                    [void]$indices.Add($value)
                }
            }
        $root = Join-Path $dataRoot $type
        $legacyPattern = '^' + [regex]::Escape($Prefix) + '_([0-9]{3,4})$'
        Get-ChildItem -LiteralPath $root -Directory -ErrorAction SilentlyContinue |
            ForEach-Object {
                $match = [regex]::Match($_.Name, $legacyPattern)
                if ($match.Success) { [void]$indices.Add([int]$match.Groups[1].Value) }
            }
    }
    foreach ($spec in @(
        @{ Type = 'aligned'; Filter = '*.jsonl' },
        @{ Type = 'export'; Filter = '*.hdf5' }
    )) {
        $batchDir = Join-Path (Join-Path $dataRoot $spec.Type) $Prefix
        Get-ChildItem -LiteralPath $batchDir -Filter $spec.Filter -File `
            -ErrorAction SilentlyContinue | ForEach-Object {
                $value = 0
                if ($_.BaseName -match '^[0-9]{3,4}$' -and
                    [int]::TryParse($_.BaseName, [ref]$value) -and $value -gt 0) {
                    [void]$indices.Add($value)
                }
            }
    }
    $legacyAligned = Join-Path $dataRoot 'aligned'
    $legacyExport = Join-Path $dataRoot 'export'
    $legacyPattern = '^' + [regex]::Escape($Prefix) + '_([0-9]{3,4})$'
    Get-ChildItem -LiteralPath $legacyAligned -Filter '*.jsonl' -File `
        -ErrorAction SilentlyContinue | ForEach-Object {
            $match = [regex]::Match($_.BaseName, $legacyPattern)
            if ($match.Success) { [void]$indices.Add([int]$match.Groups[1].Value) }
        }
    $legacyExportPattern = '^' + [regex]::Escape($Prefix) + '_([0-9]{3,4})_check$'
    Get-ChildItem -LiteralPath $legacyExport -Filter '*_check.hdf5' -File `
        -ErrorAction SilentlyContinue | ForEach-Object {
            $match = [regex]::Match($_.BaseName, $legacyExportPattern)
            if ($match.Success) { [void]$indices.Add([int]$match.Groups[1].Value) }
        }
    return @($indices | Sort-Object)
}

function Get-LatestRawSessionName {
    [CmdletBinding()]
    param([Parameter(Mandatory = $true)][string]$ProjectRoot)
    $dataRoot = Join-Path ([IO.Path]::GetFullPath($ProjectRoot)) 'data'
    $sessionsRoot = Join-Path $dataRoot 'sessions'
    $latest = Get-ChildItem -LiteralPath $sessionsRoot -Filter 'pico.jsonl' -File -Recurse `
        -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($null -ne $latest -and $latest.Directory.Name -eq 'raw') {
        $sessionDir = $latest.Directory.Parent
        if ($sessionDir.Parent.FullName -ieq $sessionsRoot) { return $sessionDir.Name }
        if ($sessionDir.Parent.Parent.FullName -ieq $sessionsRoot) {
            return $sessionDir.Parent.Name + '_' + $sessionDir.Name
        }
    }
    $rawRoot = Join-Path $dataRoot 'raw'
    $latest = Get-ChildItem -LiteralPath $rawRoot -Filter 'pico.jsonl' -File -Recurse `
        -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($null -eq $latest) { return $null }
    $sessionDir = $latest.Directory
    if ($sessionDir.Parent.FullName -ieq $rawRoot) { return $sessionDir.Name }
    if ($sessionDir.Parent.Parent.FullName -ieq $rawRoot) {
        return $sessionDir.Parent.Name + '_' + $sessionDir.Name
    }
    return $null
}

function Get-SessionAssetSpecs {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string]$ProjectRoot,
        [Parameter(Mandatory = $true)][string]$Session
    )
    $paths = Get-SessionStoragePaths -ProjectRoot $ProjectRoot -Session $Session -PreferExisting
    if ($paths.TaskFirst) {
        return @([pscustomobject]@{ Source = $paths.SessionRoot; Target = 'session' })
    }
    @(
        [pscustomobject]@{ Source = $paths.Raw; Target = 'raw' },
        [pscustomobject]@{ Source = $paths.Tactile; Target = 'tactile_raw' },
        [pscustomobject]@{ Source = $paths.Aligned; Target = 'aligned.jsonl' },
        [pscustomobject]@{ Source = $paths.Export; Target = 'export.hdf5' },
        [pscustomobject]@{ Source = $paths.ReviewOverlay; Target = 'overlay.mp4' },
        [pscustomobject]@{ Source = $paths.ReviewProbes; Target = 'probes' }
    )
}

function Move-SessionAssetsToDirectory {
    <# 从正式目录移走 session；目标只允许位于 deleted/rejected，失败时倒序恢复。 #>
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string]$ProjectRoot,
        [Parameter(Mandatory = $true)][string]$Session,
        [Parameter(Mandatory = $true)][string]$Destination
    )

    if ([string]::IsNullOrWhiteSpace($Session) -or
        $Session.IndexOfAny([IO.Path]::GetInvalidFileNameChars()) -ge 0 -or
        $Session -ne [IO.Path]::GetFileName($Session)) {
        throw "不安全的 session 名称: $Session"
    }

    $projectFull = [IO.Path]::GetFullPath($ProjectRoot)
    $destinationFull = [IO.Path]::GetFullPath($Destination)
    $allowedRoots = @(
        [IO.Path]::GetFullPath((Join-Path $projectFull 'data\deleted')),
        [IO.Path]::GetFullPath((Join-Path $projectFull 'data\rejected'))
    )
    $allowed = $false
    foreach ($root in $allowedRoots) {
        if ($destinationFull.StartsWith($root + [IO.Path]::DirectorySeparatorChar,
                [StringComparison]::OrdinalIgnoreCase)) {
            $allowed = $true
            break
        }
    }
    if (-not $allowed) { throw "归档目标不安全: $destinationFull" }
    if (Test-Path -LiteralPath $destinationFull) {
        throw "归档目标已经存在，拒绝覆盖: $destinationFull"
    }

    New-Item -ItemType Directory -Path $destinationFull -Force | Out-Null
    $journal = @()
    try {
        foreach ($asset in @(Get-SessionAssetSpecs -ProjectRoot $projectFull -Session $Session)) {
            if (-not (Test-Path -LiteralPath $asset.Source)) { continue }
            $target = Join-Path $destinationFull $asset.Target
            Move-Item -LiteralPath $asset.Source -Destination $target
            $journal += [pscustomobject]@{
                Source = $asset.Source
                Target = $target
                Name = $asset.Target
            }
        }
        if ($journal.Count -eq 0) { throw "没有找到 session 的任何正式资产: $Session" }
    } catch {
        $originalError = $_
        for ($i = $journal.Count - 1; $i -ge 0; $i--) {
            $entry = $journal[$i]
            if ((Test-Path -LiteralPath $entry.Target) -and
                -not (Test-Path -LiteralPath $entry.Source)) {
                Move-Item -LiteralPath $entry.Target -Destination $entry.Source `
                    -ErrorAction SilentlyContinue
            }
        }
        throw $originalError
    }
    return @($journal | ForEach-Object { $_.Name })
}
