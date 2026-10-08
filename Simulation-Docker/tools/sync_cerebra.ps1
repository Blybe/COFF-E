<#
.SYNOPSIS
    Copies Cerebra Studio data (maps, waypoints, paths, zones, ...) between
    this laptop's Cerebra Studio and the shared cerebra-data/ folder in the
    COFF-E repository.

.DESCRIPTION
    Cerebra Studio keeps one JSON file per item in
      %APPDATA%\Avular\Cerebra Studio\Mission Planner\<type>\{id}.json
    cerebra-data/ mirrors that layout, so the files can be shared with Git
    and read by the simulation without Cerebra Studio installed.

    Export (default): Cerebra Studio -> cerebra-data/   (then git commit + push)
    Import:           cerebra-data/  -> Cerebra Studio  (after git pull; close Studio first)

    Existing files are only overwritten when their content differs. Nothing
    is deleted unless -Prune is given.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File Simulation-Docker\tools\sync_cerebra.ps1
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File Simulation-Docker\tools\sync_cerebra.ps1 -Direction Import
#>
param(
    [ValidateSet('Export', 'Import')]
    [string]$Direction = 'Export',
    # Also remove files from the target that no longer exist in the source.
    [switch]$Prune,
    [string]$StudioDir = (Join-Path $env:APPDATA 'Avular\Cerebra Studio\Mission Planner'),
    [string]$RepoDir = (Join-Path $PSScriptRoot '..\..\cerebra-data')
)

$ErrorActionPreference = 'Stop'
$types = 'maps', 'waypoints', 'paths', 'typedpolygons', 'locations', 'jobs'

if (-not (Test-Path $StudioDir)) {
    if ($Direction -eq 'Export') { throw "Cerebra Studio data not found at '$StudioDir'." }
    New-Item -ItemType Directory -Path $StudioDir | Out-Null
}
$RepoDir = [System.IO.Path]::GetFullPath($RepoDir)
if ($Direction -eq 'Export') { $src = $StudioDir; $dst = $RepoDir } else { $src = $RepoDir; $dst = $StudioDir }

Write-Host "$Direction  $src  ->  $dst"
$changed = 0
foreach ($t in $types) {
    $s = Join-Path $src $t
    $d = Join-Path $dst $t
    New-Item -ItemType Directory -Force -Path $d | Out-Null
    $srcFiles = @(if (Test-Path $s) { Get-ChildItem -LiteralPath $s -Filter '*.json' -File })
    foreach ($f in $srcFiles) {
        $target = Join-Path $d $f.Name
        $label = $f.Name
        try { $label = (Get-Content -LiteralPath $f.FullName -Raw | ConvertFrom-Json).name } catch { }
        if (Test-Path -LiteralPath $target) {
            if ((Get-FileHash -LiteralPath $target).Hash -eq (Get-FileHash -LiteralPath $f.FullName).Hash) { continue }
            Write-Host ("  updated  {0,-14} {1}" -f $t, $label)
        } else {
            Write-Host ("  new      {0,-14} {1}" -f $t, $label)
        }
        Copy-Item -LiteralPath $f.FullName -Destination $target -Force
        $changed++
    }
    if ($Prune) {
        $names = $srcFiles | ForEach-Object Name
        foreach ($f in @(Get-ChildItem -LiteralPath $d -Filter '*.json' -File)) {
            if ($names -notcontains $f.Name) {
                Write-Host ("  removed  {0,-14} {1}" -f $t, $f.Name)
                Remove-Item -LiteralPath $f.FullName
                $changed++
            }
        }
    }
}

Write-Host "$changed file(s) changed."
if ($Direction -eq 'Export' -and $changed -gt 0) {
    Write-Host "Share them with:  git add cerebra-data; git commit -m 'Update Cerebra data'; git push"
}
if ($Direction -eq 'Import' -and $changed -gt 0) {
    Write-Host "Restart Cerebra Studio to see the imported items."
}
