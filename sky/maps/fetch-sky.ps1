#Requires -Version 7
<#
.SYNOPSIS
    Downloads one of NASA's Deep Star Maps 2020 into sky/maps/ and verifies its SHA-256.

.DESCRIPTION
    The maps come from the NASA Goddard Scientific Visualization Studio, entry 4851
    (https://svs.gsfc.nasa.gov/4851). They are OpenEXR files, too large to keep in git, so this
    script fetches them on demand. See sky/maps/README.md for what the maps contain, their
    geometry, and the credit line that any published video must carry.

    The script downloads to a temporary name ending in ".part" and renames the file only after
    the download has finished and the checksum has matched. When the file is already present
    and its checksum matches the one on record, the script downloads nothing.

    Checksums are recorded in checksums.sha256, next to this script, in the format that
    sha256sum writes. When no checksum is on record for the chosen file, the script prints the
    hash it computed and succeeds with a warning.

    The script needs PowerShell 7 and curl.exe, and nothing else.

.PARAMETER Product
    "starmap" is the full map: the Milky Way background from Gaia DR2 plus the bright stars from
    Hipparcos-2 and Tycho-2. "milkyway" is the same map without the Hipparcos and Tycho stars.

.PARAMETER Coordinates
    "galactic" centres the map on galactic longitude 0 with the galactic plane along the middle
    row. "celestial" centres it on right ascension 0h with the celestial equator along the middle
    row.

.PARAMETER Size
    Width of the map in pixels: 4k is 4096 x 2048, 8k is 8192 x 4096, 16k is 16384 x 8192,
    32k is 32768 x 16384, 64k is 65536 x 32768. The 32k and 64k sizes need -Force.

.PARAMETER Force
    Allows the 32k and 64k sizes, which are between 1.2 GB and 4.1 GB each.

.EXAMPLE
    pwsh sky/maps/fetch-sky.ps1
    Fetches starmap_2020_8k_gal.exr, the default.

.EXAMPLE
    pwsh sky/maps/fetch-sky.ps1 -Product milkyway -Coordinates celestial -Size 4k
#>
[CmdletBinding()]
param(
    [ValidateSet('starmap', 'milkyway')]
    [string] $Product = 'starmap',

    [ValidateSet('celestial', 'galactic')]
    [string] $Coordinates = 'galactic',

    [ValidateSet('4k', '8k', '16k', '32k', '64k')]
    [string] $Size = '8k',

    [switch] $Force
)

$ErrorActionPreference = 'Stop'

# Every file NASA offers for these two products, with the byte count its server reported on
# 2026-09-27. The file names were read from the links on https://svs.gsfc.nasa.gov/4851.
$BaseUrl = 'https://svs.gsfc.nasa.gov/vis/a000000/a004800/a004851'
$ExpectedBytes = @{
    'starmap_2020_4k.exr'       = 35997085
    'starmap_2020_8k.exr'       = 130530278
    'starmap_2020_16k.exr'      = 443485587
    'starmap_2020_32k.exr'      = 1535359333
    'starmap_2020_64k.exr'      = 4034745301
    'starmap_2020_4k_gal.exr'   = 40550793
    'starmap_2020_8k_gal.exr'   = 160735772
    'starmap_2020_16k_gal.exr'  = 384152397
    'starmap_2020_32k_gal.exr'  = 1278747829
    'starmap_2020_64k_gal.exr'  = 3286663034
    'milkyway_2020_4k.exr'      = 36436668
    'milkyway_2020_8k.exr'      = 137307727
    'milkyway_2020_16k.exr'     = 434041232
    'milkyway_2020_32k.exr'     = 1509917679
    'milkyway_2020_64k.exr'     = 3951844673
    'milkyway_2020_4k_gal.exr'  = 34771211
    'milkyway_2020_8k_gal.exr'  = 125076308
    'milkyway_2020_16k_gal.exr' = 373794299
    'milkyway_2020_32k_gal.exr' = 1251925430
    'milkyway_2020_64k_gal.exr' = 3206374253
}

function Stop-WithMessage([string] $Message) {
    [Console]::Error.WriteLine("fetch-sky: $Message")
    exit 1
}

function Format-Bytes([long] $Bytes) {
    if ($Bytes -ge 1GB) { return '{0:N1} GB' -f ($Bytes / 1GB) }
    return '{0:N0} MB' -f ($Bytes / 1MB)
}

function Get-Sha256([string] $Path) {
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

try {
    $suffix = if ($Coordinates -eq 'galactic') { '_gal' } else { '' }
    $name = "${Product}_2020_${Size}${suffix}.exr"
    $url = "$BaseUrl/$name"
    $bytes = [long] $ExpectedBytes[$name]

    if ($Size -in @('32k', '64k') -and -not $Force) {
        Stop-WithMessage ("$name is $(Format-Bytes $bytes) ($bytes bytes). " +
            'Run the script again with -Force to download a file this large.')
    }

    if (-not (Get-Command curl.exe -ErrorAction SilentlyContinue)) {
        Stop-WithMessage 'curl.exe was not found on PATH, and this script needs curl.exe to download.'
    }

    $dir = $PSScriptRoot
    $target = Join-Path $dir $name
    $partial = "$target.part"
    $checksumFile = Join-Path $dir 'checksums.sha256'

    # Read the recorded checksum, if there is one. Lines look like "<hex>  <file name>".
    $recorded = $null
    if (Test-Path -LiteralPath $checksumFile) {
        foreach ($line in Get-Content -LiteralPath $checksumFile) {
            if ($line -match '^\s*([0-9a-fA-F]{64})\s+\*?(\S+)\s*$' -and $Matches[2] -eq $name) {
                $recorded = $Matches[1].ToLowerInvariant()
                break
            }
        }
    }

    # A file that is already present and matches its checksum needs no download.
    if (Test-Path -LiteralPath $target) {
        Write-Host "Checking the existing $name ..."
        $existing = Get-Sha256 $target
        if ($recorded -and $existing -eq $recorded) {
            Write-Host "$name is already present and its SHA-256 matches the record. Nothing to do."
            exit 0
        }
        if (-not $recorded) {
            Write-Warning ("$name is already present, but checksums.sha256 has no checksum for it. " +
                "Its SHA-256 is $existing. The file was kept as it is and was not verified.")
            exit 0
        }
        Write-Warning "$name is present but its SHA-256 does not match the record, so it will be downloaded again."
    }

    Write-Host "Downloading $name ($(Format-Bytes $bytes)) from $url"
    if (Test-Path -LiteralPath $partial) { Remove-Item -LiteralPath $partial -Force }
    & curl.exe --fail --location --retry 3 --progress-bar --output $partial $url
    if ($LASTEXITCODE -ne 0) {
        if (Test-Path -LiteralPath $partial) { Remove-Item -LiteralPath $partial -Force }
        Stop-WithMessage "The download of $url failed (curl exit code $LASTEXITCODE)."
    }

    $got = (Get-Item -LiteralPath $partial).Length
    if ($got -ne $bytes) {
        Remove-Item -LiteralPath $partial -Force
        Stop-WithMessage "The download of $name gave $got bytes, but the server reported $bytes bytes in 2026-09. The partial file was deleted."
    }

    Write-Host 'Computing the SHA-256 ...'
    $hash = Get-Sha256 $partial
    if ($recorded) {
        if ($hash -ne $recorded) {
            Remove-Item -LiteralPath $partial -Force
            Stop-WithMessage ("The SHA-256 of the downloaded $name is $hash, but checksums.sha256 records " +
                "$recorded. The download was deleted. NASA may have replaced the file; check " +
                'https://svs.gsfc.nasa.gov/4851 before changing the record.')
        }
        Move-Item -LiteralPath $partial -Destination $target -Force
        Write-Host "Saved $target. Its SHA-256 matches the record."
    }
    else {
        Move-Item -LiteralPath $partial -Destination $target -Force
        Write-Warning ("Saved $target, but checksums.sha256 has no checksum for $name, so the file " +
            "was not verified. Its SHA-256 is $hash. To record it, add this line to checksums.sha256:")
        Write-Host "$hash  $name"
    }
    exit 0
}
catch {
    Stop-WithMessage "An unexpected error stopped the script: $($_.Exception.Message)"
}
