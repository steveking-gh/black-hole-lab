# Packs a built copy of Black Hole Lab into an MSIX package for the Microsoft Store.
#
# The release workflow runs this on the Windows runner after it has made the zip, on the same
# folder; it runs the same way on a developer's machine with the Windows SDK installed. The
# package is not signed: the Store signs it after review. To try it on this machine without a
# signature, turn on Developer Mode and register the unpacked layout that -Layout keeps:
#     Add-AppxPackage -Register <layout>\AppxManifest.xml
#
# Usage:
#     pwsh packaging/windows/make-msix.ps1 -Source <built folder> -Version 1.0.294.0 `
#         -Output black-hole-lab.msix [-Layout <folder to keep the layout in>]
#
# -Source is the folder the workflow zips: black-hole-lab.exe with the sky tools beside it, the
# maps and demos folders, the licence and notices. -Version has four parts and the last must be
# 0, as the Store requires.

param(
    [Parameter(Mandatory)] [string] $Source,
    [Parameter(Mandatory)] [string] $Version,
    [Parameter(Mandatory)] [string] $Output,
    [string] $Layout
)

$ErrorActionPreference = 'Stop'

if ($Version -notmatch '^\d+\.\d+\.\d+\.0$') {
    throw "The version $Version is not four numbers ending in .0, which the Store requires."
}
if (-not (Test-Path -LiteralPath (Join-Path $Source 'black-hole-lab.exe'))) {
    throw "$Source holds no black-hole-lab.exe."
}

# The newest Windows SDK's tools for this machine's architecture.
$kits = 'C:\Program Files (x86)\Windows Kits\10\bin'
$bin = Get-ChildItem -LiteralPath $kits -Directory |
    Where-Object { $_.Name -match '^10\.' -and (Test-Path (Join-Path $_.FullName 'x64\makeappx.exe')) } |
    Sort-Object { [version] $_.Name } -Descending |
    Select-Object -First 1
if (-not $bin) { throw "No Windows SDK with makeappx.exe was found under $kits." }
$makeappx = Join-Path $bin.FullName 'x64\makeappx.exe'
$makepri = Join-Path $bin.FullName 'x64\makepri.exe'

# The layout: a copy of the built folder, the logos, and the manifest with the version in it.
if (-not $Layout) {
    $Layout = Join-Path ([IO.Path]::GetTempPath()) "black-hole-lab-msix-$PID"
}
if (Test-Path -LiteralPath $Layout) { Remove-Item -LiteralPath $Layout -Recurse -Force }
New-Item -ItemType Directory -Force $Layout | Out-Null
Copy-Item -Path (Join-Path $Source '*') -Destination $Layout -Recurse
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'Assets') -Destination $Layout -Recurse
$manifest = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot 'AppxManifest.xml')
Set-Content -LiteralPath (Join-Path $Layout 'AppxManifest.xml') -Value ($manifest -replace '\{VERSION\}', $Version) -NoNewline

# resources.pri: the index of the logos' scale and target-size variants, by which Windows picks
# the image for each place a logo is shown. The configuration is made fresh each time, outside
# the layout, so that it is not packed.
$work = Join-Path ([IO.Path]::GetTempPath()) "black-hole-lab-pri-$PID"
New-Item -ItemType Directory -Force $work | Out-Null
$config = Join-Path $work 'priconfig.xml'
& $makepri createconfig /cf $config /dq en-US /pv 10.0.0 /o | Out-Null
if ($LASTEXITCODE -ne 0) { throw "makepri createconfig failed ($LASTEXITCODE)." }
# The default configuration splits each scale into a resources file of its own, for resource
# packages in a bundle; a single package keeps every variant in one resources.pri.
[xml] $xml = Get-Content -Raw -LiteralPath $config
$packaging = $xml.SelectSingleNode('//packaging')
if ($packaging) { [void] $packaging.ParentNode.RemoveChild($packaging) }
$xml.Save($config)
& $makepri new /pr $Layout /cf $config /mn (Join-Path $Layout 'AppxManifest.xml') `
    /of (Join-Path $Layout 'resources.pri') /o | Out-Null
if ($LASTEXITCODE -ne 0) { throw "makepri new failed ($LASTEXITCODE)." }
Remove-Item -LiteralPath $work -Recurse -Force

& $makeappx pack /d $Layout /p $Output /o
if ($LASTEXITCODE -ne 0) { throw "makeappx pack failed ($LASTEXITCODE)." }
Write-Host "Packed $Output (version $Version)."
