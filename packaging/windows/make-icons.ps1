# Renders the Windows package's logos from the app icon, assets/icons/black_hole_lab_icon.svg,
# into packaging/windows/Assets, where make-msix.ps1 takes them from.
#
# Run by hand after the drawing changes, and commit the PNGs it writes: the release workflow
# packages the committed PNGs and never runs Inkscape, as convert_svgs.ps1 in assets/images is
# run by hand for the same reason.
#
# Needs Inkscape (verified with 1.4.4). Usage, from anywhere:
#     pwsh packaging/windows/make-icons.ps1
#
# The names carry the qualifiers Windows chooses an image by: scale-N for the display scaling
# (100 = 100 %), targetsize-N for an exact pixel size, as the taskbar and Start menu ask for, and
# altform-unplated for the same drawn without the coloured plate Windows otherwise puts behind a
# small logo. The manifest names each logo without its qualifiers; make-msix.ps1 indexes the
# variants into resources.pri, from which Windows picks.

$ErrorActionPreference = 'Stop'

$inkscape = 'C:\Program Files\Inkscape\bin\inkscape.com'
$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$svg = Join-Path $root 'assets\icons\black_hole_lab_icon.svg'
$out = Join-Path $PSScriptRoot 'Assets'

if (-not (Test-Path -LiteralPath $inkscape)) {
    throw "Inkscape was not found at $inkscape."
}
New-Item -ItemType Directory -Force $out | Out-Null

# Every file: its name, and its width and height in pixels. All the logos are square.
$renders = @(
    @('Square44x44Logo.scale-100.png', 44),
    @('Square44x44Logo.scale-200.png', 88),
    @('Square44x44Logo.scale-400.png', 176),
    @('Square150x150Logo.scale-100.png', 150),
    @('Square150x150Logo.scale-200.png', 300),
    @('Square150x150Logo.scale-400.png', 600),
    @('StoreLogo.scale-100.png', 50),
    @('StoreLogo.scale-200.png', 100),
    @('StoreLogo.scale-400.png', 200)
)
foreach ($size in 16, 24, 32, 48, 256) {
    $renders += , @("Square44x44Logo.targetsize-$size.png", $size)
    $renders += , @("Square44x44Logo.targetsize-${size}_altform-unplated.png", $size)
}

foreach ($render in $renders) {
    $name, $pixels = $render
    $file = Join-Path $out $name
    & $inkscape $svg --export-type=png "--export-width=$pixels" "--export-height=$pixels" `
        "--export-filename=$file" 2>$null
    if ($LASTEXITCODE -ne 0) { throw "Inkscape failed on $name." }
    Write-Host "$name ($pixels px)"
}

# The program's own icon, which build.rs embeds in black-hole-lab.exe and Explorer shows for the
# file: the five unplated target sizes in one .ico. An .ico is a six-byte header, a sixteen-byte
# directory entry for each image, and then the images, which may be PNG files as they stand.
$sizes = 16, 24, 32, 48, 256
$images = foreach ($size in $sizes) {
    , [IO.File]::ReadAllBytes((Join-Path $out "Square44x44Logo.targetsize-${size}_altform-unplated.png"))
}
$ico = [IO.MemoryStream]::new()
$writer = [IO.BinaryWriter]::new($ico)
$writer.Write([uint16]0); $writer.Write([uint16]1); $writer.Write([uint16]$sizes.Count)
$offset = 6 + 16 * $sizes.Count
for ($i = 0; $i -lt $sizes.Count; $i++) {
    # Width and height, where 0 stands for 256; no palette; reserved; one plane; 32 bits a pixel.
    $side = [byte]($sizes[$i] % 256)
    $writer.Write($side); $writer.Write($side); $writer.Write([byte]0); $writer.Write([byte]0)
    $writer.Write([uint16]1); $writer.Write([uint16]32)
    $writer.Write([uint32]$images[$i].Length); $writer.Write([uint32]$offset)
    $offset += $images[$i].Length
}
foreach ($image in $images) { $writer.Write($image) }
[IO.File]::WriteAllBytes((Join-Path $PSScriptRoot 'black-hole-lab.ico'), $ico.ToArray())
Write-Host "black-hole-lab.ico ($($sizes -join ', ') px)"
