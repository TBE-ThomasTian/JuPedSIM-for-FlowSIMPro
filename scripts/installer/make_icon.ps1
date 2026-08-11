<#
.SYNOPSIS
    Draws the multi-resolution .ico the Inno Setup installer uses.

.DESCRIPTION
    A floor plan seen from above: walls, a door opening on the right, and the
    people inside as dots streaming towards it. Everything is drawn from scratch
    at each target resolution rather than scaled down from one large image - a
    badge or a fine line that has been through a 256->16 resample is mud.

    Drawn, not derived, so no artwork from another repository has to be copied
    in here to build the installer.

.EXAMPLE
    powershell -File scripts\installer\make_icon.ps1
    powershell -File scripts\installer\make_icon.ps1 -Palette paper
#>
param(
    [ValidateSet('blue', 'paper', 'dark')]
    [string]$Palette = 'blue',
    [string]$Output = (Join-Path $PSScriptRoot 'flowsimpro-evac-addon.ico')
)

Add-Type -AssemblyName System.Drawing
$ErrorActionPreference = 'Stop'

$sizes = @(16, 20, 24, 32, 40, 48, 64, 128, 256)

# exit: ISO 7010 escape-route green, the colour the exit signage in the buildings
# being simulated uses.
$palettes = @{
    blue  = @{ bg='F1F6F9'; grid='D6E3EA'; wall='1F4E63'; dot='2E7D9A'; exit='009646' }
    paper = @{ bg='F5F4F0'; grid='DEDCD4'; wall='2E3B42'; dot='2E3B42'; exit='009646' }
    dark  = @{ bg='2E3B42'; grid='3E4D57'; wall='E8EDEF'; dot='E8EDEF'; exit='35C46E' }
}

function C([string]$hex) {
    [System.Drawing.Color]::FromArgb(255,
        [Convert]::ToInt32($hex.Substring(0,2),16),
        [Convert]::ToInt32($hex.Substring(2,2),16),
        [Convert]::ToInt32($hex.Substring(4,2),16))
}

function New-PlanIcon([int]$s, [hashtable]$p) {
    $bmp = New-Object System.Drawing.Bitmap($s, $s, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $g.Clear((C $p.bg))

    # Drawing-paper grid. Skipped on the small sizes, where it is only noise.
    if ($s -ge 48) {
        $pen = New-Object System.Drawing.Pen((C $p.grid), [single]([Math]::Max(1, $s/128)))
        for ($i = 1; $i -lt 8; $i++) {
            $v = [single]($s * $i / 8)
            $g.DrawLine($pen, 0, $v, $s, $v)
            $g.DrawLine($pen, $v, 0, $v, $s)
        }
        $pen.Dispose()
    }

    $x0 = 0.17 * $s; $x1 = 0.83 * $s
    $y0 = 0.17 * $s; $y1 = 0.83 * $s
    $t  = [Math]::Max(1.6, 0.062 * $s)

    # Below 64 px, snap the walls to whole pixels. Leftover fractions are what
    # make one wall come out 1 px and the opposite one 2 px.
    if ($s -lt 64) {
        $t = [Math]::Max(2, [Math]::Round(0.062 * $s))
        $x0 = [Math]::Round($x0); $x1 = [Math]::Round($x1)
        $y0 = [Math]::Round($y0); $y1 = [Math]::Round($y1)
    }

    $gapHalf = 0.098 * $s
    $gapTop = 0.5 * $s - $gapHalf
    $gapBot = 0.5 * $s + $gapHalf

    # One open polyline running from one side of the opening all the way round to
    # the other, so the corners join cleanly and the opening stays a real gap.
    $path = New-Object System.Drawing.Drawing2D.GraphicsPath
    $pts = @(
        (New-Object System.Drawing.PointF([single]$x1, [single]$gapTop)),
        (New-Object System.Drawing.PointF([single]$x1, [single]$y0)),
        (New-Object System.Drawing.PointF([single]$x0, [single]$y0)),
        (New-Object System.Drawing.PointF([single]$x0, [single]$y1)),
        (New-Object System.Drawing.PointF([single]$x1, [single]$y1)),
        (New-Object System.Drawing.PointF([single]$x1, [single]$gapBot))
    )
    $path.AddLines([System.Drawing.PointF[]]$pts)
    $wallPen = New-Object System.Drawing.Pen((C $p.wall), [single]$t)
    $wallPen.LineJoin = [System.Drawing.Drawing2D.LineJoin]::Miter
    $wallPen.StartCap = [System.Drawing.Drawing2D.LineCap]::Flat
    $wallPen.EndCap = [System.Drawing.Drawing2D.LineCap]::Flat
    $g.DrawPath($wallPen, $path)
    $wallPen.Dispose(); $path.Dispose()

    # People, converging on the opening. Fewer and larger as the icon shrinks:
    # six dots at 16 px would be a smudge. Coordinates are fractions of the room,
    # values above 1.0 are outside it - those are the ones who are already out.
    if ($s -ge 48) {
        $inside = @(@(0.18,0.22), @(0.18,0.78), @(0.21,0.50), @(0.46,0.35), @(0.46,0.65), @(0.69,0.50))
        $out = @(@(1.01,0.50), @(1.16,0.50))
        $r = 0.042 * $s
    } elseif ($s -ge 32) {
        $inside = @(@(0.20,0.28), @(0.20,0.72), @(0.53,0.50))
        # Leading comma: @(@(a,b)) flattens to @(a,b), and the loop below would
        # then walk two scalars instead of one point.
        $out = ,@(1.06,0.50)
        $r = 0.058 * $s
    } else {
        $inside = @(@(0.26,0.32), @(0.26,0.68))
        $out = ,@(1.02,0.50)
        $r = 0.075 * $s
    }

    $w = $x1 - $x0; $h = $y1 - $y0
    $dotBrush = New-Object System.Drawing.SolidBrush((C $p.dot))
    $exitBrush = New-Object System.Drawing.SolidBrush((C $p.exit))
    foreach ($d in $inside) {
        $cx = $x0 + $d[0] * $w; $cy = $y0 + $d[1] * $h
        $g.FillEllipse($dotBrush, [single]($cx-$r), [single]($cy-$r), [single](2*$r), [single](2*$r))
    }
    foreach ($d in $out) {
        $cx = $x0 + $d[0] * $w; $cy = $y0 + $d[1] * $h
        $g.FillEllipse($exitBrush, [single]($cx-$r), [single]($cy-$r), [single](2*$r), [single](2*$r))
    }
    $dotBrush.Dispose(); $exitBrush.Dispose(); $g.Dispose()
    return $bmp
}

# 32-bit DIB (BITMAPINFOHEADER + bottom-up BGRA + AND mask), the format every
# Windows shell surface reads. PNG only for the big entries, as icon editors do.
function Get-DibBytes([System.Drawing.Bitmap]$b) {
    $w = $b.Width; $h = $b.Height
    $rect = New-Object System.Drawing.Rectangle(0, 0, $w, $h)
    $data = $b.LockBits($rect, [System.Drawing.Imaging.ImageLockMode]::ReadOnly,
                        [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $raw = New-Object byte[] ($data.Stride * $h)
    [System.Runtime.InteropServices.Marshal]::Copy($data.Scan0, $raw, 0, $raw.Length)
    $b.UnlockBits($data)

    $ms = New-Object System.IO.MemoryStream
    $bw = New-Object System.IO.BinaryWriter($ms)
    $bw.Write([uint32]40); $bw.Write([int32]$w); $bw.Write([int32]($h * 2))
    $bw.Write([uint16]1);  $bw.Write([uint16]32); $bw.Write([uint32]0)
    $bw.Write([uint32]($w * $h * 4))
    $bw.Write([int32]0); $bw.Write([int32]0); $bw.Write([uint32]0); $bw.Write([uint32]0)

    for ($y = $h - 1; $y -ge 0; $y--) { $bw.Write($raw, $y * $data.Stride, $w * 4) }

    $maskRow = [int][Math]::Floor(($w + 31) / 32) * 4
    $bw.Write((New-Object byte[] ($maskRow * $h)))

    $bw.Flush()
    # Leading comma again: without it PowerShell unrolls the byte[] into the
    # pipeline, the caller gets an Object[] of boxed bytes, and the file below
    # ends up silently truncated.
    return ,$ms.ToArray()
}

$p = $palettes[$Palette]
"palette     : {0}" -f $Palette

$entries = @()
foreach ($n in $sizes) {
    $b = New-PlanIcon $n $p
    if ($n -ge 128) {
        $ms = New-Object System.IO.MemoryStream
        $b.Save($ms, [System.Drawing.Imaging.ImageFormat]::Png)
        $bytes = $ms.ToArray()
        $kind = 'PNG'
    } else {
        $bytes = [byte[]](Get-DibBytes $b)
        $kind = 'DIB'
    }
    $b.Dispose()
    $entries += [pscustomobject]@{ Size = $n; Bytes = [byte[]]$bytes; Kind = $kind }
}

# Assembled by hand rather than through BinaryWriter, which does not reliably
# write a whole buffer here.
$headerLen = 6 + 16 * $entries.Count
$total = $headerLen
foreach ($e in $entries) { $total += $e.Bytes.Length }

$ico = New-Object byte[] $total
[System.Buffer]::BlockCopy([BitConverter]::GetBytes([uint16]0), 0, $ico, 0, 2)
[System.Buffer]::BlockCopy([BitConverter]::GetBytes([uint16]1), 0, $ico, 2, 2)
[System.Buffer]::BlockCopy([BitConverter]::GetBytes([uint16]$entries.Count), 0, $ico, 4, 2)

$offset = $headerLen
for ($i = 0; $i -lt $entries.Count; $i++) {
    $e = $entries[$i]
    $o = 6 + 16 * $i
    $dim = if ($e.Size -ge 256) { 0 } else { $e.Size }
    $ico[$o] = [byte]$dim; $ico[$o+1] = [byte]$dim; $ico[$o+2] = 0; $ico[$o+3] = 0
    [System.Buffer]::BlockCopy([BitConverter]::GetBytes([uint16]1),  0, $ico, $o+4,  2)
    [System.Buffer]::BlockCopy([BitConverter]::GetBytes([uint16]32), 0, $ico, $o+6,  2)
    [System.Buffer]::BlockCopy([BitConverter]::GetBytes([uint32]$e.Bytes.Length), 0, $ico, $o+8,  4)
    [System.Buffer]::BlockCopy([BitConverter]::GetBytes([uint32]$offset),         0, $ico, $o+12, 4)
    [System.Buffer]::BlockCopy($e.Bytes, 0, $ico, $offset, $e.Bytes.Length)
    $offset += $e.Bytes.Length
}
[System.IO.File]::WriteAllBytes($Output, $ico)

""
$entries | ForEach-Object { "  {0,3}x{1,-3} {2}  {3,7} bytes" -f $_.Size, $_.Size, $_.Kind, $_.Bytes.Length }
""
"wrote {0} ({1} bytes)" -f $Output, (Get-Item $Output).Length
