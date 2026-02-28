param(
    [switch]$WithDxf2wkt,
    [string]$OutputDir = "build/standalone-win",
    [string]$PythonExe = "python"
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
if ([System.IO.Path]::IsPathRooted($OutputDir)) {
    $OutputRoot = $OutputDir
} else {
    $OutputRoot = Join-Path $RepoRoot $OutputDir
}

& $PythonExe --version *> $null
if ($LASTEXITCODE -ne 0) {
    throw "error: '$PythonExe' not found."
}

& $PythonExe -c "import PyInstaller" *> $null
if ($LASTEXITCODE -ne 0) {
    throw "error: PyInstaller is missing. Install with: $PythonExe -m pip install pyinstaller"
}

New-Item -ItemType Directory -Force -Path (Join-Path $OutputRoot "bin") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $OutputRoot "spec") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $OutputRoot "work") | Out-Null

function Build-OneFile {
    param(
        [string]$BinaryName,
        [string]$ScriptRelativePath
    )

    $scriptPath = Join-Path $RepoRoot $ScriptRelativePath
    & $PythonExe -m PyInstaller `
        --clean `
        --noconfirm `
        --onefile `
        --name $BinaryName `
        --distpath (Join-Path $OutputRoot "bin") `
        --specpath (Join-Path $OutputRoot "spec") `
        --workpath (Join-Path $OutputRoot "work/$BinaryName") `
        $scriptPath

    if ($LASTEXITCODE -ne 0) {
        throw "error: PyInstaller build failed for $BinaryName."
    }
}

Write-Host "Building evac-uncertainty-cli.exe..."
Build-OneFile -BinaryName "evac-uncertainty-cli" -ScriptRelativePath "scripts/evac_uncertainty_cli.py"

if ($WithDxf2wkt) {
    Write-Host "Checking python dependencies for dxf2wkt..."
    & $PythonExe -c "import ezdxf, geopandas, matplotlib, numpy, rich, shapely, typer" *> $null
    if ($LASTEXITCODE -ne 0) {
        throw "error: Missing dependencies for dxf2wkt packaging."
    }

    Write-Host "Building dxf2wkt.exe..."
    Build-OneFile -BinaryName "dxf2wkt" -ScriptRelativePath "scripts/dxf2wkt.py"
}

Write-Host ""
Write-Host "Done. Binaries are in:"
Write-Host "  $(Join-Path $OutputRoot 'bin')"
