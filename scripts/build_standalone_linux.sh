#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${ROOT_DIR}/build/standalone"
WITH_DXF2WKT=0
PYTHON_BIN="${PYTHON_BIN:-python3}"
PYINSTALLER_BIN="${PYINSTALLER_BIN:-pyinstaller}"

usage() {
    cat <<'EOF'
Build standalone Linux executables for JuPedSim helper scripts.

Usage:
  scripts/build_standalone_linux.sh [--with-dxf2wkt] [--output-dir DIR]

Options:
  --with-dxf2wkt     Also build dxf2wkt (requires heavy geo/python deps).
  --output-dir DIR   Output root directory (default: build/standalone).
  -h, --help         Show this help.

Environment:
  PYINSTALLER_BIN    Override pyinstaller executable (default: pyinstaller)
  PYTHON_BIN         Override python executable (default: python3)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-dxf2wkt)
            WITH_DXF2WKT=1
            shift
            ;;
        --output-dir)
            if [[ $# -lt 2 ]]; then
                echo "error: --output-dir needs a value" >&2
                exit 1
            fi
            OUT_DIR="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown argument: $1" >&2
            usage
            exit 1
            ;;
    esac
done

if ! command -v "${PYINSTALLER_BIN}" >/dev/null 2>&1; then
    echo "error: '${PYINSTALLER_BIN}' not found." >&2
    echo "Install it first, e.g.:" >&2
    echo "  ${PYTHON_BIN} -m pip install pyinstaller" >&2
    exit 1
fi

mkdir -p "${OUT_DIR}/bin" "${OUT_DIR}/spec" "${OUT_DIR}/work"

build_onefile() {
    local binary_name="$1"
    local script_path="$2"

    "${PYINSTALLER_BIN}" \
        --clean \
        --noconfirm \
        --onefile \
        --name "${binary_name}" \
        --distpath "${OUT_DIR}/bin" \
        --specpath "${OUT_DIR}/spec" \
        --workpath "${OUT_DIR}/work/${binary_name}" \
        "${ROOT_DIR}/${script_path}"
}

echo "Building evac-uncertainty-cli..."
build_onefile "evac-uncertainty-cli" "scripts/evac_uncertainty_cli.py"

if [[ "${WITH_DXF2WKT}" -eq 1 ]]; then
    echo "Checking python dependencies for dxf2wkt..."
    if ! "${PYTHON_BIN}" -c "import ezdxf, geopandas, matplotlib, numpy, rich, shapely, typer" >/dev/null 2>&1; then
        echo "error: Missing dependencies for dxf2wkt packaging." >&2
        echo "Install required packages, then rerun with --with-dxf2wkt." >&2
        exit 1
    fi

    echo "Building dxf2wkt..."
    build_onefile "dxf2wkt" "scripts/dxf2wkt.py"
fi

echo
echo "Done. Binaries are in:"
echo "  ${OUT_DIR}/bin"
