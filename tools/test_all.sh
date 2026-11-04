#!/usr/bin/env sh
# Builds everything and runs every test: C unit tests (MSVC on Windows,
# cc elsewhere), the Python suite (simulator + C core through ctypes).
#   ./tools/test_all.sh
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"

./tools/build.sh

EXE=""
case "$(uname -s 2>/dev/null)" in MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;; esac

echo "== C unit tests"
for t in test_detect test_crypto test_auth; do
    ./build/$t$EXE | tail -n 1
done

echo "== Python tests"
PY=${PYTHON:-python}
command -v "$PY" >/dev/null 2>&1 || PY=python3
"$PY" -m unittest discover -s tests -v 2>&1 | tail -n 4
echo "== all tests passed"
