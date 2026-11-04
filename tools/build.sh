#!/usr/bin/env sh
# Builds the host C unit tests, the footprint benchmark and the shared
# library the Python simulator loads through ctypes.
#
#   Windows (Git Bash): uses MSVC through tools/msvc.bat
#   Linux / macOS:      uses $CC (default cc)
#
# Outputs go to build/.
set -eu

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
mkdir -p build

FW=firmware/telltale
CORE="$FW/telltale_detect.c $FW/telltale_sha256.c $FW/telltale_auth.c \
$FW/telltale_motion.c $FW/telltale_ina219.c $FW/telltale_persist.c"
TESTS="test_detect test_crypto test_auth bench"

is_windows() {
    case "$(uname -s 2>/dev/null)" in
        MINGW*|MSYS*|CYGWIN*) return 0 ;;
        *) return 1 ;;
    esac
}

if is_windows; then
    win_core=$(echo "$CORE" | sed 's#/#\\\\#g')
    for t in $TESTS; do
        echo "[build] $t.exe (MSVC)"
        cmd //c "tools\\msvc.bat /O2 /Fobuild\\ /Fe:build\\$t.exe tests\\c\\$t.c $win_core" >build/$t.log 2>&1 \
            || { cat build/$t.log; exit 1; }
    done
    echo "[build] telltale_core.dll (MSVC)"
    cmd //c "tools\\msvc.bat /O2 /LD /Fobuild\\ /Fe:build\\telltale_core.dll $win_core /link /DEF:tools\\telltale_core.def" \
        >build/telltale_core.log 2>&1 || { cat build/telltale_core.log; exit 1; }
    # MSVC prints file names on success; surface warnings only.
    if grep -qi "warning" build/*.log; then grep -hi "warning" build/*.log; fi
else
    CC=${CC:-cc}
    CFLAGS="-std=c99 -O2 -Wall -Wextra -pedantic -Wshadow -Wconversion"
    for t in $TESTS; do
        echo "[build] $t ($CC)"
        $CC $CFLAGS -o build/$t tests/c/$t.c $CORE -lm
    done
    case "$(uname -s)" in
        Darwin) LIB=build/libtelltale_core.dylib ;;
        *) LIB=build/libtelltale_core.so ;;
    esac
    echo "[build] $LIB ($CC)"
    $CC $CFLAGS -shared -fPIC -o "$LIB" $CORE -lm
fi
echo "[build] done"
