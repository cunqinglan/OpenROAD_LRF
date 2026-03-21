#!/bin/bash
set -uo pipefail

# =============================================================================
# run_test.sh — LRF Test Runner
#
# Usage:
#   ./run_test.sh [OPTIONS]
#
# Options:
#   -v NUM          Version number (auto-detect next if omitted)
#   -b BENCHMARK    Benchmark suite: mlcad (default) or iccad
#   -m "MESSAGE"    Tag/note for this experiment
#   -j NUM          Max parallel jobs (default: 3)
#   -d DESIGN,...   Comma-separated design list (default: ac97_top)
#   -t NUM          Thread count for OpenROAD (default: 10)
#   -c CMD          LR command: resize, buffering, precheck, lr, lrbuf
#   --dry-run       Show what would run without executing
#
# Examples:
#   ./run_test.sh                           # ac97_top, precheck
#   ./run_test.sh -d ac97_top,fpu -j 2     # specific designs
#   ./run_test.sh -c buffering             # test buffering mode
#   ./run_test.sh --dry-run                # preview commands
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
EXPERIMENT_DIR="${SCRIPT_DIR}/experiment"
PYTHON_SCRIPT="${SCRIPT_DIR}/test_lrf.py"
OPENROAD_EXE="${REPO_ROOT}/build/bin/openroad"
SINSHELL="/home/culan/sinshell.sh"

# Auto-detect: skip sinshell if already inside singularity
if [ -n "${SINGULARITY_CONTAINER:-}" ]; then
    SINSHELL=""
fi

BENCHMARK="mlcad"
LR_MODE="RAPIDLRHELPER"
LR_CALL=""
MAX_JOBS=3
THREAD_COUNT=10
VERSION=""
DESIGN_LIST="ac97_top"
DRY_RUN=0
TAG=""

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        -v)       VERSION="$2"; shift 2 ;;
        -b)       BENCHMARK="$2"; shift 2 ;;
        -m)       TAG="$2"; shift 2 ;;
        -j)       MAX_JOBS="$2"; shift 2 ;;
        -d)       DESIGN_LIST="$2"; shift 2 ;;
        -t)       THREAD_COUNT="$2"; shift 2 ;;
        -c)       LR_CALL="$2"; shift 2 ;;
        --dry-run)  DRY_RUN=1; shift ;;
        -h|--help)
            sed -n '3,20p' "$0" | sed 's/^# \?//'
            exit 0 ;;
        *)
            echo "Unknown option: $1"; exit 1 ;;
    esac
done

# --- Check openroad binary ---
if [ ! -x "$OPENROAD_EXE" ]; then
    echo "ERROR: openroad not found at $OPENROAD_EXE"
    echo "Run 'cmake --build build -j\$(nproc)' first."
    exit 1
fi

# --- Auto-detect version ---
if [ -z "$VERSION" ]; then
    mkdir -p "$EXPERIMENT_DIR"
    MAX_V=0
    shopt -s nullglob
    for d in "$EXPERIMENT_DIR"/v*/; do
        V_NUM=$(basename "$d" | sed 's/^v//')
        if [[ "$V_NUM" =~ ^[0-9]+$ ]] && [ "$V_NUM" -gt "$MAX_V" ]; then
            MAX_V=$V_NUM
        fi
    done
    shopt -u nullglob
    VERSION=$((MAX_V + 1))
    echo "Auto-detected version: v${VERSION}"
fi

LOG_DIR="${EXPERIMENT_DIR}/v${VERSION}"
mkdir -p "$LOG_DIR"

# --- Resolve design list ---
IFS=',' read -ra DESIGNS <<< "$DESIGN_LIST"

if [ ${#DESIGNS[@]} -eq 0 ]; then
    echo "ERROR: No designs to run."
    exit 1
fi

# --- Build optional --lr_call arg ---
LR_CALL_ARG=""
if [ -n "$LR_CALL" ]; then
    LR_CALL_ARG="--lr_call ${LR_CALL}"
fi

# --- Print config ---
echo "============================================"
echo "  LRF Test — v${VERSION}"
echo "============================================"
echo "  Benchmark:   ${BENCHMARK}"
echo "  LR mode:     ${LR_MODE}"
if [ -n "$LR_CALL" ]; then
echo "  LR command:  ${LR_CALL}"
fi
echo "  Designs:     ${DESIGNS[*]}"
echo "  Max jobs:    ${MAX_JOBS}"
echo "  Threads:     ${THREAD_COUNT}"
echo "  Log dir:     ${LOG_DIR}"
echo "  OpenROAD:    ${OPENROAD_EXE}"
if [ -n "$TAG" ]; then
echo "  Tag:         ${TAG}"
fi
echo "============================================"

if [ "$DRY_RUN" -eq 1 ]; then
    echo ""
    echo "[DRY RUN] Commands that would be executed:"
    for DESIGN in "${DESIGNS[@]}"; do
        echo "  ${OPENROAD_EXE} -exit -python ${PYTHON_SCRIPT} --design_name ${DESIGN} --benchmark ${BENCHMARK} --lr_mode ${LR_MODE} ${LR_CALL_ARG} --thread_count ${THREAD_COUNT}"
    done
    exit 0
fi

# --- Save experiment metadata ---
cat > "${LOG_DIR}/meta.txt" <<EOF
version: v${VERSION}
date: $(date '+%Y-%m-%d %H:%M:%S')
benchmark: ${BENCHMARK}
tag: ${TAG}
lr_mode: ${LR_MODE}
lr_command: ${LR_CALL:-precheck}
thread_count: ${THREAD_COUNT}
max_jobs: ${MAX_JOBS}
openroad: ${OPENROAD_EXE}
designs: ${DESIGNS[*]}
git_commit: $(cd "$REPO_ROOT" && git rev-parse --short HEAD 2>/dev/null || echo "unknown")
git_branch: $(cd "$REPO_ROOT" && git branch --show-current 2>/dev/null || echo "unknown")
EOF

# --- Run experiments ---
PIDS=()
for DESIGN in "${DESIGNS[@]}"; do
    # Wait if at max parallel jobs
    while [ $(jobs -rp | wc -l) -ge "$MAX_JOBS" ]; do
        sleep 5
    done

    LOG_FILE="${LOG_DIR}/${DESIGN}.log"
    echo "[$(date +'%H:%M:%S')] Starting: ${DESIGN} → ${LOG_FILE}"

    if [ -n "$SINSHELL" ]; then
        ${SINSHELL} "${OPENROAD_EXE} -exit -python ${PYTHON_SCRIPT} \
            --design_name ${DESIGN} \
            --benchmark ${BENCHMARK} \
            --lr_mode ${LR_MODE} \
            ${LR_CALL_ARG} \
            --thread_count ${THREAD_COUNT}" \
            > "$LOG_FILE" 2>&1 &
    else
        ${OPENROAD_EXE} -exit -python ${PYTHON_SCRIPT} \
            --design_name "$DESIGN" \
            --benchmark "$BENCHMARK" \
            --lr_mode "$LR_MODE" \
            ${LR_CALL_ARG} \
            --thread_count "$THREAD_COUNT" \
            > "$LOG_FILE" 2>&1 &
    fi
    PIDS+=("$!:$DESIGN")
done

# --- Wait and collect results ---
echo ""
echo "All tasks submitted. Waiting for completion..."
FAIL_COUNT=0
FAILED_DESIGNS=""
for entry in "${PIDS[@]}"; do
    pid="${entry%%:*}"
    name="${entry##*:}"
    if wait "$pid"; then
        echo "[OK]   ${name} (pid ${pid})"
    else
        EXIT_CODE=$?
        echo "[FAIL] ${name} (pid ${pid}, exit code ${EXIT_CODE})"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        FAILED_DESIGNS="${FAILED_DESIGNS} ${name}"
    fi
done

echo ""
if [ "$FAIL_COUNT" -gt 0 ]; then
    echo "ERROR: ${FAIL_COUNT} job(s) failed:${FAILED_DESIGNS}"
    exit 1
fi
echo "All jobs finished successfully. Logs in: ${LOG_DIR}"
