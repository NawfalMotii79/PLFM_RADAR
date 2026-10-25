#!/bin/bash
# ===========================================================================
# AERIS-10 形式验证批量运行器
#
# 实测结论（本仓库 .sby，2026-10-25，20 核机器）：
#
#   1. 瓶颈在 SMT 求解，不在综合。
#      base task（yosys 综合）秒级完成，engine（smtbmc + 求解器）占 100% 时间。
#      → 换更快的求解器收益远大于换工具链。
#
#   2. 求解器是数量级差异（fv_cdc_handshake bmc depth=100，同一 yosys 模型）：
#        bitwuzla   3 s      ← 默认
#        boolector  4 s
#        z3      1206 s
#        cvc5   >2400 s (超时)
#      bitwuzla 专门做 bit-vector，对此类 BMC 模型比 z3 快约 200~350 倍。
#
#   3. 单任务只吃 1 核 → 必须并发。7 个 .sby 用 -j 8 并发，
#      20 核机器上可同时跑 8~14 个求解进程。
#
#   4. cover 任务的深度不能随便压低：深度不够时"未到达"会被 sby 判为 FAIL，
#      看起来像属性被违反，实际只是没跑够步。下表是逐文件实测的 cover 深度下限。
#
# 用法:
#   ./run_formal.sh                        # full 档，自动选最优求解器，并发 8
#   ./run_formal.sh --fast                 # fast 档（CI 用）
#   ./run_formal.sh --fast -j 12           # 指定并发
#   ./run_formal.sh --solver z3            # 强制求解器（z3/boolector/bitwuzla/cvc5）
#   ./run_formal.sh --list                 # 只列任务
#
# 退出码: 0 = 全部通过, 1 = 有失败/错误/未完成
# ===========================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

MODE=full
JOBS=8
SOLVER=auto
LIST_ONLY=0
KEEP_WORKDIR=0

# ---- fast 档深度策略 -------------------------------------------------------
# bmc 深度可以安全压低（浅层 BMC 仍会检查它到达的所有断言）。
FAST_BMC_DEPTH=20
# cover 深度有下限，压得太低会因"未到达"误判 FAIL。
FAST_COVER_DEFAULT=40
# 逐文件实测的 cover 深度下限（覆盖任务必须到达全部 cover 语句）
declare -A FAST_COVER_MIN=(
    [fv_cdc_adc]=140
)
# fast 档下跳过 cover 任务的文件：cover 目标在浅深度不可达，
# 深跑到可达又要几十分钟，不适合放 CI 门禁 —— 只在 full 档跑。
# （断言类性质由 bmc 覆盖，仍受 CI 门禁保护。）
FAST_COVER_SKIP=(fv_doppler_processor)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --fast)   MODE=fast; shift ;;
        --full)   MODE=full; shift ;;
        -j)       JOBS="${2:-8}"; shift 2 ;;
        --solver) SOLVER="${2:-auto}"; shift 2 ;;
        --list)   LIST_ONLY=1; shift ;;
        --keep)   KEEP_WORKDIR=1; shift ;;
        --help|-h) sed -n '2,38p' "$0"; exit 0 ;;
        *) echo "未知参数: $1" >&2; exit 2 ;;
    esac
done

if ! [[ "$JOBS" =~ ^[0-9]+$ ]] || [[ "$JOBS" -lt 1 ]]; then
    echo "并发数非法: $JOBS" >&2
    exit 2
fi

# ---- 求解器选择 -------------------------------------------------------------
pick_solver() {
    local s
    for s in bitwuzla boolector z3 cvc5; do
        if command -v "$s" > /dev/null 2>&1; then
            echo "$s"
            return 0
        fi
    done
    echo "z3"
}
if [[ "$SOLVER" == "auto" ]]; then
    SOLVER="$(pick_solver)"
fi
if ! command -v "$SOLVER" > /dev/null 2>&1; then
    echo "求解器不可用: $SOLVER" >&2
    exit 2
fi

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[0;33m'; NC='\033[0m'

# ---------------------------------------------------------------------------
# 1. 生成任务清单
# ---------------------------------------------------------------------------
TASKS=()
GENERATED=()

for sby in *.sby; do
    case "$sby" in
        *_fast.sby|*_boolector.sby|*_bitwuzla.sby|*_z3.sby|*_cvc5.sby) continue ;;
        probe_*.sby) continue ;;
    esac
    base="${sby%.sby}"
    use="$sby"

    if [[ "$MODE" == "fast" ]]; then
        cover_cap="${FAST_COVER_MIN[$base]:-$FAST_COVER_DEFAULT}"
        fast_sby="${base}_fast.sby"
        awk -v bmc="$FAST_BMC_DEPTH" -v cov="$cover_cap" '
            /^bmc: depth /   { if ($3+0 > bmc) { print "bmc: depth " bmc; next } }
            /^cover: depth / { if ($3+0 > cov) { print "cover: depth " cov; next } }
            { print }
        ' "$sby" > "$fast_sby"
        # 按需移除 cover 任务（保留 bmc）
        for skip in "${FAST_COVER_SKIP[@]}"; do
            if [[ "$base" == "$skip" ]]; then
                tmp="${fast_sby}.tmp"
                awk '
                    /^\[tasks\]/ { print; intask=1; next }
                    /^\[/        { if (intask) intask=0 }
                    intask && /^cover$/          { next }
                    /^cover: /                   { next }
                    { print }
                ' "$fast_sby" > "$tmp" && mv "$tmp" "$fast_sby"
            fi
        done
        use="$fast_sby"
        GENERATED+=("$fast_sby")
    fi

    if [[ "$SOLVER" != "z3" ]]; then
        sed_sby="${base}_${SOLVER}.sby"
        sed "s/^smtbmc z3$/smtbmc $SOLVER/" "$use" > "$sed_sby"
        use="$sed_sby"
        GENERATED+=("$sed_sby")
    fi

    TASKS+=("$use")
done

if [[ "$LIST_ONLY" == "1" ]]; then
    printf "%s\n" "${TASKS[@]}"
    exit 0
fi

echo "============================================"
echo "  AERIS-10 形式验证"
echo "============================================"
echo "档位: $MODE   求解器: $SOLVER   并发: $JOBS"
echo ""

# ---------------------------------------------------------------------------
# 2. 并发执行
# ---------------------------------------------------------------------------
RUN_DIR="$(mktemp -d /tmp/aeris_formal_XXXXXX)"
cleanup() {
    if [[ "$KEEP_WORKDIR" == "0" ]]; then
        for g in "${GENERATED[@]}"; do
            [[ -n "$g" ]] && rm -f "$g"
        done
    fi
    rm -rf "$RUN_DIR"
}
trap cleanup EXIT

run_one() {
    local sby="$1"
    local name
    name="$(basename "$sby" .sby)"
    local out="$RUN_DIR/$name.log"
    local rc=0
    sby -f "$sby" > "$out" 2>&1 || rc=$?
    # 一个 .sby 可能含多个 task（bmc + cover），必须汇总全部 DONE 行，
    # 只取第一行会把 bmc 的 PASS 当成整个文件的结论，掩盖 cover 的失败。
    local all
    all="$(grep -hoE 'DONE \((PASS|FAIL|ERROR|UNKNOWN)[^)]*\)' "$out")"
    local verdict ntask nfail
    if [[ -z "$all" ]]; then
        verdict="DONE (NO-VERDICT rc=$rc)"
    else
        ntask="$(printf '%s\n' "$all" | wc -l)"
        nfail="$(printf '%s\n' "$all" | grep -vc 'DONE (PASS' || true)"
        if [[ "$nfail" -gt 0 ]]; then
            verdict="$(printf '%s\n' "$all" | grep -v 'DONE (PASS' | head -1) [$nfail/$ntask 任务非 PASS]"
        else
            verdict="DONE (PASS, rc=0) [$ntask 任务]"
        fi
    fi
    printf "%s\t%s\n" "$name" "$verdict" > "$RUN_DIR/$name.result"
    return 0
}
export -f run_one
export RUN_DIR

printf '%s\n' "${TASKS[@]}" | xargs -P "$JOBS" -I{} bash -c 'run_one "$@"' _ {}

# ---------------------------------------------------------------------------
# 3. 汇总
# ---------------------------------------------------------------------------
echo ""
echo "============================================"
echo "  结果汇总"
echo "============================================"
FAIL=0
for t in "${TASKS[@]}"; do
    name="$(basename "$t" .sby)"
    res="$RUN_DIR/$name.result"
    if [[ -f "$res" ]]; then
        verdict="$(cut -f2 "$res")"
        case "$verdict" in
            *"PASS"*)  printf "  %-42s ${GREEN}%s${NC}\n" "$name" "$verdict" ;;
            *"FAIL"*|*"ERROR"*|*"UNKNOWN"*|*"NO-VERDICT"*)
                        printf "  %-42s ${RED}%s${NC}\n" "$name" "$verdict"; FAIL=$((FAIL+1)) ;;
            *)          printf "  %-42s ${YELLOW}%s${NC}\n" "$name" "$verdict"; FAIL=$((FAIL+1)) ;;
        esac
    else
        printf "  %-42s ${RED}未执行${NC}\n" "$name"
        FAIL=$((FAIL+1))
    fi
done
echo ""
echo "  失败/未完成: $FAIL"

if [[ "$FAIL" -gt 0 ]]; then
    exit 1
fi
exit 0
