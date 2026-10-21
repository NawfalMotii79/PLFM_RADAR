#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TEST_DIR=$(mktemp -d)
trap 'rm -rf "$TEST_DIR"' EXIT

cp "$SCRIPT_DIR/run_regression.sh" "$TEST_DIR/run_regression.sh"
mkdir "$TEST_DIR/bin" "$TEST_DIR/tb"

cat > "$TEST_DIR/bin/iverilog" <<'EOF'
#!/bin/bash
if [[ "$1" == "-V" ]]; then
    echo 'Icarus Verilog (test stub)'
fi
exit 0
EOF

cat > "$TEST_DIR/bin/vvp" <<'EOF'
#!/bin/bash
echo '[PASS] initial check'
case "$TEST_SCENARIO" in
    exit_error)
        echo 'FATAL: simulator stopped' >&2
        exit 1
        ;;
    explicit_fail) echo '[FAIL] later check' ;;
esac
EOF

cat > "$TEST_DIR/bin/timeout" <<'EOF'
#!/bin/bash
shift
"$@"
status=$?
if [[ "$TEST_SCENARIO" == "timeout" ]]; then
    exit 124
fi
exit "$status"
EOF
chmod +x "$TEST_DIR/bin/iverilog" "$TEST_DIR/bin/vvp" "$TEST_DIR/bin/timeout"

failures=0
check_runner() {
    local scenario="$1" expected_status="$2" expected_output="$3"
    local status=0
    local log="$TEST_DIR/$scenario.log"
    TEST_SCENARIO="$scenario" PATH="$TEST_DIR/bin:$PATH" \
        bash "$TEST_DIR/run_regression.sh" --quick --skip-lint > "$log" 2>&1 || status=$?

    if [[ "$status" -eq "$expected_status" ]] && grep -q "$expected_output" "$log"; then
        echo "PASS: $scenario (runner exit $status)"
    else
        echo "FAIL: $scenario (runner exit $status, expected $expected_status; expected output: $expected_output)"
        cat "$log"
        failures=$((failures + 1))
    fi
}

check_runner success 0 'Tests: .* passed, 0 failed,'
check_runner exit_error 1 'FATAL: simulator stopped'
check_runner timeout 1 'exit status 124'
check_runner explicit_fail 1 'failure(s)'

if [[ "$failures" -gt 0 ]]; then
    echo "$failures runner test(s) failed"
    exit 1
fi
