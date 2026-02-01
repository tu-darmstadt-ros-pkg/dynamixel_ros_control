#!/bin/bash
#
# Compute test coverage for dynamixel_ros_control
#
# Usage:
#   ./scripts/coverage.sh [--open]
#
# Options:
#   --open    Open the HTML report in browser after generation
#
# Prerequisites:
#   sudo apt install lcov
#
# The script will:
#   1. Build the package with coverage instrumentation
#   2. Run all tests
#   3. Generate HTML coverage report in build/dynamixel_ros_control/coverage/
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
PACKAGE_NAME="dynamixel_ros_control"

# Find workspace root (where build/ install/ src/ are)
WORKSPACE_ROOT="$SCRIPT_DIR"
while [[ "$WORKSPACE_ROOT" != "/" ]]; do
    if [[ -d "$WORKSPACE_ROOT/build" && -d "$WORKSPACE_ROOT/src" ]]; then
        break
    fi
    WORKSPACE_ROOT="$(dirname "$WORKSPACE_ROOT")"
done

if [[ "$WORKSPACE_ROOT" == "/" ]]; then
    # Workspace not built yet, assume standard layout
    WORKSPACE_ROOT="$(cd "$PACKAGE_DIR/../../.." && pwd)"
fi

BUILD_DIR="$WORKSPACE_ROOT/build/$PACKAGE_NAME"
COVERAGE_DIR="$BUILD_DIR/coverage"

echo "========================================"
echo "Dynamixel ROS Control - Coverage Report"
echo "========================================"
echo "Workspace: $WORKSPACE_ROOT"
echo "Package:   $PACKAGE_DIR"
echo "Build:     $BUILD_DIR"
echo "Coverage:  $COVERAGE_DIR"
echo ""

# Check for lcov
if ! command -v lcov &> /dev/null; then
    echo "Error: lcov is not installed."
    echo "Install with: sudo apt install lcov"
    exit 1
fi

# Verify ROS environment is properly set up
# Check both ROS_DISTRO and that ament_package is importable
if ! python3 -c "import ament_package" 2>/dev/null; then
    echo "Error: ROS Python environment not properly set up."
    echo "Please source your workspace before running this script:"
    echo ""
    echo "  source ~/hector/setup.bash"
    echo "  # or: source install/setup.bash"
    echo ""
    exit 1
fi

# Step 1: Clean previous coverage data
echo "[1/5] Cleaning previous coverage data..."
rm -rf "$COVERAGE_DIR"
mkdir -p "$COVERAGE_DIR"

# Also clean any existing .gcda files from previous runs
find "$BUILD_DIR" -name "*.gcda" -delete 2>/dev/null || true

# Step 2: Build with coverage flags
echo "[2/5] Building with coverage instrumentation..."
cd "$WORKSPACE_ROOT"
colcon build \
    --packages-select "$PACKAGE_NAME" \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_FLAGS="--coverage -fprofile-arcs -ftest-coverage -fprofile-update=atomic" \
        -DCMAKE_C_FLAGS="--coverage -fprofile-arcs -ftest-coverage -fprofile-update=atomic" \
        -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
        -DCMAKE_SHARED_LINKER_FLAGS="--coverage"

# Step 3: Run tests
echo "[3/5] Running tests..."
colcon test --packages-select "$PACKAGE_NAME" --return-code-on-test-failure || true

# Show test results
echo ""
echo "Test Results:"
colcon test-result --verbose --test-result-base "$WORKSPACE_ROOT/build/$PACKAGE_NAME" 2>/dev/null || true
echo ""

# Step 4: Capture coverage data
echo "[4/5] Capturing coverage data..."

# Common lcov options to handle multithreaded code and other issues
LCOV_OPTS="--ignore-errors mismatch,negative,empty,unused --rc lcov_branch_coverage=1"

# Capture coverage after tests (skip baseline, it's not needed and causes issues)
lcov --capture \
    --directory "$BUILD_DIR" \
    --output-file "$COVERAGE_DIR/coverage_raw.info" \
    $LCOV_OPTS

# Filter out system headers, test files, and external dependencies
lcov --remove "$COVERAGE_DIR/coverage_raw.info" \
    '/usr/*' \
    '/opt/*' \
    '*/test/*' \
    '*/gtest/*' \
    '*/gmock/*' \
    '*/_deps/*' \
    --output-file "$COVERAGE_DIR/coverage_filtered.info" \
    $LCOV_OPTS

# Step 5: Generate HTML report
echo "[5/5] Generating HTML report..."
genhtml "$COVERAGE_DIR/coverage_filtered.info" \
    --output-directory "$COVERAGE_DIR/html" \
    --title "dynamixel_ros_control Coverage" \
    --legend \
    --show-details \
    --branch-coverage \
    --ignore-errors negative \
    --rc lcov_branch_coverage=1

# Print summary
echo ""
echo "========================================"
echo "Coverage Summary"
echo "========================================"
lcov --summary "$COVERAGE_DIR/coverage_filtered.info" $LCOV_OPTS 2>&1 | grep -E "(lines|functions|branches)" || echo "See HTML report for details"
echo ""
echo "HTML report: $COVERAGE_DIR/html/index.html"
echo ""

# Open browser if requested
if [[ "$1" == "--open" ]]; then
    if command -v xdg-open &> /dev/null; then
        xdg-open "$COVERAGE_DIR/html/index.html"
    elif command -v open &> /dev/null; then
        open "$COVERAGE_DIR/html/index.html"
    else
        echo "Could not find browser opener. Please open manually:"
        echo "  $COVERAGE_DIR/html/index.html"
    fi
fi
