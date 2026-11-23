#!/usr/bin/env bash
#
# Build and run the dynamixel_ros_control test suite under AddressSanitizer or
# ThreadSanitizer. Manual, no CI job runs it: TSan costs roughly ten times the
# wall clock and the reports need a human to read them.
#
# Only dynamixel_ros_control is instrumented. Its dependencies are not, which is
# why tools/sanitizers/*.supp exist. Their comments say what that hides.
#
#   tools/run_sanitizers.sh                 # asan, then tsan, whole suite
#   tools/run_sanitizers.sh tsan            # tsan only
#   tools/run_sanitizers.sh asan -R wire    # only tests matching a regex
#   tools/run_sanitizers.sh tsan --no-build # re-run against an existing tree
#   tools/run_sanitizers.sh tsan --isolate  # one gtest case per process, counted separately
#
# --isolate exists because a sanitizer's state is per process and sticky: once TSan sees one mutex
# misused, every later access under that mutex is reported too, so the count for a whole binary is
# neither additive nor attributable. One case per process makes each number mean something, at the
# cost of repeating every fixture setup. Use it to decide what is real; use the default to decide
# whether anything changed.
#
# Each mode builds into its own <workspace>/build-<mode>-<package> tree, so a normal colcon
# build is never clobbered, switching back needs no rebuild, and another repository's copy of
# this script cannot reach into this one's.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUPPRESSION_DIR="${REPO_ROOT}/tools/sanitizers"
PACKAGES=(dynamixel_ros_control)
# The package whose build directory holds the test binaries.
PRIMARY_PACKAGE=dynamixel_ros_control

usage()
{
  # The header comment above is the help text; print it up to the first line of code.
  awk 'NR > 1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
  exit "${1:-0}"
}

MODES=()
TEST_REGEX=""
CTEST_JOBS=1
DO_BUILD=1
ISOLATE=0
WORKSPACE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    asan | tsan) MODES+=("$1") ;;
    all) MODES+=(asan tsan) ;;
    -R | --tests) TEST_REGEX="$2"; shift ;;
    -j | --jobs) CTEST_JOBS="$2"; shift ;;
    --no-build) DO_BUILD=0 ;;
    --isolate) ISOLATE=1 ;;
    --workspace) WORKSPACE="$2"; shift ;;
    -h | --help) usage 0 ;;
    *) echo "Unknown argument: $1" >&2; usage 2 ;;
  esac
  shift
done
[[ ${#MODES[@]} -gt 0 ]] || MODES=(asan tsan)

# The colcon workspace is the directory holding the src/ this repo sits in.
if [[ -z "${WORKSPACE}" ]]; then
  if [[ "$(basename "$(dirname "${REPO_ROOT}")")" == "src" ]]; then
    WORKSPACE="$(cd "${REPO_ROOT}/../.." && pwd)"
  else
    echo "Could not locate the colcon workspace from ${REPO_ROOT}. Pass --workspace DIR." >&2
    exit 2
  fi
fi

# ROS setup files read unset variables (AMENT_TRACE_SETUP_FILES and friends), so
# they cannot be sourced under `set -u`.
source_env()
{
  set +u
  # shellcheck disable=SC1090,SC1091
  source "$1"
  set -u
}

: "${ROS_DISTRO:=lyrical}"
source_env "/opt/ros/${ROS_DISTRO}/setup.bash"
# Dependencies come from the normal (uninstrumented) install space, if there is
# one. The instrumented install space is sourced after the build so it takes
# precedence: pluginlib resolves the hardware component through the ament index
# rather than the linker, so whichever prefix comes first is the .so that runs.
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  source_env "${WORKSPACE}/install/setup.bash"
fi

flags_for()
{
  case "$1" in
    # UBSan rides along with ASan: it is nearly free next to ASan's shadow
    # memory and catches a different class of thing entirely.
    asan) echo "-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer -O1 -g" ;;
    tsan) echo "-fsanitize=thread -fno-omit-frame-pointer -O2 -g" ;;
  esac
}

# Report everything and keep going: a run that stops at the first finding hides
# how much else there is, and these runs are too slow to repeat once per report.
#
# detect_odr_violation=0: the tests link the library and controller_manager
# dlopen()s the same library as a plugin, so every symbol legitimately has two
# definitions in the process.
export_runtime_options()
{
  case "$1" in
    asan)
      # Leak detection is off by default, as rclcpp never destroys its global
      # Context, so every run "leaks" the participant, the type support and the
      # DDS thread state. DXL_SAN_LEAKS=1 turns it on against lsan.supp.
      export ASAN_OPTIONS="detect_leaks=${DXL_SAN_LEAKS:-0}:detect_odr_violation=0:halt_on_error=0:print_stacktrace=1"
      export LSAN_OPTIONS="suppressions=${SUPPRESSION_DIR}/lsan.supp"
      export UBSAN_OPTIONS="print_stacktrace=1:suppressions=${SUPPRESSION_DIR}/ubsan.supp"
      ;;
    tsan)
      export TSAN_OPTIONS="suppressions=${SUPPRESSION_DIR}/tsan.supp:halt_on_error=0:second_deadlock_stack=1:history_size=7"
      ;;
  esac
}

# Run every gtest case of every matching binary in its own process and count the sanitizer reports
# each one raises on its own. Returns non-zero if any case reported.
run_isolated()
{
  local build_dir="$1" mode="$2"
  local out_dir="${build_dir}/sanitizer-isolate"
  rm -rf "${out_dir}"
  mkdir -p "${out_dir}"

  # A fresh domain per case, cycled: these run sequentially, but a participant from the previous
  # process can outlive it, and discovering one is how tests find each other's controller manager.
  local domain=1
  local cases=0 dirty=0 total=0

  # What counts as a finding depends on the sanitizer, and a case is equally not-clean when it
  # crashes, fails an assertion or fails to start, none of which prints a sanitizer banner.
  local pattern
  case "${mode}" in
    tsan) pattern='WARNING: ThreadSanitizer' ;;
    asan) pattern='ERROR: AddressSanitizer|ERROR: LeakSanitizer|SUMMARY: UndefinedBehaviorSanitizer|runtime error:' ;;
    *) echo "run_isolated: no report pattern for mode '${mode}'" >&2; return 2 ;;
  esac

  # Direct invocation bypasses the per-test TIMEOUT that CMake sets, so a deadlocked case would
  # stall the sweep for ever. Generous, because the point of the ceiling is to keep going.
  local case_timeout="${DXL_SAN_CASE_TIMEOUT:-600}"

  # Test names come from ctest rather than a filename glob: a gtest binary is not always called
  # test_*, and probing arbitrary executables for --gtest_list_tests would run whatever it found.
  local binary name case_id log count status listed note
  while IFS= read -r name; do
    binary="${build_dir}/${name}"
    [[ -f "${binary}" && -x "${binary}" ]] || continue

    listed=0
    while IFS= read -r case_id; do
      [[ -n "${case_id}" ]] || continue
      listed=$((listed + 1))
      cases=$((cases + 1))
      domain=$(((domain % 101) + 1))
      log="${out_dir}/${name}.${case_id}.log"

      status=0
      ROS_DOMAIN_ID="${domain}" timeout --kill-after=30s "${case_timeout}" \
        "${binary}" --gtest_filter="${case_id}" >"${log}" 2>&1 || status=$?

      count="$(grep -cE "${pattern}" "${log}" || true)"
      total=$((total + count))

      note=""
      if [[ "${status}" -eq 124 || "${status}" -eq 137 ]]; then
        note="timed out after ${case_timeout}s"
      elif [[ "${count}" -eq 0 && "${status}" -ne 0 ]]; then
        # No sanitizer banner, but the process still failed: an assertion, a crash, a loader error.
        note="exit ${status}, no sanitizer report"
      fi

      if [[ "${count}" -gt 0 || -n "${note}" ]]; then
        dirty=$((dirty + 1))
        printf '  %6s  %s / %s%s\n' "${count}" "${name}" "${case_id}" \
          "${note:+  [${note}]}"
      else
        rm -f "${log}"
      fi
    done < <("${binary}" --gtest_list_tests 2>/dev/null |
      awk '/^[^[:space:]]/ { suite = $1; next } /^[[:space:]]+[A-Za-z_]/ { print suite $1 }')

    if [[ "${listed}" -eq 0 ]]; then
      # ctest named a test whose binary lists no cases. Silence here would read as "all clean".
      echo "  ------  ${name}  [listed no test cases; --gtest_list_tests failed?]"
      dirty=$((dirty + 1))
    fi
  done < <(cd "${build_dir}" && ctest -N "${filter_args[@]}" | sed -n 's/^ *Test *#[0-9]*: *//p')

  echo
  echo "${mode^^}: ${cases} case(s), ${dirty} not clean, ${total} sanitizer report(s)."
  if [[ "${dirty}" -gt 0 ]]; then
    echo "Output for the cases that were not clean: ${out_dir}"
    return 1
  fi
  return 0
}

overall_status=0

for mode in "${MODES[@]}"; do
  # Suffixed with the package, not just the mode. Several repositories carry a copy of this
  # script; sharing one build-<mode> tree meant running another repository's copy quietly replaced
  # the sibling packages here with instrumented ones, changing what this run measures without
  # changing anything this repository owns.
  build_base="${WORKSPACE}/build-${mode}-${PRIMARY_PACKAGE}"
  install_base="${WORKSPACE}/install-${mode}-${PRIMARY_PACKAGE}"
  flags="$(flags_for "${mode}")"

  echo
  echo "=============================================================================="
  # Named, because the suppression file is per DDS implementation and CI's default is not
  # necessarily the one a workspace pins: a clean run says nothing about the other.
  echo " ${mode^^}  ->  ${build_base}"
  echo " rmw: ${RMW_IMPLEMENTATION:-<unset, using the image default>}"
  echo "=============================================================================="

  if [[ ${DO_BUILD} -eq 1 ]]; then
    colcon build \
      --packages-select "${PACKAGES[@]}" \
      --build-base "${build_base}" \
      --install-base "${install_base}" \
      --cmake-args \
      -DBUILD_TESTING=ON \
      -DCMAKE_BUILD_TYPE= \
      "-DCMAKE_C_FLAGS=${flags}" \
      "-DCMAKE_CXX_FLAGS=${flags}" \
      "-DCMAKE_EXE_LINKER_FLAGS=${flags}" \
      "-DCMAKE_SHARED_LINKER_FLAGS=${flags}"
  fi

  source_env "${install_base}/setup.bash"

  filter_args=()
  if [[ -n "${TEST_REGEX}" ]]; then
    filter_args=(-R "${TEST_REGEX}")
  fi

  # A build that ends up with BUILD_TESTING off runs zero tests and still exits
  # 0, which looks exactly like a clean sanitizer run. Refuse to report that.
  discovered=$(cd "${build_base}/${PRIMARY_PACKAGE}" &&
    ctest -N "${filter_args[@]}" | sed -n 's/^Total Tests: //p')
  if [[ "${discovered:-0}" -eq 0 ]]; then
    echo "No tests were discovered under ${mode}${TEST_REGEX:+ matching ${TEST_REGEX}}." >&2
    exit 1
  fi
  echo "Running ${discovered} test(s) under ${mode}."

  # Sanitizer runs are memory-hungry (TSan maps a large shadow region per
  # process), hence serial by default.
  export_runtime_options "${mode}"
  status=0

  if [[ ${ISOLATE} -eq 1 ]]; then
    run_isolated "${build_base}/${PRIMARY_PACKAGE}" "${mode}" || status=$?
    if [[ ${status} -ne 0 ]]; then
      overall_status=${status}
    fi
    continue
  fi

  colcon test \
    --packages-select "${PACKAGES[@]}" \
    --build-base "${build_base}" \
    --install-base "${install_base}" \
    --return-code-on-test-failure \
    --executor sequential \
    --ctest-args -j "${CTEST_JOBS}" --output-on-failure "${filter_args[@]}" || status=$?

  if [[ ${status} -ne 0 ]]; then
    echo "${mode^^} reported failures (exit ${status})." >&2
    overall_status=${status}
  else
    echo "${mode^^} clean."
  fi
done

exit ${overall_status}
