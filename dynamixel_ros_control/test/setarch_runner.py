#!/usr/bin/env python3
"""
ament test runner shim that wraps the test command with `setarch -R`
(disables ASLR via ADDR_NO_RANDOMIZE personality before exec).

Required for ThreadSanitizer-instrumented tests on Linux 6.5+ where the
default ASLR entropy can collide with TSan's reserved shadow region,
causing: "FATAL: ThreadSanitizer: unexpected memory mapping ...".

CMake's TEST_LAUNCHER property (CMake 3.29+) would be a more direct fix,
but Ubuntu 24.04 ships CMake 3.28. Using ament_add_gtest's RUNNER keyword
to swap in this script is the workaround.

This is a thin shim: it injects setarch into the command then delegates to
ament_cmake_test.main() so the XML output, timeout handling, etc. stay
identical to the default run_test.py.
"""

import os
import sys

import ament_cmake_test


def main(argv):
    # Find the --command flag and prepend setarch to its arguments.
    if "--command" in argv:
        i = argv.index("--command")
        # argv[i+1:] is the actual test invocation. Insert setarch in front.
        # `setarch $(uname -m) -R` is the canonical no-ASLR launcher.
        arch = os.uname().machine
        argv = argv[: i + 1] + ["setarch", arch, "-R"] + argv[i + 1 :]
    return ament_cmake_test.main(argv=argv)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
