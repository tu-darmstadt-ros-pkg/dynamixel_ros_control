// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause
//
// Helpers for running tests under AddressSanitizer / ThreadSanitizer.
// Designed to be included from any test main(), including mock-only tests
// that do not pull in rclcpp.

#pragma once

#include <cstdlib>

extern "C" {
#if defined(__has_feature)
#  if __has_feature(address_sanitizer) || __has_feature(leak_sanitizer)
#    define DYNAMIXEL_TEST_HAVE_LSAN 1
#  endif
#  if __has_feature(thread_sanitizer)
#    define DYNAMIXEL_TEST_HAVE_TSAN 1
#  endif
#elif defined(__SANITIZE_ADDRESS__)
#  define DYNAMIXEL_TEST_HAVE_LSAN 1
#endif
#ifdef __SANITIZE_THREAD__
#  define DYNAMIXEL_TEST_HAVE_TSAN 1
#endif
#ifdef DYNAMIXEL_TEST_HAVE_LSAN
// LSan public APIs (no-ops when LSan is not linked in).
void __lsan_do_leak_check();
int __lsan_do_recoverable_leak_check();
#endif
}

namespace dynamixel_ros_control::test {

// NOTE on TSan + ASLR: ThreadSanitizer reserves a fixed shadow region in the
// virtual address space. On Linux 6.5+ the higher mmap entropy can cause the
// binary's loaded segments to overlap that range, making TSan abort with
// "FATAL: ThreadSanitizer: unexpected memory mapping ...".
//
// Doing this in main() doesn't help because TSan's runtime initializer runs
// before main(). The cleanest workaround is to launch the test under
// `setarch -R` (ADDR_NO_RANDOMIZE personality at exec time). This is wired
// up at the CMake level for tsan builds via dxl_add_gtest.

/**
 * @brief Trigger LSan's recoverable leak check (no-op outside ASan/LSan).
 *
 * Called from run_tests_and_exit() before _Exit() so leak detection still
 * runs even though we skip glibc's __cxa_finalize. Returns 1 if leaks were
 * found, 0 otherwise.
 */
inline int trigger_lsan_check()
{
#ifdef DYNAMIXEL_TEST_HAVE_LSAN
  return __lsan_do_recoverable_leak_check();
#else
  return 0;
#endif
}

}  // namespace dynamixel_ros_control::test
