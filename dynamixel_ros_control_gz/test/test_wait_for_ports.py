"""wait_for_ports accepts only ports created while it waits."""

import os
import select
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPT = os.environ["WAIT_FOR_PORTS"]


class TestWaitForPorts(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.port = os.path.join(self.directory.name, "bus")

    def tearDown(self):
        self.directory.cleanup()

    def test_a_link_left_from_before_is_refused(self):
        os.symlink("/dev/null", self.port)
        an_hour_ago = time.time() - 3600
        os.utime(self.port, (an_hour_ago, an_hour_ago), follow_symlinks=False)
        result = subprocess.run(
            [sys.executable, SCRIPT, "--timeout", "1", self.port], check=False
        )
        self.assertNotEqual(result.returncode, 0)

    def test_a_link_created_while_waiting_is_accepted(self):
        waiting = subprocess.Popen(
            [sys.executable, SCRIPT, "--timeout", "10", self.port],
            stdout=subprocess.PIPE,
            text=True,
        )
        # Run last to first, so a script that hangs is killed before it is reaped
        self.addCleanup(waiting.stdout.close)
        self.addCleanup(waiting.wait)
        self.addCleanup(waiting.kill)
        # The script takes its start time before it says it waits
        ready, _, _ = select.select([waiting.stdout], [], [], 10)
        self.assertTrue(ready, "wait_for_ports never said it waits")
        self.assertTrue(waiting.stdout.readline().startswith("Waiting for"))
        os.symlink("/dev/null", self.port)
        self.assertEqual(waiting.wait(timeout=10), 0)


if __name__ == "__main__":
    unittest.main()
