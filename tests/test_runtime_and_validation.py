"""Offline regression tests only; never contact an UR5e or Ollama."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import limits as L
import main
import robot_sender as R


class MinimumPythonVersionTests(unittest.TestCase):
    def _check(self, version):
        fake = SimpleNamespace(version_info=SimpleNamespace(major=version[0], minor=version[1], micro=version[2]))
        with patch.object(main, "sys", fake):
            main.check_python()

    def test_python_312_is_rejected(self):
        with self.assertRaises(SystemExit):
            self._check((3, 12, 9))

    def test_python_313_is_accepted(self):
        self._check((3, 13, 0))
        self._check((3, 13, 19))

    def test_python_314_is_accepted(self):
        self._check((3, 14, 4))
        self._check((3, 14, 10))

    def test_python_315_is_accepted(self):
        self._check((3, 15, 0))

    def test_other_major_is_rejected(self):
        with self.assertRaises(SystemExit):
            self._check((4, 0, 0))


class OfflineValidatorTests(unittest.TestCase):
    def setUp(self):
        self.state = R.Robot(dry_run=True).get_state()

    def test_configuration_is_consistent(self):
        self.assertEqual(L.check_static_config(), [])
        self.assertEqual(L.check_consistency(), [])

    def test_valid_sleep_is_accepted(self):
        self.assertTrue(R.validate_body(["sleep(0.1)"], self.state).ok)

    def test_invalid_sleep_is_refused(self):
        self.assertFalse(R.validate_body(["sleep(0)"], self.state).ok)

    def test_stationary_cartesian_target_is_accepted(self):
        p = list(L.HOME_POSE)
        command = "movel(p[" + ",".join(f"{v:.9f}" for v in p) + "],a=0.050000000,v=0.020000000)"
        self.assertTrue(R.validate_body([command], self.state).ok)


if __name__ == "__main__":
    unittest.main()
