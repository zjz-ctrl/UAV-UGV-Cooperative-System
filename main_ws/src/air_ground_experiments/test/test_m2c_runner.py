#!/usr/bin/env python3

import sys
from pathlib import Path
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

import run_m2c_dynamic_validation as runner  # noqa: E402


class RunDirectoryTest(unittest.TestCase):
    def test_prepare_run_directory_removes_stale_file_and_recreates_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "scenario"
            run_dir.mkdir()
            stale_file = run_dir / "verdict.json"
            stale_file.write_text("stale")

            prepare_run_directory = getattr(runner, "prepare_run_directory", None)
            self.assertIsNotNone(prepare_run_directory)
            prepare_run_directory(run_dir)

            self.assertTrue(run_dir.is_dir())
            self.assertFalse(stale_file.exists())


if __name__ == "__main__":
    unittest.main()
