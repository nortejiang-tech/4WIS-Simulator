"""Packaging regression: a hashed vendor DLL is not an importable DLL name."""
import hashlib
import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("build_portable", Path(__file__).parents[1] / "build_portable.py")
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


class WindowsRuntimeTests(unittest.TestCase):
    def test_bundles_exact_vendor_bytes_and_provenance(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "runtime").mkdir()
            (root / "vendor/numpy.libs").mkdir(parents=True)
            source = root / "vendor/numpy.libs/msvcp140-vendorhash.dll"
            source.write_bytes(b"fixture-runtime-bytes")
            result = build.install_windows_cpp_runtime(root)
            self.assertEqual((root / result["path"]).read_bytes(), source.read_bytes())
            self.assertEqual(result["sha256"], hashlib.sha256(source.read_bytes()).hexdigest())
            self.assertEqual(result["path"], "runtime/msvcp140.dll")
            self.assertTrue(source.exists())

    def test_missing_runtime_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(ValueError, "exactly one"):
                build.install_windows_cpp_runtime(Path(temp))

    def test_ambiguous_runtime_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "vendor/numpy.libs").mkdir(parents=True)
            for name in ("msvcp140-old.dll", "msvcp140-new.dll"):
                (root / "vendor/numpy.libs" / name).write_bytes(b"fixture")
            with self.assertRaisesRegex(ValueError, "exactly one"):
                build.install_windows_cpp_runtime(root)


if __name__ == "__main__":
    unittest.main()
