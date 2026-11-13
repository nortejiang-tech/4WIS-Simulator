"""Host-native build; SIM4WIS_BUILD_NATIVE=0 explicitly builds the Python reference."""

import hashlib
import os
import sys
from pathlib import Path

from setuptools import Extension, setup

flags = (
    ["/std:c++17", "/O2", "/fp:strict"]
    if sys.platform == "win32"
    else [
        "-std=c++17",
        "-O3",
        "-fno-fast-math",
        "-ffp-contract=off",
    ]
)
root = Path(__file__).resolve().parent
sources = ["native/core.hpp", "native/module.cpp", "src/sim4wis/native.py"]
digest = hashlib.sha256(b"".join((root / p).read_bytes() for p in sources)).hexdigest()
setup(
    ext_modules=[]
    if os.environ.get("SIM4WIS_BUILD_NATIVE") == "0"
    else [
        Extension(
            "sim4wis._native",
            ["native/module.cpp"],
            depends=sources,
            language="c++",
            extra_compile_args=flags,
            define_macros=[("SIM4WIS_NATIVE_SOURCE_SHA256", '"' + digest + '"')],
        ),
    ]
)
