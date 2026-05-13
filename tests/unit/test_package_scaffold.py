from __future__ import annotations

import vcp


def test_package_exposes_version() -> None:
    assert vcp.__version__ == "0.1.0"
