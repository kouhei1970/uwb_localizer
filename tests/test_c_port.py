"""C 版が Python 版と一致し続けているかを見る.

移植は放っておくとずれる。**同じ入力で同じ数字が出るか**を毎回確かめる。
C コンパイラが無い環境では skip する (ライブラリ本体は Python だけで動く)。
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
CDIR = ROOT / "c"

pytestmark = pytest.mark.skipif(
    shutil.which("cc") is None and shutil.which("gcc") is None,
    reason="C コンパイラが無い")


def _make(*targets: str) -> subprocess.CompletedProcess:
    return subprocess.run(["make", "-C", str(CDIR), *targets],
                          capture_output=True, text=True, timeout=600)


@pytest.fixture(scope="module")
def built():
    r = _make("examples")
    assert r.returncode == 0, f"C 版のビルドに失敗:\n{r.stdout[-3000:]}\n{r.stderr[-3000:]}"
    return True


def test_c_library_builds_without_warnings():
    """移植先のコンパイラで詰まらないよう、警告ゼロを保つ."""
    _make("clean")
    r = _make()
    assert r.returncode == 0, f"ビルド失敗:\n{r.stderr[-3000:]}"
    warnings = [ln for ln in (r.stdout + r.stderr).splitlines() if "warning:" in ln]
    assert not warnings, "警告が出ている:\n  " + "\n  ".join(warnings[:10])


def test_c_tests_pass(built):
    r = _make("test")
    assert r.returncode == 0, f"C 版のテストが失敗:\n{r.stdout[-4000:]}"
    assert "すべて通った" in r.stdout


def test_c_tests_pass_in_single_precision():
    """マイコンでは float にすることが多い。そこでも通るか."""
    r = _make("float")
    assert r.returncode == 0, f"float 版が失敗:\n{r.stdout[-4000:]}"
    assert "すべて通った" in r.stdout
    _make("clean")


def test_c_matches_python(built):
    """Lv0-Lv3 x 5 シナリオで、位置が一致するか.

    ここが落ちたら、どちらかの実装だけを直したということ。
    """
    _make("examples")
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "crossval.py"),
                        "--tol", "1e-9"],
                       capture_output=True, text=True, timeout=900, cwd=ROOT)
    assert r.returncode == 0, f"C 版と Python 版がずれている:\n{r.stdout[-4000:]}"
    assert "すべて一致" in r.stdout


def test_c_examples_run(built):
    for name in ("01_snapshot", "02_tracking"):
        exe = CDIR / "examples" / name
        assert exe.exists(), f"{name} がビルドされていない"
        r = subprocess.run([str(exe)], capture_output=True, text=True, timeout=120)
        assert r.returncode == 0, f"{name} が失敗:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}"
