"""ドキュメントが実物とずれていないかを見る.

リファレンスは手で書くと必ず古くなるので、コードから生成したものと
コミットされているファイルを突き合わせる。オプションを増やして
docs を更新し忘れたら、ここで落ちる。
"""

from __future__ import annotations

import inspect
import subprocess
import sys
from pathlib import Path

import pytest

import uwb_loc as ul

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "docs"))

from build_reference import first_line, public_methods, render  # noqa: E402


def test_reference_is_up_to_date():
    """docs/REFERENCE.md が生成し直した結果と一致するか."""
    current = (ROOT / "docs" / "REFERENCE.md").read_text(encoding="utf-8")
    assert current == render(), (
        "docs/REFERENCE.md が古い。python docs/build_reference.py で更新してください")


def test_every_public_symbol_is_documented():
    """__all__ に足したのにリファレンスに出ない、を防ぐ."""
    text = render()
    for name in ul.__all__:
        # サブモジュールは `uwb_loc.geometry` の形で載る
        assert f"`{name}" in text or f"`uwb_loc.{name}`" in text, \
            f"{name} がリファレンスに出ていない"


@pytest.mark.parametrize("name", sorted(ul.__all__))
def test_public_api_has_docstrings(name):
    """公開 API は説明が無いと使えない. 空の docstring を許さない."""
    obj = getattr(ul, name)
    if inspect.isclass(obj):
        assert first_line(obj), f"{name} に docstring が無い"
        for meth, _sig, doc in public_methods(obj):
            assert doc, f"{name}.{meth} に docstring が無い"
    elif inspect.isfunction(obj):
        assert first_line(obj), f"{name} に docstring が無い"


def test_every_cli_option_has_help():
    """--help が空欄だらけだと初心者が詰まる."""
    from uwb_loc.cli import build_parser

    parser = build_parser()
    subs = next(a for a in parser._actions
                if isinstance(getattr(a, "choices", None), dict))
    missing = []
    for cmd, sub in subs.choices.items():
        assert sub.description, f"サブコマンド {cmd} に description が無い"
        for action in sub._actions:
            if action.dest != "help" and not action.help:
                missing.append(f"{cmd} {action.dest}")
    assert not missing, f"help の無いオプション: {missing}"


def test_examples_run():
    """README が案内する例が実際に動くか (壊れた例ほど有害なものはない)."""
    for path in sorted((ROOT / "examples").glob("*.py")):
        r = subprocess.run([sys.executable, str(path)], cwd=ROOT,
                           capture_output=True, text=True, timeout=600)
        assert r.returncode == 0, f"{path.name} が失敗:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}"


def test_no_broken_local_links():
    """ドキュメント内のローカルリンクが実在するか.

    初心者ほどリンクを辿るので、切れていると一番効く。
    """
    import re

    link = re.compile(r"\[[^\]]*\]\(([^)#]+)(?:#[^)]*)?\)")
    broken = []
    for md in sorted(ROOT.rglob("*.md")):
        if ".git" in md.parts:
            continue
        for target in link.findall(md.read_text(encoding="utf-8")):
            if target.startswith(("http://", "https://", "mailto:")):
                continue
            if not (md.parent / target).exists():
                broken.append(f"{md.relative_to(ROOT)} -> {target}")
    assert not broken, "リンク切れ:\n  " + "\n  ".join(broken)
