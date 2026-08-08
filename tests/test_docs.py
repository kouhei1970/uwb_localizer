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


def _strip_inline_code(line: str) -> str:
    """行内のコードスパン (`...`) を落とす.

    ``\`$\`` のように「$ という文字」を説明している箇所を数式と
    取り違えないため。中身は空白に置き換えて桁をずらさない。
    """
    import re

    return re.sub(r"`[^`]*`", lambda m: " " * len(m.group(0)), line)



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


def test_python_blocks_in_docs_compile():
    """ドキュメントの ```python ブロックが構文として通るか.

    省略のつもりで書いた `...` は、dict の中では SyntaxError になる
    (``{"A0": [0, 0, 2.4], ...}`` など)。写して動かす読者が最初に踏むので、
    省略は「裸の ...」ではなくコメントで書く。
    """
    import re

    block = re.compile(r"```python\n(.*?)```", re.S)
    broken = []
    for md in sorted(ROOT.rglob("*.md")):
        if ".git" in md.parts:
            continue
        for i, code in enumerate(block.findall(md.read_text(encoding="utf-8")), 1):
            try:
                compile(code, str(md), "exec")
            except SyntaxError as e:
                broken.append(f"{md.relative_to(ROOT)} ブロック{i} 行{e.lineno}: {e.msg}")
    assert not broken, "構文エラーのあるコードブロック:\n  " + "\n  ".join(broken)


def test_display_math_uses_github_block_form():
    """表示数式が GitHub で壊れない形になっているか.

    GitHub の Markdown は 1 行形式の ``$$...$$`` の中身にもインラインの
    エスケープを掛けるので、行列の行区切り ``\\\\`` が ``\\`` に潰れて
    ``\\begin{bmatrix}`` が壊れる。公式に案内されている

        $$
        式
        $$

    の形なら中身はそのまま数式として扱われる。
    """
    import re

    bad = []
    for md in sorted(ROOT.rglob("*.md")):
        if ".git" in md.parts:
            continue
        in_code = False
        for i, line in enumerate(md.read_text(encoding="utf-8").split("\n"), 1):
            if line.strip().startswith("```"):
                in_code = not in_code
                continue
            if in_code:
                continue
            line = _strip_inline_code(line)
            if re.search(r"\$\$.*\S.*\$\$", line):
                bad.append(f"{md.relative_to(ROOT)}:{i} 1 行形式の $$ — {line.strip()[:60]}")
    assert not bad, "GitHub で壊れる表示数式:\n  " + "\n  ".join(bad)


#: GitHub の KaTeX が禁止しているコマンド。
#: 使うとページに「The following macros are not allowed: ...」と出て、
#: その数式が丸ごと表示されなくなる。マクロを定義できる系が中心。
GITHUB_BANNED_MACROS = (
    r"\def", r"\gdef", r"\edef", r"\xdef", r"\let", r"\futurelet",
    r"\newcommand", r"\renewcommand", r"\providecommand",
    r"\global", r"\operatorname", r"\includegraphics",
)


def test_no_macros_that_github_rejects():
    """GitHub が弾くコマンドを使っていないか.

    ``\operatorname`` は普通の KaTeX では通るが GitHub では通らない。
    代わりに ``\mathrm`` を使う。
    """
    import re

    bad = []
    for md in sorted(ROOT.rglob("*.md")):
        if ".git" in md.parts:
            continue
        in_code = False
        for i, line in enumerate(md.read_text(encoding="utf-8").split("\n"), 1):
            if line.strip().startswith("```"):
                in_code = not in_code
                continue
            line = _strip_inline_code(line)
            if in_code or "$" not in line:
                continue
            for macro in GITHUB_BANNED_MACROS:
                # \def などはそのままだと正規表現の \d と衝突するのでエスケープする
                if re.search(re.escape(macro) + r"(?![a-zA-Z])", line):
                    bad.append(f"{md.relative_to(ROOT)}:{i} {macro} — {line.strip()[:50]}")
    assert not bad, ("GitHub が表示できないコマンド (\\mathrm などに置き換える):\n  "
                     + "\n  ".join(bad))


def test_inline_math_is_preceded_by_ascii_space():
    """開き ``$`` の直前が非 ASCII だと GitHub は数式として認識しない.

    GitHub の実際の出力で確かめた規則。描画される数式だけが
    ``<math-renderer>`` に包まれるので、包まれているかを数えると分かる。

        直前の文字      描画された   未描画
        ASCII 空白           101        5
        行頭                  19        1
        非 ASCII               0       26     ← 全滅

    日本語の直後に置くと出ない (``。$x$``)。半角空白を 1 つ挟めばよい。
    実物の確認は ``python tools/check_github_math.py docs/UWB_ALGORITHMS.md``。
    """
    import re

    inline = re.compile(r"(?<!\$)\$([^$\n]+?)\$(?!\$)")
    bad = []
    for md in sorted(ROOT.rglob("*.md")):
        if ".git" in md.parts:
            continue
        in_code = in_block = False
        for i, line in enumerate(md.read_text(encoding="utf-8").split("\n"), 1):
            if line.strip().startswith("```"):
                in_code = not in_code
                continue
            if in_code:
                continue
            if line.strip() == "$$":
                in_block = not in_block
                continue
            line = _strip_inline_code(line)
            if in_block or "$" not in line:
                continue
            for m in inline.finditer(line):
                prev = line[m.start() - 1] if m.start() > 0 else None
                if prev is not None and ord(prev) >= 128:
                    bad.append(f"{md.relative_to(ROOT)}:{i} 直前が「{prev}」 — "
                               f"{m.group(0)[:40]}")
    assert not bad, ("GitHub が描画しない数式 (開き $ の前に半角空白を入れる):\n  "
                     + "\n  ".join(bad))


def test_inline_math_delimiters_are_balanced():
    """インライン数式の $ が閉じているか (奇数個だと以降が数式扱いになる)."""
    bad = []
    for md in sorted(ROOT.rglob("*.md")):
        if ".git" in md.parts:
            continue
        in_code = in_block = False
        for i, line in enumerate(md.read_text(encoding="utf-8").split("\n"), 1):
            if line.strip().startswith("```"):
                in_code = not in_code
                continue
            if in_code:
                continue
            if line.strip() == "$$":
                in_block = not in_block
                continue
            if in_block:
                continue
            if _strip_inline_code(line).count("$") % 2 != 0:
                bad.append(f"{md.relative_to(ROOT)}:{i} $ が奇数個 — {line.strip()[:60]}")
        if in_block:
            bad.append(f"{md.relative_to(ROOT)} $$ ブロックが閉じていない")
    assert not bad, "数式の区切りが合っていない:\n  " + "\n  ".join(bad)
