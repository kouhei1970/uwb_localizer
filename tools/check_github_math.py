"""GitHub が実際に描画した HTML を取ってきて、数式が出ているか数える。

GitHub は数式をクライアント側で描画するので、サーバが返す HTML には
生の ``$...$`` が残っている。**描画されるものだけ** ``<math-renderer>``
で包まれているので、包まれていない ``$...$`` = 表示されない数式。

    python tools/check_github_math.py docs/UWB_ALGORITHMS.md

ローカルの Markdown を見るだけでは分からない (GitHub 固有の規則があるため)。
push 済みの内容に対して使う。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.request

REPO = "kouhei1970/uwb_localizer"
INLINE = re.compile(r"\$[^$\n]+?\$")


def fetch(path: str, branch: str) -> str:
    url = f"https://github.com/{REPO}/blob/{branch}/{path}"
    req = urllib.request.Request(url, headers={"User-Agent": "uwb-loc-doc-check"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read().decode("utf-8", "replace")


def rendered_body(html_text: str) -> str:
    """blob ページの <script type="application/json"> から描画済み HTML を取り出す。"""
    key = html_text.find('"richText"')
    if key < 0:
        raise SystemExit("richText が見つからない (ページ構造が変わった?)")
    start = html_text.rfind("<script", 0, key)
    open_end = html_text.find(">", start) + 1
    end = html_text.find("</script>", open_end)
    data = json.loads(html_text[open_end:end])

    def walk(o):
        if isinstance(o, dict):
            for k, v in o.items():
                if k == "richText" and isinstance(v, str):
                    return v
                found = walk(v)
                if found:
                    return found
        elif isinstance(o, list):
            for v in o:
                found = walk(v)
                if found:
                    return found
        return None

    body = walk(data)
    if not body:
        raise SystemExit("richText を取り出せなかった")
    return body


def unrendered(body: str) -> list[str]:
    """<math-renderer> に包まれていない $...$ を集める。"""
    # まず math-renderer の中身を取り除く
    stripped = re.sub(r"<math-renderer.*?</math-renderer>", "", body, flags=re.S)
    # タグを落としてから探す (属性値の $ を拾わないように)
    text = re.sub(r"<[^>]+>", "", stripped)
    return INLINE.findall(text)


def main() -> int:
    ap = argparse.ArgumentParser(description="GitHub 上で数式が描画されているか確かめる")
    ap.add_argument("paths", nargs="+", help="リポジトリ内のパス (例 docs/UWB_ALGORITHMS.md)")
    ap.add_argument("--branch", default="main")
    args = ap.parse_args()

    total_bad = 0
    for path in args.paths:
        body = rendered_body(fetch(path, args.branch))
        ok = body.count("<math-renderer")
        bad = unrendered(body)
        total_bad += len(bad)
        print(f"{path}: 描画 {ok} 個 / **未描画 {len(bad)} 個**")
        for s in bad[:20]:
            print(f"    {s[:80]}")
        if len(bad) > 20:
            print(f"    ... ほか {len(bad) - 20} 個")

    print()
    if total_bad:
        print(f"合計 {total_bad} 個の数式が GitHub で表示されていない")
        return 1
    print("すべて描画されている")
    return 0


if __name__ == "__main__":
    sys.exit(main())
