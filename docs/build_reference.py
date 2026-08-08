"""``docs/REFERENCE.md`` をコードから生成する.

手で書くとすぐ実物とずれるので、**argparse の定義と公開シンボルを
そのまま読んで**出す。`tests/test_docs.py` が「生成し直した結果と
コミットされている REFERENCE.md が一致するか」を見ているので、
オプションを増やしたのに docs を直し忘れると CI で落ちる。

    python docs/build_reference.py            # 生成して書き出す
    python docs/build_reference.py --check    # ずれていないか見るだけ
"""

from __future__ import annotations

import argparse
import enum
import inspect
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import uwb_loc as ul  # noqa: E402
from uwb_loc.cli import build_parser  # noqa: E402

OUT = ROOT / "docs" / "REFERENCE.md"

#: 章立て。公開シンボルをこの順に並べる。ここに載っていないものが
#: __all__ に増えたら「その他」に落ちるので、気づける。
SECTIONS: list[tuple[str, str, list[str]]] = [
    ("データ型", "HAL と測位器の間を流れるもの。単位は m / rad / s、右手系で z が上。",
     ["Anchor", "Measurement", "MeasurementBatch", "MeasKind", "Fix", "MeasurementModel"]),
    ("HAL — 観測の入り口", "チップごとの差をここで吸収する。測位側のコードは変わらない。",
     ["UwbHal", "TextHal", "JsonLinesHal", "JsonLinesWriter", "PushHal",
      "Ryuw122Hal", "Ryuw122Config", "Ryuw122Terminal", "Ryuw122Tag", "sniff"]),
    ("測位器", "同じインターフェイスで差し替えられる。`make_estimator` が入り口。",
     ["make_estimator", "LEVELS", "PositionEstimator", "SolveConfig", "RobustLoss",
      "Lv0Trilateration", "Lv1WeightedNLS", "Lv2RobustNLS", "Lv3TightlyCoupledEKF"]),
    ("パイプライン", "HAL と測位器をつないで回す。",
     ["Pipeline", "run_offline"]),
    ("シミュレータ", "実機なしで動かす。HAL と同じインターフェイスなので差し替えられる。",
     ["SimulatedHal", "ErrorModel", "Scenario", "trajectory",
      "make_anchors", "room_anchors"]),
    ("配置の評価", "現場で精度が出ない原因はたいてい設営。置く前に見る。",
     ["gdop_at", "gdop_map", "crlb_at", "anchor_condition"]),
    ("キャリブレーション", "巻き尺で全台測らずに済ませる道具。",
     ["self_survey", "align_to_reference", "fit_range_bias", "estimate_antenna_delays"]),
    ("精度の評価", "推定と真値を突き合わせる。",
     ["error_stats", "error_series", "error_cdf"]),
    ("その他", "",
     ["__version__", "WIRE_VERSION", "geometry", "metrics", "calibration"]),
]


def first_line(obj: object) -> str:
    doc = inspect.getdoc(obj)
    if not doc:
        return ""
    out: list[str] = []
    for line in doc.splitlines():
        if not line.strip():
            break
        out.append(line.strip())
    return " ".join(out)


def signature_of(obj: object) -> str:
    try:
        if inspect.isclass(obj):
            sig = inspect.signature(obj.__init__)
            params = list(sig.parameters.values())[1:]      # self を落とす
            sig = sig.replace(parameters=params)
        else:
            sig = inspect.signature(obj)                    # type: ignore[arg-type]
    except (TypeError, ValueError):
        return ""
    return str(sig)


def public_methods(cls: type) -> list[tuple[str, str, str]]:
    out = []
    for name, member in vars(cls).items():
        if name.startswith("_"):
            continue
        if isinstance(member, (staticmethod, classmethod)):
            member = member.__func__
        if isinstance(member, property):
            out.append((name, "", first_line(member.fget) or "(プロパティ)"))
            continue
        if not callable(member):
            continue
        out.append((name, signature_of(member), first_line(member)))
    return out


def render_symbol(name: str) -> list[str]:
    obj = getattr(ul, name, None)
    if obj is None:
        return [f"### `{name}`", "", "(見つからない)", ""]

    if inspect.ismodule(obj):
        return [f"### `uwb_loc.{name}`", "", first_line(obj) or "(説明なし)", ""]

    if not (inspect.isclass(obj) or inspect.isfunction(obj)):
        value = repr(obj)
        if len(value) > 90:
            value = value[:87] + "..."
        return [f"### `{name}`", "", f"`{value}`", ""]

    lines = [f"### `{name}{signature_of(obj)}`", ""]
    summary = first_line(obj)
    if summary:
        lines += [summary, ""]

    if inspect.isclass(obj):
        if issubclass(obj, enum.Enum):
            members = ", ".join(f"`{m.name}` (`{m.value}`)" for m in obj)
            lines += [f"値: {members}", ""]
        else:
            methods = public_methods(obj)
            if methods:
                lines += ["| メソッド | 説明 |", "|---|---|"]
                for mname, msig, mdoc in methods:
                    label = f"`{mname}{msig}`" if msig else f"`{mname}`"
                    lines.append(f"| {label} | {mdoc or ''} |")
                lines.append("")
    return lines


def format_option(action: argparse.Action) -> tuple[str, str]:
    if action.option_strings:
        flag = ", ".join(f"`{s}`" for s in action.option_strings)
    else:
        flag = f"`{action.dest}` (位置引数)"
    bits = []
    if action.choices:
        bits.append("選択: " + " / ".join(f"`{c}`" for c in action.choices))
    if action.default is not None and action.default is not argparse.SUPPRESS:
        if not isinstance(action.default, bool) or action.default:
            bits.append(f"既定 `{action.default}`")
    if action.required and action.option_strings:
        bits.append("**必須**")
    desc = action.help or ""
    if bits:
        desc = f"{desc} ({'、'.join(bits)})" if desc else "、".join(bits)
    return flag, desc


def render_cli() -> list[str]:
    parser = build_parser()
    subparsers = next(a for a in parser._actions
                      if isinstance(getattr(a, "choices", None), dict))

    lines = ["## コマンドライン", "",
             "`python -m uwb_loc <コマンド>` か、インストール時に入る",
             "`uwb-loc <コマンド>` で呼ぶ。どちらも同じ。", "",
             "| コマンド | 何をする |", "|---|---|"]
    for name, sub in subparsers.choices.items():
        lines.append(f"| [`{name}`](#{name}) | {sub.description or ''} |")
    lines.append("")

    for name, sub in subparsers.choices.items():
        lines += [f"### `{name}`", ""]
        if sub.description:
            lines += [sub.description, ""]
        usage = sub.format_usage().replace("usage: ", "").strip()
        lines += ["```", usage, "```", ""]
        rows = [format_option(a) for a in sub._actions
                if a.dest not in ("help",)]
        if rows:
            lines += ["| 引数 | 説明 |", "|---|---|"]
            lines += [f"| {flag} | {desc} |" for flag, desc in rows]
            lines.append("")
    return lines


def render() -> str:
    lines = [
        "# リファレンス",
        "",
        "**このファイルはコードから生成している** (`python docs/build_reference.py`)。",
        "手で直しても次の生成で消えるので、直すならコードの docstring と",
        "argparse の `help` を直す。",
        "",
        f"対象バージョン: `uwb_loc {ul.__version__}`",
        "",
        "使い方の流れは [TUTORIAL.md](TUTORIAL.md)、動くコードは",
        "[../examples/](../examples/) にある。ここは「何があるか」の一覧。",
        "",
    ]
    lines += render_cli()
    lines += ["## Python API", "",
              "`import uwb_loc as ul` で全部触れる。", ""]

    listed = {n for _, _, names in SECTIONS for n in names}
    extra = [n for n in ul.__all__ if n not in listed]

    for title, blurb, names in SECTIONS:
        names = list(names)
        if title == "その他":
            names += extra
        if not names:
            continue
        lines += [f"### {title}", ""]
        if blurb:
            lines += [blurb, ""]
        for name in names:
            lines += ["#" + line if line.startswith("### ") else line
                      for line in render_symbol(name)]
    return "\n".join(lines).rstrip() + "\n"


if __name__ == "__main__":
    text = render()
    if "--check" in sys.argv:
        current = OUT.read_text(encoding="utf-8") if OUT.exists() else ""
        if current != text:
            print("REFERENCE.md が古い。python docs/build_reference.py で更新を。",
                  file=sys.stderr)
            raise SystemExit(1)
        print("REFERENCE.md は最新")
    else:
        OUT.write_text(text, encoding="utf-8")
        print(f"書き出しました: {OUT}  ({len(text.splitlines())} 行)")
