"""コマンドライン.

    python -m uwb_loc ui                     # ブラウザ UI を起動
    python -m uwb_loc sim --levels Lv0,Lv2,Lv3
    python -m uwb_loc replay log.jsonl --anchors anchors.json --level Lv3
    python -m uwb_loc gdop --room 8 6 2.6
    python -m uwb_loc sniff --serial /dev/ttyUSB0   # 実機の出力を覗いて解釈できるか見る
    python -m uwb_loc survey dist.csv        # 相互測距からアンカー配置を推定
"""

from __future__ import annotations

import argparse
import json
import sys
import warnings
from pathlib import Path
from typing import Any

import numpy as np

from .calibration import self_survey
from .geometry import anchor_condition, crlb_at, gdop_map
from .hal.jsonl import JsonLinesHal, JsonLinesWriter
from .hal.text import TextHal, sniff
from .metrics import error_stats
from .pipeline import run_offline
from .sim import ErrorModel, SimulatedHal, room_anchors, trajectory
from .solvers import SolveConfig
from .types import Anchor

__all__ = ["main"]


def _load_anchors(path: str | None) -> list[Anchor] | None:
    if not path:
        return None
    obj = json.loads(Path(path).read_text(encoding="utf-8"))
    items = obj["anchors"] if isinstance(obj, dict) else obj
    return [Anchor.from_dict(a) for a in items]


def _print_stats(name: str, stats: dict[str, float]) -> None:
    print(
        f"{name:5s} 測位率 {stats['availability']*100:5.1f}%  "
        f"RMSE3D {stats['rmse_3d']:6.3f}  RMSE2D {stats['rmse_2d']:6.3f}  "
        f"RMSEz {stats['rmse_z']:6.3f}  CEP50 {stats['cep50']:6.3f}  "
        f"CEP95 {stats['cep95']:6.3f}  最大 {stats['max_3d']:6.3f}  [m]"
    )


# --------------------------------------------------------------------------- sim


def cmd_sim(args: argparse.Namespace) -> int:
    anchors = _load_anchors(args.anchors) or room_anchors(tuple(args.room), n_low=args.n_low)
    cond = anchor_condition(anchors)
    print(f"アンカー {int(cond['n'])} 台  同一平面={cond['coplanar']}  "
          f"平面からの広がり {cond['planarity']:.2f} m")
    if cond["coplanar"] and args.dim == 3:
        print("  警告: 同一平面配置で 3D を解こうとしています。高さがほぼ観測できないうえ、")
        print("        その平面に関する鏡像が測距値では区別できません")
        print("        (水平は正しいまま、高さだけ丸ごと折り返ることがあります)。")
        print("        --dim 2 で高さを固定するのが確実です。")
        # 同じ内容をライブラリ側も warnings で出すので, ここでは二重に見せない.
        warnings.filterwarnings("ignore", message=".*同一平面.*")

    error = ErrorModel(
        sigma0=args.sigma0,
        nlos_prob=args.nlos,
        nlos_bias_mean=args.nlos_bias,
        loss_rate=args.loss,
    )
    center = np.array([args.room[0] / 2, args.room[1] / 2, args.height])
    traj = {
        "figure8": lambda: trajectory.figure8(center, args.size, args.period),
        "circle": lambda: trajectory.circle(center, args.size, args.period),
        "static": lambda: trajectory.static(center),
        "random_walk": lambda: trajectory.random_walk(center, 0.6, None, args.seed),
    }[args.traj]()

    hal = SimulatedHal(anchors, traj, error, rate_hz=args.rate, seed=args.seed)
    times, truths, batches = hal.generate(args.duration)
    truth = np.array(truths)
    print(f"{len(batches)} エポック  1 エポックあたり平均 "
          f"{np.mean([len(b) for b in batches]):.1f} 本\n")

    if args.log:
        with JsonLinesWriter(args.log, anchors) as w:
            for b in batches:
                w.write(b)
        print(f"観測ログを書き出しました: {args.log}\n")

    config = SolveConfig(dim=args.dim, z_fixed=args.height)
    for level in args.levels.split(","):
        fixes = run_offline(batches, anchors, level=level.strip(), config=config)
        _print_stats(level.strip(), error_stats(truth, np.array([f.position for f in fixes])))
    print(f"\n理論下限 (CRLB, 軌道中心) {crlb_at(truth.mean(axis=0), anchors, dim=args.dim):.3f} m"
          "  — 1 エポックだけで解く場合の下限。Lv3 は時間方向の情報も使うので下回りうる。")
    return 0


# --------------------------------------------------------------------------- replay


def cmd_replay(args: argparse.Namespace) -> int:
    hal = JsonLinesHal.from_path(args.path, anchors=_load_anchors(args.anchors) or [])
    hal.open()
    batches = []
    while hal.is_open:
        got = hal.poll(0.3)
        if not got:
            break
        batches.extend(got)
    hal.close()

    anchors = hal.anchors
    if not anchors:
        print("アンカー座標が分かりません。--anchors を指定するか、"
              "ログに anchors メッセージを入れてください。", file=sys.stderr)
        return 2
    print(f"{len(batches)} エポック / アンカー {len(anchors)} 台")

    config = SolveConfig(dim=args.dim, z_fixed=args.height)
    fixes = run_offline(batches, anchors, level=args.level, config=config)
    ok = sum(1 for f in fixes if f.ok)
    print(f"{args.level}: 測位成功 {ok}/{len(fixes)}")

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write("t,x,y,z,sigma,n_used,gdop\n")
            for fx in fixes:
                f.write(f"{fx.t:.6f},{fx.position[0]:.4f},{fx.position[1]:.4f},"
                        f"{fx.position[2]:.4f},{fx.sigma:.4f},{fx.n_used},{fx.gdop:.3f}\n")
        print(f"書き出しました: {args.out}")
    return 0


# --------------------------------------------------------------------------- gdop


def cmd_gdop(args: argparse.Namespace) -> int:
    anchors = _load_anchors(args.anchors) or room_anchors(tuple(args.room), n_low=args.n_low)
    x, y, g = gdop_map(anchors, ((0.0, args.room[0]), (0.0, args.room[1])), args.height,
                       nx=args.nx, ny=args.ny, dim=args.dim)
    finite = g[np.isfinite(g)]
    print(f"GDOP  中央値 {np.median(finite):.2f}  最悪 {finite.max():.2f}  "
          f"最良 {finite.min():.2f}  (高さ {args.height} m, {args.dim}D)")

    # 端末で見えるように粗い等高線で描く.
    chars = " .:-=+*#%@"
    step_y = max(len(y) // 18, 1)
    step_x = max(len(x) // 60, 1)
    for j in range(len(y) - 1, -1, -step_y):
        row = ""
        for i in range(0, len(x), step_x):
            v = g[j, i]
            k = int(np.clip((v - 1.0) / 5.0, 0, 1) * (len(chars) - 1)) if np.isfinite(v) else -1
            row += chars[k] if k >= 0 else "@"
        print(row)
    print(f"(左下が原点 / {chars[0]!r} が良く {chars[-1]!r} が悪い)")
    return 0


# --------------------------------------------------------------------------- survey


def cmd_survey(args: argparse.Namespace) -> int:
    rows = [r for r in Path(args.matrix).read_text(encoding="utf-8").splitlines() if r.strip()]
    ids: list[str] | None = None
    if rows and not rows[0].split(",")[0].strip().lstrip("-").replace(".", "").isdigit():
        ids = [c.strip() for c in rows[0].split(",")]
        rows = rows[1:]
    dmat = np.array([[float(c) if c.strip() else np.nan for c in r.split(",")] for r in rows])
    anchors = self_survey(dmat, ids, dim=args.dim)
    print(json.dumps({"anchors": [a.to_dict() for a in anchors]}, ensure_ascii=False, indent=2))
    return 0


# --------------------------------------------------------------------------- sniff


def cmd_sniff(args: argparse.Namespace) -> int:
    """実機の出力を覗いて、測距として解釈できるかを確かめる.

    実機立ち上げの最初にやること。ここで単位とアンカー ID を確定させておかないと、
    位置がおかしいときに「測距が変」なのか「座標が変」なのか切り分けられない。
    """
    if args.serial:
        import io

        import serial  # type: ignore[import-not-found]

        ser = serial.Serial(args.serial, args.baud, timeout=1.0)
        stream: Any = io.TextIOWrapper(ser, encoding="utf-8", errors="replace")
    elif args.tcp:
        import socket

        host, _, port = args.tcp.rpartition(":")
        stream = socket.create_connection((host, int(port))).makefile(
            "r", encoding="utf-8", errors="replace")
    elif args.file:
        stream = open(args.file, "r", encoding="utf-8", errors="replace")
    else:
        stream = sys.stdin

    r = sniff(stream, args.pattern, n=args.lines, unit=args.unit,
              anchor_prefix=args.prefix)

    print(f"読んだ行  {r['lines']}")
    print(f"解釈できた行  {r['matched']}")
    print(f"使った正規表現  {r['pattern']}")
    print()
    print("生の行:")
    for ln in r["samples"]:
        print(f"  | {ln}")
    print()
    if not r["matched"]:
        print("測距として解釈できませんでした。--pattern で正規表現を指定してください。")
        print(r"  例: --pattern '(?P<anchor>A\d+):\s*(?P<dist>[\d.]+)'")
        return 1

    print(f"見つかったアンカー ID  {r['anchors']}")
    lo, hi = r["ranges"]
    print(f"距離の範囲  {lo:.3f} 〜 {hi:.3f} m  (単位 --unit {args.unit} として換算)")
    if hi > 200.0:
        print("  警告: 距離が大きすぎます。--unit mm か --unit cm ではありませんか。")
    elif hi < 0.05:
        print("  警告: 距離が小さすぎます。--unit の指定を見直してください。")
    else:
        print("  → 妥当な範囲です。次はアンカー座標を用意してください。")
    print()
    print("この ID と単位をそのまま TextHal に渡せば測位できます:")
    print(f"    ul.TextHal.from_serial(port, baud, r\"{r['pattern']}\",")
    print(f"                           anchors=anchors, unit=\"{args.unit}\""
          + (f", anchor_prefix=\"{args.prefix}\"" if args.prefix else "") + ")")
    return 0


# --------------------------------------------------------------------------- ui


def cmd_ui(args: argparse.Namespace) -> int:
    from .ui.server import serve

    serve(args.host, args.port, open_browser=not args.no_browser)
    return 0


# --------------------------------------------------------------------------- main


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="uwb-loc", description="UWB 測位ライブラリ")
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp: argparse.ArgumentParser) -> None:
        sp.add_argument("--anchors", help="アンカー座標の JSON")
        sp.add_argument("--room", nargs=3, type=float, default=[8.0, 6.0, 2.6],
                        metavar=("X", "Y", "Z"))
        sp.add_argument("--n-low", type=int, default=4, help="下段アンカーの数 (0 で天井のみ)")
        sp.add_argument("--dim", type=int, default=3, choices=(2, 3))
        sp.add_argument("--height", type=float, default=1.2, help="タグ高さ / 2D の固定高さ [m]")

    sp = sub.add_parser("sim", help="シミュレーションして精度を出す")
    common(sp)
    sp.add_argument("--levels", default="Lv0,Lv1,Lv2,Lv3")
    sp.add_argument("--duration", type=float, default=40.0)
    sp.add_argument("--rate", type=float, default=10.0)
    sp.add_argument("--seed", type=int, default=0)
    sp.add_argument("--traj", default="figure8",
                    choices=("figure8", "circle", "static", "random_walk"))
    sp.add_argument("--size", type=float, default=2.0)
    sp.add_argument("--period", type=float, default=24.0)
    sp.add_argument("--sigma0", type=float, default=0.08)
    sp.add_argument("--nlos", type=float, default=0.15, help="NLOS 確率")
    sp.add_argument("--nlos-bias", type=float, default=0.8)
    sp.add_argument("--loss", type=float, default=0.03)
    sp.add_argument("--log", help="観測を JSON Lines で書き出す")
    sp.set_defaults(func=cmd_sim)

    sp = sub.add_parser("replay", help="記録した JSON Lines を測位し直す")
    common(sp)
    sp.add_argument("path")
    sp.add_argument("--level", default="Lv2")
    sp.add_argument("--out", help="結果を CSV で書き出す")
    sp.set_defaults(func=cmd_replay)

    sp = sub.add_parser("gdop", help="アンカー配置の GDOP を評価する")
    common(sp)
    sp.add_argument("--nx", type=int, default=60)
    sp.add_argument("--ny", type=int, default=40)
    sp.set_defaults(func=cmd_gdop)

    sp = sub.add_parser("survey", help="相互測距行列 (CSV) からアンカー配置を推定する")
    sp.add_argument("matrix")
    sp.add_argument("--dim", type=int, default=3, choices=(2, 3))
    sp.set_defaults(func=cmd_survey)

    sp = sub.add_parser("sniff", help="実機の出力を覗いて測距として読めるか確かめる")
    sp.add_argument("--serial", help="シリアルポート (例 /dev/ttyUSB0)")
    sp.add_argument("--baud", type=int, default=115200)
    sp.add_argument("--tcp", help="host:port")
    sp.add_argument("--file", help="保存したログ")
    sp.add_argument("--pattern", help="正規表現 (省略すると推測する)")
    sp.add_argument("--unit", default="m", choices=("m", "cm", "mm"))
    sp.add_argument("--prefix", default="", help="アンカー ID の接頭辞 (例 A)")
    sp.add_argument("--lines", type=int, default=40, help="読む行数")
    sp.set_defaults(func=cmd_sniff)

    sp = sub.add_parser("ui", help="ブラウザ UI を起動する")
    sp.add_argument("--host", default="127.0.0.1")
    sp.add_argument("--port", type=int, default=8765)
    sp.add_argument("--no-browser", action="store_true")
    sp.set_defaults(func=cmd_ui)

    args = p.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
