"""コマンドライン.

    python -m uwb_loc ui                     # ブラウザ UI を起動
    python -m uwb_loc sim --levels Lv0,Lv2,Lv3
    python -m uwb_loc replay log.jsonl --anchors anchors.json --level Lv3
    python -m uwb_loc gdop --room 8 6 2.6
    python -m uwb_loc sniff --serial /dev/ttyUSB0   # 実機の出力を覗いて解釈できるか見る
    python -m uwb_loc survey dist.csv        # 相互測距からアンカー配置を推定
    python -m uwb_loc ryuw122 info --serial /dev/ttyUSB0     # RYUW122 の設定を読む
    python -m uwb_loc ryuw122 tag-setup --serial /dev/ttyUSB1 --address TAG00001
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
    """記録したログを測位し直す (JSON Lines / テキストのどちらでも)."""
    given = _load_anchors(args.anchors) or []
    if args.format == "text":
        # 時刻を持たないテキストログは、一定レートとみなして時刻を合成する。
        # そうしないと全観測が同時刻になり Lv3 の予測が止まる。
        hal = TextHal.from_path(
            args.path,
            args.pattern or r"(?P<anchor>[A-Za-z]*\d+)\s*[:=,]\s*(?P<dist>-?[\d.]+)",
            anchors=given, unit=args.unit, anchor_prefix=args.prefix,
            rate_hz=args.rate,
        )
    else:
        hal = JsonLinesHal.from_path(args.path, anchors=given)
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

    matched = getattr(hal, "n_matched", None)
    if matched is not None:
        print(f"解釈できた行 {matched} / 捨てた行 {hal.n_unmatched}")
        if matched == 0:
            print("  測距として解釈できませんでした。--pattern を見直すか、"
                  "sniff で確かめてください。", file=sys.stderr)
            return 2
    if not batches:
        print("観測が 1 つも読めませんでした。--format の指定を確認してください。",
              file=sys.stderr)
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


def _is_number(cell: str) -> bool:
    """空欄 (欠測) も数値扱いにする. ラベルかどうかの判定に使う."""
    if not cell:
        return True
    try:
        float(cell)
    except ValueError:
        return False
    return True


def read_distance_matrix(path: str | Path) -> tuple[np.ndarray, list[str] | None]:
    """相互測距の CSV を読む.

    表計算からそのまま出したものを想定して、**ヘッダ行と ID 列があっても
    無くても読める**。空欄は欠測 (NaN) として扱う。

        ,       A0,    A1,    A2        ← ヘッダ行 (任意)
        A0,     0,     4.12,  6.03      ← ID 列 (任意)
        A1,     4.12,  0,     4.55
        A2,     6.03,  4.55,  0
    """
    rows = [r for r in Path(path).read_text(encoding="utf-8").splitlines() if r.strip()]
    if not rows:
        raise ValueError("CSV が空です")
    cells = [[c.strip() for c in r.split(",")] for r in rows]

    ids: list[str] | None = None
    # ヘッダ行なら先頭以外が全部ラベル。ID 列つきのデータ行 (A0,0,4.12) は
    # 先頭だけがラベルで残りは数値なので、ここで取り違えない。
    if len(cells[0]) > 1 and all(not _is_number(c) for c in cells[0][1:]):
        ids = [c for c in cells[0] if c]                # 隅の空セルは捨てる
        cells = cells[1:]
    if cells and all(row and not _is_number(row[0]) for row in cells):   # 先頭列は ID
        row_ids = [row[0] for row in cells]
        cells = [row[1:] for row in cells]
        if ids is None:
            ids = row_ids

    n = len(cells)
    for i, row in enumerate(cells):
        if len(row) != n:
            raise ValueError(
                f"{i + 1} 行目の列数 {len(row)} が行数 {n} と一致しません。"
                "相互測距行列は正方 (N×N) で渡してください")
    if ids is not None and len(ids) != n:
        raise ValueError(f"ID の数 {len(ids)} が行数 {n} と一致しません")

    return np.array([[float(c) if c else np.nan for c in row] for row in cells]), ids


def cmd_survey(args: argparse.Namespace) -> int:
    try:
        dmat, ids = read_distance_matrix(args.matrix)
    except ValueError as e:
        print(f"CSV を読めませんでした: {e}\n", file=sys.stderr)
        print("期待する形 (ヘッダ行と ID 列は省略可、空欄は欠測):", file=sys.stderr)
        print("        ,A0  ,A1  ,A2", file=sys.stderr)
        print("      A0,0   ,4.12,6.03", file=sys.stderr)
        print("      A1,4.12,0   ,4.55", file=sys.stderr)
        print("      A2,6.03,4.55,0", file=sys.stderr)
        return 2
    try:
        anchors = self_survey(dmat, ids, dim=args.dim)
    except ValueError as e:
        print(f"配置を復元できませんでした: {e}", file=sys.stderr)
        if len(dmat) <= args.dim:
            print(f"  --dim {args.dim} で解くには最低 {args.dim + 1} 台の相互測距が要ります "
                  f"(渡されたのは {len(dmat)} 台)。高さが分かっているなら --dim 2 で 3 台から解けます。",
                  file=sys.stderr)
        return 2
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

    if r.get("looks_like_json"):
        print("この出力は JSON Lines のようです。正規表現は要りません。")
        print("  UI なら形式に「JSON Lines」を、CLI なら --format jsonl を選んでください。")
        print()
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


# --------------------------------------------------------------------------- ryuw122


def _ryuw122_config(args: argparse.Namespace, address: str | None) -> Any:
    from .hal.ryuw122 import Ryuw122Config

    return Ryuw122Config(
        network_id=args.network_id,
        address=address,
        password=args.cpin,
        channel=args.channel,
        bandwidth=args.bandwidth,
        power=args.power,
        calibration_cm=args.cal,
    )


def cmd_ryuw122(args: argparse.Namespace) -> int:
    """RYUW122 を 1 台ずつ設定する / 今の設定を読む / TAG として動かす.

    測位そのものは ``ui`` か Python API がやる。ここは **その前の準備**、
    つまり「並べる前に 1 台ずつ設定を入れる」ための道具。
    設定は Flash に残るので、一度書けば次から要らない。
    """
    import time

    from .hal.ryuw122 import Ryuw122Tag, Ryuw122Terminal

    if args.action == "tag":
        if not args.address:
            print("TAG として動かすには --address が要ります"
                  " (機体ごとに違う 8 バイト ASCII)", file=sys.stderr)
            return 2
        tag = Ryuw122Tag.from_serial(args.serial, args.baud,
                                     config=_ryuw122_config(args, args.address),
                                     payload=args.payload)
        tag.open()
        for line in tag.setup_log:
            print(f"  {line}")
        if tag.last_error:
            print(f"警告: {tag.last_error}", file=sys.stderr)
        print(f"\nTAG {args.address} として AT+TAG_SEND を積み続けます。Ctrl-C で終了。")
        try:
            last = -1
            while True:
                time.sleep(1.0)
                if tag.n_rcv != last:
                    last = tag.n_rcv
                    print(f"  積んだ {tag.n_sent} 回 / 読まれた {tag.n_rcv} 回")
        except KeyboardInterrupt:
            pass
        finally:
            tag.close()
        if tag.n_rcv == 0:
            print("\n一度も読まれていません。ANCHOR 側の NETWORKID / CPIN /"
                  " CHANNEL / BANDWIDTH と、呼んでいるアドレスを確かめてください。")
            return 1
        return 0

    with Ryuw122Terminal.from_serial(args.serial, args.baud) as t:
        if args.action == "info":
            for name, value in t.info().items():
                print(f"  {name:<11} {value if value is not None else '(応答なし)'}")
            return 0

        as_anchor = args.action == "anchor"
        ok = t.provision(_ryuw122_config(args, args.address), as_anchor=as_anchor)
        for line in t.log:
            print(f"  {line}")
        print()
        if not ok:
            print("いくつかのコマンドが +OK を返しませんでした。", file=sys.stderr)
            return 1
        print(f"{'ANCHOR' if as_anchor else 'TAG'} として設定しました (Flash に保存済み)。")
        return 0


# --------------------------------------------------------------------------- ui


def cmd_ui(args: argparse.Namespace) -> int:
    from .ui.server import serve

    serve(args.host, args.port, open_browser=not args.no_browser)
    return 0


# --------------------------------------------------------------------------- main


def build_parser() -> argparse.ArgumentParser:
    """コマンドラインの定義。

    ``main`` から切り出してあるのは、``docs/build_reference.py`` が
    ここを読んでリファレンスを生成するため。定義はここ 1 箇所にある。
    """
    p = argparse.ArgumentParser(prog="uwb-loc", description="UWB 測位ライブラリ")
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp: argparse.ArgumentParser) -> None:
        sp.add_argument("--anchors", help="アンカー座標の JSON。省略すると --room から自動生成")
        sp.add_argument("--room", nargs=3, type=float, default=[8.0, 6.0, 2.6],
                        metavar=("X", "Y", "Z"), help="部屋の大きさ [m]")
        sp.add_argument("--n-low", type=int, default=4,
                        help="下段アンカーの数 (0 で天井のみ = 同一平面になる)")
        sp.add_argument("--dim", type=int, default=3, choices=(2, 3),
                        help="2 なら高さを --height に固定して解く")
        sp.add_argument("--height", type=float, default=1.2, help="タグ高さ / 2D の固定高さ [m]")

    sp = sub.add_parser("sim", help="シミュレーションして精度を出す",
                        description="実機なしで観測を作り、測位レベルごとの精度を出す。")
    common(sp)
    sp.add_argument("--levels", default="Lv0,Lv1,Lv2,Lv3",
                    help="比較する測位レベルをカンマ区切りで")
    sp.add_argument("--duration", type=float, default=40.0, help="測る長さ [s]")
    sp.add_argument("--rate", type=float, default=10.0, help="観測レート [Hz]")
    sp.add_argument("--seed", type=int, default=0, help="乱数の種。変えると誤差の出方が変わる")
    sp.add_argument("--traj", default="figure8",
                    choices=("figure8", "circle", "static", "random_walk"),
                    help="タグの動き方")
    sp.add_argument("--size", type=float, default=2.0, help="軌道の大きさ [m]")
    sp.add_argument("--period", type=float, default=24.0, help="軌道を 1 周する時間 [s]")
    sp.add_argument("--sigma0", type=float, default=0.08, help="測距ノイズの標準偏差 [m]")
    sp.add_argument("--nlos", type=float, default=0.15,
                    help="NLOS (見通しが切れて距離が伸びる) 確率")
    sp.add_argument("--nlos-bias", type=float, default=0.8, help="NLOS のとき伸びる量の平均 [m]")
    sp.add_argument("--loss", type=float, default=0.03, help="測距が欠測する確率")
    sp.add_argument("--log", help="観測を JSON Lines で書き出す (replay で読み直せる)")
    sp.set_defaults(func=cmd_sim)

    sp = sub.add_parser("replay", help="記録した JSON Lines を測位し直す",
                        description="記録したログを別の測位レベルで解き直す。"
                                    "現場で録っておけば、あとで何度でも試せる。")
    common(sp)
    sp.add_argument("path", help="読むログ (sim --log か JsonLinesWriter が書いたもの)")
    sp.add_argument("--level", default="Lv2", help="測位レベル (Lv0/Lv1/Lv2/Lv3)")
    sp.add_argument("--format", default="jsonl", choices=("jsonl", "text"),
                    help="ログの形式")
    sp.add_argument("--pattern", help="text のときの正規表現 (省略時は汎用パターン)")
    sp.add_argument("--unit", default="m", choices=("m", "cm", "mm"),
                    help="text のときの距離の単位")
    sp.add_argument("--prefix", default="", help="アンカー ID の接頭辞 (例 A)")
    sp.add_argument("--rate", type=float, default=10.0,
                    help="text のとき時刻を合成するレート [Hz]。Lv3 に要る")
    sp.add_argument("--out", help="結果を CSV で書き出す")
    sp.set_defaults(func=cmd_replay)

    sp = sub.add_parser("gdop", help="アンカー配置の GDOP を評価する",
                        description="置く前に配置の良し悪しを見る。"
                                    "GDOP が大きい場所は、測距が同じ精度でも位置が暴れる。")
    common(sp)
    sp.add_argument("--nx", type=int, default=60, help="ヒートマップの横の分割数")
    sp.add_argument("--ny", type=int, default=40, help="ヒートマップの縦の分割数")
    sp.set_defaults(func=cmd_gdop)

    sp = sub.add_parser("survey", help="相互測距行列 (CSV) からアンカー配置を推定する",
                        description="アンカー同士で測った距離の表から座標を復元する。"
                                    "巻き尺で全台測らずに済む。")
    sp.add_argument("matrix", help="N×N の距離 [m] の CSV。ヘッダ行と ID 列は省略可、空欄は欠測")
    sp.add_argument("--dim", type=int, default=3, choices=(2, 3),
                    help="3 なら最低 4 台、2 なら最低 3 台の相互測距が要る")
    sp.set_defaults(func=cmd_survey)

    sp = sub.add_parser("sniff", help="実機の出力を覗いて測距として読めるか確かめる",
                        description="実機立ち上げの最初にやること。"
                                    "ここで単位とアンカー ID を確定させておく。")
    sp.add_argument("--serial", help="シリアルポート (例 /dev/ttyUSB0)")
    sp.add_argument("--baud", type=int, default=115200, help="シリアルのボーレート")
    sp.add_argument("--tcp", help="TCP で読む場合の host:port")
    sp.add_argument("--file", help="保存したログを読む")
    sp.add_argument("--pattern", help="正規表現 (省略すると推測する)")
    sp.add_argument("--unit", default="m", choices=("m", "cm", "mm"), help="距離の単位")
    sp.add_argument("--prefix", default="", help="アンカー ID の接頭辞 (例 A)")
    sp.add_argument("--lines", type=int, default=40, help="読む行数")
    sp.set_defaults(func=cmd_sniff)

    sp = sub.add_parser("ryuw122", help="REYAX RYUW122 を 1 台ずつ設定する",
                        description="並べる前の準備。設定は Flash に残るので一度書けば済む。")
    sp.add_argument("action", choices=("info", "anchor", "tag-setup", "tag"),
                    help="info: 今の設定を読む / anchor: ANCHOR に設定 / "
                         "tag-setup: TAG に設定 / tag: TAG に設定して動かし続ける")
    sp.add_argument("--serial", required=True, help="シリアルポート (例 /dev/ttyUSB0)")
    sp.add_argument("--baud", type=int, default=115200, help="シリアルのボーレート")
    sp.add_argument("--address", help="この機体のアドレス (8 バイト ASCII, 機体ごとに変える)")
    sp.add_argument("--network-id", help="NETWORKID (8 バイト ASCII, 全機で同じ)")
    sp.add_argument("--cpin", help="AES128 パスワード (32 文字, 全機で同じ)")
    sp.add_argument("--channel", type=int, choices=(5, 9),
                    help="RF チャネル 5: 6489.6MHz / 9: 7987.2MHz (全機で同じ)")
    sp.add_argument("--bandwidth", type=int, choices=(0, 1),
                    help="データレート 0: 850kbps / 1: 6.8Mbps (全機で同じ)")
    sp.add_argument("--power", type=int, choices=range(6), help="送信出力 0-5 (5 が最大)")
    sp.add_argument("--cal", type=int, help="AT+CAL 距離校正 [cm], -100〜100")
    sp.add_argument("--payload", default="RNGE",
                    help="TAG が積むデータ (ANCHOR 側と 3 バイト以内の差にする)")
    sp.set_defaults(func=cmd_ryuw122)

    sp = sub.add_parser("ui", help="ブラウザ UI を起動する",
                        description="配置・誤差モデル・測位レベルの比較と、実機のライブ表示。")
    sp.add_argument("--host", default="127.0.0.1",
                    help="0.0.0.0 にすると同じ LAN の端末から開ける (認証は無い)")
    sp.add_argument("--port", type=int, default=8765, help="待ち受けポート")
    sp.add_argument("--no-browser", action="store_true", help="ブラウザを自動で開かない")
    sp.set_defaults(func=cmd_ui)

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
