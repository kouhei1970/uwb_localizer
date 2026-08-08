"""Python 版と C 版に同じ観測を食わせて、結果を突き合わせる。

移植が正しいかを確かめる一番早い方法。数式を読み比べるより、
**同じ入力で同じ数字が出るか**を見た方が早いし確実。

    python tools/crossval.py                 # 既定のシナリオ一式
    python tools/crossval.py --tol 1e-6      # 一致の判定を厳しくする

C 側は c/examples/03_replay_jsonl が CSV を吐くので、それを読んで比べる。
`make -C c` が通っていることが前提。
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import uwb_loc as ul  # noqa: E402

REPLAY = ROOT / "c" / "examples" / "03_replay"

#: C 版と Python 版で既定が違うところを、突き合わせのためにそろえる。
#:
#: * RANSAC — C 版は持たない (組込みには重いので入れていない)
#: * warm_start — C のスナップショット解は状態を持たない (毎回初期値から解く)
#:
#: どちらも c/README.md「Python 版との違い」に書いてある。
LEVEL_KW = {
    "Lv1": {"warm_start": False},
    "Lv2": {"use_ransac": False, "warm_start": False},
}


def make_case(name, anchors, traj, *, nlos=0.0, sigma0=0.08, n=60, seed=0, rate=10.0):
    """観測列を作って JSON Lines に書き出す。"""
    hal = ul.SimulatedHal(anchors, traj,
                          ul.ErrorModel(sigma0=sigma0, nlos_prob=nlos, loss_rate=0.0),
                          rate_hz=rate, seed=seed)
    batches = []
    hal.open()
    while len(batches) < n:
        got = hal.poll(0.2)
        if not got:
            break
        batches.extend(got)
    hal.close()
    return {"name": name, "anchors": anchors, "batches": batches[:n]}


def write_inputs(case, tmp: Path):
    """C 側が読む素朴な行形式で書き出す (JSON を C で読ませない)。"""
    path = tmp / "input.txt"
    index = {a.id: i for i, a in enumerate(case["anchors"])}

    def r(v):
        """numpy スカラーの repr (np.float64(0.2)) を C に渡さないようにする。
        17 桁あれば double を往復しても値が変わらない。"""
        return repr(float(v))

    with path.open("w", encoding="utf-8") as f:
        for a in case["anchors"]:
            f.write(f"A {a.id} {r(a.position[0])} {r(a.position[1])} {r(a.position[2])} "
                    f"{r(a.antenna_delay_m)} {r(a.sigma0)} {r(a.sigma_per_m)}\n")
        for b in case["batches"]:
            for m in b.measurements:
                if m.anchor_id not in index:
                    continue
                sigma = m.sigma if m.sigma is not None else 0.0
                quality = m.quality if m.quality is not None else -1.0
                f.write(f"M {index[m.anchor_id]} {r(m.value)} {r(sigma)} {r(quality)}\n")
            f.write(f"E {r(b.t)}\n")
    return path


def run_python(case, level):
    kw = dict(LEVEL_KW.get(level, {}))
    if level == "Lv3":
        # EKF の立ち上げは Lv2 を使う。その Lv2 も C 側とそろえる。
        kw["init_estimator"] = ul.Lv2RobustNLS(
            case["anchors"], use_ransac=False, warm_start=False)
    est = ul.make_estimator(level, case["anchors"], **kw)
    out = []
    for b in case["batches"]:
        fix = est.update(b)
        out.append((b.t, fix))
    return out


def run_c(path, level):
    r = subprocess.run([str(REPLAY), str(path), level],
                       capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        raise RuntimeError(f"C 側が失敗: {r.stderr}")
    rows = list(csv.DictReader(r.stdout.splitlines()))
    return rows


def compare(case, level, tmp, tol):
    path = write_inputs(case, tmp)
    py = run_python(case, level)
    c = run_c(path, level)

    if len(py) != len(c):
        return {"name": case["name"], "level": level, "ok": False,
                "why": f"行数が違う: Python {len(py)} / C {len(c)}"}

    worst = 0.0
    worst_at = -1
    mismatched_ok = 0
    n_ok = 0
    for i, ((t, fix), row) in enumerate(zip(py, c)):
        c_ok = int(row["ok"]) == 1
        if bool(fix.ok) != c_ok:
            mismatched_ok += 1
            continue
        if not fix.ok:
            continue
        n_ok += 1
        cp = np.array([float(row["x"]), float(row["y"]), float(row["z"])])
        d = float(np.linalg.norm(fix.position - cp))
        if d > worst:
            worst, worst_at = d, i

    return {"name": case["name"], "level": level,
            "ok": mismatched_ok == 0 and worst <= tol and n_ok > 0,
            "worst": worst, "worst_at": worst_at,
            "n_ok": n_ok, "mismatched_ok": mismatched_ok}


def build_cases():
    room = [
        [0.2, 0.2, 2.4], [7.8, 0.2, 0.3], [7.8, 5.8, 2.4], [0.2, 5.8, 0.3],
        [4.0, 0.2, 2.4], [4.0, 5.8, 0.3], [0.2, 3.0, 0.3], [7.8, 3.0, 2.4],
    ]
    a8 = ul.make_anchors(np.array(room), prefix="A")
    a4 = ul.make_anchors(np.array(room[:4]), prefix="A")
    center = [4.0, 3.0, 1.2]

    return [
        make_case("静止・無雑音・8台", a8, ul.trajectory.static(center), sigma0=1e-9),
        make_case("静止・雑音あり・8台", a8, ul.trajectory.static(center), sigma0=0.08, seed=1),
        make_case("8の字・雑音あり・8台", a8, ul.trajectory.figure8(center), sigma0=0.08, seed=2),
        make_case("8の字・NLOS 30%・8台", a8, ul.trajectory.figure8(center),
                  sigma0=0.08, nlos=0.30, seed=3),
        make_case("円・最小構成 4台", a4, ul.trajectory.circle(center), sigma0=0.05, seed=4),
    ]


def main():
    ap = argparse.ArgumentParser(description="Python 版と C 版の突き合わせ")
    ap.add_argument("--tol", type=float, default=1e-6, help="一致とみなす位置差 [m]")
    ap.add_argument("--levels", default="Lv0,Lv1,Lv2,Lv3")
    args = ap.parse_args()

    if not REPLAY.exists():
        print(f"C 側の実行ファイルが無い: {REPLAY}\n  先に `make -C c examples` を実行してください",
              file=sys.stderr)
        return 2

    cases = build_cases()
    levels = args.levels.split(",")
    results = []
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        for case in cases:
            for level in levels:
                results.append(compare(case, level, tmp, args.tol))

    width = max(len(r["name"]) for r in results) + 2
    print(f"{'シナリオ':<{width}}{'Lv':>5}{'一致':>7}{'最大差 [m]':>14}{'測位数':>8}")
    print("-" * (width + 34))
    bad = 0
    for r in results:
        if "why" in r:
            print(f"{r['name']:<{width}}{r['level']:>5}{'NG':>7}   {r['why']}")
            bad += 1
            continue
        mark = "OK" if r["ok"] else "NG"
        if not r["ok"]:
            bad += 1
        note = ""
        if r["mismatched_ok"]:
            note = f"  ok が {r['mismatched_ok']} 回食い違う"
        print(f"{r['name']:<{width}}{r['level']:>5}{mark:>7}{r['worst']:>14.2e}"
              f"{r['n_ok']:>8}{note}")

    print("-" * (width + 34))
    if bad == 0:
        print(f"すべて一致 (許容 {args.tol:g} m)")
        return 0
    print(f"{bad} 件が不一致")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
