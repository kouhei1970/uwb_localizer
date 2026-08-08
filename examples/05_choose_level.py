"""05 — 測位レベル Lv0〜Lv3 の選び方 (チュートリアル第 4 章)

「とりあえず一番良さそうなの」で Lv3 を選ぶと、条件によっては損をする。
**どの条件でどれが効くのか**を測って決める。

    python examples/05_choose_level.py

見どころ
--------
* NLOS が増えるとレベル間の差が開く (見通しが良ければ Lv0 でも足りる)
* **アンカーが少ないと Lv3 は Lv2 に負ける** — 台数が先、アルゴリズムは後
* Lv3 は時刻が正しくないと使えない。時刻が無いログの救い方
"""

from __future__ import annotations

import numpy as np

import uwb_loc as ul

CENTER = np.array([4.0, 3.0, 1.2])
LEVELS = ("Lv0", "Lv1", "Lv2", "Lv3")

EIGHT = [[0.2, 0.2, 2.4], [7.8, 0.2, 0.3], [7.8, 5.8, 2.4], [0.2, 5.8, 0.3],
         [4.0, 0.2, 2.4], [4.0, 5.8, 0.3], [0.2, 3.0, 0.3], [7.8, 3.0, 2.4]]


def run(anchors, level, *, nlos=0.15, n_seed=3, duration=20.0, rate=10.0):
    """1 条件ぶん走らせて RMSE3D と測位率を返す."""
    errs: list[float] = []
    ok = total = 0
    for seed in range(n_seed):
        hal = ul.SimulatedHal(anchors, ul.trajectory.figure8(CENTER),
                              ul.ErrorModel(sigma0=0.08, nlos_prob=nlos),
                              rate_hz=rate, seed=seed)
        for fix in ul.Pipeline(hal, level=level).run(duration=duration):
            total += 1
            if fix.ok:
                ok += 1
                errs.append(float(np.linalg.norm(fix.position - hal.truth(fix.t))))
    rmse = float(np.sqrt(np.mean(np.square(errs)))) if errs else float("nan")
    return rmse, 100.0 * ok / max(total, 1)


anchors8 = ul.make_anchors(np.array(EIGHT), prefix="A")

# ------------------------------------------------------------------ NLOS
print("=" * 72)
print("(1) NLOS が増えるとどうなるか  — アンカー 8 台、σ=8cm")
print("=" * 72)
print(f"{'NLOS 率':>8}" + "".join(f"{lv:>9}" for lv in LEVELS) + "   効く順")

for nlos in (0.0, 0.15, 0.35):
    row = {lv: run(anchors8, lv, nlos=nlos)[0] for lv in LEVELS}
    best = min(row, key=lambda k: row[k])
    line = f"{nlos * 100:>7.0f}%" + "".join(f"{row[lv]:>9.3f}" for lv in LEVELS)
    print(f"{line}   {best} が最良 (Lv0 比 {row['Lv0'] / row[best]:.1f} 倍改善)")

print("""
  見通しが良い (NLOS 0%) なら Lv0 と Lv2 の差は小さい。**手法の差は
  NLOS があってはじめて出る。** 屋内で人が動くなら Lv2 以上にする。
""")

# ------------------------------------------------------------------ 台数
print("=" * 72)
print("(2) 台数が少ないと Lv3 は「速いが時々外す」  — NLOS 15%、8 seed")
print("=" * 72)
print("  平均だけ見ると分からないので、seed ごとに出して中央値と最悪を並べる。")
print(f"{'台数':>5}{'Lv2中央':>9}{'Lv2最悪':>9}{'Lv3中央':>9}{'Lv3最悪':>9}"
      f"{'測位率':>8}   判断")


def per_seed(anchors, level, seeds=8, duration=30.0):
    """seed ごとの RMSE を返す (平均に埋もれる破綻を見つけるため)."""
    out = []
    ok = total = 0
    for seed in range(seeds):
        hal = ul.SimulatedHal(anchors, ul.trajectory.figure8(CENTER),
                              ul.ErrorModel(sigma0=0.08, nlos_prob=0.15),
                              rate_hz=10.0, seed=seed)
        errs = []
        for fix in ul.Pipeline(hal, level=level).run(duration=duration):
            total += 1
            if fix.ok:
                ok += 1
                errs.append(float(np.linalg.norm(fix.position - hal.truth(fix.t))))
        out.append(float(np.sqrt(np.mean(np.square(errs)))) if errs else float("nan"))
    return np.array(out), 100.0 * ok / max(total, 1)


for n in (4, 5, 6, 8):
    sub = ul.make_anchors(np.array(EIGHT[:n]), prefix="A")
    a, rate = per_seed(sub, "Lv2")
    b, _ = per_seed(sub, "Lv3")
    blowup = b.max() > a.max()
    verdict = "Lv3 が時々外す" if blowup else "Lv3 が安定して有利"
    print(f"{n:>5}{np.median(a):>9.3f}{a.max():>9.3f}{np.median(b):>9.3f}"
          f"{b.max():>9.3f}{rate:>7.1f}%   {verdict}")

print("""
  **Lv3 は中央値ではどの台数でも Lv2 より良い。** 差が出るのは最悪ケース。
  4〜5 台では Lv3 の最悪が Lv2 の最悪を大きく上回る —— 幾何が痩せている
  ところに欠測が重なると、EKF が一度ずれてから戻るのに時間がかかるため。

  つまり選び方はこうなる:
    * 平均精度がほしい          → 台数が少なくても Lv3
    * **外れないことが大事**    → 台数が少ないうちは Lv2 の方が読める
    * 6 台以上にすれば          → Lv3 が中央値も最悪も上回る (悩まなくてよい)
""")

# ------------------------------------------------------------------ 時刻
print("=" * 72)
print("(3) Lv3 は時刻が要る  — 時刻が無いログをどう救うか")
print("=" * 72)

# ログを 1 本作る (時刻つき)
hal = ul.SimulatedHal(anchors8, ul.trajectory.figure8(CENTER),
                      ul.ErrorModel(sigma0=0.08, nlos_prob=0.10), rate_hz=10.0, seed=0)
batches, truths = [], []
hal.open()
for _ in range(200):
    for b in hal.poll(0.1):
        batches.append(b)
        truths.append(hal.truth(b.t))
hal.close()


def score(estimator, feed) -> float:
    errs = []
    for batch, truth in zip(feed, truths):
        fix = estimator.update(batch)
        if fix.ok:
            errs.append(float(np.linalg.norm(fix.position - truth)))
    return float(np.sqrt(np.mean(np.square(errs)))) if errs else float("nan")


# (a) ファームが出した正しい時刻
a = score(ul.make_estimator("Lv3", anchors8), batches)

# (b) 時刻が無い = 全部 0 のまま流し込む (よくある失敗)
no_time = [ul.MeasurementBatch(t=0.0, measurements=b.measurements) for b in batches]
b_ = score(ul.make_estimator("Lv3", anchors8), no_time)

# (c) 一定レートだと分かっているので時刻を合成する
synth = [ul.MeasurementBatch(t=i * 0.1, measurements=x.measurements)
         for i, x in enumerate(batches)]
c = score(ul.make_estimator("Lv3", anchors8), synth)

# (d) そもそも時刻を使わない Lv2 なら影響を受けない
d = score(ul.make_estimator("Lv2", anchors8), no_time)

print(f"  (a) ファームが出す時刻で Lv3          RMSE3D {a:.3f} m")
print(f"  (b) 時刻が無いまま Lv3               RMSE3D {b_:.3f} m   ← 壊れる")
print(f"  (c) 10Hz と分かっているので合成 + Lv3  RMSE3D {c:.3f} m   ← 戻る")
print(f"  (d) 時刻を使わない Lv2                RMSE3D {d:.3f} m   ← 影響なし")

print("""
  時刻が取れないなら Lv2 を使う。一定レートで届くと分かっているなら、
  TextHal(rate_hz=10.0) や replay --rate 10 で合成すれば Lv3 が使える。
""")

print("=" * 72)
print("まとめ — 迷ったら Lv2")
print("=" * 72)
print("""  Lv0   配線と座標系の確認用。最初の 1 回だけ使う
  Lv1   見通しが良いと分かっている環境
  Lv2   屋内の既定。時刻が要らず、NLOS に強い ← 迷ったらこれ
  Lv3   移動体 + 正しい時刻 + アンカー 6 台以上。条件が揃えば最良

  差し替えは名前を変えるだけ:  ul.make_estimator("Lv2", anchors)""")
