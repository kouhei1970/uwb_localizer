"""04 — アンカーの置き方と台数で精度がどう変わるか (チュートリアル第 3 章)

現場で精度が出ない原因は、たいていアルゴリズムではなく設営。
**測位器も測距誤差も固定したまま、配置だけ変えて**測ってみる。

    python examples/04_anchor_layout.py

結論を先に書くと、実際に測ると次のようになる。

1. **GDOP は「置く前に」ひどい配置を弾ける。** 実測しなくても分かる
2. **4 台ではどう置いても苦しい。** 配置を工夫するより台数を足す方が効く
3. **同一平面の害は RMSE に出ない。** 鏡像解という別種の壊れ方をする

3 が厄介で、平均誤差だけ見ていると「天井 4 隅で十分」に見えてしまう。
"""

from __future__ import annotations

import warnings

import numpy as np

import uwb_loc as ul

CENTER = np.array([4.0, 3.0, 1.2])       # 8x6x2.6 m の部屋の真ん中あたり

LAYOUTS: dict[str, list[list[float]]] = {
    # 天井の 4 隅に貼っただけ。いちばんやりがち
    "4台 天井4隅": [
        [0.2, 0.2, 2.4], [7.8, 0.2, 2.4], [7.8, 5.8, 2.4], [0.2, 5.8, 2.4]],
    # 同じ 4 台を壁の片側だけ低くした。平面ではあるが細長い
    "4台 隣り合う2台を低く": [
        [0.2, 0.2, 2.4], [7.8, 0.2, 2.4], [7.8, 5.8, 0.3], [0.2, 5.8, 0.3]],
    # 対角で高さを変える。平面ではなくなる
    "4台 対角で高さを変える": [
        [0.2, 0.2, 2.4], [7.8, 0.2, 0.3], [7.8, 5.8, 2.4], [0.2, 5.8, 0.3]],
    # 台数を足す
    "6台 高さをばらす": [
        [0.2, 0.2, 2.4], [7.8, 0.2, 0.3], [7.8, 5.8, 2.4], [0.2, 5.8, 0.3],
        [4.0, 0.2, 2.4], [4.0, 5.8, 0.3]],
}


def evaluate(anchors: list[ul.Anchor], n_seed: int = 3) -> dict[str, float]:
    """同じ誤差モデル・同じ測位器で走らせて誤差を集める."""
    e3: list[float] = []
    ez: list[float] = []
    ok = total = ambiguous = 0
    for seed in range(n_seed):
        hal = ul.SimulatedHal(anchors, ul.trajectory.figure8(CENTER),
                              ul.ErrorModel(sigma0=0.08, nlos_prob=0.15),
                              rate_hz=10.0, seed=seed)
        # 同一平面の警告は承知のうえ (それが見せたいことなので)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            for fix in ul.Pipeline(hal, level="Lv2").run(duration=20.0):
                total += 1
                if not fix.ok:
                    continue
                ok += 1
                ambiguous += bool(fix.ambiguous)
                d = fix.position - hal.truth(fix.t)
                e3.append(float(np.linalg.norm(d)))
                ez.append(abs(float(d[2])))

    def rms(v: list[float]) -> float:
        return float(np.sqrt(np.mean(np.square(v)))) if v else float("nan")

    return {"rmse3d": rms(e3), "rmsez": rms(ez),
            "rate": 100.0 * ok / max(total, 1),
            "ambiguous": 100.0 * ambiguous / max(ok, 1)}


print("=" * 78)
print("配置だけ変えて比べる — 測距 σ=8cm、NLOS 15%、Lv2、3 seed、20 秒 x 3")
print("=" * 78)
print(f"{'配置':<24}{'同一平面':>9}{'GDOP':>7}{'RMSE3D':>9}{'RMSEz':>8}"
      f"{'測位率':>8}{'鏡像疑い':>9}")

for name, positions in LAYOUTS.items():
    anchors = ul.make_anchors(np.array(positions), prefix="A")
    cond = ul.anchor_condition(anchors)
    r = evaluate(anchors)
    print(f"{name:<24}{str(cond['coplanar']):>9}{ul.gdop_at(CENTER, anchors):>7.2f}"
          f"{r['rmse3d']:>9.3f}{r['rmsez']:>8.3f}{r['rate']:>7.1f}%{r['ambiguous']:>8.0f}%")

print("""
読み取り方

  GDOP を先に見る
      「隣り合う 2 台を低く」は GDOP が桁違いに大きい。これは**置く前に**
      計算できるので、現場に脚立を持ち込む前に弾ける。

  4 台はどう置いても苦しい
      配置を変えても RMSE は 0.6〜1.0 m の幅を出ない。測位率も 90% 前後
      —— NLOS や欠測で 1 本落ちると、4 台では解けなくなるため。
      **配置を工夫するより 6 台にする方がずっと効く。**

  RMSE だけ見ていると同一平面の害を見落とす
      「天井 4 隅」は RMSE では悪くない。しかし次に見るとおり、
      これは**平均に出ない壊れ方**を隠している。
""")

# ---------------------------------------------------------------- 鏡像解
print("=" * 78)
print("同一平面の本当の害 — 鏡像解")
print("=" * 78)

flat = ul.make_anchors(np.array(LAYOUTS["4台 天井4隅"]), prefix="A")
truth = np.array([4.0, 3.0, 1.2])
mirror = np.array([4.0, 3.0, 2.0 * 2.4 - 1.2])     # z=2.4 の平面に関する鏡映

for label, p in (("真値", truth), ("鏡像", mirror)):
    d = [round(float(np.linalg.norm(p - a.position)), 4) for a in flat]
    print(f"  {label} {np.round(p, 2)}  各アンカーからの距離 {d}")

print("""
  **距離が 1 本も違わない。** 誤差の問題ではなく、測距値という情報だけでは
  原理的に選べない。真値は床上 1.2 m、鏡像は天井の上 3.6 m。

  ライブラリの対処:
    * 平面配置で 3D の測位器を作ると、コンストラクタで警告を出す
    * どちらの側かを追い続け、途中で飛び移らないようにする
    * それでも確信が持てないときは fix.ambiguous = True で知らせる

  設計側の対処 (上から順に良い):
    1. 高さをばらす。ただし上の表のとおり 4 台では限界がある
    2. SolveConfig(z_bounds=(0.0, 2.3)) で「床と天井の間」と教える
    3. SolveConfig(dim=2, z_fixed=1.2) で高さを固定して 2D で解く
""")

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    plain = ul.make_estimator("Lv2", flat)
    bounded = ul.make_estimator("Lv2", flat, ul.SolveConfig(z_bounds=(0.0, 2.3)))
    flat2d = ul.make_estimator("Lv2", flat, ul.SolveConfig(dim=2, z_fixed=1.2))

hal = ul.SimulatedHal(flat, ul.trajectory.figure8(CENTER),
                      ul.ErrorModel(sigma0=0.08, nlos_prob=0.15), rate_hz=10.0, seed=3)
hal.open()
scores: dict[str, list[float]] = {"制約なし": [], "z_bounds": [], "2D (z 固定)": []}
for _ in range(200):
    for batch in hal.poll(0.1):
        t = hal.truth(batch.t)
        for label, est in (("制約なし", plain), ("z_bounds", bounded), ("2D (z 固定)", flat2d)):
            fix = est.update(batch)
            if fix.ok:
                scores[label].append(float(np.linalg.norm(fix.position - t)))
hal.close()

print("  天井 4 隅のまま、制約の与え方だけ変えたときの RMSE3D:")
for label, v in scores.items():
    if v:
        print(f"    {label:<12} {np.sqrt(np.mean(np.square(v))):.3f} m  ({len(v)} 回)")

# ------------------------------------------------------ 座標を測らずに済ませる
print()
print("=" * 78)
print("巻き尺で全台測らずに済ませる — 相互測距から座標を復元")
print("=" * 78)

anchors = ul.make_anchors(np.array(LAYOUTS["6台 高さをばらす"]), prefix="A")
P = np.array([a.position for a in anchors])
rng = np.random.default_rng(0)
D = np.linalg.norm(P[:, None] - P[None, :], axis=-1) + rng.normal(0.0, 0.05, (6, 6))
D = (D + D.T) / 2.0                     # 距離行列は対称
np.fill_diagonal(D, 0.0)

recovered = ul.self_survey(D, [a.id for a in anchors], dim=3)
# 復元しただけでは向きと原点が決まらない。実測した 4 台に合わせる
recovered = ul.align_to_reference(recovered, {a.id: a.position for a in anchors[:4]})

err = [float(np.linalg.norm(r.position - a.position))
       for r, a in zip(recovered, anchors)]
print(f"  測距に 5 cm の誤差を入れて 6 台を復元 → 座標誤差 "
      f"平均 {np.mean(err) * 100:.1f} cm / 最大 {np.max(err) * 100:.1f} cm")
print("  実測が要るのは基準の 4 台だけ。残りは相互測距から出せる。")
print("  CLI からも:  python -m uwb_loc survey distances.csv --dim 3")
