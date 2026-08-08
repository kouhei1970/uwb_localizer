"""06 — 位置が出ない・飛ぶときの切り分け (チュートリアル第 6 章)

現場でハマる典型的な状況を**わざと作って**、どの数字を見れば原因が
分かるかを確かめる。症状はどれも「位置がおかしい」に見えるが、
`Fix` の中身を見れば区別できる。

    python examples/06_troubleshooting.py

扱う症状
--------
1. 距離が足りず解けない          → fix.ok / n_total
2. アンカー座標が間違っている      → Lv0 の residual_rms
3. 特定の 1 台だけ悪い            → excluded
4. アンテナ遅延で距離がずれている  → 既知距離で測って分離・校正
5. 単位を取り違えている            → sniff で気づく
"""

from __future__ import annotations

import numpy as np

import uwb_loc as ul

ANCHORS = [
    ul.Anchor("A0", [0.2, 0.2, 2.4]), ul.Anchor("A1", [7.8, 0.2, 0.3]),
    ul.Anchor("A2", [7.8, 5.8, 2.4]), ul.Anchor("A3", [0.2, 5.8, 0.3]),
    ul.Anchor("A4", [4.0, 0.2, 2.4]), ul.Anchor("A5", [4.0, 5.8, 0.3]),
]
TRUTH = np.array([3.1, 2.4, 1.2])


def clean_batch(anchors=ANCHORS, truth=TRUTH, *, noise=0.0, seed=0):
    """真値どおりの距離を作る (誤差を入れたければ noise を指定)."""
    rng = np.random.default_rng(seed)
    return ul.MeasurementBatch(t=0.0, measurements=[
        ul.Measurement(a.id, float(np.linalg.norm(truth - a.position)
                                   + rng.normal(0.0, noise)))
        for a in anchors])


def show(label: str, fix: ul.Fix) -> None:
    if not fix.ok:
        print(f"  {label:<26} 測位できず  (届いた {fix.n_total} 本)")
        return
    err = float(np.linalg.norm(fix.position - TRUTH))
    print(f"  {label:<26} 誤差 {err:>6.3f} m  残差 {fix.residual_rms:>6.3f} m  "
          f"σ {fix.sigma:>5.2f}  使用 {fix.n_used}/{fix.n_total}  "
          f"除外 {fix.excluded or '-'}")


print("=" * 78)
print("症状 1: そもそも位置が出ない  →  fix.ok と fix.n_total を見る")
print("=" * 78)

est = ul.make_estimator("Lv2", ANCHORS)
full = clean_batch(noise=0.03)
for keep in (6, 4, 3, 0):
    b = ul.MeasurementBatch(t=0.0, measurements=full.measurements[:keep])
    show(f"距離 {keep} 本", est.update(b))

print("""
  3D は未知数が 3 つなので **最低 4 本**要る。3 本以下は解けない。
  n_total が 0 なら測位以前の問題 — HAL か配線。sniff に戻る。
""")

# ------------------------------------------------------------------ 座標ミス
print("=" * 78)
print("症状 2: アンカー座標が間違っている  →  Lv0 の残差で見る")
print("=" * 78)

wrong = [ul.Anchor(a.id, a.position.copy()) for a in ANCHORS]
wrong[2].position[1] += 2.0            # A2 の y を 2 m 間違えて入力した

for level in ("Lv0", "Lv1", "Lv2"):
    print(f"  --- {level} ---")
    show("正しい座標", ul.make_estimator(level, ANCHORS).update(clean_batch(noise=0.03)))
    show("A2 の y を 2m 間違えた",
         ul.make_estimator(level, wrong).update(clean_batch(noise=0.03)))

print("""
  ここが落とし穴。**Lv2 は座標ミスを「隠して」しまう。**
  A2 を外れ値として捨てるので位置は保たれ、残差も小さいまま。
  一方 Lv0 はロバスト化しないので、残差が 0.008 → 0.4 m に跳ね、
  位置も 1.7 m ずれる —— **壊れていることがすぐ分かる。**

  だから切り分けはこうする:
    * **立ち上げ時は Lv0 で確認する。** 座標と単位の誤りが素直に出る
    * 運用は Lv2。ただし **excluded を必ず見る**。同じ ID が出続けるなら、
      それは「ロバスト化が肩代わりしている座標ミス」かもしれない
""")

# ------------------------------------------------------------------ 1 台だけ悪い
print("=" * 78)
print("症状 3: 特定の 1 台だけ悪い  →  excluded に同じ ID が出続ける")
print("=" * 78)

est = ul.make_estimator("Lv2", ANCHORS)
counts: dict[str, int] = {}
for i in range(60):
    b = clean_batch(noise=0.03, seed=i)
    # A3 だけ常に 1.5 m 長く出る (見通しが悪い場所に置いてしまった)
    for m in b.measurements:
        if m.anchor_id == "A3":
            m.value += 1.5
    fix = est.update(b)
    for aid in fix.excluded:
        counts[aid] = counts.get(aid, 0) + 1

print(f"  60 エポック中、除外された回数: {counts or '(なし)'}")
print("""
  **いつも同じ ID が出るなら、そのアンカーが原因。** 座標の入力ミス、
  設置場所 (金属の裏、人の動線)、あるいはアンテナ遅延を疑う。
  Lv2 はこれを自動で外してくれるので位置は保たれるが、
  「外れていることに気づく」ために excluded は毎回見る価値がある。
""")

# ------------------------------------------------------------------ アンテナ遅延
print("=" * 78)
print("症状 4: 距離に一定のずれがある  →  アンテナ遅延。分離して校正できる")
print("=" * 78)

# 実際には台ごとに違う遅延が乗り、さらにタグ側にも共通の遅延がある
PER_ANCHOR = {"A0": 0.10, "A1": -0.05, "A2": 0.22, "A3": 0.03, "A4": -0.12, "A5": 0.07}
TAG_DELAY = 0.18


def delayed_batch(anchors=ANCHORS, *, noise=0.01, seed=0):
    b = clean_batch(anchors, noise=noise, seed=seed)
    for m in b.measurements:
        m.value += PER_ANCHOR[m.anchor_id] + TAG_DELAY
    return b


show("補正なし", ul.make_estimator("Lv2", ANCHORS).update(delayed_batch()))

# 真の距離が分かる場所に置いて何度か測り、遅延を分離する
true_d = np.array([float(np.linalg.norm(TRUTH - a.position)) for a in ANCHORS])
ids = [a.id for a in ANCHORS] * 8
measured = np.concatenate([
    true_d + np.array([PER_ANCHOR[a.id] for a in ANCHORS]) + TAG_DELAY
    for _ in range(8)])
delays = ul.estimate_antenna_delays(ids, measured, np.concatenate([true_d] * 8))

print("  推定した遅延 [m] (真値と比べる):")
for a in ANCHORS:
    print(f"    {a.id}  推定 {delays[a.id]:+.3f}   真値 {PER_ANCHOR[a.id]:+.3f}"
          f"   差 {delays[a.id] - PER_ANCHOR[a.id]:+.3f}")
print(f"    タグ側 推定 {delays['__tag__']:+.3f}   真値 {TAG_DELAY:+.3f}"
      f"   差 {delays['__tag__'] - TAG_DELAY:+.3f}")

print("""
  **どれも同じ量だけずれている。** アンカー側とタグ側の遅延は、定数分だけ
  分離できない (どちらに寄せても測距値は同じ) ため。ライブラリはアンカー
  遅延の平均を 0 に固定して解き、残りを __tag__ に寄せている。

  **合計は正しい**ので、補正するときは両方を足して入れる:""")

calibrated = [
    ul.Anchor(a.id, a.position,
              antenna_delay_m=delays[a.id] + delays["__tag__"])
    for a in ANCHORS]
show("遅延を補正", ul.make_estimator("Lv2", calibrated).update(delayed_batch()))

print("""
  RYUW122 なら AT+CAL でも粗く合わせられるが、そちらは 1 cm 刻みの
  モジュール単体の設定。Anchor.antenna_delay_m は台ごとに小数で持てるので、
  仕上げはこちらの方が扱いやすい。
""")

# ------------------------------------------------------------------ 単位
print("=" * 78)
print("症状 5: 単位を取り違えている  →  sniff が範囲で教えてくれる")
print("=" * 78)

import io  # noqa: E402

raw = "\n".join(f"range,{i},{int(d * 1000)}" for i, d in enumerate(true_d))
for unit in ("m", "mm"):
    r = ul.sniff(io.StringIO(raw), r"range,(?P<anchor>\d+),(?P<dist>\d+)",
                 n=10, unit=unit, anchor_prefix="A")
    lo, hi = r["ranges"]
    verdict = "妥当" if 0.05 < hi < 200 else "**おかしい**"
    print(f"  --unit {unit:<3} と解釈 → 距離 {lo:.3f} 〜 {hi:.3f} m   {verdict}")

print("""
  ファームは mm で出しているのに m と解釈すると、距離が 1000 倍になる。
  **実機をつなぐ前に必ず sniff を通す。**
      python -m uwb_loc sniff --serial /dev/ttyUSB0 --unit mm
""")

# ------------------------------------------------------------------ 録る
print("=" * 78)
print("現場では悩まず、まず録る")
print("=" * 78)
print("""  writer = ul.JsonLinesWriter("run.jsonl")
  writer.write_anchors(anchors)      # 先頭にアンカー座標を 1 行
  for batch in ...:
      writer.write(batch)            # 観測を 1 エポック 1 行
  writer.close()

  録っておけば、あとから何度でも解き直せる:
      python -m uwb_loc replay run.jsonl --level Lv3
      python -m uwb_loc replay run.jsonl --level Lv2 --out fixes.csv

  座標を間違えていても、ログさえあれば座標を直して解き直せる。
  **現場で正解にたどり着こうとしないこと。**""")
