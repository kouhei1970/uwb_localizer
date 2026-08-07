# uwb_loc — チップ非依存の UWB 測位ライブラリ

各 UWB 用に書いた HAL から観測をもらい、位置・共分散・品質指標を返す。
DW1000 でも DW3000 でも SR150 でも、距離になってしまえば同じコードが動く。

- 測位アルゴリズムの選定理由 → [UWB_POSITIONING.md](UWB_POSITIONING.md)
- **アルゴリズムの導出と実装の詳細 → [UWB_ALGORITHMS.md](UWB_ALGORITHMS.md)**
- HAL とのデータ交換仕様 → [UWB_PROTOCOL.md](UWB_PROTOCOL.md)

依存は **numpy だけ**。scipy を使わず Gauss-Newton も EKF も自前で書いてあるので、
そのまま C に移せる (扱う行列は最大 6×6、一般化固有値も 4×4 まで)。

---

## 30 秒で動かす

```bash
python -m uwb_loc ui        # ブラウザ UI (http://127.0.0.1:8765)
```

ハードがなくても、部屋とアンカーを置いて誤差を振れば、Lv0〜Lv3 の精度差がそのまま見える。

```bash
python -m uwb_loc sim --nlos 0.2 --duration 40
```

```
アンカー 8 台  同一平面=False  平面からの広がり 1.05 m
400 エポック  1 エポックあたり平均 7.7 本

Lv0   測位率 100.0%  RMSE3D  0.631  RMSE2D  0.292  CEP50  0.134  CEP95  0.650  最大  3.955  [m]
Lv1   測位率 100.0%  RMSE3D  0.368  RMSE2D  0.160  CEP50  0.092  CEP95  0.264  最大  1.913  [m]
Lv2   測位率 100.0%  RMSE3D  0.312  RMSE2D  0.146  CEP50  0.081  CEP95  0.207  最大  2.758  [m]
Lv3   測位率 100.0%  RMSE3D  0.153  RMSE2D  0.081  CEP50  0.062  CEP95  0.138  最大  0.595  [m]
```

---

## 測位レベル

**同一インターフェイスで差し替えられる**測位器を 4 段階そろえてある。
呼び出し側のコードはレベルが変わっても 1 行も変わらない。

| Lv | 中身 | 想定 |
|---|---|---|
| **Lv0** | LLS 三辺測量 (閉形式・反復なし) | 配線と座標系の確認、初期値供給 |
| **Lv1** | 重み付き非線形最小二乗 (GN/LM) + χ² ゲート | 見通しの良い環境 |
| **Lv2** | Beck 厳密解 + Huber-IRLS + 片側損失 (+RANSAC) | **NLOS のある屋内の既定** |
| **Lv3** | 密結合 EKF (CV/CA) | **移動体・ドローン** |

```python
import uwb_loc as ul

est = ul.make_estimator("Lv2", anchors)
fix = est.update(batch)
print(fix.position, fix.sigma, fix.gdop, fix.excluded)
```

### なぜ Lv2 が既定なのか

屋内 UWB の精度を決めるのは測距ノイズではなく **NLOS** で、遮蔽されると距離は
**正側にしか伸びない**。ガウス仮定の最小二乗 (Lv1) はこの偏りに弱い。
Lv2 はその物理をそのまま重みに落とす。

- **Beck の厳密解 (GTRS)** を初期解にする — `y = [p; ||p||²]` と置くと 2 次制約付きの
  線形問題になり、1 変数の単調な永年方程式を二分法で解くだけで**大域最適解**が出る。
  Gauss-Newton の初期値依存が原理的に消える
- **Huber-IRLS** — 外れ値を捨てずに重みを下げる。アンカーが少ないときに
  「捨てすぎて解けない」を避けられる
- **片側損失** — 正側の残差だけしきい値を絞る。LOS 側の情報を犠牲にせず NLOS だけ抑える
- **RANSAC は保険**。常時走らせると、最小構成 (4 本) から作る仮解自体の誤差で
  まともな観測まで落としてしまうので、通常の解き方が明らかに失敗したときだけ起動する

### なぜ Lv3 は「密結合」なのか

スナップショット位置をフィルタに入れる (疎結合) のではなく、**測距値そのものを
観測として EKF に直接入れる**。

- アンカーが 3 本未満しか見えないエポックでも**更新できる**。
  疎結合ならそのエポックは丸ごと捨てになる
- TWR はアンカーを順にポーリングするので観測はもともと非同期に届く。
  「1 スキャン = 1 エポック」に束ねる必要がなく、届いた瞬間に predict→update できる
- 幾何が悪い方向の情報だけを部分的に取り込める

調整するパラメータは実質 `sigma_a` (加速度の白色雑音強度) だけにしてある。

```python
est = ul.make_estimator("Lv3", anchors, motion="cv", sigma_a=1.0, gate=3.0)
```

`sigma_a` の目安は**対象が実際に出す加速度の RMS**。歩行者や台車なら 0.2〜0.5、
機敏なドローンなら 2〜5。**外すなら大きめに外すこと** — 小さすぎると機動に
追従できず発散しうるのに対し、大きすぎても精度が少し落ちるだけで済む
(実測: 8 の字周期 5 秒の機動で `sigma_a=0.2` は RMSE 4.4 m、`1.0` なら 0.24 m。
逆に静かな軌道では `0.2` が 0.13 m、`1.0` でも 0.17 m にとどまる)。

`gate` はイノベーションゲート [σ]。NLOS 対策の本体だが、狭くしすぎると
フィルタが自分の誤りを守り続ける。連続して全観測を弾いたら
スナップショット測位からやり直すようにしてある (`max_rejects`)。

---

## 使い方

### 実機

```python
import uwb_loc as ul

hal = ul.JsonLinesHal.from_serial("/dev/ttyUSB0", 115200)   # 方法 B
# hal = MyChipHal("/dev/ttyUSB0")                           # 方法 A (UwbHal を継承)

for fix in ul.Pipeline(hal, level="Lv3", sigma_a=1.0).run():
    if fix.ok and fix.sigma < 0.3:
        print(f"{fix.t:.2f}  {fix.position}  ±{fix.sigma:.2f} m  ({fix.n_used}/{fix.n_total} 本)")
```

HAL の書き方は [UWB_PROTOCOL.md](UWB_PROTOCOL.md)。

### ハードなし (シミュレータ)

`SimulatedHal` は実機と同じ `UwbHal` なので、**測位側のコードは何も変わらない**。

```python
anchors = ul.room_anchors((8.0, 6.0, 2.6))          # 部屋の四隅に上下 2 段
hal = ul.SimulatedHal(
    anchors,
    ul.trajectory.figure8([4, 3, 1.2]),
    ul.ErrorModel(sigma0=0.08, nlos_prob=0.2, nlos_bias_mean=0.8, loss_rate=0.03),
    rate_hz=10, seed=0,
)
times, truth, batches = hal.generate(40.0)

for level in ("Lv0", "Lv2", "Lv3"):
    fixes = ul.run_offline(batches, anchors, level=level)
    stats = ul.error_stats(np.array(truth), np.array([f.position for f in fixes]))
    print(level, stats["rmse_3d"], stats["cep95"])
```

`run_offline` は**同じ観測列**を各レベルに通すので、アルゴリズムの差だけが出る。

### 記録と再生

```python
with ul.JsonLinesWriter("run01.jsonl", anchors) as w:
    for batch in hal.stream():
        w.write(batch)
```

```bash
python -m uwb_loc replay run01.jsonl --level Lv3 --out fixes.csv
```

---

## 設営を助ける道具

現場で精度が出ない原因は、たいていアルゴリズムではなく設営とキャリブレーション。

### 配置を決める (設置前)

```python
ul.gdop_at([4, 3, 1.2], anchors)        # その点の幾何精度劣化係数
ul.crlb_at([4, 3, 1.2], anchors)        # 測距 σ から決まる位置誤差の理論下限 [m]
ul.anchor_condition(anchors)            # 同一平面かどうか
```

```bash
python -m uwb_loc gdop --room 8 6 2.6   # 端末に GDOP マップを描く
```

**3 次元測位では、アンカーが同一平面に並んでいないことが本質的に効く。**
天井の 4 隅に貼っただけでは高さがほとんど観測できず、しかも
その誤差が水平方向にも回り込む。`anchor_condition` が警告を出す。
平面配置しか取れないなら `SolveConfig(dim=2, z_fixed=...)` で高さを固定する方が、
水平精度まで良くなる。

### アンカー座標を測る

```python
anchors = ul.self_survey(dist_matrix, ids, dim=3)     # 相互測距 → 配置
anchors = ul.align_to_reference(anchors, {"A0": [0,0,2.4], ...})  # 実座標に載せる
```

巻き尺で全台測る作業が消える。実測するのは 3〜4 台だけでよい。

### アンテナ遅延

```python
delays = ul.estimate_antenna_delays(anchor_ids, measured, true_distance)
for a in anchors:
    a.antenna_delay_m = delays[a.id]
```

未補正のアンテナ遅延は数十 cm の系統誤差になる。距離バイアスの 1 次モデルは
`ul.fit_range_bias(measured, true)` で当てられる。

---

## 評価指標

RMSE だけ見ていると、たまに出る大外れ (NLOS で 2 m 飛ぶ) を見落とす。
実運用で効くのは分布の裾なので、CEP50/CEP95 と可用性を必ず併記する。

```python
stats = ul.error_stats(truth, estimates)
# availability, rmse_3d, rmse_2d, rmse_x/y/z, cep50, cep95, p95_3d, max_3d, bias_x/y/z
x, p = ul.error_cdf(errors)
```

---

## ブラウザ UI

```bash
python -m uwb_loc ui
```

外部 CDN を一切使わない単一ページなので、ネットワークのない現場でも動く。

- アンカーを平面図上でドラッグして配置し、**GDOP ヒートマップ**で弱い場所を確認
- 誤差モデル (σ、NLOS 確率・継続時間・バイアス、ロス率、アンテナ遅延、設置誤差) を振る
- Lv0〜Lv3 を**同じ観測列**に通して、平面図・側面図・誤差時系列・誤差 CDF・精度表で比較
- 「ライブ」タブから実機の観測 (JSON Lines: ファイル / TCP / シリアル) を流し込んで表示。
  推定 σ の円が位置と一緒に描かれる

アンテナ遅延と設置座標誤差は**測位側に知らせない**ので、設営の失敗がどれだけ効くかを
そのまま観察できる。

---

## CLI

```bash
python -m uwb_loc sim --levels Lv0,Lv2,Lv3 --nlos 0.2 --log run.jsonl
python -m uwb_loc replay run.jsonl --level Lv3 --out fixes.csv
python -m uwb_loc gdop --room 8 6 2.6 --n-low 0     # 天井のみ配置を評価
python -m uwb_loc survey distances.csv --dim 3
python -m uwb_loc ui --port 8765
```

---

## C への移植

Lv0〜Lv3 のどれも scipy を使わず、行列は小さい。移植で必要になるのは

- 3×3 / 4×4 の線形方程式ソルバ (LU で十分)
- 4×4 の一般化固有値 — Beck 法の λ 下限にだけ使う。
  実装が面倒なら、λ を十分小さい負値から二分法で探しても実用上は同じ
- 6×6 の対称行列演算 (EKF。逐次更新なので逆行列は不要、スカラー除算だけ)

観測モデル (`model.py`) の残差とヤコビアンをそのまま写し、
`solve_nls` の Gauss-Newton ループと `Lv3TightlyCoupledEKF` の predict/update を
順に移せばよい。動作の照合には、同じ JSON Lines を Python 版と C 版に食わせて
`Fix` を突き合わせるのが早い。
