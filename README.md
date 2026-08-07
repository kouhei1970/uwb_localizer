# uwb_localizer

UWB のように**距離情報だけ**からタグの位置を割り出すツール。

特定の UWB チップ (DW1000/DW3000, NXP SR150, Qorvo, ESP32-UWB …) に依存しない。
各チップ用に書いた HAL から測距値を受け取り、位置・共分散・品質指標を返す。
チップごとに違うのは「時刻をどう測るか」までで、**距離になってしまえば同じコードが動く**。

依存は **numpy だけ**。scipy を使わず Gauss-Newton も EKF も自前で書いてあるので、
そのまま C に移植できる (扱う行列は最大 6×6)。

```
 ┌─────────┐   Measurement    ┌──────────────────────────────────────┐
 │  HAL    │ ───────────────→ │  前処理    スナップショット    追跡    │ → 位置
 │ (chip)  │  距離 / TDoA /   │  バイアス   閉形式 + WNLS      EKF    │   共分散
 └─────────┘  角度 / 品質値   │  外れ値除去  (Lv0-Lv2)        (Lv3)   │   品質指標
      ↑                       └──────────────────────────────────────┘
 実機 or シミュレータ (同じインターフェイスなので差し替え可能)
```

## インストール

```bash
pip install -e .              # numpy のみ
pip install -e ".[serial]"    # + pyserial (シリアル接続する場合)
pip install -e ".[dev]"       # + pytest
```

## 30 秒で動かす

ハードがなくても動く。部屋とアンカーを置いて誤差を振れば、精度がそのまま見える。

```bash
python -m uwb_loc ui          # ブラウザ UI (http://127.0.0.1:8765)
```

![UI](docs/images/ui.png)

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

## 実機につなぐ

```python
import uwb_loc as ul

hal = ul.JsonLinesHal.from_serial("/dev/ttyUSB0", 115200)   # or 自前の UwbHal
for fix in ul.Pipeline(hal, level="Lv3").run():
    if fix.ok:
        print(f"{fix.t:.2f}  {fix.position}  ±{fix.sigma:.2f} m  ({fix.n_used}/{fix.n_total} 本)")
```

HAL の書き方は 2 通りある。どちらでも測位側のコードは変わらない。

| 方法 | 実装するもの | 向いている場面 |
|---|---|---|
| **Python HAL クラス** | `UwbHal` を継承して `anchors` と `poll` の 2 つだけ | Python から直接チップを叩く |
| **JSON Lines** | ファームウェアが 1 行 1 JSON を print するだけ | Python を書かずに繋ぎたい |

```json
{"v":1,"type":"meas","t":12.345,"tag":"tag0","meas":[
  {"a":"A0","d":3.214,"q":0.93},
  {"a":"A1","d":2.887,"q":0.41}]}
```

## 測位レベル

**同一インターフェイスで差し替えられる**測位器を 4 段階そろえてある。

| Lv | 中身 | 想定 |
|---|---|---|
| **Lv0** | LLS 三辺測量 (閉形式・反復なし) | 配線と座標系の確認、初期値供給 |
| **Lv1** | 重み付き非線形最小二乗 (GN/LM) + χ² ゲート | 見通しの良い環境 |
| **Lv2** | Beck 厳密解 + Huber-IRLS + 片側損失 | **NLOS のある屋内の既定** |
| **Lv3** | 密結合 EKF (CV/CA) | **移動体・ドローン** |

```python
est = ul.make_estimator("Lv2", anchors)
fix = est.update(batch)
print(fix.position, fix.sigma, fix.gdop, fix.excluded)
```

屋内の精度を決めるのは測距ノイズではなく **NLOS** (見通しが切れると距離が伸びる)。
NLOS 率を上げるほどレベル間の差が開く。

| NLOS 率 | Lv0 | Lv1 | Lv2 | Lv3 |
|---|---|---|---|---|
| 0 % | 0.193 | 0.177 | 0.177 | **0.116** |
| 15 % | 0.607 | 0.376 | 0.311 | **0.172** |
| 35 % | 1.049 | 0.799 | 0.637 | **0.328** |

(RMSE 3D [m]、8 台立体配置、σ₀ = 8 cm、10 Hz、5 seed 平均)

## 設営を助ける道具

現場で精度が出ない原因は、たいていアルゴリズムではなく設営とキャリブレーション。

```python
ul.gdop_at([4, 3, 1.2], anchors)      # その点の幾何精度劣化係数
ul.crlb_at([4, 3, 1.2], anchors)      # 位置誤差の理論下限 [m]
ul.anchor_condition(anchors)          # 同一平面かどうか (3D では致命的)

anchors = ul.self_survey(dist_matrix, ids, dim=3)   # 相互測距からアンカー配置を推定
anchors = ul.align_to_reference(anchors, {"A0": [0, 0, 2.4], ...})

delays = ul.estimate_antenna_delays(anchor_ids, measured, true_distance)
```

自己測量があるので、**巻き尺で全台測る作業は要らない**。実測するのは 3〜4 台だけでよい。

**3 次元測位では、アンカーが同一平面に並んでいないことが本質的に効く。**
天井の 4 隅に貼っただけでは高さがほとんど観測できないうえ、平面の上下に
**距離が厳密に一致する 2 点** (鏡像解) ができて、測距値だけでは選べない。
この構成で 3D の測位器を作ると警告が出るので、`SolveConfig(dim=2, z_fixed=...)` で
高さを固定するか、`z_bounds=(0, 2.3)` のような事前情報を与える。

## ブラウザ UI

```bash
python -m uwb_loc ui
```

外部 CDN を一切使わない単一ページなので、ネットワークのない現場でも動く。

- アンカーを平面図上でドラッグして配置し、**GDOP ヒートマップ**で弱い場所を確認
- 誤差モデル (σ、NLOS 確率・継続時間・バイアス、ロス率、アンテナ遅延、設置誤差) を振る
- Lv0〜Lv3 を**同じ観測列**に通して比較。アルゴリズムの差だけが見える
- 「ライブ」タブから実機の観測 (JSON Lines: ファイル / TCP / シリアル) を流し込んで表示

スマホ・タブレットからも使える (1 列レイアウト、指でのドラッグ対応)。
同じ LAN の端末から開くには `python -m uwb_loc ui --host 0.0.0.0` —
起動時に表示される LAN アドレスをブラウザに入れる。
**認証はないので信用できる回線でだけ**使うこと。

## CLI

```bash
python -m uwb_loc sim --levels Lv0,Lv2,Lv3 --nlos 0.2 --log run.jsonl
python -m uwb_loc replay run.jsonl --level Lv3 --out fixes.csv
python -m uwb_loc gdop --room 8 6 2.6 --n-low 0     # 天井のみ配置を評価
python -m uwb_loc survey distances.csv --dim 3      # 相互測距 → アンカー配置
python -m uwb_loc ui --port 8765
```

## ドキュメント

| | |
|---|---|
| [docs/UWB.md](docs/UWB.md) | 使い方 |
| [docs/UWB_PROTOCOL.md](docs/UWB_PROTOCOL.md) | **HAL とのデータ交換仕様** (単位・座標系・時刻の規約、JSON Lines) |
| [docs/UWB_ALGORITHMS.md](docs/UWB_ALGORITHMS.md) | **アルゴリズムの導出** (式と実装の対応、踏んだ罠) |
| [docs/UWB_POSITIONING.md](docs/UWB_POSITIONING.md) | 手法選定の経緯 |

## 例

```bash
python examples/01_quickstart.py      # ハードなしで Lv0-Lv3 を比較
python examples/02_custom_hal.py      # 自前の UWB 用 HAL を書く
```

## 開発

```bash
python -m pytest -q      # 52 件
```

テストは数値の一致だけでなく、**アルゴリズムが持つべき性質**を検証している
(無雑音なら閉形式が真値を返す、Beck は LLS より偏りが小さい、NLOS 1 本で
Lv2 が Lv1 より崩れない、EKF がアンカー 2 本でも追従する、
ゲートに閉じ込められても復帰する、自己測量が配置を復元する、など)。

## ライセンス

MIT
