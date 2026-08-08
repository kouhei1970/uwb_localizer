# UWB HAL ↔ ライブラリ データ交換仕様

> **この文書**: HAL とライブラリの約束事 (単位・座標系・型・JSON Lines スキーマ)。手を動かす順序は [TUTORIAL.md](TUTORIAL.md)。 → [ドキュメント一覧](README.md)

`uwb_loc` は特定の UWB チップに依存しない。チップごとに違うのは「時刻をどう測るか」
までで、**距離になってしまえば下流は同じ**なので、その境界をこの仕様で切る。

境界は 2 通りある。どちらを使っても測位側のコードは変わらない。

| 方法 | 実装するもの | 向いている場面 |
|---|---|---|
| **A. Python HAL クラス** | `UwbHal` を継承した 1 クラス | Python から直接チップを叩く (SPI/シリアル) |
| **B. JSON Lines** | ファームウェア側で 1 行 1 JSON を print | Python を書かずに繋ぎたい / 既存ファームがある |

---

## 0. 共通規約

この 3 つを守れば、どちらの方法でも噛み合う。

**単位** — 長さ `m`、角度 `rad`、時刻 `s`。例外なし。ミリメートルもセンチも受け付けない。

**座標系** — 右手系、z 軸が上向き (ENU 相当)。アンカー座標と推定位置は同じ系。
原点はどこでもよいが、部屋の隅を `(0,0,0)` に取ると UI の描画と揃う。

**時刻** — 単調増加する秒。絶対時刻でなくてよい (起動からの経過秒で十分) が、
全観測で同じ基準を使うこと。入れるのは
**「測距が成立した時刻」であって、ホストにデータが届いた時刻ではない**。
密結合 EKF の予測ステップがこの時刻差で回るので、ここがずれると移動中の精度が直接落ちる。
シリアル経由で 50 ms 遅れて届くなら、その 50 ms は誤差になる。

---

## 1. 観測 (`Measurement`)

観測 1 本を表す。HAL が作るのはこれだけ。

| フィールド | 型 | 必須 | 意味 |
|---|---|:--:|---|
| `anchor_id` | str | ✓ | 相手アンカーの ID |
| `value` | float | ✓ | 観測値。`kind` により [m] か [rad] |
| `kind` | MeasKind | | `range` (既定) / `tdoa` / `azimuth` / `elevation` |
| `t` | float | ✓ | 測距が成立した時刻 [s] |
| `sigma` | float | | この観測の 1σ。None ならアンカーのノイズモデルを使う |
| `quality` | float | | 0–1 の見通し (LOS) 尤度。1 が完全な見通し |
| `ref_anchor_id` | str | TDoA のみ | 距離差の基準アンカー |
| `tag_id` | str | | 測位対象の ID (既定 `"tag0"`) |
| `raw` | dict | | チップ固有の生値。診断・後解析用 |

### `quality` の作り方 — ここを埋めると屋内精度が変わる

`quality` は **NLOS 対策の入口**で、ライブラリはこの値で σ を膨らませる
(`q=1` で等倍、`q=0` で 4 倍)。チップ非依存にするため、生値ではなく
0–1 に正規化して入れる。

DW1000/DW3000 系なら、次のような指標から作れる。

- **first path power と peak path power の差** — NLOS では直達波が弱く、
  後から来た反射波の方が強いので差が開く。いちばん効く指標
- **受信電力 (RX power)** — 距離から期待される値より弱ければ遮蔽を疑う
- **first path index の立ち上がり時間** — なまっていれば NLOS
- **測距の分散** — 直近数回のばらつき

分類器を持たないなら、first path power と peak power の差 `Δ` [dB] から
`q = clip(1 - Δ/10, 0, 1)` 程度の素朴な写像で十分効く。
判断がつかないなら **`quality` を省く方がよい** (省略時は `q=1` 扱い) —
でたらめな値を入れると、正しい観測の重みまで下げてしまう。

生の指標は `raw` に入れておく。あとから記録を見返して分類器を作れる。

### アンテナ遅延をどちらで引くか

`value` は**アンテナ遅延を引く前の生の値**でよい。ライブラリが
`Anchor.antenna_delay_m` で補正する。HAL 側で補正済みなら
`antenna_delay_m=0` にしておく。**二重に引かないこと** — 数十 cm の系統誤差になる。

---

## 2. アンカー (`Anchor`)

| フィールド | 型 | 既定 | 意味 |
|---|---|---|---|
| `id` | str | — | 観測の `anchor_id` と一致させる |
| `position` | (3,) | — | 設置座標 [m] |
| `enabled` | bool | True | False なら測位に使わない |
| `antenna_delay_m` | float | 0.0 | アンテナ遅延に相当する距離 [m] |
| `sigma0` | float | 0.10 | 測距ノイズの定数項 [m] |
| `sigma_per_m` | float | 0.0 | 測距ノイズの距離比例項 [m/m] |
| `position_sigma` | float | 0.0 | 設置座標そのものの不確かさ [m] |

座標が分からなくても始められる。相互測距から
[`self_survey`](#6-アンカー座標が分からないとき) で推定できる。

---

## 3. 測位結果 (`Fix`)

位置だけでなく **共分散と品質指標が必ず付く**。運用時に「今の値を信じてよいか」を
判断できないと使い物にならないため。

| フィールド | 意味 |
|---|---|
| `position` / `covariance` | 推定位置 [m] と共分散 [m²] |
| `ok` | 測位が成立したか。False なら `position` は NaN |
| `sigma` | 位置誤差の代表値 (共分散のトレースの平方根) [m] |
| `n_used` / `n_total` | 使った観測数 / 入力された観測数 |
| `residual_rms` | 採用した観測の残差 RMS [m] |
| `gdop` | 幾何精度劣化係数 |
| `excluded` | 外れ値として落としたアンカー ID |
| `velocity` | 速度 [m/s]。Lv3 のみ |
| `level` | どのレベルが出したか |

`n_used` が急に減った、`residual_rms` が σ の数倍に跳ねた、`sigma` が膨らんだ —
このどれかが起きていれば、現場で何かが変わっている (人が立った、アンカーが動いた)。

---

## 4. 方法 A — Python HAL クラス

実装するのは 2 つだけ。

```python
from uwb_loc import Anchor, Measurement, MeasurementBatch, UwbHal

class MyChipHal(UwbHal):
    name = "dw3000"

    def __init__(self, port):
        self._dev = open_my_chip(port)
        self._anchors = [
            Anchor("A0", [0.2, 0.2, 2.4], antenna_delay_m=0.154),
            Anchor("A1", [7.8, 0.2, 2.4], antenna_delay_m=0.151),
        ]

    @property
    def anchors(self):
        return self._anchors

    def poll(self, timeout=0.0):
        """溜まっている観測を返す。ブロックしないこと。"""
        out = []
        for frame in self._dev.read_ranging_results(timeout):
            out.append(MeasurementBatch(
                t=frame.timestamp_s,
                measurements=[
                    Measurement(
                        anchor_id=r.anchor_id,
                        value=r.distance_m,
                        t=frame.timestamp_s,
                        quality=clip(1 - (r.peak_power - r.first_path_power) / 10, 0, 1),
                        raw={"rx_power": r.rx_power, "fp_index": r.fp_index},
                    )
                    for r in frame.ranges
                ],
            ))
        return out
```

任意で `open()` / `close()` / `is_open` も実装できる。あとは

```python
from uwb_loc import Pipeline

with MyChipHal("/dev/ttyUSB0") as hal:
    for fix in Pipeline(hal, level="Lv3").run():
        print(fix.position, fix.sigma)
```

**守ること**

1. `poll` はブロックしないか、`timeout` で必ず戻る
2. 通信エラーは例外ではなく `is_open = False` で表す
3. 1 エポックに束ねられないなら、観測 1 本の `MeasurementBatch` を並べて返してよい。
   密結合 EKF は届いた順に処理できる

---

## 5. 方法 B — JSON Lines

ファームウェアが 1 行 1 JSON を吐くだけでよい。Python 側は 1 行も書かなくてよい。

### アンカー通知 (最初に 1 回、または変更時)

```json
{"v":1,"type":"anchors","anchors":[
  {"id":"A0","p":[0.2,0.2,2.4],"antenna_delay_m":0.154},
  {"id":"A1","p":[7.8,0.2,2.4],"antenna_delay_m":0.151}]}
```

### 観測 (毎エポック)

```json
{"v":1,"type":"meas","t":12.345,"tag":"tag0","meas":[
  {"a":"A0","d":3.214,"q":0.93},
  {"a":"A1","d":2.887,"q":0.41,"sigma":0.22},
  {"a":"A2","d":4.550,"raw":{"rx":-79.4,"fp":-86.1}}]}
```

観測 1 本のキーは短縮形を使う。

| キー | 意味 |
|---|---|
| `a` | アンカー ID (必須) |
| `d` または `v` | 観測値 (必須)。`d` は距離としての別名 |
| `type` | `range` (既定) / `tdoa` / `azimuth` / `elevation` |
| `t` | 個別の時刻 [s]。省略時は親の `t` |
| `sigma` | 1σ |
| `q` | 品質 0–1 |
| `ref` | TDoA の基準アンカー ID |
| `raw` | チップ固有の生値 |

### 受け側

```python
from uwb_loc import JsonLinesHal, Pipeline

hal = JsonLinesHal.from_serial("/dev/ttyUSB0", 115200)   # シリアル
# hal = JsonLinesHal.from_tcp("192.168.1.50", 9000)      # TCP
# hal = JsonLinesHal.from_path("log.jsonl")              # 記録の再生

for fix in Pipeline(hal, level="Lv3").run():
    print(fix.position)
```

**JSON でない行は黙って捨てる。** 実機のシリアルには起動メッセージや
デバッグ出力が混ざるのが普通なので、それでパイプラインが止まらないようにしてある。
壊れた JSON も同様。

### 記録して後から比較する

```python
from uwb_loc import JsonLinesWriter

with JsonLinesWriter("run01.jsonl", hal.anchors) as w:
    for batch in hal.stream():
        w.write(batch)
```

記録しておけば、**同じ観測列**を Lv0〜Lv3 に通してアルゴリズムの差だけを比較できる。
UI の「ライブ → ファイル」からも読める。

---

## 6. アンカー座標が分からないとき

アンカー同士で相互測距できるなら、巻き尺は要らない。

```python
import numpy as np
from uwb_loc import self_survey, align_to_reference

# アンカー間の距離行列 (対称・対角 0・欠測は NaN)
anchors = self_survey(dist_matrix, ids=["A0","A1","A2","A3","A4","A5"], dim=3)

# 出てくる座標は「形は正しいが向きと原点は任意」。
# 何台か実測した値に合わせると実世界座標に載る。
anchors = align_to_reference(anchors, {"A0": [0,0,2.4], "A1": [7.6,0,2.4], "A2": [7.6,5.6,2.4]})
```

3 次元で意味のある解を得るには、**アンカーが同一平面に並んでいないこと**が必要。
天井の 4 隅だけ、のような平面配置では `dim=2` で解いて高さは実測する。

CLI からも使える。

```bash
python -m uwb_loc survey distances.csv --dim 3
```

---

## 7. よくある失敗

| 症状 | 原因 |
|---|---|
| 位置が一定方向に数十 cm ずれる | アンテナ遅延の未補正、または二重補正 |
| z だけ大きく外れる / 暴れる | アンカーが同一平面。`dim=2` にするか配置を立体にする |
| 動くと遅れる | `t` にホスト到着時刻を入れている |
| たまに数 m 飛ぶ | NLOS。`quality` を入れて Lv2 以上にする |
| `ok=False` が続く | アンカー ID の不一致。観測の `a` とアンカーの `id` を照合する |
| 静止しているのにふらつく | Lv3 の `sigma_a` が大きすぎる |
