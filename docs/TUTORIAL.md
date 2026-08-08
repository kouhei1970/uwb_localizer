# チュートリアル — 手を動かして覚える

UWB も測位も初めて、という前提で書いてある。**上から順に写して動かせば、
最後には実機の距離から位置が出せるようになる。**

各段階に対応する動くコードが [`../examples/`](../examples/) にある。
「何があるか」の一覧は [REFERENCE.md](REFERENCE.md)。

| | 何をする | ハード |
|---|---|---|
| [第 0 章](#第-0-章-そもそも何をしているのか) | 用語と原理 | 不要 |
| [第 1 章](#第-1-章-とりあえず動かす) | インストールして UI を開く | 不要 |
| [第 2 章](#第-2-章-python-から-3-行で位置を出す) | 3 行で位置を出す | 不要 |
| [第 3 章](#第-3-章-アンカーを置く) | 配置を決める・評価する | 不要 |
| [第 4 章](#第-4-章-測位レベルを選ぶ) | Lv0〜Lv3 の選び方 | 不要 |
| [第 5 章](#第-5-章-実機の距離をつなぐ) | 実機の出力を読む | 要 |
| [第 6 章](#第-6-章-精度が出ないときにやること) | 切り分けと校正 | 要 |

---

## 第 0 章: そもそも何をしているのか

### 用語

| 言葉 | 意味 |
|---|---|
| **UWB** | 超広帯域無線。電波の到達時間を正確に測れるので、距離が測れる |
| **アンカー** | 座標が分かっている固定点。基準になる |
| **タグ** | 位置を知りたい移動する側 |
| **測距 (ranging)** | アンカーとタグの間の距離を測ること。単位はこのライブラリでは **m** |
| **測位 (positioning)** | 複数の距離からタグの座標を求めること ← **これが本ライブラリの仕事** |
| **エポック** | 「ほぼ同時刻に取れた距離のひとまとまり」。1 エポックで 1 回位置が出る |

### 原理

アンカー 1 台からの距離が分かると、タグは**その球面のどこか**にいる。
2 台で球面の交わり (円)、3 台で 2 点、**4 台で 1 点に決まる**。

```
  アンカーA ─── 3.2m ───┐
  アンカーB ─── 2.9m ───┼──→  この 3 つを同時に満たす点 = タグの位置
  アンカーC ─── 4.1m ───┘
```

現実には距離に誤差が乗るので、**球面はきれいに 1 点で交わらない**。
だから「一番もっともらしい点」を探す最小二乗問題になる。さらに屋内では

- **NLOS**: 人や壁で見通しが切れると、電波が回り込んで**距離が実際より長く出る**
- **同一平面問題**: アンカーを平らに並べると高さが決まらない

この 2 つが精度を決める。ライブラリの Lv1〜Lv3 は、ほぼこの対策。

---

## 第 1 章: とりあえず動かす

### 入れる

```bash
git clone https://github.com/kouhei1970/uwb_localizer.git
cd uwb_localizer
python3 -m venv venv
source venv/bin/activate          # Windows は venv\Scripts\activate
pip install -e .
```

### 動かす

```bash
python -m uwb_loc ui
```

ブラウザで <http://127.0.0.1:8765> が開く。**ハードは 1 台も要らない。**
左でアンカーをドラッグして動かすと、右の精度がその場で変わる。

まずこれで遊んでみると、「アンカーを 4 隅に固めると真ん中は良いが端が悪い」
といったことが体で分かる。

### 数字で見る

```bash
python -m uwb_loc sim --nlos 0.2 --duration 40
```

```
Lv0   測位率 100.0%  RMSE3D  0.631  ...
Lv1   測位率 100.0%  RMSE3D  0.368  ...
Lv2   測位率 100.0%  RMSE3D  0.312  ...
Lv3   測位率 100.0%  RMSE3D  0.153  ...
```

読み方:

- **測位率** — 位置が出せたエポックの割合。100% でないなら距離が足りていない
- **RMSE3D** — 真値との誤差の二乗平均平方根 [m]。**小さいほど良い**
- **CEP50 / CEP95** — 誤差がこの値以下に収まる割合が 50% / 95%

`--nlos 0` にすると全部良くなる。`--nlos 0.4` にすると Lv0 が崩れて
Lv2/Lv3 との差が開く。**動かしてみるのが一番早い。**

→ 対応する例: [`examples/01_quickstart.py`](../examples/01_quickstart.py)

---

## 第 2 章: Python から 3 行で位置を出す

ライブラリの中心はこれだけ。**アンカー座標を渡して測位器を作り、
距離を渡すと位置が返る。**

```python
import uwb_loc as ul

# 1. アンカー (座標が分かっている固定点)。単位は m、右手系で z が上
anchors = [
    ul.Anchor("A0", [0.0, 0.0, 2.4]),
    ul.Anchor("A1", [8.0, 0.0, 0.3]),
    ul.Anchor("A2", [8.0, 6.0, 2.4]),
    ul.Anchor("A3", [0.0, 6.0, 0.3]),
    ul.Anchor("A4", [4.0, 0.0, 2.4]),
]

# 2. 測位器を作る (Lv2 = 屋内の既定)
est = ul.make_estimator("Lv2", anchors)

# 3. 距離を渡す
batch = ul.MeasurementBatch(t=0.0, measurements=[
    ul.Measurement("A0", 4.85),      # アンカー ID と距離 [m] だけ
    ul.Measurement("A1", 5.21),
    ul.Measurement("A2", 5.02),
    ul.Measurement("A3", 4.43),
    ul.Measurement("A4", 3.31),
])
fix = est.update(batch)

print(fix.position)      # [x, y, z]  numpy 配列
print(fix.sigma)         # 位置の不確かさ [m]
print(fix.ok)            # 測位できたか
```

### `Fix` の読み方

`update()` が返す `Fix` には、位置以外にも判断材料が入っている。

| 属性 | 意味 | 使いどころ |
|---|---|---|
| `position` | 位置 `[x, y, z]` [m] | これが答え |
| `ok` | 測位できたか | **必ず見る。**False の位置は意味が無い |
| `sigma` | 位置誤差の代表値 [m] | 大きいなら信用しない |
| `n_used` / `n_total` | 使った本数 / 届いた本数 | 差が大きい = 外れ値が多い |
| `excluded` | 外れ値として捨てたアンカー ID | いつも同じ ID なら座標か設置を疑う |
| `gdop` | 幾何精度劣化係数 | 5 を超えたら配置が悪い |
| `residual_rms` | 残差 [m] | 大きい = 距離と座標が噛み合っていない |
| `ambiguous` | 鏡像解かもしれない | True なら高さが信用できない |

**最低限このように書く:**

```python
fix = est.update(batch)
if fix.ok and fix.sigma < 0.5:
    use(fix.position)
```

### 時刻について

`Lv0`〜`Lv2` は時刻を使わないので `t=0.0` のままで構わない。
**`Lv3` (EKF) だけは正しい時刻が要る** — 前回からの経過時間で予測するため。

→ 対応する例: [`examples/03_minimal_integration.py`](../examples/03_minimal_integration.py)

---

## 第 3 章: アンカーを置く

**現場で精度が出ない原因は、たいていアルゴリズムではなく置き方。**

### 何台要るか

- **3D なら最低 4 台**、実用は **6 台**から
- **2D (高さが分かっている) なら最低 3 台**

### 平らに並べない

3D 測位で一番やりがちな失敗が、**天井の 4 隅に貼る**こと。

```
天井:  A0 ─────────── A1        ← 全部同じ高さ = 同一平面
        │             │
        │             │
       A3 ─────────── A2
```

こうすると 2 つ困る。

1. **高さがほとんど観測できない。** どの距離も高さの変化に鈍い
2. **鏡像解が出る。** 平面の上下に「距離が全部一致する 2 点」ができ、
   測距値だけでは選べない。**天井の上に推定が飛ぶ**

確かめ方:

```python
print(ul.anchor_condition(anchors))
# {'coplanar': True, ...}   ← True なら危ない
```

この配置で 3D の測位器を作ると警告が出る。対処は 3 つ:

```python
# 1. 高さをばらす ← 一番よい
anchors = [ul.Anchor("A0", [0, 0, 2.4]), ul.Anchor("A1", [8, 0, 0.3]), ...]

# 2. 2D で解く (高さが分かっているなら)
est = ul.make_estimator("Lv2", anchors, ul.SolveConfig(dim=2, z_fixed=1.2))

# 3. 高さの範囲を教える
est = ul.make_estimator("Lv2", anchors, ul.SolveConfig(z_bounds=(0.0, 2.3)))
```

### 置く前に配置を評価する

**GDOP** は「測距の精度が同じでも、置き方が悪いと位置がどれだけ暴れるか」の指標。

```python
ul.gdop_at([4.0, 3.0, 1.2], anchors)   # その点の GDOP
ul.crlb_at([4.0, 3.0, 1.2], anchors)   # 位置誤差の理論下限 [m]
```

| GDOP | 評価 |
|---|---|
| 〜2 | 良い |
| 2〜5 | 使える |
| 5〜 | **配置を見直す** |

部屋全体を見るなら:

```bash
python -m uwb_loc gdop --room 8 6 2.6
python -m uwb_loc gdop --room 8 6 2.6 --n-low 0     # 天井のみだとどうなるか
```

### 座標を測るのが面倒なら

アンカー同士で測った距離があれば、**座標を復元できる**。

```bash
python -m uwb_loc survey distances.csv --dim 3
```

CSV はこの形 (ヘッダ行と ID 列は省略可、届かなかったペアは空欄):

```
        ,A0  ,A1  ,A2  ,A3
A0      ,0   ,8.00,10.0,6.00
A1      ,8.00,0   ,6.00,10.0
A2      ,10.0,6.00,0   ,8.00
A3      ,6.00,10.0,8.00,0
```

出てくるのは**形だけ合った座標**で、向きと原点は決まっていない。
実測した 3〜4 台を基準に実寸へ合わせる:

```python
anchors = ul.self_survey(dist_matrix, ids, dim=3)
anchors = ul.align_to_reference(anchors, {"A0": [0, 0, 2.4], "A1": [8, 0, 0.3],
                                          "A2": [8, 6, 2.4], "A3": [0, 6, 0.3]})
```

→ 対応する例: [`examples/04_anchor_layout.py`](../examples/04_anchor_layout.py)

---

## 第 4 章: 測位レベルを選ぶ

**同じ書き方で差し替えられる**測位器が 4 つある。迷ったら **Lv2**。

| Lv | 中身 | 使うとき |
|---|---|---|
| **Lv0** | 閉形式の三辺測量 (反復なし) | 配線・座標系の確認。**最初の動作確認に** |
| **Lv1** | 重み付き非線形最小二乗 + χ² ゲート | 見通しが良い環境 |
| **Lv2** | Beck 厳密解 + Huber ロバスト化 | **屋内の既定。迷ったらこれ** |
| **Lv3** | 密結合 EKF | **移動体。時刻が正しく取れるなら** |

```python
est = ul.make_estimator("Lv2", anchors)     # 名前を変えるだけ
```

### 選び方

```
時刻が正しく取れる? ──いいえ──→ Lv2
        │
       はい
        │
   タグは動く? ──いいえ──→ Lv2 (静止なら EKF の利点が薄い)
        │
       はい
        │
   アンカーは 6 台以上? ──いいえ──→ 外れないことが大事なら Lv2
        │
       はい
        ↓
      Lv3
```

**Lv3 は平均では強いが、台数が少ないと時々外す。** 実測 (8 seed) では、
4 台のとき Lv3 の中央値は Lv2 より良い (0.56 m 対 0.96 m) 一方、
**最悪ケースは 2.43 m まで伸びた** (Lv2 は 1.11 m)。幾何が痩せたところに
欠測が重なると、EKF が一度ずれてから戻るのに時間がかかるため。
5 台以上ではこの傾向は消え、Lv3 が中央値も最悪も上回る。

### 動きに合わせる

```python
# 等速モデル (既定)。普通の移動体
est = ul.make_estimator("Lv3", anchors)

# 加速度まで持つモデル。急に速度が変わるもの (ドローンなど)
est = ul.Lv3TightlyCoupledEKF(anchors, motion="ca", sigma_a=2.0)
```

`sigma_a` は「どれくらい機敏に動くと思うか」。**大きくすると追従が速く
なるがノイズを拾い、小さくすると滑らかになるが遅れる。**

→ 対応する例: [`examples/05_choose_level.py`](../examples/05_choose_level.py)

---

## 第 5 章: 実機の距離をつなぐ

### まず出力を覗く

**いきなりコードを書かない。** 何が出ているか見るのが先。

```bash
python -m uwb_loc sniff --serial /dev/ttyUSB0
```

```
解釈できた行  39 / 40
使った正規表現  (?P<anchor>\d+)\s*,\s*(?P<dist>-?[\d.]+)
見つかったアンカー ID  ['0', '1', '2', '3']
距離の範囲  2.887 〜 5.102 m
  → 妥当な範囲です。
```

距離が 3000 とか出たら**単位が mm**。`--unit mm` を付けて測り直す。

### つなぐ — 4 通り。どれでも測位側は同じ

#### (a) 既に自分でパースしている → HAL は要らない

```python
est = ul.make_estimator("Lv2", anchors)
readings = my_uart_read()        # [("A0", 3.214), ("A1", 2.887), ...]
fix = est.update(ul.MeasurementBatch(
    t=time.monotonic(),
    measurements=[ul.Measurement(a, d) for a, d in readings]))
```

#### (b) テキストが出ている → 正規表現 1 本

```python
hal = ul.TextHal.from_serial("/dev/ttyUSB0", 115200,
                             r"range,(?P<anchor>\d+),(?P<dist>\d+)",
                             anchors=anchors, unit="mm", anchor_prefix="A")
for fix in ul.Pipeline(hal, level="Lv2").run():
    if fix.ok:
        print(fix.position.round(2))
```

`sniff` が出した正規表現をそのまま貼れる。名前付きグループ
`(?P<anchor>...)` と `(?P<dist>...)` の 2 つが必須。

#### (c) ファームを書ける → JSON Lines

```json
{"t":12.345,"meas":[{"a":"A0","d":3.214,"q":0.93},{"a":"A1","d":2.887}]}
```

`a` (アンカー ID) と `d` (距離 m) だけ必須。`t` (時刻) と `q` (品質 0-1) は任意。

```python
hal = ul.JsonLinesHal.from_serial("/dev/ttyUSB0", 115200, anchors=anchors)
```

#### (d) 読みに行けない (BLE 通知 / MQTT / ROS) → PushHal

```python
hal = ul.PushHal(anchors)
def on_ble_notify(anchor_id, dist_m):        # コールバックの中で
    hal.push(anchor_id, dist_m)
```

#### RYUW122 なら専用 HAL

```python
hal = ul.Ryuw122Hal.from_serial("/dev/ttyUSB0", ["TAG00001", "TAG00002", ...],
                                anchors=anchors)
```

→ 詳しくは [RYUW122.md](RYUW122.md)

→ 対応する例: [`examples/02_custom_hal.py`](../examples/02_custom_hal.py)、
[`examples/03_minimal_integration.py`](../examples/03_minimal_integration.py)

---

## 第 6 章: 精度が出ないときにやること

**上から順に潰す。** 下から手をつけると時間を溶かす。

### 1. そもそも位置が出ない (`fix.ok == False`)

```python
print(fix.n_total)      # 届いた本数
```

- `n_total` が 0 → 距離が届いていない。HAL の問題。`sniff` に戻る
- `n_total` が `dim+1` 未満 → **アンカーが足りない**。3D なら 4 本要る

### 2. 位置が出るが飛ぶ

```python
print(fix.residual_rms, fix.excluded, fix.gdop)
```

| 症状 | たぶん原因 |
|---|---|
| `residual_rms` が大きい (> 0.5 m) | **アンカー座標が間違っている**か、単位が違う |
| `excluded` にいつも同じ ID | その 1 台の座標か設置が悪い |
| `gdop` が大きい (> 5) | **置き方が悪い**。第 3 章へ |
| 高さだけおかしい | **同一平面問題**。`anchor_condition` を見る |
| `ambiguous` が True | 鏡像解。`z_bounds` を与える |

### 3. 距離に一定のずれがある

アンテナ遅延で、**全部の距離が同じだけ長い/短い**ことがよくある。
真の距離が分かる場所で測って補正する:

```python
delays = ul.estimate_antenna_delays(anchor_ids, measured, true_distance)
for a in anchors:
    a.antenna_delay_m = delays[a.id]
```

### 4. NLOS が多い

人が動くと距離が伸びる。**Lv2 にする**のが第一手。それでも駄目なら
アンカーを増やして、遮られない経路を確保する。

### 現場で録っておく

**その場で悩まず、まず録る。**

```python
writer = ul.JsonLinesWriter("run.jsonl")
writer.write_anchors(anchors)
for batch in ...:
    writer.write(batch)
```

あとから何度でも解き直せる:

```bash
python -m uwb_loc replay run.jsonl --level Lv3
python -m uwb_loc replay run.jsonl --level Lv2 --out fixes.csv
```

→ 対応する例: [`examples/06_troubleshooting.py`](../examples/06_troubleshooting.py)

---

## 次に読むもの

| | |
|---|---|
| [REFERENCE.md](REFERENCE.md) | **全コマンド・全メソッドの一覧** |
| [../examples/](../examples/) | **動くコードと解説** |
| [RYUW122.md](RYUW122.md) | REYAX RYUW122 の手順 |
| [BRINGUP.md](BRINGUP.md) | 実機立ち上げ (モジュール別の目安) |
| [UWB_PROTOCOL.md](UWB_PROTOCOL.md) | HAL とのデータ交換仕様 |
| [UWB_ALGORITHMS.md](UWB_ALGORITHMS.md) | 中で何をしているかの導出 |
