# 実機立ち上げ — 何を用意して、何を渡せばよいか

このライブラリは**測位の計算だけ**を担当する。UWB を動かす部分は含まない。
どこまでを自分で用意する必要があるのかを、はっきりさせておく。

## このライブラリがやること / やらないこと

| | |
|---|---|
| **やる** | 距離 → 位置の計算 (最小二乗・ロバスト化・カルマンフィルタ) |
| | 外れ値 (NLOS) の除去、共分散と品質指標の算出 |
| | アンカー配置の評価 (GDOP/CRLB)、相互測距からの自己測量 |
| | アンテナ遅延の推定、ハードなしでの精度検証 |
| **やらない** | **UWB チップの制御** (SPI/レジスタ設定、割り込み処理) |
| | **測距シーケンス** (DS-TWR のフレーム往復、タイムスタンプ処理) |
| | 無線チャネル設定、PAN/アドレス管理、省電力制御 |
| | アンカーの設置と座標の実測 |

つまり **「測距値が既に取れている」ところから始まる**。
そこがまだなら、まずモジュールのファームウェアを動かすのが先で、
このライブラリは何も助けにならない。

逆に言うと、**距離さえ出ていれば残りは全部ある**。

---

## 必要なものは 3 つだけ

### 1. 測距値 — アンカー ID と距離

これが本体。単位は m に直して渡す。最低限これだけあれば測位できる。

```python
ul.Measurement(anchor_id="A0", value=3.214)     # [m]
```

**経路 (UART か否か) は問わない。** ライブラリはシリアルポートも
ソケットも直接は触らない。必要なのは上の 3 つの値だけで、それが
どう届くかは呼ぶ側の自由 — シリアル・TCP・UDP・BLE・MQTT・ROS・
ファイル・共有メモリ、何でもよい。

### 2. アンカー座標

どこにアンカーがあるかを知らないと位置は出せない。3 つの入手方法がある。

```python
# (a) 巻き尺で測る
anchors = [ul.Anchor("A0", [0.2, 0.2, 2.4]), ...]

# (b) 相互測距から推定して、実測した 3〜4 台に合わせる
anchors = ul.self_survey(dist_matrix, ids, dim=3)
anchors = ul.align_to_reference(anchors, {"A0": [0, 0, 2.4], ...})

# (c) とりあえず概算で置いて、精度を見ながら詰める
```

### 3. 時刻 — **Lv3 (EKF) を使う場合のみ**

Lv0〜Lv2 は 1 エポックの観測だけで解くので時刻を使わない。
Lv3 は予測ステップを時刻差で回すので、これが無いと動かない。

どれくらい効くかは実測してある (8 台配置、10 Hz、NLOS 10%、RMSE 3D)。

| 時刻の与え方 | Lv3 | Lv2 |
|---|---|---|
| ファームウェアが出す時刻 | **0.21 m** | 0.43 m |
| 無し (記録したログを一気に流す) | 1.33 m | 0.43 m |
| 無し + `rate_hz=10` で合成 | **0.21 m** | 0.43 m |

**時刻が無いなら Lv2 を使うか、`rate_hz` を指定する。**

---

## 手順

### 手順 1 — まず出力を覗く

モジュールが既に何を吐いているかを見る。ここで**単位とアンカー ID を
確定させておかないと**、後で位置がおかしいときに「測距が変」なのか
「座標が変」なのか切り分けられない。

```bash
python -m uwb_loc sniff --serial /dev/ttyUSB0 --unit mm --prefix A
```

```
読んだ行  40
解釈できた行  39
使った正規表現  (?P<anchor>\d+)\s*,\s*(?P<dist>-?[\d.]+)

生の行:
  | boot ok
  | range,0,3214
  | range,1,2887

見つかったアンカー ID  ['A0', 'A1', 'A2', 'A3']
距離の範囲  2.887 〜 5.102 m  (単位 --unit mm として換算)
  → 妥当な範囲です。次はアンカー座標を用意してください。
```

正規表現は省略すれば推測する。当たらなければ `--pattern` で指定する。
距離が桁違いなら単位の指定が違うので、`--unit` を変えて出し直す。

### 手順 2 — つなぐ

`sniff` が出した正規表現と単位をそのまま渡す。**ファームウェアの改造も
Python のクラス書きも要らない。**

```python
import uwb_loc as ul

anchors = [ul.Anchor("A0", [0.2, 0.2, 2.4]),
           ul.Anchor("A1", [7.8, 0.2, 2.4]),
           ul.Anchor("A2", [7.8, 5.8, 0.3]),
           ul.Anchor("A3", [0.2, 5.8, 0.3])]

hal = ul.TextHal.from_serial(
    "/dev/ttyUSB0", 115200,
    r"range,(?P<anchor>\d+),(?P<dist>\d+)",
    anchors=anchors, unit="mm", anchor_prefix="A")

for fix in ul.Pipeline(hal, level="Lv2").run():
    if fix.ok:
        print(f"{fix.position.round(2)}  ±{fix.sigma:.2f} m  ({fix.n_used} 本)")
```

### 手順 3 — 確かめる

いきなり位置を信じない。次の順で潰す。

1. **`fix.ok` が立つか** — 立たないならアンカー ID の不一致か本数不足。
   `fix.n_total` が 0 なら ID が合っていない
2. **`fix.residual_rms`** — 測距の σ と同じ桁 (数 cm) に収まるか。
   桁違いに大きいなら座標の入力ミスか単位の取り違え
3. **静止させて `fix.position` のばらつき** — これが測距ノイズ由来の下限
4. **既知の 2 点間を往復させて距離が合うか** — 合わなければ
   アンテナ遅延が未補正 (数十 cm の系統誤差として出る)

```python
delays = ul.estimate_antenna_delays(anchor_ids, measured, true_distance)
for a in anchors:
    a.antenna_delay_m = delays[a.id]
```

### 手順 4 — レベルを上げる

動くものを追うなら Lv3 に上げる。ただし**時刻が要る** (上の表)。

---

## ID と距離が取れているなら — 書き方 3 通りの最小形

「UART から何か読めて、ID と距離が整理できる」ところまで来ていれば、
あとは短い。実際に動かせる形は `examples/03_minimal_integration.py` にある。

```bash
python examples/03_minimal_integration.py
```

### A. HAL を使わない — 自分でパース済みならこれが最短

**ストリームの面倒を自分で見るなら、HAL クラスは要らない。**
`Measurement` を並べて `update()` に渡すだけ。

```python
import uwb_loc as ul

anchors = [ul.Anchor("A0", [0.2, 0.2, 2.4]),
           ul.Anchor("A1", [7.8, 0.2, 2.4]),
           ul.Anchor("A2", [7.8, 5.8, 0.3]),
           ul.Anchor("A3", [0.2, 5.8, 0.3])]

est = ul.make_estimator("Lv2", anchors)                       # 1

while True:
    readings = my_uart_read()          # [("A0", 3.214), ("A1", 2.887), ...]  距離は m

    batch = ul.MeasurementBatch(                              # 2
        t=time.monotonic(),
        measurements=[ul.Measurement(aid, d) for aid, d in readings])
    fix = est.update(batch)                                   # 3

    if fix.ok:
        print(fix.position, fix.sigma)
```

**実質 3 行。** `Measurement` に必須なのは `anchor_id` と `value` [m] だけ。

### B. JSON Lines — ファームウェアが 1 行 print するだけ

Python 側にパースのコードを 1 行も書きたくない場合。

マイコン側 (C):

```c
printf("{\"t\":%.3f,\"meas\":["
       "{\"a\":\"A0\",\"d\":%.3f},"
       "{\"a\":\"A1\",\"d\":%.3f},"
       "{\"a\":\"A2\",\"d\":%.3f},"
       "{\"a\":\"A3\",\"d\":%.3f}]}\n",
       t_sec, d0, d1, d2, d3);
```

出てくる行:

```json
{"t":12.345,"meas":[{"a":"A0","d":3.214},{"a":"A1","d":2.887}]}
```

Python 側:

```python
hal = ul.JsonLinesHal.from_serial("/dev/ttyUSB0", 115200, anchors=anchors)
for fix in ul.Pipeline(hal, level="Lv2").run():
    print(fix.position, fix.sigma)
```

キーは短縮形。`a` (アンカー ID) と `d` (距離 [m]) が必須で、
`t` (時刻 [s])、`q` (品質 0-1)、`sigma` (1σ [m]) は任意。
アンカー座標もファームから送れる:

```json
{"type":"anchors","anchors":[{"id":"A0","p":[0.2,0.2,2.4]}]}
```

**JSON でない行は黙って捨てる**ので、起動メッセージやデバッグ出力が
混ざっていても構わない。

### C. HAL クラス — ストリームを自分で管理したいとき

実装するのは `anchors` と `poll` の 2 つだけ。

```python
class MyUartHal(ul.UwbHal):
    name = "my-uart"

    def __init__(self, port, anchors):
        self._ser = serial.Serial(port, 115200, timeout=0.1)
        self._anchors = anchors

    @property
    def anchors(self):
        return self._anchors

    def poll(self, timeout=0.0):
        """溜まっている観測を返す。ブロックしないこと。"""
        out = []
        while self._ser.in_waiting:
            aid, dist, t = my_parse(self._ser.readline())
            out.append(ul.MeasurementBatch(
                t=t, measurements=[ul.Measurement(aid, dist, t=t)]))
        return out

    @property
    def is_open(self):
        return self._ser.is_open      # 通信が切れたら False
```

```python
for fix in ul.Pipeline(MyUartHal("/dev/ttyUSB0", anchors), level="Lv3").run():
    print(fix.position)
```

1 エポックに束ねなくてよい (上のように 1 本ずつ返してよい) のは、
Lv3 が測距を届いた順に処理できるため。Lv0-Lv2 を使うなら
1 エポックに 4 本以上まとめる必要がある。

### D. 読みに行けない経路 — BLE 通知 / MQTT / ROS / UDP

`readline()` できる経路 (シリアル・TCP・ファイル) は上の 3 通りで足りるが、
**「届いたら呼ばれる」形の経路**はそもそも読みに行けない。
その場合は `PushHal` に押し込む。

```python
hal = ul.PushHal(anchors)

def on_ble_notify(_, data):          # BLE の通知コールバック
    aid, dist = my_decode(data)      # 距離は m に直しておく
    hal.push(aid, dist)              # ← 押し込むだけ

client.start_notify(CHAR_UUID, on_ble_notify)

for fix in ul.Pipeline(hal, level="Lv2").run():
    print(fix.position, fix.sigma)
```

MQTT の `on_message`、ROS のサブスクライバ、UDP の受信ループ、USB HID、
WebSocket — どれも同じ形になる。**UART である必要はまったくない。**

1 本ずつ押し込んでも、Lv0-Lv2 が解けるようにエポックへ束ねてくれる
(同じアンカーが再び来たら 1 巡完了とみなす)。Lv3 だけなら `group=False` で
素通しにしてもよく、**そのときも精度は束ねた場合と変わらない**
(フィルタの状態が厳密に一致することをテストで固定している)。

### どれを選ぶか

| | 書く量 | 選ぶ場面 |
|---|---|---|
| **A. HAL なし** | 3 行 | 既に自分でパースしている。**まずこれで試す** |
| **B. JSON Lines** | ファームに printf 1 つ | ファームを触れる。Python を書きたくない |
| **C. HAL クラス** | 20 行 | 再接続処理など、ストリームを自分で握りたい |
| **D. `PushHal`** | 押し込む 1 行 | BLE 通知 / MQTT / ROS / UDP など読みに行けない経路 |
| (`TextHal`) | 正規表現 1 本 | ファームを触れず、出力形式も変えられない |

**測位側のコードはどれでも同じ。** 違うのは観測の入り口だけ。

---

## モジュール別の目安

実機の出力形式はファームウェアのバージョンで変わるので、
**必ず `sniff` で実物を確認すること**。以下は「何が既に手に入るか」の目安。

| モジュール | 既に出ているもの | 追加で要るもの |
|---|---|---|
| **DWM1001 / DWM1001-DEV**<br>(工場出荷の PANS ファーム) | UART シェルから距離を出せる。<br>位置計算まで載っている | 出力形式を `sniff` で確認して正規表現を決めるだけ。**いちばん楽** |
| **ESP32 + DW1000**<br>(Makerfabs 等 / Arduino ライブラリ) | 例のスケッチが距離を `Serial.print` する | 単位 (m か mm か) の確認。アンカーを複数にする改造 |
| **DW3000 / DWM3001C**<br>(Qorvo SDK の TWR 例) | 例が 2 台間の距離を出す | **アンカー複数台への拡張が必要**。例は 1 対 1 |
| **NXP SR150 / Android UWB API** | 距離 + 方位角 | Android 側からホストへ渡す経路 |
| **自作 (DW1000/DW3000 直叩き)** | 何も | DS-TWR の実装から。**ここが一番重い** |

**下に行くほど手前の作業が増える。** DWM1001 なら今日つながるが、
DW3000 を素から立ち上げるなら、このライブラリに届くまでに数週間かかる。

`TextHal` で始めて、精度を詰める段になったら `JsonLinesHal` に移るのが
現実的な順序。品質値 (`q`) を載せられるようになると NLOS に強くなる。

---

## 正直な話

**「誰でもすぐ使える」かというと、UWB モジュールが既に距離を出している人なら
すぐ使える。そうでない人は、まずそこまで到達する必要がある。**

このライブラリが引き受けているのは、測距値が手に入ってから先の

- 幾何的に正しく解く (退化した配置での破綻を含めて)
- NLOS で伸びた測距に引きずられない
- 移動体を追う
- 出た位置を信用してよいか判断できるようにする

という部分で、これは自分で書くと地味に大変なところではある。
逆に、UWB を動かすこと自体の面倒さは何も減らしていない。

ハードが無くても [ブラウザ UI](UWB.md#ブラウザ-ui) で
アルゴリズムと配置の検討はできるので、**モジュールを買う前に
「その部屋で何 cm 出そうか」を見ておく**使い方はできる。
