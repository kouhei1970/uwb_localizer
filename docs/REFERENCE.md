# リファレンス

**このファイルはコードから生成している** (`python docs/build_reference.py`)。
手で直しても次の生成で消えるので、直すならコードの docstring と
argparse の `help` を直す。

対象バージョン: `uwb_loc 0.1.0`

使い方の流れは [TUTORIAL.md](TUTORIAL.md)、動くコードは
[../examples/](../examples/) にある。ここは「何があるか」の一覧。

## コマンドライン

`python -m uwb_loc <コマンド>` か、インストール時に入る
`uwb-loc <コマンド>` で呼ぶ。どちらも同じ。

| コマンド | 何をする |
|---|---|
| [`sim`](#sim) | 実機なしで観測を作り、測位レベルごとの精度を出す。 |
| [`replay`](#replay) | 記録したログを別の測位レベルで解き直す。現場で録っておけば、あとで何度でも試せる。 |
| [`gdop`](#gdop) | 置く前に配置の良し悪しを見る。GDOP が大きい場所は、測距が同じ精度でも位置が暴れる。 |
| [`survey`](#survey) | アンカー同士で測った距離の表から座標を復元する。巻き尺で全台測らずに済む。 |
| [`sniff`](#sniff) | 実機立ち上げの最初にやること。ここで単位とアンカー ID を確定させておく。 |
| [`ryuw122`](#ryuw122) | 並べる前の準備。設定は Flash に残るので一度書けば済む。 |
| [`ui`](#ui) | 配置・誤差モデル・測位レベルの比較と、実機のライブ表示。 |

### `sim`

実機なしで観測を作り、測位レベルごとの精度を出す。

```
uwb-loc sim [-h] [--anchors ANCHORS] [--room X Y Z] [--n-low N_LOW]
                   [--dim {2,3}] [--height HEIGHT] [--levels LEVELS]
                   [--duration DURATION] [--rate RATE] [--seed SEED]
                   [--traj {figure8,circle,static,random_walk}] [--size SIZE]
                   [--period PERIOD] [--sigma0 SIGMA0] [--nlos NLOS]
                   [--nlos-bias NLOS_BIAS] [--loss LOSS] [--log LOG]
```

| 引数 | 説明 |
|---|---|
| `--anchors` | アンカー座標の JSON。省略すると --room から自動生成 |
| `--room` | 部屋の大きさ [m] (既定 `[8.0, 6.0, 2.6]`) |
| `--n-low` | 下段アンカーの数 (0 で天井のみ = 同一平面になる) (既定 `4`) |
| `--dim` | 2 なら高さを --height に固定して解く (選択: `2` / `3`、既定 `3`) |
| `--height` | タグ高さ / 2D の固定高さ [m] (既定 `1.2`) |
| `--levels` | 比較する測位レベルをカンマ区切りで (既定 `Lv0,Lv1,Lv2,Lv3`) |
| `--duration` | 測る長さ [s] (既定 `40.0`) |
| `--rate` | 観測レート [Hz] (既定 `10.0`) |
| `--seed` | 乱数の種。変えると誤差の出方が変わる (既定 `0`) |
| `--traj` | タグの動き方 (選択: `figure8` / `circle` / `static` / `random_walk`、既定 `figure8`) |
| `--size` | 軌道の大きさ [m] (既定 `2.0`) |
| `--period` | 軌道を 1 周する時間 [s] (既定 `24.0`) |
| `--sigma0` | 測距ノイズの標準偏差 [m] (既定 `0.08`) |
| `--nlos` | NLOS (見通しが切れて距離が伸びる) 確率 (既定 `0.15`) |
| `--nlos-bias` | NLOS のとき伸びる量の平均 [m] (既定 `0.8`) |
| `--loss` | 測距が欠測する確率 (既定 `0.03`) |
| `--log` | 観測を JSON Lines で書き出す (replay で読み直せる) |

### `replay`

記録したログを別の測位レベルで解き直す。現場で録っておけば、あとで何度でも試せる。

```
uwb-loc replay [-h] [--anchors ANCHORS] [--room X Y Z] [--n-low N_LOW]
                      [--dim {2,3}] [--height HEIGHT] [--level LEVEL]
                      [--format {jsonl,text}] [--pattern PATTERN]
                      [--unit {m,cm,mm}] [--prefix PREFIX] [--rate RATE]
                      [--out OUT]
                      path
```

| 引数 | 説明 |
|---|---|
| `--anchors` | アンカー座標の JSON。省略すると --room から自動生成 |
| `--room` | 部屋の大きさ [m] (既定 `[8.0, 6.0, 2.6]`) |
| `--n-low` | 下段アンカーの数 (0 で天井のみ = 同一平面になる) (既定 `4`) |
| `--dim` | 2 なら高さを --height に固定して解く (選択: `2` / `3`、既定 `3`) |
| `--height` | タグ高さ / 2D の固定高さ [m] (既定 `1.2`) |
| `path` (位置引数) | 読むログ (sim --log か JsonLinesWriter が書いたもの) |
| `--level` | 測位レベル (Lv0/Lv1/Lv2/Lv3) (既定 `Lv2`) |
| `--format` | ログの形式 (選択: `jsonl` / `text`、既定 `jsonl`) |
| `--pattern` | text のときの正規表現 (省略時は汎用パターン) |
| `--unit` | text のときの距離の単位 (選択: `m` / `cm` / `mm`、既定 `m`) |
| `--prefix` | アンカー ID の接頭辞 (例 A) (既定 ``) |
| `--rate` | text のとき時刻を合成するレート [Hz]。Lv3 に要る (既定 `10.0`) |
| `--out` | 結果を CSV で書き出す |

### `gdop`

置く前に配置の良し悪しを見る。GDOP が大きい場所は、測距が同じ精度でも位置が暴れる。

```
uwb-loc gdop [-h] [--anchors ANCHORS] [--room X Y Z] [--n-low N_LOW]
                    [--dim {2,3}] [--height HEIGHT] [--nx NX] [--ny NY]
```

| 引数 | 説明 |
|---|---|
| `--anchors` | アンカー座標の JSON。省略すると --room から自動生成 |
| `--room` | 部屋の大きさ [m] (既定 `[8.0, 6.0, 2.6]`) |
| `--n-low` | 下段アンカーの数 (0 で天井のみ = 同一平面になる) (既定 `4`) |
| `--dim` | 2 なら高さを --height に固定して解く (選択: `2` / `3`、既定 `3`) |
| `--height` | タグ高さ / 2D の固定高さ [m] (既定 `1.2`) |
| `--nx` | ヒートマップの横の分割数 (既定 `60`) |
| `--ny` | ヒートマップの縦の分割数 (既定 `40`) |

### `survey`

アンカー同士で測った距離の表から座標を復元する。巻き尺で全台測らずに済む。

```
uwb-loc survey [-h] [--dim {2,3}] matrix
```

| 引数 | 説明 |
|---|---|
| `matrix` (位置引数) | N×N の距離 [m] の CSV。ヘッダ行と ID 列は省略可、空欄は欠測 |
| `--dim` | 3 なら最低 4 台、2 なら最低 3 台の相互測距が要る (選択: `2` / `3`、既定 `3`) |

### `sniff`

実機立ち上げの最初にやること。ここで単位とアンカー ID を確定させておく。

```
uwb-loc sniff [-h] [--serial SERIAL] [--baud BAUD] [--tcp TCP]
                     [--file FILE] [--pattern PATTERN] [--unit {m,cm,mm}]
                     [--prefix PREFIX] [--lines LINES]
```

| 引数 | 説明 |
|---|---|
| `--serial` | シリアルポート (例 /dev/ttyUSB0) |
| `--baud` | シリアルのボーレート (既定 `115200`) |
| `--tcp` | TCP で読む場合の host:port |
| `--file` | 保存したログを読む |
| `--pattern` | 正規表現 (省略すると推測する) |
| `--unit` | 距離の単位 (選択: `m` / `cm` / `mm`、既定 `m`) |
| `--prefix` | アンカー ID の接頭辞 (例 A) (既定 ``) |
| `--lines` | 読む行数 (既定 `40`) |

### `ryuw122`

並べる前の準備。設定は Flash に残るので一度書けば済む。

```
uwb-loc ryuw122 [-h] --serial SERIAL [--baud BAUD] [--address ADDRESS]
                       [--network-id NETWORK_ID] [--cpin CPIN]
                       [--channel {5,9}] [--bandwidth {0,1}]
                       [--power {0,1,2,3,4,5}] [--cal CAL] [--payload PAYLOAD]
                       {info,anchor,tag-setup,tag}
```

| 引数 | 説明 |
|---|---|
| `action` (位置引数) | info: 今の設定を読む / anchor: ANCHOR に設定 / tag-setup: TAG に設定 / tag: TAG に設定して動かし続ける (選択: `info` / `anchor` / `tag-setup` / `tag`) |
| `--serial` | シリアルポート (例 /dev/ttyUSB0) (**必須**) |
| `--baud` | シリアルのボーレート (既定 `115200`) |
| `--address` | この機体のアドレス (8 バイト ASCII, 機体ごとに変える) |
| `--network-id` | NETWORKID (8 バイト ASCII, 全機で同じ) |
| `--cpin` | AES128 パスワード (32 文字, 全機で同じ) |
| `--channel` | RF チャネル 5: 6489.6MHz / 9: 7987.2MHz (全機で同じ) (選択: `5` / `9`) |
| `--bandwidth` | データレート 0: 850kbps / 1: 6.8Mbps (全機で同じ) (選択: `0` / `1`) |
| `--power` | 送信出力 0-5 (5 が最大) (選択: `0` / `1` / `2` / `3` / `4` / `5`) |
| `--cal` | AT+CAL 距離校正 [cm], -100〜100 |
| `--payload` | TAG が積むデータ (ANCHOR 側と 3 バイト以内の差にする) (既定 `RNGE`) |

### `ui`

配置・誤差モデル・測位レベルの比較と、実機のライブ表示。

```
uwb-loc ui [-h] [--host HOST] [--port PORT] [--no-browser]
```

| 引数 | 説明 |
|---|---|
| `--host` | 0.0.0.0 にすると同じ LAN の端末から開ける (認証は無い) (既定 `127.0.0.1`) |
| `--port` | 待ち受けポート (既定 `8765`) |
| `--no-browser` | ブラウザを自動で開かない |

## Python API

`import uwb_loc as ul` で全部触れる。

### データ型

HAL と測位器の間を流れるもの。単位は m / rad / s、右手系で z が上。

#### `Anchor(id: 'str', position: 'np.ndarray', enabled: 'bool' = True, antenna_delay_m: 'float' = 0.0, sigma0: 'float' = 0.1, sigma_per_m: 'float' = 0.0, position_sigma: 'float' = 0.0) -> None`

アンカー (固定局).

| メソッド | 説明 |
|---|---|
| `range_sigma(self, distance: 'float') -> 'float'` | 距離 ``distance`` におけるモデル上の測距標準偏差 [m]. |
| `to_dict(self) -> 'dict[str, Any]'` | JSON に載る形の辞書にする. 座標は m のまま. |
| `from_dict(cls, d: 'dict[str, Any]') -> "'Anchor'"` | ``to_dict`` が出した辞書から復元する. |

#### `Measurement(anchor_id: 'str', value: 'float', kind: 'MeasKind' = <MeasKind.RANGE: 'range'>, t: 'float' = 0.0, sigma: 'float | None' = None, quality: 'float | None' = None, ref_anchor_id: 'str | None' = None, tag_id: 'str' = 'tag0', raw: 'dict[str, Any]' = <factory>) -> None`

観測 1 本.

| メソッド | 説明 |
|---|---|
| `to_dict(self) -> 'dict[str, Any]'` | JSON Lines の 1 観測ぶんの辞書にする (``a`` / ``d`` などの短縮キー). |
| `from_dict(cls, d: 'dict[str, Any]', *, tag_id: 'str' = 'tag0', t: 'float' = 0.0) -> "'Measurement'"` | 観測 1 本の辞書から復元する. ``d`` は ``v`` の別名として受け付ける. |

#### `MeasurementBatch(t: 'float', measurements: 'list[Measurement]' = <factory>, tag_id: 'str' = 'tag0') -> None`

同一エポックにまとめた観測.

| メソッド | 説明 |
|---|---|
| `of_kind(self, kind: 'MeasKind') -> 'list[Measurement]'` | その種別の観測だけ取り出す. |
| `to_dict(self) -> 'dict[str, Any]'` | JSON Lines の 1 行ぶんの辞書にする. |
| `from_dict(cls, d: 'dict[str, Any]') -> "'MeasurementBatch'"` | JSON Lines の 1 行から復元する. |

#### `MeasKind(*args, **kwds)`

観測量の種別.

値: `RANGE` (`range`), `TDOA` (`tdoa`), `AZIMUTH` (`azimuth`), `ELEVATION` (`elevation`)

#### `Fix(position: 'np.ndarray', covariance: 'np.ndarray', t: 'float' = 0.0, ok: 'bool' = True, n_used: 'int' = 0, n_total: 'int' = 0, residual_rms: 'float' = nan, gdop: 'float' = nan, excluded: 'list[str]' = <factory>, iterations: 'int' = 0, level: 'str' = '', velocity: 'np.ndarray | None' = None, ambiguous: 'bool' = False) -> None`

測位結果.

| メソッド | 説明 |
|---|---|
| `sigma` | 位置誤差の代表値 (共分散のトレースの平方根) [m]. |
| `failed(cls, t: 'float' = 0.0, n_total: 'int' = 0, level: 'str' = '') -> "'Fix'"` | 測位できなかったことを表す Fix を作る (``ok=False``). |
| `to_dict(self) -> 'dict[str, Any]'` | 位置・共分散・品質指標を JSON に載る形にする. |

#### `MeasurementModel(anchors: 'list[Anchor]', *, apply_antenna_delay: 'bool' = True) -> 'None'`

アンカー表を保持して観測を評価する.

| メソッド | 説明 |
|---|---|
| `known(self, m: 'Measurement') -> 'bool'` | その観測を評価できるか (アンカー座標が既知で有効か). |
| `corrected_value(self, m: 'Measurement') -> 'float'` | アンテナ遅延を補正した観測値. |
| `sigma(self, m: 'Measurement', distance: 'float') -> 'float'` | 観測の 1σ. |
| `evaluate(self, p: 'np.ndarray', m: 'Measurement') -> 'tuple[float, np.ndarray, float]'` | 観測 1 本を評価する. |
| `assemble(self, p: 'np.ndarray', meas: 'list[Measurement]') -> 'tuple[np.ndarray, np.ndarray, np.ndarray]'` | 観測のリストをまとめて評価する. |

### HAL — 観測の入り口

チップごとの差をここで吸収する。測位側のコードは変わらない。

#### `UwbHal(*args, **kwargs)`

UWB ハードウェア抽象化層.

| メソッド | 説明 |
|---|---|
| `anchors` | アンカー一覧. |
| `poll(self, timeout: 'float' = 0.0) -> 'list[MeasurementBatch]'` | 溜まっている観測を返す. |
| `open(self) -> 'None'` | デバイスを開く. 既定は何もしない. |
| `close(self) -> 'None'` | デバイスを閉じる. 既定は何もしない. |
| `is_open` | 通信が生きているか. 切れたら False を返す. |
| `stream(self, timeout: 'float' = 1.0) -> 'Iterator[MeasurementBatch]'` | 観測を延々と流すジェネレータ. |

#### `TextHal(stream: 'IO[str]', pattern: 'str', anchors: 'list[Anchor] | None' = None, *, unit: 'str' = 'm', anchor_prefix: 'str' = '', group: 'bool' = True, max_span: 'float' = 0.5, rate_hz: 'float | None' = None, scale: 'float' = 1.0, offset: 'float' = 0.0, name: 'str' = 'text') -> 'None'`

行指向のテキスト出力を正規表現で読む HAL.

| メソッド | 説明 |
|---|---|
| `from_path(cls, path: 'str', pattern: 'str', **kw: 'Any') -> "'TextHal'"` | 保存したログファイルを読む. |
| `from_serial(cls, port: 'str', baudrate: 'int', pattern: 'str', **kw: 'Any') -> "'TextHal'"` | シリアルポートを開いて読む (pyserial が要る). |
| `from_tcp(cls, host: 'str', port: 'int', pattern: 'str', **kw: 'Any') -> "'TextHal'"` | TCP で繋いで読む. |
| `parse(self, line: 'str', now: 'float') -> 'list[Measurement]'` | 1 行から測距を取り出す (テスト・sniff から直接呼べる). |
| `open(self) -> 'None'` | デバイスを開く. 既定は何もしない. |
| `close(self) -> 'None'` | デバイスを閉じる. 既定は何もしない. |
| `is_open` | (プロパティ) |
| `anchors` | (プロパティ) |
| `poll(self, timeout: 'float' = 0.0) -> 'list[MeasurementBatch]'` | 溜まっている観測を返す. |

#### `JsonLinesHal(stream: 'IO[str]', anchors: 'list[Anchor] | None' = None, *, name: 'str' = 'jsonl') -> 'None'`

JSON Lines を読むだけの HAL.

| メソッド | 説明 |
|---|---|
| `from_path(cls, path: 'str', **kw: 'Any') -> "'JsonLinesHal'"` | ログファイルを読む (リプレイ用). |
| `from_tcp(cls, host: 'str', port: 'int', **kw: 'Any') -> "'JsonLinesHal'"` | TCP でファームウェア/ブリッジに繋ぐ. |
| `from_serial(cls, port: 'str', baudrate: 'int' = 115200, **kw: 'Any') -> "'JsonLinesHal'"` | シリアルポートから読む (pyserial が要る). |
| `open(self) -> 'None'` | デバイスを開く. 既定は何もしない. |
| `close(self) -> 'None'` | デバイスを閉じる. 既定は何もしない. |
| `is_open` | (プロパティ) |
| `anchors` | (プロパティ) |
| `poll(self, timeout: 'float' = 0.0) -> 'list[MeasurementBatch]'` | 溜まっている観測を返す. |

#### `JsonLinesWriter(path: 'str', anchors: 'list[Anchor] | None' = None) -> 'None'`

観測を JSON Lines で記録する.

| メソッド | 説明 |
|---|---|
| `write_anchors(self, anchors: 'list[Anchor]') -> 'None'` | アンカー座標の行を書く. ログの先頭に 1 度だけ置く. |
| `write(self, batch: 'MeasurementBatch') -> 'None'` | 観測 1 エポックを 1 行書く. |
| `flush(self) -> 'None'` | バッファを吐き出す. |
| `close(self) -> 'None'` | 閉じる (未書き出しがあれば吐く). |

#### `PushHal(anchors: 'list[Anchor] | None' = None, *, group: 'bool' = True, max_span: 'float' = 0.5, clock=<built-in function monotonic>) -> 'None'`

外から観測を押し込む HAL.

| メソッド | 説明 |
|---|---|
| `push(self, anchor_id: 'str', distance: 'float', *, t: 'float | None' = None, quality: 'float | None' = None, sigma: 'float | None' = None, tag_id: 'str' = 'tag0') -> 'None'` | 測距 1 本を押し込む. |
| `push_many(self, readings, *, t: 'float | None' = None, **kw) -> 'None'` | ``[(アンカー ID, 距離), ...]`` をまとめて押し込む. |
| `push_batch(self, batch: 'MeasurementBatch') -> 'None'` | 組み立て済みのエポックをそのまま流す (束ね直さない). |
| `set_anchors(self, anchors: 'list[Anchor]') -> 'None'` | アンカー表を差し替える (自己測量の結果を反映するときなど). |
| `close(self) -> 'None'` | もう押し込まれないことを伝える (溜まっている分は流し切る). |
| `is_open` | (プロパティ) |
| `anchors` | (プロパティ) |
| `poll(self, timeout: 'float' = 0.0) -> 'list[MeasurementBatch]'` | 溜まっている観測を返す. |

#### `Ryuw122Hal(stream: 'IO[str]', tag_addresses: 'list[str]', anchors: 'list[Anchor] | None' = None, *, config: 'Ryuw122Config | None' = None, payload: 'str' = 'RNGE', timeout: 'float' = 0.35, period: 'float' = 0.0, group: 'bool' = True) -> 'None'`

RYUW122 を ANCHOR にして, 固定した TAG を順に呼ぶ HAL.

| メソッド | 説明 |
|---|---|
| `from_serial(cls, port: 'str', tag_addresses: 'list[str]', baudrate: 'int' = 115200, **kw: 'Any') -> "'Ryuw122Hal'"` | シリアルポートに繋ぐ (既定ボーレート 115200 は仕様書 4 節の既定値). |
| `command(self, cmd: 'str', timeout: 'float' = 1.0) -> 'list[str]'` | AT コマンドを 1 つ送り, ``+OK`` / ``+ERR`` が返るまでの行を返す. |
| `setup(self) -> 'bool'` | モジュールを ANCHOR に設定する. 成功したら True. |
| `range_once(self, tag: 'str') -> 'Measurement | None'` | TAG を 1 つ呼んで距離を得る. |
| `open(self) -> 'None'` | デバイスを開く. 既定は何もしない. |
| `close(self) -> 'None'` | デバイスを閉じる. 既定は何もしない. |
| `is_open` | (プロパティ) |
| `anchors` | (プロパティ) |
| `set_anchors(self, anchors: 'list[Anchor]') -> 'None'` | アンカー (= 固定した TAG) の座標を差し替える. |
| `poll(self, timeout: 'float' = 0.0) -> 'list[MeasurementBatch]'` | 溜まっている観測を返す. |

#### `Ryuw122Config(*, network_id: 'str | None' = None, address: 'str | None' = None, password: 'str | None' = None, channel: 'int | None' = None, bandwidth: 'int | None' = None, power: 'int | None' = None, calibration_cm: 'int | None' = None, rssi: 'bool' = True, duty_cycle: 'tuple[int, int] | None' = None) -> 'None'`

モジュールに流し込む設定 (仕様書 3-12 節).

| メソッド | 説明 |
|---|---|
| `commands(self, *, as_anchor: 'bool' = True) -> 'list[str]'` | 流し込む AT コマンドを順に並べる. |
| `for_tag(self, address: 'str') -> "'Ryuw122Config'"` | 無線側の設定はそのままに, アドレスだけ差し替えた複製を返す. |

#### `Ryuw122Terminal(stream: 'IO[str]') -> 'None'`

AT コマンドを打つだけの最小セッション (測距はしない).

| メソッド | 説明 |
|---|---|
| `from_serial(cls, port: 'str', baudrate: 'int' = 115200) -> "'Ryuw122Terminal'"` | シリアルポートを開く (既定 115200 は仕様書 4 節の既定値). |
| `command(self, cmd: 'str', timeout: 'float' = 1.0) -> 'list[str]'` | AT コマンドを 1 つ送り, 応答の行を返す. 送受信は :attr:`log` に残る. |
| `query(self, cmd: 'str', timeout: 'float' = 1.0) -> 'str | None'` | ``AT+ADDRESS?`` → ``"DAVID123"`` のように値だけ取り出す. |
| `info(self) -> 'dict[str, str | None]'` | 今の設定をまとめて読む. 応答しない項目は None. |
| `provision(self, config: 'Ryuw122Config', *, as_anchor: 'bool') -> 'bool'` | 設定を流し込む. 全コマンドが ``+OK`` を返したら True. |
| `factory_reset(self) -> 'list[str]'` | ``AT+FACTORY``. 出荷時の値に戻す (仕様書 20 節). |
| `close(self) -> 'None'` | ポートを閉じる. |

#### `Ryuw122Tag(stream: 'IO[str]', *, config: 'Ryuw122Config | None' = None, payload: 'str' = 'RNGE', refill: 'float' = 0.2, setup: 'bool' = True) -> 'None'`

TAG 側の面倒を見る: 設定を入れて ``AT+TAG_SEND`` を積み続ける.

| メソッド | 説明 |
|---|---|
| `from_serial(cls, port: 'str', baudrate: 'int' = 115200, **kw: 'Any') -> "'Ryuw122Tag'"` | シリアルポートを開く (既定 115200 は仕様書 4 節の既定値). |
| `setup(self) -> 'bool'` | モジュールを TAG に設定する. |
| `open(self) -> 'None'` | TAG として設定し, ``AT+TAG_SEND`` を積み続けるスレッドを起こす. |
| `close(self) -> 'None'` | スレッドを止めてポートを閉じる. |

#### `sniff(stream: 'IO[str]', pattern: 'str | None' = None, *, n: 'int' = 40, unit: 'str' = 'm', anchor_prefix: 'str' = '') -> 'dict[str, Any]'`

流れてくる行を覗いて、正規表現が効いているか確かめる.

### 測位器

同じインターフェイスで差し替えられる。`make_estimator` が入り口。

#### `make_estimator(level: 'str', anchors: 'list[Anchor]', config: 'SolveConfig | None' = None, **kwargs: 'object') -> 'PositionEstimator'`

レベル名から測位器を作る.

#### `LEVELS`

`{'Lv0': <class 'uwb_loc.solvers.nls.Lv0Trilateration'>, 'Lv1': <class 'uwb_loc.solvers....`

#### `PositionEstimator(anchors: 'list[Anchor]', config: 'SolveConfig | None' = None) -> 'None'`

測位器の基底クラス.

| メソッド | 説明 |
|---|---|
| `set_anchors(self, anchors: 'list[Anchor]') -> 'None'` | アンカー表を差し替える (自己測量の結果を反映するときなど). |
| `update(self, batch: 'MeasurementBatch') -> 'Fix'` | 観測を 1 エポック分与えて位置を得る. |
| `reset(self) -> 'None'` | 内部状態を捨てる. ステートレスな実装では何もしない. |
| `resolve_mirror(self, p: 'np.ndarray', cov: 'np.ndarray | None' = None) -> 'tuple[np.ndarray, np.ndarray | None, bool]'` | 同一平面配置で生じる鏡像解を片側に寄せる. |

#### `SolveConfig(*, dim: 'int' = 3, z_fixed: 'float' = 0.0, z_bounds: 'tuple[float, float] | None' = None, max_iter: 'int' = 30, tol: 'float' = 0.0001) -> 'None'`

測位の共通設定.

| メソッド | 説明 |
|---|---|
| `free_mask` | 自由に動かす座標のマスク, shape (3,). |
| `project(self, p: 'np.ndarray') -> 'np.ndarray'` | 拘束を位置に適用する. |

#### `RobustLoss(kind: 'str' = 'huber', k: 'float' = 1.345, one_sided: 'bool' = True, k_pos_scale: 'float' = 0.6) -> None`

ロバスト損失の設定.

| メソッド | 説明 |
|---|---|
| `thresholds(self, residual: 'np.ndarray') -> 'np.ndarray'` | 残差ごとのしきい値 (片側損失を反映). |

#### `Lv0Trilateration(anchors: 'list[Anchor]', config: 'SolveConfig | None' = None) -> 'None'`

Lv0 — 線形最小二乗による三辺測量.

| メソッド | 説明 |
|---|---|
| `update(self, batch: 'MeasurementBatch') -> 'Fix'` | 観測を 1 エポック分与えて位置を得る. |

#### `Lv1WeightedNLS(anchors, config: 'SolveConfig | None' = None, *, chi2_threshold: 'float | None' = 3.5, loss: 'RobustLoss | None' = None, use_physical_gate: 'bool' = True, use_ransac: 'bool' = False, ransac_trigger: 'float' = 3.0, warm_start: 'bool' = True) -> 'None'`

Lv1 — 重み付き非線形最小二乗 + χ² ゲート.

| メソッド | 説明 |
|---|---|
| `reset(self) -> 'None'` | 内部状態を捨てる. ステートレスな実装では何もしない. |
| `update(self, batch: 'MeasurementBatch') -> 'Fix'` | 観測を 1 エポック分与えて位置を得る. |

#### `Lv2RobustNLS(anchors, config: 'SolveConfig | None' = None, **kw) -> 'None'`

Lv2 — Beck 初期解 + Huber-IRLS + 片側損失 (+RANSAC).

#### `Lv3TightlyCoupledEKF(anchors: 'list[Anchor]', config: 'SolveConfig | None' = None, *, motion: 'str' = 'cv', sigma_a: 'float' = 1.0, gate: 'float' = 3.0, max_dt: 'float' = 2.0, max_rejects: 'int' = 5, init_estimator: 'PositionEstimator | None' = None) -> 'None'`

密結合拡張カルマンフィルタ.

| メソッド | 説明 |
|---|---|
| `set_anchors(self, anchors: 'list[Anchor]') -> 'None'` | アンカー表を差し替える (自己測量の結果を反映するときなど). |
| `reset(self) -> 'None'` | 内部状態を捨てる. ステートレスな実装では何もしない. |
| `predict(self, dt: 'float') -> 'None'` | ``dt`` 秒ぶん状態を進める (観測なしの時間経過). |
| `update(self, batch: 'MeasurementBatch') -> 'Fix'` | 観測を 1 エポック分与えて位置を得る. |

### パイプライン

HAL と測位器をつないで回す。

#### `Pipeline(hal: 'UwbHal', *, level: 'str' = 'Lv2', config: 'SolveConfig | None' = None, estimator: 'PositionEstimator | None' = None, anchors: 'list[Anchor] | None' = None, on_fix: 'Callable[[Fix], None] | None' = None, **kwargs: 'object') -> 'None'`

観測 -> 測位 -> 結果 の一連の流れ.

| メソッド | 説明 |
|---|---|
| `process(self, batch: 'MeasurementBatch') -> 'Fix'` | 観測 1 エポックを処理する. |
| `run(self, duration: 'float | None' = None, *, max_epochs: 'int | None' = None, timeout: 'float' = 1.0) -> 'Iterator[Fix]'` | HAL から読みながら測位し続けるジェネレータ. |
| `positions(self) -> 'np.ndarray'` | これまでの推定位置, shape (n, 3). 失敗したエポックは NaN. |
| `times(self) -> 'np.ndarray'` | これまでに測位した時刻の一覧 [s]. |

#### `run_offline(batches: 'list[MeasurementBatch]', anchors: 'list[Anchor]', *, level: 'str' = 'Lv2', config: 'SolveConfig | None' = None, **kwargs: 'object') -> 'list[Fix]'`

記録済みの観測列をまとめて処理する.

### シミュレータ

実機なしで動かす。HAL と同じインターフェイスなので差し替えられる。

#### `SimulatedHal(anchors: 'list[Anchor]', traj: 'Trajectory', error: 'ErrorModel | None' = None, *, rate_hz: 'float' = 10.0, kind: 'MeasKind' = <MeasKind.RANGE: 'range'>, seed: 'int' = 0, t0: 'float' = 0.0, tag_id: 'str' = 'tag0') -> 'None'`

模擬 UWB.

| メソッド | 説明 |
|---|---|
| `anchors` | (プロパティ) |
| `truth(self, t: 'float | None' = None) -> 'np.ndarray'` | 真の位置. |
| `step(self) -> 'tuple[np.ndarray, MeasurementBatch]'` | 1 エポック進めて (真位置, 観測) を返す. |
| `poll(self, timeout: 'float' = 0.0) -> 'list[MeasurementBatch]'` | 溜まっている観測を返す. |
| `generate(self, duration: 'float') -> 'tuple[np.ndarray, list[np.ndarray], list[MeasurementBatch]]'` | ``duration`` 秒ぶんまとめて生成する. |

#### `ErrorModel(sigma0: 'float' = 0.08, sigma_per_m: 'float' = 0.004, nlos_prob: 'float' = 0.15, nlos_hold: 'float' = 1.5, nlos_bias_mean: 'float' = 0.8, loss_rate: 'float' = 0.03, max_range: 'float' = 40.0, antenna_delay: 'float' = 0.0, anchor_position_error: 'float' = 0.0, report_sigma: 'bool' = True, report_quality: 'bool' = True) -> None`

測距の誤差モデル.

| メソッド | 説明 |
|---|---|
| `sigma_at(self, d: 'float') -> 'float'` | その距離での測距標準偏差 [m]. |

#### `Scenario(anchors: 'list[Anchor]', traj: 'Trajectory', error: 'ErrorModel' = <factory>, rate_hz: 'float' = 10.0, duration: 'float' = 60.0, kind: 'MeasKind' = <MeasKind.RANGE: 'range'>, seed: 'int' = 0) -> None`

シミュレーション条件一式 (UI と CLI が共有する).

| メソッド | 説明 |
|---|---|
| `hal(self) -> 'SimulatedHal'` | この設定どおりの :class:`SimulatedHal` を作る. |

#### `trajectory(*args, **kwargs)`

よく使う軌道.

| メソッド | 説明 |
|---|---|
| `static(p: 'np.ndarray') -> 'Trajectory'` | 静止. |
| `line(p0: 'np.ndarray', p1: 'np.ndarray', period: 'float' = 20.0) -> 'Trajectory'` | 2 点間を往復. |
| `circle(center: 'np.ndarray', radius: 'float' = 2.0, period: 'float' = 20.0, z_amp: 'float' = 0.0) -> 'Trajectory'` | 水平円 (``z_amp`` を与えると螺旋). |
| `figure8(center: 'np.ndarray', size: 'float' = 2.0, period: 'float' = 24.0, z_amp: 'float' = 0.3) -> 'Trajectory'` | 8 の字. 加減速と旋回が入るのでフィルタの追従性を見るのに向く. |
| `random_walk(start: 'np.ndarray', speed: 'float' = 0.5, bounds: 'tuple[tuple[float, float], ...] | None' = None, seed: 'int' = 0, dt: 'float' = 0.05) -> 'Trajectory'` | ランダムウォーク (時刻をキャッシュして再現性を保つ). |

#### `make_anchors(positions: 'np.ndarray', prefix: 'str' = 'A', **kw: 'object') -> 'list[Anchor]'`

座標配列からアンカー一覧を作る.

#### `room_anchors(size: 'tuple[float, float, float]' = (8.0, 6.0, 2.6), *, n_low: 'int' = 4, z_low: 'float' = 0.3, z_high: 'float | None' = None) -> 'list[Anchor]'`

部屋の四隅に上下 2 段でアンカーを置く既定配置.

### 配置の評価

現場で精度が出ない原因はたいてい設営。置く前に見る。

#### `gdop_at(point: 'np.ndarray', anchors: 'list[Anchor]', *, dim: 'int' = 3) -> 'float'`

ある点における GDOP.

#### `gdop_map(anchors: 'list[Anchor]', bounds: 'tuple[tuple[float, float], tuple[float, float]]', z: 'float' = 1.0, *, nx: 'int' = 40, ny: 'int' = 40, dim: 'int' = 3) -> 'tuple[np.ndarray, np.ndarray, np.ndarray]'`

水平面を格子で切って GDOP を評価する.

#### `crlb_at(point: 'np.ndarray', anchors: 'list[Anchor]', *, dim: 'int' = 3) -> 'float'`

クラメール・ラオ下限 (位置誤差の理論下限) [m].

#### `anchor_condition(anchors: 'list[Anchor]') -> 'dict[str, float | bool]'`

アンカー配置の素性を調べる.

### キャリブレーション

巻き尺で全台測らずに済ませる道具。

#### `self_survey(distances: 'np.ndarray', ids: 'list[str] | None' = None, *, dim: 'int' = 3, max_iter: 'int' = 100, weights: 'np.ndarray | None' = None) -> 'list[Anchor]'`

アンカー間の相互測距からアンカー配置を推定する.

#### `align_to_reference(anchors: 'list[Anchor]', reference: 'dict[str, np.ndarray]', *, allow_reflection: 'bool' = True) -> 'list[Anchor]'`

自己測量の結果を実世界の座標系に合わせる.

#### `fit_range_bias(measured: 'np.ndarray', true: 'np.ndarray') -> 'tuple[float, float]'`

距離バイアスの 1 次モデルを当てる.

#### `estimate_antenna_delays(anchor_ids: 'list[str]', measured: 'np.ndarray', true_distance: 'np.ndarray', *, tag_delay: 'bool' = True) -> 'dict[str, float]'`

アンテナ遅延をアンカーごとに推定する [m].

### 精度の評価

推定と真値を突き合わせる。

#### `error_stats(truth: 'np.ndarray', est: 'np.ndarray') -> 'dict[str, float]'`

誤差統計をまとめて返す.

#### `error_series(truth: 'np.ndarray', est: 'np.ndarray') -> 'tuple[np.ndarray, np.ndarray]'`

各時刻の 3 次元誤差と水平誤差 [m].

#### `error_cdf(errors: 'np.ndarray', n: 'int' = 100) -> 'tuple[np.ndarray, np.ndarray]'`

誤差の累積分布.

### その他

#### `__version__`

`'0.1.0'`

#### `WIRE_VERSION`

`1`

#### `uwb_loc.geometry`

幾何評価 — アンカー配置の良し悪しを測る.

#### `uwb_loc.metrics`

評価指標 — 「で, 何 cm 出るのか」を答えるための集計.

#### `uwb_loc.calibration`

キャリブレーションと設営支援.
