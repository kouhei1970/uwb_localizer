# REYAX RYUW122 / RYUW122_Lite で測位する

仕様書: REYAX "RYUW122 AT COMMAND GUIDE" (2024-03-12 版)。
リポジトリには**同梱していない** (再配布できないため)。入手先と、コードが
参照している節の対応は [`docs/datasheets/README.md`](datasheets/README.md)。

## このモジュールの性質を先に理解する

**距離は ANCHOR 側の UART に出る。** ANCHOR が `AT+ANCHOR_SEND` で TAG を
指名して初めて測距が成立し、その応答として返る。

```
送信:  AT+ANCHOR_SEND=DAVID123,4,RNGE
応答:  +OK
       +ANCHOR_RCV=DAVID123,5,HELLO,40 cm,-72
                   └ TAG      └len └data └距離(cm) └RSSI
```

ここから 2 つ効いてくる。

1. **1 台の ANCHOR は 1 度に 1 つの TAG としか測距しない。**
   複数点の距離を集めるには宛先を変えて順に呼ぶ (ポーリング)
2. **距離は ANCHOR の UART に出る。** 固定局を ANCHOR にすると、
   距離が固定局それぞれの UART に散らばる

### TAG 側のホストは距離を受け取れない

仕様書 16 節 (`+ANCHOR_RCV`) と 17 節 (`+TAG_RCV`) を並べると、
**TAG 側には距離のフィールドが無い**。

| | ANCHOR 側 `+ANCHOR_RCV` | TAG 側 `+TAG_RCV` |
|---|---|---|
| 相手のアドレス | **あり** (8 バイト ASCII) | **無い** |
| ペイロード長 | あり | あり |
| データ本体 | あり | あり |
| **距離** | **あり (cm)** | **無い** |
| RSSI | あり | あり |

```
+ANCHOR_RCV=DAVID123,5,HELLO,40 cm,-72     ← 距離が出る
+TAG_RCV=4,TEST                            ← 距離が無い。送信元も分からない
```

TAG に繋いだホストは「誰かに読まれた」ことしか分からない。
**移動体の上で自己位置を知りたいなら、移動体側を ANCHOR にするしかない。**
一般的な UWB システム (タグは小さく安く、アンカーは有線給電・LAN 接続) の
感覚とは逆になるので注意。

> **これは仕様書から読み取れることで、実機では確認していない。**
> 実機があれば TAG 側のポートを `python -m uwb_loc sniff --serial <port>` で
> 覗いてみてほしい。`+TAG_RCV` に距離が載っているようなら話が変わるので、
> 生の出力を Issue に貼ってもらえれば対応する。

**抜け道 (これも未検証)**: ANCHOR は測距の直後に距離を知っているので、
**次の `AT+ANCHOR_SEND` のペイロードに距離を載せて TAG に返す**ことはできる。
ペイロードは最大 12 バイトなので cm 値 (`"1234"` の 4 バイト) は十分入り、
ペイロード長の差 3 バイト以内の制約も満たせる。ただし **1 エポック遅れ**の
情報になり、**ANCHOR 側に MCU が要る**。

## 配置の取り方 — どちらを ANCHOR にするか

### (A) 移動側を ANCHOR にする ← **本 HAL が想定する構成**

```
PC ── UART ── RYUW122 (ANCHOR, 移動する)
                 ├── 測距 ──> RYUW122 (TAG) 部屋の隅に固定
                 ├── 測距 ──> RYUW122 (TAG)
                 └── 測距 ──> RYUW122 (TAG) ...
```

**UART 1 本で全部の距離が揃う。** 配線が最も簡単で、PC につないだ 1 台が
そのまま「位置を知りたいもの」になる。

このとき、ライブラリでいう**「アンカー座標」は固定した TAG の座標**で、
`Anchor.id` には TAG の `AT+ADDRESS` (8 バイト ASCII) を入れる。
名前が逆に感じるが、「距離の基準になる固定点」という意味では同じもの。

### (B) 移動側を TAG にする

一般的な UWB の構成だが、距離が各 ANCHOR の UART に出るので
**ANCHOR の数だけホスト接続が要る**。各 ANCHOR の MCU から距離を集めて
1 箇所に流す仕組みを自分で用意し、[`TextHal`](UWB.md) か `PushHal` に
渡す方が早い。

**移動体の上で自己位置を知る用途には使えない。** 上記のとおり TAG 側の
UART には距離が出ないので、集約した結果を無線などで移動体に送り返す
必要がある。

## TAG 側の準備 (これを忘れると距離が出ない)

**ANCHOR だけ設定しても距離は出ない。TAG 側にも設定が要る。**

| 項目 | どうする | 理由 |
|---|---|---|
| `AT+MODE=0` | TAG にする | 出荷時が TAG なので、買ったままなら不要 |
| `AT+NETWORKID` | **全機で同じ値** | 違うと通信しない |
| `AT+CPIN` | **全機で同じ値** | AES128。違うとデータを認識できない |
| `AT+CHANNEL` / `AT+BANDWIDTH` | **全機で同じ値** | 物理層が合わない |
| `AT+ADDRESS` | **機体ごとに違う値** | 後述。出荷時は全機同じ |
| `AT+TAG_SEND` | **積み続ける** | 積んでいないと呼びかけに応答できない |

### アドレスは「設定するもの」で、機体固有値ではない

`AT+ADDRESS` は 8 バイト ASCII の**書き換えられる設定**で、Flash に残る。
`AT+FACTORY` の出荷時値は `DAVID123`、つまり**買ってきた 5 台は全部同じ
アドレス**。そのまま並べると `AT+ANCHOR_SEND=DAVID123,...` がどれを呼んで
いるのか決まらないので、**並べる前に 1 台ずつ変える**。

書き換えられない機体固有値は `AT+UID?` (96 bit) の方。ただし
`AT+ANCHOR_SEND` の宛先に使えるのは `ADDRESS` なので、測位で使うのは
自分で振った 8 バイトの方になる。UID は「どの個体か」の判別用。

### 1 台ずつ設定する

```bash
# 今どうなっているか読む (立ち上げで詰まったら最初にこれ)
python -m uwb_loc ryuw122 info --serial /dev/ttyUSB0

# TAG として設定して Flash に焼く (1 台ずつ、--address だけ変えて繰り返す)
python -m uwb_loc ryuw122 tag-setup --serial /dev/ttyUSB0 \
    --address TAG00001 --network-id MYROOM01 \
    --cpin FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF --channel 9 --bandwidth 1

# PC につないだ 1 台を ANCHOR にする (UI から使うなら UI がやるので不要)
python -m uwb_loc ryuw122 anchor --serial /dev/ttyUSB0 \
    --address ANCHOR01 --network-id MYROOM01 \
    --cpin FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF --channel 9 --bandwidth 1
```

設定は Flash に残るので、**一度書けば次回からは電源を入れるだけ**。
ここで振った `--address` を、そのままアンカー一覧の ID に入れる。

Python からなら、ANCHOR 用の設定からアドレスだけ差し替えられる:

```python
base = ul.Ryuw122Config(network_id="MYROOM01", password="F" * 32,
                        channel=9, bandwidth=1)

with ul.Ryuw122Terminal.from_serial("/dev/ttyUSB0") as t:
    t.provision(base.for_tag("TAG00001"), as_anchor=False)
    print(t.info())        # 書けたか読み返す
```

### `AT+TAG_SEND` は「送信」ではなく「郵便受けに入れておく」

このモジュールには**測距コマンドが無い**。あるのはデータを送るコマンド
(`AT+ANCHOR_SEND`) だけで、**距離はそのデータ交換のついでに出てくる**。
距離が出る仕組みは電波の往復時間 (DS-TWR) なので、**TAG からの返信が
必ず要る**。その返信の中身が、TAG が事前に積んでおいたデータ。

仕様書 14 節の英語がそのまま動作の説明になっている:

> **AT+TAG_SEND** — Send data to the module and wait for the anchor to read it.
> (モジュールにデータを渡し、**ANCHOR が読みに来るのを待つ**)

つまり打った瞬間に電波は飛ばない。モジュールの中のバッファに、返信用の中身を
1 通置くだけ。

```
TAG:    AT+TAG_SEND=4,RNGE     → [バッファ: RNGE]   まだ何も飛ばない
                                        ↓
ANCHOR: AT+ANCHOR_SEND=...     → 往復。RNGE を持っていく
                                        ↓
                                  [バッファ: 空]
TAG の UART: +TAG_RCV=4,RNGE   ← 「読まれた」の合図
```

ここから 2 つ効いてくる。

1. **空だと距離が出ない。** 持ち帰るものが無い = 往復が成立しない =
   時間差が測れない。ANCHOR 側には `+ANCHOR_RCV` が一切出ず、
   ライブラリからは「応答なし」に見える。圏外・設定ミス・バッファ空が
   **全部同じ症状**になるので厄介
2. **1 回読まれると空になる。** 測距 1 回につき 1 通消費されるので、
   10 Hz で測位したいなら 10 Hz で入れ直す。これが「積み**続ける**」の意味

ペイロード長の差 3 バイト以内という制約も同じ理屈で、行きと帰りで電波に
乗っている時間が非対称すぎると往復時間の計算が狂うため。

なお「空だと距離が出ない」は上の一文と DS-TWR の原理からの読みで、
**仕様書に明文があるわけではない**。実機で空でも距離が出るようなら、
積み直しの頻度 (`refill`) は落とせる。

### `AT+TAG_SEND` を誰が打つか

本来は TAG 側の MCU の役目で、`+TAG_RCV` が「次を入れて」の合図になる。
**MCU を用意する前に PC で試したい**なら、ライブラリが代わりに打てる:

```bash
python -m uwb_loc ryuw122 tag --serial /dev/ttyUSB1 --address TAG00001 \
    --network-id MYROOM01
# TAG00001 として AT+TAG_SEND を積み続けます。Ctrl-C で終了。
#   積んだ 42 回 / 読まれた 37 回
```

「読まれた」が増えていれば ANCHOR との通信は成立している。0 のままなら
`NETWORKID` / `CPIN` / `CHANNEL` / `BANDWIDTH` か、ANCHOR が呼んでいる
アドレスが合っていない。

- **ANCHOR と TAG のペイロード長の差は 3 バイト以内**。超えると距離が
  計算されない (仕様書 13, 14 節)。どちらも既定 (`RNGE`、4 バイト) のまま
  使う分には気にしなくてよい
- 電池で長く持たせたいときだけ `AT+TAGD`
  (`Ryuw122Config(duty_cycle=...)`)。測位用途では出荷時の `0,0` =
  常時有効のままがよい

## 使い方

### ブラウザ UI から (いちばん早い)

```bash
pip install -e ".[serial]"      # pyserial が要る
python -m uwb_loc ui
```

0. 先に TAG 側を 1 台ずつ設定しておく (上記「TAG 側の準備」)。
   これを飛ばすと接続はできても距離が 1 本も返ってこない
1. 左パネルでアンカーを配置し、**ID を TAG のアドレスにする**
   (`TAG00001` など、8 バイト ASCII)
2. 「ライブ」タブ → ソースに **RYUW122 (シリアル)**
3. ポートを入れる。`NETWORKID` / `CPIN` は全機で揃えるなら入力する
4. 開始

**TAG アドレスを別に入力する欄は無い。** アンカー一覧の ID をそのまま
順に呼ぶので、2 箇所に同じものを書いてずれる事故が起きない。

状態表示に `測距 N 回 / 応答なし M 回` が出る。応答なしばかりなら
アドレス・`NETWORKID`・`CPIN` のいずれかが合っていない。

### Python から

```python
import uwb_loc as ul

anchors = [                                   # 固定した TAG の座標
    ul.Anchor("TAG00001", [0.2, 0.2, 2.4]),
    ul.Anchor("TAG00002", [7.8, 0.2, 2.4]),
    ul.Anchor("TAG00003", [7.8, 5.8, 0.3]),
    ul.Anchor("TAG00004", [0.2, 5.8, 0.3]),
    ul.Anchor("TAG00005", [0.2, 3.0, 0.3]),
]

hal = ul.Ryuw122Hal.from_serial(
    "/dev/ttyUSB0",
    [a.id for a in anchors],                  # 呼ぶ TAG の順番
    anchors=anchors,
    config=ul.Ryuw122Config(
        network_id="REYAX123",                # 全機で揃える (8 バイト)
        address="ANCHOR01",                   # この機体のアドレス
        password="F" * 32,                    # 全機で揃える (32 文字)
        channel=9,                            # 5: 6489.6MHz / 9: 7987.2MHz
        bandwidth=1,                          # 0: 850kbps / 1: 6.8Mbps
        calibration_cm=-11,                   # AT+CAL による粗補正
    ),
)

for fix in ul.Pipeline(hal, level="Lv2").run():
    if fix.ok:
        print(f"{fix.position.round(2)}  ±{fix.sigma:.2f} m  ({fix.n_used} 本)")
```

`open()` した時点で `AT+MODE=1` から順に設定を流し込み、そのあと
TAG を順に呼び続ける。流し込んだ結果は `hal.setup_log` に残る。

## 単位と精度まわり

| | |
|---|---|
| `+ANCHOR_RCV` の距離 | **cm**。HAL が m に直して渡す |
| `AT+CAL` | -100〜+100 cm の粗補正。Flash に残る |
| 細かい補正 | `Anchor.antenna_delay_m` の方が扱いやすい (機体ごとに持てる) |
| RSSI | `AT+RSSI=1` で `+ANCHOR_RCV` に載る。HAL が品質値 0-1 に写す |

RSSI から品質値への写像は `-95 dBm → 0` / `-60 dBm → 1` の粗いもので、
**現場で調整する前提**。うまく効かないようなら `Ryuw122Hal.range_once` を
参考に自分で書き換えるのが早い。

## 更新レートの見積もり

TAG を 1 台ずつ順に呼ぶので、**1 エポックの時間 ≒ TAG 台数 × 1 回の測距時間**。
`timeout` の既定は 0.35 秒なので、圏外の TAG があるとそこで待たされる。
台数が多いなら `timeout` を詰めるか、`tag_addresses` から外す。

## 動作確認について

**実機での確認は行っていない。** 仕様書の応答どおりに振る舞う模擬モジュールを
作って、AT コマンドの順序・`+ANCHOR_RCV` の解析・cm→m 換算・ポーリング・
TAG 側の設定と `AT+TAG_SEND` の積み直し・測位までを検証している
(`tests/test_ryuw122.py`、23 件)。
UI からは仮想シリアルポート (pty) 経由で一発接続と測位まで通してある。

実機で動かして違いがあれば、まず `python -m uwb_loc sniff --serial <port>` で
生の出力を見てほしい。`+ANCHOR_RCV` の形が仕様書と違っていれば
`parse_anchor_rcv` を直せば済む。
