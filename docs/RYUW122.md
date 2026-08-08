# REYAX RYUW122 / RYUW122_Lite で測位する

仕様書: [`docs/datasheets/RYUW122_AT_Command_Guide.pdf`](datasheets/RYUW122_AT_Command_Guide.pdf)
(2024-03-12 版)

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

## TAG 側の準備 (これを忘れると距離が出ない)

- TAG は `AT+TAG_SEND=<len>,<data>` でデータを積んでおかないと応答できない。
  **TAG 側の MCU が定期的に打つ必要がある**
- **ANCHOR と TAG のペイロード長の差は 3 バイト以内**。超えると距離が
  計算されない (仕様書 13, 14 節)
- `NETWORKID` と `CPIN` が全機で揃っていないと通信しない

## 使い方

### ブラウザ UI から (いちばん早い)

```bash
pip install -e ".[serial]"      # pyserial が要る
python -m uwb_loc ui
```

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
測位までを検証している (`tests/test_ryuw122.py`、15 件)。
UI からは仮想シリアルポート (pty) 経由で一発接続と測位まで通してある。

実機で動かして違いがあれば、まず `python -m uwb_loc sniff --serial <port>` で
生の出力を見てほしい。`+ANCHOR_RCV` の形が仕様書と違っていれば
`parse_anchor_rcv` を直せば済む。
