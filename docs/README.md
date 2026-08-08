# ドキュメントの歩き方

**各ファイルは 1 つの問いに答える。** 内容が重なるのは構わないが、
「どれを読めばいいか分からない」を避けるため、役割をここに書いておく。

## 目的から引く

| やりたいこと | 読むもの |
|---|---|
| とりあえず動かしたい | [../README.md](../README.md) |
| 使い方を順を追って覚えたい | [TUTORIAL.md](TUTORIAL.md) |
| コマンドや API を調べたい | [REFERENCE.md](REFERENCE.md) |
| 動くコードを見たい | [../examples/README.md](../examples/README.md) |
| **RYUW122** を使いたい | [RYUW122.md](RYUW122.md) |
| ほかのチップを実機で立ち上げたい | [BRINGUP.md](BRINGUP.md) |
| 自分の HAL を書きたい / ログ形式を知りたい | [UWB_PROTOCOL.md](UWB_PROTOCOL.md) |
| 中で何をしているか知りたい | [UWB_ALGORITHMS.md](UWB_ALGORITHMS.md) |
| **マイコン (C) で動かしたい** | [../c/README.md](../c/README.md) |
| なぜこの手法なのか知りたい | [DESIGN.md](DESIGN.md) |

## 各ファイルの役割

| ファイル | 答える問い | 答えないこと |
|---|---|---|
| [../README.md](../README.md) | **これは何で、なぜあり、最短で動かすには?** | 網羅的な使い方・API |
| [TUTORIAL.md](TUTORIAL.md) | **どう使うか** — 用語から実機接続・切り分けまで 6 章 | 全 API の網羅、式の導出 |
| [REFERENCE.md](REFERENCE.md) | **何があるか** — 全コマンド・全オプション・全 API | 使い方の筋道 (順序立てはしない) |
| [../examples/README.md](../examples/README.md) | **どのコードを見ればいいか** — 6 本の解説 | — |
| [RYUW122.md](RYUW122.md) | **RYUW122 固有の手順** — AT 設定、配置、TAG 側の準備 | 他チップの話 |
| [BRINGUP.md](BRINGUP.md) | **実機で何を用意し、何を渡すか** — モジュール別の目安 | チップ固有の手順 (→ RYUW122.md) |
| [UWB_PROTOCOL.md](UWB_PROTOCOL.md) | **HAL とライブラリの約束事** — 単位・座標系・型・JSON Lines スキーマ | 測位の中身 |
| [UWB_ALGORITHMS.md](UWB_ALGORITHMS.md) | **中で何をしているか** — 式の導出、実装との対応、C 移植 | 使い方 |
| [DESIGN.md](DESIGN.md) | **なぜこの設計にしたか** — 手法選定の記録、採らなかった案 | 現在の使い方 (古くなりうる) |
| [../c/README.md](../c/README.md) | **C 版の使い方と移植** — 大きさ、Python 版との違い | アルゴリズムの導出 (→ UWB_ALGORITHMS.md) |

### `UWB_ALGORITHMS.md` と `DESIGN.md` の違い

紛らわしいので明記しておく。

- **ALGORITHMS** = 「**どう動くか**」。今のコードの説明。式と実装が対応している
- **DESIGN** = 「**なぜそれを選んだか**」。実装前の検討と、採らなかった案の記録。
  設計判断を追うためのもので、**現在の使い方の情報源としては使わない**

## 書くときの決まり

このリポジトリのドキュメントを直すときは:

- **リファレンスは手で書かない。** `docs/REFERENCE.md` は
  `python docs/build_reference.py` が生成する。直すのはコードの docstring と
  argparse の `help` の方
- **コード例は動く形で書く。** 省略を `...` で書かない —— dict の中では
  SyntaxError になる。省略はコメントで示す
- **数式は GitHub で表示できる書き方に限る。** 引っかかるのは 2 つ:
  - 表示数式は `$$` を独立した行に置く (1 行形式だと行列の `\\` が潰れる)
  - `\operatorname` は使えない (`\mathrm` を使う)。GitHub の KaTeX が
    マクロを定義できる系のコマンドを禁止しているため
  ```bash
  npm install katex && node tools/check_math.js docs/*.md   # 実際に描画して確かめる
  ```
- 数字を書くときは**測ってから**。`tests/test_docs.py` がサンプルの実行と
  リンク切れとコード例の構文と数式の書き方を見ている
