# データシートの置き場

**ここにベンダのデータシートは置いていない。** 配布物の著作権はメーカーに
あり、本リポジトリの MIT ライセンスでは再配布できないため。

## RYUW122 / RYUW122_Lite AT Command Guide

REYAX の製品ページから入手する。

- <https://reyax.com/products/RYUW122>

本ライブラリの実装は **2024-03-12 版**を参照している。コード中の
「仕様書 N 節」はこの版の節番号。

手元に置いておきたい場合はこのディレクトリに入れてよい。
`.gitignore` で PDF を除外してあるので、誤ってコミットされることはない。

## 実装が参照している箇所

| 節 | 内容 | 実装 |
|---|---|---|
| 2 | `AT+MODE` (0: TAG / 1: ANCHOR) を最初に送る | `Ryuw122Config.commands` |
| 4 | `AT+IPR` 既定ボーレート 115200 | `Ryuw122Hal.from_serial` |
| 7, 8 | `AT+NETWORKID` / `AT+ADDRESS` は 8 バイト ASCII | `Ryuw122Config` |
| 9 | `AT+UID?` は書き換え不可の機体固有値 (96 bit) | `Ryuw122Terminal.info` |
| 11 | `AT+TAGD` デューティは 0 か 10〜28000 ms | `Ryuw122Config` |
| 13, 14 | ペイロード長の差は 3 バイト以内 | `Ryuw122Hal.payload` / `Ryuw122Tag.payload` |
| 16 | `+ANCHOR_RCV` の距離は **cm** | `parse_anchor_rcv` |
| 17 | `+TAG_RCV` は TAG が読まれた合図 | `Ryuw122Tag._worker` |
| 20 | `AT+FACTORY` の出荷時値は全機共通 | `docs/RYUW122.md` |
