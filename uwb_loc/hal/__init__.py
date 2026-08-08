"""UWB HAL — チップごとの差をここで吸収する.

* :class:`~uwb_loc.hal.base.UwbHal` — 実装すべき 2 メソッドだけの抽象基底
* :class:`~uwb_loc.hal.text.TextHal` — **既存ファームの出力をそのまま読む**.
  正規表現 1 本で済むので, ファームも Python も書かなくてよい
* :class:`~uwb_loc.hal.jsonl.JsonLinesHal` — JSON Lines を読む汎用 HAL.
  ファームウェアを自分で書けるならこちらが確実 (時刻・品質値を載せられる)
* :class:`~uwb_loc.hal.push.PushHal` — **読みに行けない経路**用.
  BLE 通知や MQTT のように「届いたら呼ばれる」形は, こちらに押し込む
* :class:`~uwb_loc.hal.ryuw122.Ryuw122Hal` — **REYAX RYUW122 専用**.
  AT コマンドで設定し, TAG を順に呼んで測距する
* :class:`~uwb_loc.sim.SimulatedHal` — 実機なしで動かす模擬 HAL
"""

from __future__ import annotations

from .base import UwbHal
from .jsonl import JsonLinesHal, JsonLinesWriter, parse_line
from .push import PushHal
from .ryuw122 import Ryuw122Config, Ryuw122Hal
from .text import TextHal, sniff

__all__ = ["UwbHal", "TextHal", "PushHal", "sniff",
           "Ryuw122Hal", "Ryuw122Config",
           "JsonLinesHal", "JsonLinesWriter", "parse_line"]
