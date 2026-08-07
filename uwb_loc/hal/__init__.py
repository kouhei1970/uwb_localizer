"""UWB HAL — チップごとの差をここで吸収する.

* :class:`~uwb_loc.hal.base.UwbHal` — 実装すべき 2 メソッドだけの抽象基底
* :class:`~uwb_loc.hal.jsonl.JsonLinesHal` — JSON Lines を読む汎用 HAL.
  ファームウェアが 1 行 1 JSON を吐けば, Python 側を書かずに繋がる
* :class:`~uwb_loc.sim.SimulatedHal` — 実機なしで動かす模擬 HAL
"""

from __future__ import annotations

from .base import UwbHal
from .jsonl import JsonLinesHal, JsonLinesWriter, parse_line

__all__ = ["UwbHal", "JsonLinesHal", "JsonLinesWriter", "parse_line"]
